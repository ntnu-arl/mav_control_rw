/*
 Copyright (c) 2015, Mina Kamel, ASL, ETH Zurich, Switzerland
 Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 Copyright (c) 2015, Michael Burri, ASL, ETH Zurich, Switzerland

 You can contact the author at <mina.kamel@mavt.ethz.ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <mav_linear_mpc/linear_mpc.h>

namespace mav_control {

constexpr int LinearModelPredictiveController::kStateSize;
constexpr int LinearModelPredictiveController::kInputSize;
constexpr int LinearModelPredictiveController::kMeasurementSize;
constexpr int LinearModelPredictiveController::kDisturbanceSize;
constexpr double LinearModelPredictiveController::kGravity;
constexpr int LinearModelPredictiveController::kPredictionHorizonSteps;

LinearModelPredictiveController::LinearModelPredictiveController(const ros::NodeHandle& nh,
                                                                 const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      initialized_parameters_(false),
      position_error_integration_(0, 0, 0),
      command_thrust_sigma_(0, 0, 0),
      command_moment_sigma_(0, 0, 0),
      encoders_(0,0,0),
      mpc_queue_(nh, private_nh, kPredictionHorizonSteps),
      disturbance_observer_(nh, private_nh),
      verbose_(false),
      solve_time_average_(0),
      steady_state_calculation_(nh, private_nh),
      received_first_odometry1_(false),
      received_first_odometryL_(false),
      received_first_encoders_(false)
{
  reset_integrator_service_server_ = nh_.advertiseService(
        "reset_integrator", &LinearModelPredictiveController::resetIntegratorServiceCallback, this);

  initializeParameters();

  mpc_queue_.initializeQueue(sampling_time_, prediction_sampling_time_);
  std::string stick_odom_topic = "stick/odom";
  odom_L_publisher = nh_.advertise<nav_msgs::Odometry>("stick/odom", 1, false);  
}

LinearModelPredictiveController::~LinearModelPredictiveController()
{

}

bool LinearModelPredictiveController::resetIntegratorServiceCallback(std_srvs::Empty::Request &req,
                                                                     std_srvs::Empty::Response &res)
{
  position_error_integration_.setZero();
  return true;
}

void LinearModelPredictiveController::initializeParameters()
{
  std::vector<double> drag_coefficients;
  std::vector<double> d1;
  std::vector<double> d2;

  //Get parameters from RosParam server
  private_nh_.param<bool>("verbose", verbose_, false);

  if (!private_nh_.getParam("mass1", mass1_)) {
    ROS_ERROR("mass1 in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("mass2", mass2_)) {
    ROS_ERROR("mass2 in MPC is not loaded from ros parameter server");
    abort();
  }  

  if (!private_nh_.getParam("length", length_)) {
    ROS_ERROR("length in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("joint1_offset", d1)) {
    ROS_ERROR("joint1 offset in MPC is not loaded from ros parameter server");
    abort();
  }

  d1_ << d1.at(0), d1.at(1), d1.at(2);  

  if (!private_nh_.getParam("joint2_offset", d2)) {
    ROS_ERROR("joint2 offset in MPC is not loaded from ros parameter server");
    abort();
  }  

  d2_ << d2.at(0), d2.at(1), d2.at(2);

  if (!private_nh_.getParam("drag_coefficients", drag_coefficients)) {
    ROS_ERROR("drag_coefficients in MPC is not loaded from ros parameter server");
    abort();
  }

  drag_coefficients_ << drag_coefficients.at(0), drag_coefficients.at(1), drag_coefficients.at(2);

  if (!private_nh_.getParam("position_error_integration_limit",
                            position_error_integration_limit_)) {
    ROS_ERROR("position_error_integration_limit in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("antiwindup_ball", antiwindup_ball_)) {
    ROS_ERROR("antiwindup_ball in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("prediction_sampling_time", prediction_sampling_time_)) {
    ROS_ERROR("prediction_sampling_time in MPC is not loaded from ros parameter server");
    abort();
  }

  //construct model matrices
  Eigen::MatrixXd A_continous_time(kStateSize, kStateSize);
  A_continous_time = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  Eigen::MatrixXd B_continous_time;
  B_continous_time = Eigen::MatrixXd::Zero(kStateSize, kInputSize);
  Eigen::MatrixXd Bd_continous_time;
  Bd_continous_time = Eigen::MatrixXd::Zero(kStateSize, kDisturbanceSize);

  A_continous_time(0, 3) = 1;
  A_continous_time(1, 4) = 1;
  A_continous_time(2, 5) = 1;
  A_continous_time(3, 3) = -drag_coefficients.at(0);
  A_continous_time(4, 4) = -drag_coefficients.at(1);
  A_continous_time(5, 5) = -drag_coefficients.at(2);

  B_continous_time(3, 0) = 1.0;
  B_continous_time(4, 1) = 1.0;
  B_continous_time(5, 2) = 1.0;

  Bd_continous_time(3, 0) = 1.0;
  Bd_continous_time(4, 1) = 1.0;
  Bd_continous_time(5, 2) = 1.0;

  model_A_ = (prediction_sampling_time_ * A_continous_time).exp();

  Eigen::MatrixXd integral_exp_A;
  integral_exp_A = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  const int count_integral_A = 100;

  for (int i = 0; i < count_integral_A; i++) {
    integral_exp_A += (A_continous_time * prediction_sampling_time_ * i / count_integral_A).exp()
        * prediction_sampling_time_ / count_integral_A;
  }

  model_B_ = integral_exp_A * B_continous_time;
  model_Bd_ = integral_exp_A * Bd_continous_time;

  steady_state_calculation_.initialize(model_A_, model_B_, model_Bd_);

  if (verbose_) {
    ROS_INFO_STREAM("A: \n" << model_A_);
    ROS_INFO_STREAM("B: \n" << model_B_);
    ROS_INFO_STREAM("B_d: \n" << model_Bd_);
  }

  //Solver initialization
  set_defaults();
  setup_indexing();

  //Solver settings
  settings.verbose = 0;

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.A), kStateSize, kStateSize) = model_A_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.B), kStateSize, kInputSize) = model_B_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Bd), kStateSize, kDisturbanceSize) =
      model_Bd_;

  initialized_parameters_ = true;

  ROS_INFO("Linear MPC: initialized correctly");
}

void LinearModelPredictiveController::applyParameters()
{
  Eigen::Matrix<double, kStateSize, kStateSize> Q;
  Eigen::Matrix<double, kStateSize, kStateSize> Q_final;
  Eigen::Matrix<double, kInputSize, kInputSize> R;
  Eigen::Matrix<double, kInputSize, kInputSize> R_delta;

  Q.setZero();
  Q_final.setZero();
  R.setZero();
  R_delta.setZero();

  Q.block(0, 0, 3, 3) = q_position_.asDiagonal();
  Q.block(3, 3, 3, 3) = q_velocity_.asDiagonal();

  R = r_command_.asDiagonal();

  R_delta = r_delta_command_.asDiagonal();

  //Compute terminal cost
  //Q_final(k+1) = A'*Q_final(k)*A - (A'*Q_final(k)*B)*inv(B'*Q_final(k)*B+R)*(B'*Q_final(k)*A)+ Q;
  Q_final = Q;
  for (int i = 0; i < 1000; i++) {
    Eigen::MatrixXd temp = (model_B_.transpose() * Q_final * model_B_ + R);
    Q_final = model_A_.transpose() * Q_final * model_A_
        - (model_A_.transpose() * Q_final * model_B_) * temp.inverse()
            * (model_B_.transpose() * Q_final * model_A_) + Q;
  }

  Eigen::MatrixXd temporary_matrix = model_B_.transpose() * Q_final * model_B_ + R;
  LQR_K_ = temporary_matrix.inverse() * (model_B_.transpose() * Q_final * model_A_);

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q_x), kStateSize, kStateSize) = Q;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.P), kStateSize, kStateSize) =
      Q_final;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R_u), kInputSize, kInputSize) = R;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R_delta), kInputSize, kInputSize) = R_delta
      * (1.0 / sampling_time_ * sampling_time_);

  params.u_max[0] = thrust_x_max_;
  params.u_max[1] = thrust_y_max_;
  params.u_max[2] = thrust_z_max_;

  params.u_min[0] = thrust_x_min_;
  params.u_min[1] = thrust_y_min_;
  params.u_min[2] = thrust_z_min_;

  ROS_INFO("Linear MPC: Tuning parameters updated...");
  if (verbose_) {
    ROS_INFO_STREAM("diag(Q) = \n" << Q.diagonal().transpose());
    ROS_INFO_STREAM("diag(R) = \n" << R.diagonal().transpose());
    ROS_INFO_STREAM("diag(R_delta) = \n " << R_delta.diagonal().transpose());
    ROS_INFO_STREAM("Q_final = \n" << Q_final);
  }
}

/* FAKE LENGTH */
void LinearModelPredictiveController::setOdometry2(const mav_msgs::EigenOdometry& odometry2)
{
  if (!received_first_encoders_)
  {
    return;
  }

  Eigen::Vector3d joint1_pos;
  Eigen::Vector3d joint2_pos;
  Eigen::Vector3d stick_dir;
  
  Eigen::Vector3d quad2_pos_ = odometry2.position_W;
  Eigen::Quaterniond quad2_orientation_ = odometry2.orientation_W_B;
  
  Eigen::Vector3d quad1_pos_ = odometry1_.position_W;
  Eigen::Quaterniond quad1_orientation_ = odometry1_.orientation_W_B;

  joint1_pos = quad1_pos_ + quad1_orientation_ * d1_;
  joint2_pos = quad2_pos_ + quad2_orientation_ * d2_;
  stick_dir = joint1_pos - joint2_pos;
  length_ = stick_dir.norm();
  ROS_INFO_STREAM_THROTTLE(1, "length:" << length_);
} 

void LinearModelPredictiveController::setOdometry(const mav_msgs::EigenOdometry& odometry1, mav_msgs::EigenOdometry* odometry_L_out)
{
  static mav_msgs::EigenOdometry previous_odometry1 = odometry1;
  if (!received_first_encoders_)
  {
    return;
  }

  if (odometry1.position_W.allFinite() == false) {
    odometry1_.position_W = previous_odometry1.position_W;
    ROS_WARN("Odometry.position has a non finite element");
  } else {
    odometry1_.position_W = odometry1.position_W;
    previous_odometry1.position_W = odometry1.position_W;
  }

  if (odometry1.velocity_B.allFinite() == false) {
    odometry1_.velocity_B = previous_odometry1.velocity_B;
    ROS_WARN("Odometry.velocity has a non finite element");
  } else {
    odometry1_.velocity_B = odometry1.velocity_B;
    previous_odometry1.velocity_B = odometry1.velocity_B;
  }

  if (odometry1.angular_velocity_B.allFinite() == false) {
    odometry1_.angular_velocity_B = previous_odometry1.angular_velocity_B;
    ROS_WARN("Odometry.angular_velocity has a non finite element");
  } else {
    odometry1_.angular_velocity_B = odometry1.angular_velocity_B;
    previous_odometry1.angular_velocity_B = odometry1.angular_velocity_B;
  }

  odometry1_.orientation_W_B = odometry1.orientation_W_B;
  previous_odometry1.orientation_W_B = odometry1.orientation_W_B;

  // calculate rotL
  Eigen::Matrix3d R_1L;
  R_1L =  Eigen::AngleAxisd(encoders_(0), Eigen::Vector3d::UnitZ())  // yaw
  * Eigen::AngleAxisd(encoders_(1), Eigen::Vector3d::UnitY())  // pitch
  * Eigen::AngleAxisd(encoders_(2), Eigen::Vector3d::UnitX());  // roll  
  
  odometryL_.orientation_W_B = odometry1_.orientation_W_B.toRotationMatrix() * R_1L;

  // calculate omegaL
  Eigen::Vector3d euler_angles_L;
  odometryL_.getEulerAngles(&euler_angles_L);
  static Eigen::Vector3d previous_euler_angles_L = euler_angles_L;
  if (!received_first_odometry1_)
  {
    odometryL_.angular_velocity_B << 0 , 0 , 0;
  }
  else
  {
    Eigen::Vector3d euler_L_rate = (euler_angles_L - previous_euler_angles_L)/ sampling_time_;
    odometryL_.angular_velocity_B(0) = euler_L_rate(0) - euler_L_rate(2) * sin(euler_angles_L(1));
    odometryL_.angular_velocity_B(1) = euler_L_rate(1)*cos(euler_angles_L(0)) + euler_L_rate(2)*cos(euler_angles_L(1))*sin(euler_angles_L(0));
    odometryL_.angular_velocity_B(2) = -euler_L_rate(1)*sin(euler_angles_L(0)) + euler_L_rate(2)*cos(euler_angles_L(1))*cos(euler_angles_L(0));
    previous_euler_angles_L = euler_angles_L;
  }

  // calculate xL
  Eigen::Vector3d L_half(-length_ * mass2_/(mass1_ + mass2_), 0, 0);
  /* TODO: CHANGE L_HALF -> depends on m1, m2 */
  odometryL_.position_W = odometry1_.position_W +  odometry1_.orientation_W_B * d1_ + odometryL_.orientation_W_B * L_half;

  // calculate vL
  Eigen::Vector3d velocity_L = odometry1_.getVelocityWorld() + odometry1_.orientation_W_B * odometry1_.angular_velocity_B.cross(d1_) 
          + odometryL_.orientation_W_B * odometryL_.angular_velocity_B.cross(L_half) ;
  odometryL_.setVelocityWorld(velocity_L);

  if (!received_first_odometry1_) {
    // HUAN

    disturbance_observer_.reset(odometryL_.position_W, odometryL_.getVelocityWorld(), Eigen::Vector3d::Zero());

    received_first_odometry1_ = true;
  }  
  odometryL_.timestamp_ns = ros::Time::now().toNSec(); 
  *odometry_L_out = odometryL_;
  nav_msgs::Odometry odom_L_cmd;
  mav_msgs::msgOdometryFromEigen(odometryL_, &odom_L_cmd);
  static uint32_t odomL_seq = 0;
  odom_L_cmd.header.seq = odomL_seq++;
  odom_L_cmd.header.frame_id = "/world";
  odom_L_publisher.publish(odom_L_cmd);
}

void LinearModelPredictiveController::setEncoder(const Eigen::Vector3d& encoders)// encoder yaw, pitch, roll in rad
{
  encoders_ = encoders;
  if (!received_first_encoders_)
  {
    received_first_encoders_ = true;
  }
}

void LinearModelPredictiveController::setCommandTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory)
{
  mpc_queue_.insertReference(command_trajectory);
}

void LinearModelPredictiveController::setCommandTrajectory(
    const mav_msgs::EigenTrajectoryPointDeque& command_trajectory_array)
{
  int array_size = command_trajectory_array.size();
  if (array_size < 1) {
    return;
  }

  mpc_queue_.insertReferenceTrajectory(command_trajectory_array);
}

void LinearModelPredictiveController::calculateAttitudeThrustCommand(
    Eigen::Vector4d *thrust_yaw_quad1, Eigen::Vector4d *thrust_yaw_quad2)
{
  //ROS_INFO("LinearModelPredictiveController::calculateAttitudeThrustCommand");
  assert(thrust_yaw_quad1 != nullptr);
  assert(thrust_yaw_quad2 != nullptr);
  assert(initialized_parameters_ == true);
  ros::WallTime starting_time = ros::WallTime::now();

  //Declare variables
  Eigen::Matrix<double, kMeasurementSize, 1> reference;
  Eigen::VectorXd KF_estimated_state;
  Eigen::Matrix<double, kDisturbanceSize, 1> estimated_disturbances;
  Eigen::Matrix<double, kStateSize, 1> x_0;

  // update mpc queue
  mpc_queue_.updateQueue();
  // Copy out the whole queues
  mpc_queue_.getQueue(position_ref_, velocity_ref_, acceleration_ref_, orientation_reference_, angular_velocity_W_reference_);

  // update the disturbance observer
  disturbance_observer_.feedThrustCommand(command_thrust_sigma_);
  disturbance_observer_.feedPositionMeasurement(odometryL_.position_W);
  disturbance_observer_.feedVelocityMeasurement(odometryL_.getVelocityWorld());
  //disturbance_observer_.feedRotationMatrix(odometry_.orientation_W_B.toRotationMatrix());

  bool observer_update_successful = disturbance_observer_.updateEstimator();

  if (!observer_update_successful) {
    // reset the disturbance observer
    disturbance_observer_.reset(odometryL_.position_W, odometryL_.getVelocityWorld(), Eigen::Vector3d::Zero());
  }

  disturbance_observer_.getEstimatedState(&KF_estimated_state);

  if (enable_offset_free_ == true) {
    estimated_disturbances = KF_estimated_state.segment(6, kDisturbanceSize);
  } else {
    estimated_disturbances.setZero();
  }

  if (enable_integrator_) {
    Eigen::Vector3d position_error = position_ref_.front() - odometryL_.position_W;
    if (position_error.norm() < antiwindup_ball_) {
      position_error_integration_ += position_error * sampling_time_;
    } else {
      position_error_integration_.setZero();
    }

    position_error_integration_ = position_error_integration_.cwiseMax(
        Eigen::Vector3d(-position_error_integration_limit_, -position_error_integration_limit_,
                        -position_error_integration_limit_));

    position_error_integration_ = position_error_integration_.cwiseMin(
        Eigen::Vector3d(position_error_integration_limit_, position_error_integration_limit_,
                        position_error_integration_limit_));

    estimated_disturbances -= Eigen::Vector3d(Ki_xy_, Ki_xy_, Ki_altitude_).asDiagonal()
        * position_error_integration_;
  }

  Eigen::Matrix<double, kStateSize, 1> target_state;
  Eigen::Matrix<double, kInputSize, 1> target_input;
  Eigen::VectorXd ref(kMeasurementSize);

  CVXGEN_queue_.clear();
  for (int i = 0; i < kPredictionHorizonSteps - 1; i++) {
    ref << position_ref_.at(i), velocity_ref_.at(i);
    steady_state_calculation_.computeSteadyState(estimated_disturbances, ref, &target_state,
                                                 &target_input);
    CVXGEN_queue_.push_back(target_state);
    // HUAN
    //if (i == 0) {
      Eigen::Map<Eigen::Matrix<double, kInputSize, 1>>(const_cast<double*>(params.u_ss[i])) =
          target_input;
    //}
  }
  ref << position_ref_.at(kPredictionHorizonSteps - 1), velocity_ref_.at(
      kPredictionHorizonSteps - 1);

  steady_state_calculation_.computeSteadyState(estimated_disturbances, ref, &target_state,
                                               &target_input);
  CVXGEN_queue_.push_back(target_state);

  for (int i = 0; i < kPredictionHorizonSteps; i++) {
    Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.x_ss[i]), kStateSize, 1) =
        CVXGEN_queue_[i];
  }

  x_0 << odometryL_.position_W, odometryL_.getVelocityWorld();

  //Solve using CVXGEN
  Eigen::Map<Eigen::Matrix<double, kDisturbanceSize, 1>>(const_cast<double*>(params.d)) =
      estimated_disturbances;
  Eigen::Map<Eigen::Matrix<double, kInputSize, 1>>(const_cast<double*>(params.u_prev)) =
      command_thrust_sigma_;
  Eigen::Map<Eigen::Matrix<double, kStateSize, 1>>(const_cast<double*>(params.x_0)) = x_0;

  tic();
  int solver_status = solve();
  solve_time_average_ += tocq();

  command_thrust_sigma_ << vars.u_0[0], vars.u_0[1], vars.u_0[2];

  if (solver_status < 0) {
    ROS_WARN("Linear MPC: Solver faild, use LQR backup");
    command_thrust_sigma_ = LQR_K_ * (target_state - x_0);
    command_thrust_sigma_ = command_thrust_sigma_.cwiseMax(
        Eigen::Vector3d(thrust_x_max_, thrust_y_max_, thrust_z_max_));
    command_thrust_sigma_ = command_thrust_sigma_.cwiseMin(
        Eigen::Vector3d(thrust_x_min_, thrust_y_min_, thrust_z_min_));
  }

  /* TODO: uncomment next lines */
  Eigen::Vector3d acc_ref = acceleration_ref_.front();
  //Eigen::Vector3d acc_ref(0,0,0);
  command_thrust_sigma_(0) = command_thrust_sigma_(0) + acc_ref(0);
  command_thrust_sigma_(1) = command_thrust_sigma_(1) + acc_ref(1);
  command_thrust_sigma_(2) = command_thrust_sigma_(2) + acc_ref(2) + kGravity;
  
  // angular controller
  Eigen::Matrix3d R_des = orientation_reference_.at(0).toRotationMatrix();
  Eigen::Matrix3d R = odometryL_.orientation_W_B.toRotationMatrix();
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  Eigen::Vector3d angle_error;
  angle_error << angle_error_matrix(2, 1),  angle_error_matrix(0, 2), angle_error_matrix(1,0);  
  Eigen::Vector3d angular_rate_error = odometryL_.angular_velocity_B - R.transpose() * angular_velocity_W_reference_.at(0);

  command_moment_sigma_ = -1 * angle_error.cwiseProduct(gain_attitude_) - angular_rate_error.cwiseProduct(gain_angular_rate_);
  command_moment_sigma_ = R * command_moment_sigma_.cross(Eigen::Vector3d::UnitX());
  Eigen::Vector3d command_rotation_force_acc = command_moment_sigma_ / length_;// force: N
  Eigen::Vector3d command_rotation_force_;

  if (command_rotation_force_acc(0) > rot_acc_x_max_)
    command_rotation_force_acc(0) = rot_acc_x_max_;
  else if (command_rotation_force_acc(0) < rot_acc_x_min_)
    command_rotation_force_acc(0) = rot_acc_x_min_;
    
  if (command_rotation_force_acc(1) > rot_acc_y_max_)
    command_rotation_force_acc(1) = rot_acc_y_max_;
  else if (command_rotation_force_acc(1) < rot_acc_y_min_)
    command_rotation_force_acc(1) = rot_acc_y_min_;
    
  if (command_rotation_force_acc(2) > rot_acc_z_max_)
    command_rotation_force_acc(2) = rot_acc_z_max_;
  else if (command_rotation_force_acc(2) < rot_acc_z_min_)
    command_rotation_force_acc(2) = rot_acc_z_min_;        

  if (mass2_ > mass1_)
  {  
    command_rotation_force_ = command_rotation_force_acc * mass1_;
  }
  else
  {
    command_rotation_force_ = command_rotation_force_acc * mass2_;    
  }

  Eigen::Vector3d thrust_yaw_quad1_tmp = command_thrust_sigma_ * mass1_ + command_rotation_force_;
  *thrust_yaw_quad1 << thrust_yaw_quad1_tmp(0), thrust_yaw_quad1_tmp(1), thrust_yaw_quad1_tmp(2), 0; // keep current yaw angle
  Eigen::Vector3d thrust_yaw_quad2_tmp = command_thrust_sigma_ * mass2_ - command_rotation_force_;
  *thrust_yaw_quad2 << thrust_yaw_quad2_tmp(0), thrust_yaw_quad2_tmp(1), thrust_yaw_quad2_tmp(2), 0; // keep current yaw angle
  
  double diff_time = (ros::WallTime::now() - starting_time).toSec();

  if (verbose_) {
    static int counter = 0;
    if (counter > 1/sampling_time_) {
      ROS_INFO_STREAM("average solve time: " << 1000.0 * solve_time_average_ / counter << " ms");
      solve_time_average_ = 0.0;

      ROS_INFO_STREAM("Controller loop time : " << diff_time * 1000.0 << " ms");
      Eigen::Vector3d pos_ref = position_ref_.front();
      ROS_INFO_STREAM("Pos ref:" << pos_ref(0) << "," << pos_ref(1) << "," << pos_ref(2));
      ROS_INFO_STREAM("Pos current:" << odometryL_.position_W(0) << "," << odometryL_.position_W(1) << "," << odometryL_.position_W(2));
      // Eigen::Vector3d vel = odometryL_.getVelocityWorld();
      // ROS_INFO_STREAM("Vel current:" << vel(0) << "," << vel(1) << "," << vel(2));
      
      //ROS_INFO_STREAM("Angle ref:" << orientation_reference_);
      Eigen::Vector3d euler_angles_L_des;
      mav_msgs::getEulerAnglesFromQuaternion(orientation_reference_.at(0), &euler_angles_L_des);
      Eigen::Vector3d angular_rate_ref = angular_velocity_W_reference_.at(0);
      ROS_INFO_STREAM("Angle ref:" << euler_angles_L_des(0) << "," << euler_angles_L_des(1) << "," << euler_angles_L_des(2) << "\t"
          << "Angular vel ref W:" << angular_rate_ref(0) << "," << angular_rate_ref(1) << "," << angular_rate_ref(2));
      Eigen::Vector3d euler_angles_L;
      odometryL_.getEulerAngles(&euler_angles_L);      
      ROS_INFO_STREAM("Angle current:" << euler_angles_L(0) << "," << euler_angles_L(1) << "," << euler_angles_L(2));
      //ROS_INFO_STREAM("Angle error:" << angle_error);
      //ROS_INFO_STREAM("Angular rate error:" << angular_rate_error);
      ROS_INFO_STREAM(
          "Thrust force: " << command_thrust_sigma_(0) * mass1_ << "," << command_thrust_sigma_(1) * mass1_ << 
          "," << command_thrust_sigma_(2) * mass1_ << "\t" 
          << "Moment force: " << command_rotation_force_acc(0) * mass1_<< "," << command_rotation_force_acc(1) * mass1_ 
          << "," << command_rotation_force_acc(2) * mass1_);
      counter = 0;
    }
    counter++;
  }
}

bool LinearModelPredictiveController::getCurrentReference(
    mav_msgs::EigenTrajectoryPoint* reference) const
{
  assert(reference != nullptr);

  (*reference).position_W = position_ref_.front();
  (*reference).velocity_W = velocity_ref_.front();
  (*reference).setFromYaw(yaw_ref_.front());

  return true;
}

bool LinearModelPredictiveController::getCurrentReference(
    mav_msgs::EigenTrajectoryPointDeque* reference) const
{
  assert(reference != nullptr);

  (*reference).clear();

  for (int i = 0; i < position_ref_.size(); i++) {
    mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = position_ref_.at(i);
    pnt.setFromYaw(yaw_ref_.at(i));
    (*reference).push_back(pnt);
  }
  return true;
}

bool LinearModelPredictiveController::getPredictedState(
    mav_msgs::EigenTrajectoryPointDeque* predicted_state) const
{
  assert(predicted_state != nullptr);

  for (size_t i = 1; i < kPredictionHorizonSteps; i++) {
    mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = Eigen::Vector3d(vars.x[i][0], vars.x[i][1], vars.x[i][2]);
    (*predicted_state).push_back(pnt);
  }

  return true;
}

}
