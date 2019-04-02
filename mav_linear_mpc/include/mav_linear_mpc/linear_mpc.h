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

#ifndef MPC_POSITION_CONTROLLER_H
#define MPC_POSITION_CONTROLLER_H

#include <memory>
#include <mav_disturbance_observer/KF_disturbance_observer.h>
#include <mav_linear_mpc/steady_state_calculation.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/common.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_control_interface/mpc_queue.h>
#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <deque>
#include <Eigen/Eigen>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include <nav_msgs/Odometry.h>

#include <solver.h>

namespace mav_control {

class LinearModelPredictiveController
{
 public:

  LinearModelPredictiveController(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~LinearModelPredictiveController();

  // Dynamic parameters
  void setPositionPenality(const Eigen::Vector3d& q_position)
  {
    q_position_ = q_position;
  }
  void setVelocityPenality(const Eigen::Vector3d& q_velocity)
  {
    q_velocity_ = q_velocity;
  }
  
  void setCommandPenality(const Eigen::Vector3d& r_command)
  {
    r_command_ = r_command;
  }
  void setDeltaCommandPenality(const Eigen::Vector3d& r_delta_command)
  {
    r_delta_command_ = r_delta_command;
  }

  void setAltitudeIntratorGain(double Ki_altitude)
  {
    Ki_altitude_ = Ki_altitude;
  }

  void setXYIntratorGain(double Ki_xy)
  {
    Ki_xy_ = Ki_xy;
  }

  void setEnableOffsetFree(bool enable_offset_free)
  {
    enable_offset_free_ = enable_offset_free;
  }

  void setEnableIntegrator(bool enable_integrator)
  {
    enable_integrator_ = enable_integrator;
  }

  void setControlLimits(const Eigen::VectorXd& control_limits)
  {
    //roll_max, pitch_max, yaw_rate_max, thrust_min and thrust_max
    thrust_x_min_ = -control_limits(0);
    thrust_x_max_ = control_limits(0);
    thrust_y_min_ = -control_limits(1);
    thrust_y_max_ = control_limits(1);
    thrust_z_min_ = control_limits(2) - kGravity;
    thrust_z_max_ = control_limits(3) - kGravity;    
    
    rot_acc_x_min_ = -control_limits(4);
    rot_acc_x_max_ = control_limits(4);
    rot_acc_y_min_ = -control_limits(5);
    rot_acc_y_max_ = control_limits(5);
    rot_acc_z_min_ = -control_limits(6);
    rot_acc_z_max_ = control_limits(6);
  }

  void setAttitudeGain(const Eigen::Vector3d& gain_attitude)
  {
    gain_attitude_ = gain_attitude;
    ROS_INFO_STREAM("gain_attitude = \n" << gain_attitude_);
  }

  void setAngularRateGain(const Eigen::Vector3d& gain_angular_rate)
  {
    gain_angular_rate_ = gain_angular_rate;
    ROS_INFO_STREAM("gain_angular_rate = \n" << gain_angular_rate);   
  }  

  void applyParameters();

  double getMass1() const
  {
    return mass1_;
  }

  double getMass2() const
  {
    return mass2_;
  }

  double getLinkLength() const
  {
    return length_;
  }

  void getRotationL_WB(Eigen::Matrix3d* rotation_matrix) 
  {
    *rotation_matrix = odometryL_.orientation_W_B.toRotationMatrix();
  }

  // get reference and predicted state
  bool getCurrentReference(mav_msgs::EigenTrajectoryPoint* reference) const;
  bool getCurrentReference(mav_msgs::EigenTrajectoryPointDeque* reference) const;
  bool getPredictedState(mav_msgs::EigenTrajectoryPointDeque* predicted_state) const;

  // set odom and commands
  void setOdometry(const mav_msgs::EigenOdometry& odometry, mav_msgs::EigenOdometry* odometry_L_out);
  void setEncoder(const Eigen::Vector3d& encoders);  
  void setCommandTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
  void setCommandTrajectory(const mav_msgs::EigenTrajectoryPointDeque& command_trajectory);

  // compute control input
  void calculateAttitudeThrustCommand(Eigen::Vector4d *thrust_yaw_quad1, Eigen::Vector4d *thrust_yaw_quad2);
  
  /* FAKE LENGTH */
  void setOdometry2(const mav_msgs::EigenOdometry& odometry2);  

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:

  // constants
  static constexpr int kStateSize = 6;
  static constexpr int kInputSize = 3;
  static constexpr int kMeasurementSize = 6;
  static constexpr int kDisturbanceSize = 3;
  static constexpr int kPredictionHorizonSteps = 20;
  static constexpr double kGravity = 9.8066;

  // ros node handles
  ros::NodeHandle nh_, private_nh_;

  // reset integrator service
  ros::ServiceServer reset_integrator_service_server_;
  bool resetIntegratorServiceCallback(std_srvs::Empty::Request  &req,
                                      std_srvs::Empty::Response &res);


  //initialize parameters
  void initializeParameters();
  bool initialized_parameters_;

  // sampling time parameters
  double sampling_time_;
  double prediction_sampling_time_;

  // system model parameters
  // Model: A, B, Bd
  // x(k+1) = A*x(k) + B*u(k) + Bd*d(k)
  Eigen::Matrix<double, kStateSize, kStateSize> model_A_;   //dynamics matrix
  Eigen::Matrix<double, kStateSize, kInputSize> model_B_;   //transfer matrix
  Eigen::Matrix<double, kStateSize, kInputSize> model_Bd_;  //Disturbance transfer matrix
  //double roll_time_constant_;
  //double roll_gain_;
  //double pitch_time_constant_;
  //double pitch_gain_;
  Eigen::Vector3d drag_coefficients_;
  double mass1_;
  double mass2_;
  double length_;
  Eigen::Vector3d d1_;
  Eigen::Vector3d d2_;

  // controller parameters
  // state penalty
  Eigen::Vector3d q_position_;
  Eigen::Vector3d q_velocity_;
  //Eigen::Vector2d q_attitude_;

  // control penalty
  Eigen::Vector3d r_command_;
  Eigen::Vector3d r_delta_command_;

  // yaw P gain
  //double K_yaw_;
  Eigen::Vector3d gain_attitude_;
  Eigen::Vector3d gain_angular_rate_;


  // backup LQR
  Eigen::MatrixXd LQR_K_;

  // error integrator
  bool enable_integrator_;
  double Ki_altitude_;
  double Ki_xy_;
  double antiwindup_ball_;
  Eigen::Vector3d position_error_integration_;
  double position_error_integration_limit_;

  // control input limits
  double thrust_x_min_;
  double thrust_x_max_;
  double thrust_y_min_;
  double thrust_y_max_;
  double thrust_z_min_;
  double thrust_z_max_;  

  double rot_acc_x_min_;
  double rot_acc_x_max_;
  double rot_acc_y_min_;
  double rot_acc_y_max_;
  double rot_acc_z_min_;
  double rot_acc_z_max_;    

  // reference queue
  MPCQueue mpc_queue_;
  Vector3dDeque position_ref_, velocity_ref_, acceleration_ref_;
  std::deque<double> yaw_ref_, yaw_rate_ref_;
  QuaterniondDeque orientation_reference_;
  Vector3dDeque angular_velocity_W_reference_;
  // solver queue
  std::deque<Eigen::Matrix<double, kStateSize, 1>> CVXGEN_queue_;

  // disturbance observer
  bool enable_offset_free_;
  KFDisturbanceObserver disturbance_observer_;

  // commands
  Eigen::Vector3d command_thrust_sigma_;  //actual roll, pitch, yaw, thrust command
  Eigen::Vector3d command_moment_sigma_;

  // steady state calculation
  SteadyStateCalculation steady_state_calculation_;

  // debug info
  bool verbose_;
  double solve_time_average_;

  // most recent odometry information
  mav_msgs::EigenOdometry odometry1_;
  bool received_first_odometry1_;

  mav_msgs::EigenOdometry odometryL_;
  bool received_first_odometryL_;

  Eigen::Vector3d encoders_;
  bool received_first_encoders_;
  
  ros::Publisher odom_L_publisher;

};

}  // end namespace mav_control

#endif
