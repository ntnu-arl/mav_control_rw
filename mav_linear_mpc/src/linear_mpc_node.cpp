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

#include <Eigen/Geometry>

#include <mav_msgs/default_topics.h>

#include <mav_control_interface/mav_control_interface_cooperative.h>
#include <mav_control_interface/rc_interface_aci_cooperative.h>

#include <mav_linear_mpc/linear_mpc_node.h>

namespace mav_control {
// REMOVE const in node handle to FAKE LENGTH
LinearModelPredictiveControllerNode::LinearModelPredictiveControllerNode(
    ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : linear_mpc_(nh, private_nh),
      dyn_config_server_(private_nh)
{
  dynamic_reconfigure::Server<mav_linear_mpc::LinearMPCConfig>::CallbackType f;
  f = boost::bind(&LinearModelPredictiveControllerNode::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);

  /* FAKE LENGTH */
  odom2_subscriber_ = nh.subscribe("/quad2/msf_core/odometry", 10, &LinearModelPredictiveControllerNode::Odom2PoseCallback, this);  
}

LinearModelPredictiveControllerNode::~LinearModelPredictiveControllerNode()
{

}

void LinearModelPredictiveControllerNode::DynConfigCallback(mav_linear_mpc::LinearMPCConfig &config,
                                                            uint32_t level)
{
  Eigen::Vector3d q_position;
  Eigen::Vector3d q_velocity;

  Eigen::Vector3d r_command;
  Eigen::Vector3d r_delta_command;
  Eigen::VectorXd control_limits(12);

  Eigen::Vector3d K_R, K_omega;

  q_position << config.q_x, config.q_y, config.q_z;
  q_velocity << config.q_vx, config.q_vy, config.q_vz;

  r_command << config.r_thrust_x, config.r_thrust_y, config.r_thrust_z;
  r_delta_command << config.r_dthrust_x, config.r_dthrust_y, config.r_dthrust_z;

  control_limits << config.thrust_x_max, config.thrust_y_max, config.thrust_z_min, config.thrust_z_max,
    config.rot_acc_x_max, config.rot_acc_y_max, config.rot_acc_z_max;

  K_R << config.KR_x, config.KR_y, config.KR_z;
  K_omega << config.K_omega_x, config.K_omega_y, config.K_omega_z;

  linear_mpc_.setPositionPenality(q_position);
  linear_mpc_.setVelocityPenality(q_velocity);
  linear_mpc_.setCommandPenality(r_command);
  linear_mpc_.setDeltaCommandPenality(r_delta_command);
  /* TODO: replace with set params for angular controller */
  //linear_mpc_.setYawGain(config.K_yaw);
  linear_mpc_.setAttitudeGain(K_R);
  linear_mpc_.setAngularRateGain(K_omega);
  linear_mpc_.setControlLimits(control_limits);

  linear_mpc_.setAltitudeIntratorGain(config.Ki_altitude);
  linear_mpc_.setXYIntratorGain(config.Ki_xy);

  linear_mpc_.setEnableIntegrator(config.enable_integrator);
  linear_mpc_.setEnableOffsetFree(config.enable_offset_free);

  linear_mpc_.applyParameters();
}

bool LinearModelPredictiveControllerNode::setReference(
    const mav_msgs::EigenTrajectoryPoint& reference)
{
  linear_mpc_.setCommandTrajectoryPoint(reference);
  return true;
}

bool LinearModelPredictiveControllerNode::setReferenceArray(
    const mav_msgs::EigenTrajectoryPointDeque& reference_array)
{
  linear_mpc_.setCommandTrajectory(reference_array);
  return true;
}

bool LinearModelPredictiveControllerNode::setOdometry(const mav_msgs::EigenOdometry& odometry, mav_msgs::EigenOdometry* odometry_L_out)
{
  linear_mpc_.setOdometry(odometry, odometry_L_out);
  return true;
}

bool LinearModelPredictiveControllerNode::setEncoder(const Eigen::Vector3d& encoders)// encoder yaw, pitch, roll
{
  linear_mpc_.setEncoder(encoders);//rad, =2*pi/4096  
  return true;
}

bool LinearModelPredictiveControllerNode::calculateAttitudeThrustCommandCentralize(
    mav_msgs::EigenAttitudeThrust* attitude_thrust_command_quad1, mav_msgs:: EigenAttitudeThrust* attitude_thrust_command_quad2)
{
  Eigen::Vector4d cmd_quad1, cmd_quad2;
  //ROS_INFO("LinearModelPredictiveControllerNode::calculateAttitudeThrustCommandCentralize");
  linear_mpc_.calculateAttitudeThrustCommand(&cmd_quad1, &cmd_quad2);
  
  attitude_thrust_command_quad1->thrust.x() = cmd_quad1(0);
  attitude_thrust_command_quad1->thrust.y() = cmd_quad1(1);
  attitude_thrust_command_quad1->thrust.z() = cmd_quad1(2);
  attitude_thrust_command_quad1->attitude = Eigen::AngleAxisd(cmd_quad1(3), Eigen::Vector3d::UnitZ());

  attitude_thrust_command_quad2->thrust.x() = cmd_quad2(0);
  attitude_thrust_command_quad2->thrust.y() = cmd_quad2(1);
  attitude_thrust_command_quad2->thrust.z() = cmd_quad2(2);
  attitude_thrust_command_quad2->attitude = Eigen::AngleAxisd(cmd_quad2(3), Eigen::Vector3d::UnitZ());  

  return true;
}

// bool calculateRollPitchYawrateThrustCommand(
//     mav_msgs::EigenRollPitchYawrateThrust* attitude_thrust_command)
// {
//   ROS_WARN("calculateRollPitchYawrateThrustCommand not implemented");
//   return false;
// }

// bool LinearModelPredictiveControllerNode::calculateAttitudeThrustCommand(
//     mav_msgs::EigenAttitudeThrust* attitude_thrust_command)
// {
//   ROS_WARN("calculateAttitudeThrustCommand not implemented");
//   return false;
// }

bool LinearModelPredictiveControllerNode::getCurrentReference(
    mav_msgs::EigenTrajectoryPoint* reference) const
{
  assert(reference != nullptr);
  return linear_mpc_.getCurrentReference(reference);
}

bool LinearModelPredictiveControllerNode::getCurrentReference(
    mav_msgs::EigenTrajectoryPointDeque* reference) const
{
  assert(reference != nullptr);
  return linear_mpc_.getCurrentReference(reference);
}

bool LinearModelPredictiveControllerNode::getPredictedState(
    mav_msgs::EigenTrajectoryPointDeque* predicted_state) const
{
  assert(predicted_state != nullptr);
  return linear_mpc_.getPredictedState(predicted_state);
}

/* FAKE LENGTH */
void LinearModelPredictiveControllerNode::Odom2PoseCallback(const nav_msgs::OdometryConstPtr &odometry_msg)
{
  mav_msgs::EigenOdometry odometry2;  
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry2);
  linear_mpc_.setOdometry2(odometry2);  
}
}  // end namespace mav_control
int main(int argc, char** argv)
{
  ros::init(argc, argv, "LinearModelPredictiveControllerNode");

  ros::NodeHandle nh, private_nh("~");

  std::shared_ptr<mav_control::LinearModelPredictiveControllerNode> mpc(
      new mav_control::LinearModelPredictiveControllerNode(nh, private_nh));

  std::shared_ptr<mav_control_interface::RcInterfaceAciCooperative> rc(
      new mav_control_interface::RcInterfaceAciCooperative(nh));

  mav_control_interface::MavControlInterfaceCooperative control_interface(nh, private_nh, mpc, rc);

  ros::spin();

  return 0;
}
