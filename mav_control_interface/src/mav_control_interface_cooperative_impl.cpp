/*
 * Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <Eigen/Geometry>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Vector3.h>
#include "mav_control_interface_cooperative_impl.h"
#include "parameters.h"

namespace mav_control_interface {

constexpr double MavControlInterfaceCooperativeImpl::kOdometryWatchdogTimeout;

MavControlInterfaceCooperativeImpl::MavControlInterfaceCooperativeImpl(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                                                 std::shared_ptr<PositionControllerInterfaceCooperative> controller,
                                                 std::shared_ptr<RcInterfaceBase> rc_interface)
    : nh_(nh),
      private_nh_(private_nh),
      rc_interface_(rc_interface)
{
  ros::NodeHandle interface_nh(private_nh, "control_interface");

  odometry_watchdog_ = nh_.createTimer(ros::Duration(kOdometryWatchdogTimeout),
                                       &MavControlInterfaceCooperativeImpl::OdometryWatchdogCallback, this, false, true);

  encoder_watchdog_ = nh_.createTimer(ros::Duration(kOdometryWatchdogTimeout),
                                       &MavControlInterfaceCooperativeImpl::EncoderWatchdogCallback, this, false, true);                                       

  command_trajectory_subscriber_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_POSE, 1,
                                                 &MavControlInterfaceCooperativeImpl::CommandPoseCallback, this);

  command_trajectory_array_subscriber_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &MavControlInterfaceCooperativeImpl::CommandTrajectoryCallback, this);

  odometry_subscriber_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                       &MavControlInterfaceCooperativeImpl::OdometryCallback, this,
                                       ros::TransportHints().tcpNoDelay());

  encoder_subscriber_ = nh_.subscribe("encoder", 1,
                                       &MavControlInterfaceCooperativeImpl::EncoderCallback, this,
                                       ros::TransportHints().tcpNoDelay());

  rc_interface_->registerUpdatedCallback(&MavControlInterfaceCooperativeImpl::RcUpdatedCallback, this);

  takeoff_server_ = nh.advertiseService("takeoff", &MavControlInterfaceCooperativeImpl::TakeoffCallback, this);
  back_to_position_hold_server_ = nh.advertiseService("back_to_position_hold",
                                                      &MavControlInterfaceCooperativeImpl::BackToPositionHoldCallback,
                                                      this);

  state_machine_.reset(new state_machine_cooperative::StateMachineCooperative(nh_, private_nh_, controller));

  Parameters p;
  interface_nh.param("rc_teleop_max_carrot_distance_position", p.rc_teleop_max_carrot_distance_position_,
                     Parameters::kDefaultRcTeleopMaxCarrotDistancePosition);//?????

  interface_nh.param("rc_teleop_max_carrot_distance_yaw", p.rc_teleop_max_carrot_distance_yaw_,
                     Parameters::kDefaultRcTeleopMaxCarrotDistanceYaw);//?????

  if (p.rc_teleop_max_carrot_distance_yaw_ > M_PI / 2.0) {
    p.rc_teleop_max_carrot_distance_yaw_ = M_PI / 2.0;
    ROS_WARN("rc_teleop_max_carrot_distance_yaw_ was limited to pi/2. This is by far enough.");
  }

  interface_nh.param("rc_max_roll_pitch_command", p.rc_max_roll_pitch_command_,
                     Parameters::kDefaultRcMaxRollPitchCommand);
  interface_nh.param("rc_max_yaw_rate_command", p.rc_max_yaw_rate_command_,
                     Parameters::kDefaultRcMaxYawRateCommand);
  interface_nh.param("takeoff_distance", p.takeoff_distance_, Parameters::kDefaultTakeoffDistance);
  interface_nh.param("takeoff_time", p.takeoff_time_, Parameters::kDefaultTakeoffTime);

  p.stick_deadzone_ = Deadzone<double>(rc_interface->getStickDeadzone());
  state_machine_->SetParameters(p);

  ROS_INFO_STREAM("Created control interface for controller " << controller->getName() <<
                  " and RC " << rc_interface_->getName());

  state_machine_->start();
}

MavControlInterfaceCooperativeImpl::~MavControlInterfaceCooperativeImpl()
{
  state_machine_->stop();
}

void MavControlInterfaceCooperativeImpl::RcUpdatedCallback(const RcInterfaceBase& rc_interface)
{
  state_machine_->process_event(//?????: why not use rc_interface variable
      state_machine_cooperative::RcUpdate(rc_interface_->getRcData(), rc_interface_->isActive(), rc_interface_->isOn()));
}

void MavControlInterfaceCooperativeImpl::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{ 
  mav_msgs::EigenTrajectoryPoint reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*msg, &reference);

  // HUAN
  static mav_msgs::EigenTrajectoryPoint last_reference = reference;
  double dt = (reference.timestamp_ns - last_reference.timestamp_ns)*0.000000001;
  // calculate angular velocity if timestamps of both msgs are not too far from each other
  if ((dt <= 0.1) && (dt > 0))
  {
    double roll_rate, pitch_rate, yaw_rate;
    Eigen::Vector3d euler_ref;
    Eigen::Vector3d last_euler_ref;
    mav_msgs::getEulerAnglesFromQuaternion(reference.orientation_W_B, &euler_ref);
    mav_msgs::getEulerAnglesFromQuaternion(last_reference.orientation_W_B, &last_euler_ref);
    roll_rate = (euler_ref(0) - last_euler_ref(0))/dt;
    pitch_rate = (euler_ref(1) - last_euler_ref(1))/dt;
    yaw_rate = (euler_ref(2) - last_euler_ref(2))/dt;
    Eigen::Vector3d angular_velocity_B;
    angular_velocity_B(0) = roll_rate - yaw_rate*sin(euler_ref(1));
    angular_velocity_B(1) = pitch_rate*cos(euler_ref(0)) + yaw_rate*cos(euler_ref(1))*sin(euler_ref(0));
    angular_velocity_B(2) = -pitch_rate*sin(euler_ref(0)) + yaw_rate*cos(euler_ref(1))*cos(euler_ref(0));
    reference.angular_velocity_W = reference.orientation_W_B.toRotationMatrix() * angular_velocity_B;
    //ROS_INFO_STREAM("Euler rate:" << roll_rate << "," << pitch_rate << "," << yaw_rate);
    //ROS_INFO_STREAM("Ref angular vel:" << reference.angular_velocity_W.transpose());
  }
  mav_msgs::EigenTrajectoryPointDeque references;
  references.push_back(reference);

  state_machine_->process_event(state_machine_cooperative::ReferenceUpdate(references));
  last_reference = reference;
}

void MavControlInterfaceCooperativeImpl::CommandTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg)
{
  int array_size = msg->points.size();
  if (array_size == 0)
    return;

  mav_msgs::EigenTrajectoryPointDeque references;
  mav_msgs::eigenTrajectoryPointDequeFromMsg(*msg, &references);

  state_machine_->process_event(state_machine_cooperative::ReferenceUpdate(references));
}

void MavControlInterfaceCooperativeImpl::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("Control interface got first odometry message.");
  //ROS_INFO_THROTTLE(1,"Got odom");
  mav_msgs::EigenOdometry odometry;
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
  // Stamp odometry upon reception to be robust against timestamps "in the future".
  odometry.timestamp_ns = ros::Time::now().toNSec();
  state_machine_->process_event(state_machine_cooperative::OdometryUpdate(odometry));
}

void MavControlInterfaceCooperativeImpl::EncoderCallback(const geometry_msgs::Vector3::ConstPtr& encoder_msg)
{
  ROS_INFO_ONCE("Control interface got first encoder message.");
  //ROS_INFO_THROTTLE(1,"Got enc");
  //mav_msgs::EigenOdometry odometry;
  //mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
  // Stamp odometry upon reception to be robust against timestamps "in the future".
  //odometry.timestamp_ns = ros::Time::now().toNSec();
  Eigen::Vector3d encoder_data;
  encoder_data << encoder_msg->x, encoder_msg->y, encoder_msg->z;
  state_machine_->process_event(state_machine_cooperative::EncoderUpdate(encoder_data));
}

void MavControlInterfaceCooperativeImpl::OdometryWatchdogCallback(const ros::TimerEvent& e)
{
  state_machine_->process_event(state_machine_cooperative::OdometryWatchdog());
}

void MavControlInterfaceCooperativeImpl::EncoderWatchdogCallback(const ros::TimerEvent& e)
{
  state_machine_->process_event(state_machine_cooperative::EncoderWatchdog());
}

bool MavControlInterfaceCooperativeImpl::TakeoffCallback(std_srvs::Empty::Request& request,
                                              std_srvs::Empty::Response& response)
{
  ROS_INFO("Take off event sent");
  state_machine_->process_event(state_machine_cooperative::Takeoff());
  return true;
}

bool MavControlInterfaceCooperativeImpl::BackToPositionHoldCallback(std_srvs::Empty::Request& request,
                                                         std_srvs::Empty::Response& response)
{
  state_machine_->process_event(state_machine_cooperative::BackToPositionHold());
  return true;
}

}  // end namespace mav_control_interface

