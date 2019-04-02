#include <Eigen/Eigen>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include "command_converter_2.h"

static ros::Subscriber odometry_subscriber;
static ros::Subscriber attitude_thrust_subscriber;
static ros::Publisher rpyrate_thrust_publisher;
static double current_roll, current_pitch, current_yaw;
static mav_msgs::EigenOdometry odometry;
//static int counter = 0;

void odometry_cb(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
}

void attitude_thrust_cb(const mav_msgs::AttitudeThrustPtr& attitude_thrust_msg)
{
    mav_msgs::RollPitchYawrateThrustPtr rpyrate_thrust_cmd(new mav_msgs::RollPitchYawrateThrust);;
    Eigen::Vector3d current_rpy;
    odometry.getEulerAngles(&current_rpy);
    // get yaw from odometry
    double current_roll = current_rpy(0);
    double current_pitch = current_rpy(1);
    double current_yaw = current_rpy(2);    
    Eigen::Vector3d thrust_sp;
    thrust_sp << attitude_thrust_msg->thrust.x, attitude_thrust_msg->thrust.y, attitude_thrust_msg->thrust.z;
    Eigen::Vector3d thrust_norm = thrust_sp.normalized();
    
    double cphi_stheta = thrust_norm(0)*cos(current_yaw)+thrust_norm(1)*sin(current_yaw);
    double sphi = thrust_norm(0)*sin(current_yaw)-thrust_norm(1)*cos(current_yaw);
    double cphi_ctheta = thrust_norm(2);
    if (cphi_ctheta != 0)
    {
        rpyrate_thrust_cmd->pitch = atan2(cphi_stheta, cphi_ctheta);
        rpyrate_thrust_cmd->roll = atan2(sphi, sqrt(cphi_stheta*cphi_stheta + cphi_ctheta*cphi_ctheta));
    }
    else
    {
        rpyrate_thrust_cmd->pitch = 0;
        rpyrate_thrust_cmd->roll = 0;
    }
    
    rpyrate_thrust_cmd->yaw_rate = 0;// ignore yaw setpoint
    rpyrate_thrust_cmd->thrust.x = 0;
    rpyrate_thrust_cmd->thrust.y = 0;
    //rpyrate_thrust_cmd->thrust.z = thrust_sp(2);
    rpyrate_thrust_cmd->thrust.z = thrust_sp(0)*(cos(current_roll)*sin(current_pitch)*cos(current_yaw) + sin(current_roll)*sin(current_yaw))
                               + thrust_sp(1)*(cos(current_roll)*sin(current_pitch)*sin(current_yaw) - sin(current_roll)*cos(current_yaw))
                               + thrust_sp(2)*cos(current_roll)*cos(current_pitch);
    rpyrate_thrust_publisher.publish(rpyrate_thrust_cmd);

    // counter++;
    // //if (counter > 5)
    // {
    //     ROS_INFO_STREAM("Current RPY 2:" << current_roll << "," << current_pitch << "," << current_yaw);
    //     ROS_INFO_STREAM("Attitude thrust 2:" << attitude_thrust_msg->thrust.x << "," << attitude_thrust_msg->thrust.y
    //         << "," << attitude_thrust_msg->thrust.z);
    //     ROS_INFO_STREAM("RPY 2:" << rpyrate_thrust_cmd->roll << "," << rpyrate_thrust_cmd->pitch << ","
    //         << rpyrate_thrust_cmd->yaw_rate << "," << rpyrate_thrust_cmd->thrust.z);        
    //     counter = 0;
    // }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "command_converter_node");
    ros::NodeHandle nh, private_nh("~");

    std::string attitude_thrust_topic;
    private_nh.param<std::string>("attitude_thrust_topic", attitude_thrust_topic, "command/attitude_thrust");
    std::cout << "attitude thrust topic: " << attitude_thrust_topic << '\n';

    std::string rpyrate_thrust_topic;
    private_nh.param<std::string>("rpyrate_thrust_topic", rpyrate_thrust_topic, "command/roll_pitch_yawrate_thrust");
    std::cout << "rpyrate thrust topic: " << rpyrate_thrust_topic << '\n';

    std::string odometry_topic;
    private_nh.param<std::string>("odometry_topic", odometry_topic, "msf_core/odometry");
    std::cout << "odometry topic: " << odometry_topic << '\n';

    odometry_subscriber = nh.subscribe(odometry_topic, 1,
                                       odometry_cb, ros::TransportHints().tcpNoDelay());
    attitude_thrust_subscriber = nh.subscribe(attitude_thrust_topic, 10,
                                       attitude_thrust_cb, ros::TransportHints().tcpNoDelay());
    rpyrate_thrust_publisher = nh.advertise<mav_msgs::RollPitchYawrateThrust>(rpyrate_thrust_topic, 10);

    ros::spin();
    return 0;
}
