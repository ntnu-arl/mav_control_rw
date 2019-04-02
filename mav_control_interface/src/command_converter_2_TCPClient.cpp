#include "command_converter_2_TCPClient.h"

#define TCP_SERVER_PORT 8000
#define TCP_ADDRESS "127.0.0.1"

static ros::Subscriber odometry_subscriber;
//static ros::Subscriber attitude_thrust_subscriber;
static ros::Publisher rpyrate_thrust_publisher;
static ros::Publisher vicon_publisher;
//static double current_roll, current_pitch, current_yaw;
static mav_msgs::EigenOdometry odometry;
//static int counter = 0;
static TCP::TCPClient client;
static pthread_t clientThread;
static Eigen::Vector3d current_pos;
static Eigen::Quaterniond current_attitude;
//pthread_mutex_t odom_data_mutex;

void processAttitudeThrustMsg(double current_roll, double current_pitch, double current_yaw,
                              double thrust_x, double thrust_y, double thrust_z, double yaw)
{
    mav_msgs::RollPitchYawrateThrustPtr rpyrate_thrust_cmd(new mav_msgs::RollPitchYawrateThrust);
    Eigen::Vector3d thrust_sp;
    thrust_sp << thrust_x, thrust_y, thrust_z;
    Eigen::Vector3d thrust_norm = thrust_sp.normalized();

    double cphi_stheta = thrust_norm(0) * cos(current_yaw) + thrust_norm(1) * sin(current_yaw);
    double sphi = thrust_norm(0) * sin(current_yaw) - thrust_norm(1) * cos(current_yaw);
    double cphi_ctheta = thrust_norm(2);
    if (cphi_ctheta != 0)
    {
        rpyrate_thrust_cmd->pitch = atan2(cphi_stheta, cphi_ctheta);
        rpyrate_thrust_cmd->roll = atan2(sphi, sqrt(cphi_stheta * cphi_stheta + cphi_ctheta * cphi_ctheta));
    }
    else
    {
        rpyrate_thrust_cmd->pitch = 0;
        rpyrate_thrust_cmd->roll = 0;
    }

    rpyrate_thrust_cmd->yaw_rate = 0; // ignore yaw setpoint
    rpyrate_thrust_cmd->thrust.x = 0;
    rpyrate_thrust_cmd->thrust.y = 0;
    //rpyrate_thrust_cmd->thrust.z = thrust_sp(2);
    rpyrate_thrust_cmd->thrust.z = thrust_sp(0) * (cos(current_roll) * sin(current_pitch) * cos(current_yaw) + sin(current_roll) * sin(current_yaw)) + thrust_sp(1) * (cos(current_roll) * sin(current_pitch) * sin(current_yaw) - sin(current_roll) * cos(current_yaw)) + thrust_sp(2) * cos(current_roll) * cos(current_pitch);
    rpyrate_thrust_publisher.publish(rpyrate_thrust_cmd);
    //ROS_INFO("FY");
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

void odometry_cb(const nav_msgs::OdometryConstPtr &odometry_msg)
{
    mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
}

// void attitude_thrust_cb(const mav_msgs::AttitudeThrustPtr& attitude_thrust_msg)
// {
//     Eigen::Vector3d current_rpy;
//     Eigen::Vector3d euler_cmd;
//     double yaw_setpoint;
//     odometry.getEulerAngles(&current_rpy);
//     // get yaw from odometry
//     double current_roll = current_rpy(0);
//     double current_pitch = current_rpy(1);
//     double current_yaw = current_rpy(2);
//     Eigen::Quaterniond attitude_setpoint(attitude_thrust_msg->orientation.w, attitude_thrust_msg->orientation.x,
//         attitude_thrust_msg->orientation.y, attitude_thrust_msg->orientation.z);
//     mav_msgs::getEulerAnglesFromQuaternion(attitude_setpoint, &euler_cmd);
//     yaw_setpoint = euler_cmd(2);
//     processAttitudeThrustMsg(current_roll, current_pitch, current_yaw,
//         attitude_thrust_msg->thrust.x, attitude_thrust_msg->thrust.y, attitude_thrust_msg->thrust.z, yaw_setpoint);
// }

void *ClientTask(void *arg)
{
    ROS_INFO("Client Task run");
    client.start();
    client.closeClient();
}

bool msgCallback(std::string msg)
{
    //ROS_INFO_STREAM("Client received:" << msg);
    char tmp_str[TCP::MAX_PACKET_SIZE];
    std::strcpy(tmp_str, msg.c_str());
    
    std::size_t found_idx = msg.find("data:");
    if (found_idx != std::string::npos) // get pose msg
    {
        double x_tmp, y_tmp, z_tmp, quat_x_tmp, quat_y_tmp, quat_z_tmp, quat_w_tmp;
        //std::cout << "data" << std::endl;
        char *pStart = tmp_str + 5;
        char *pEnd;
        static int seq = 0;
        x_tmp = strtod(pStart, &pEnd);
        //std::cout << "pEnd:" << *pEnd << std::endl;
        if (*pEnd != ',')
        {
            return false;
        }
        pStart = pEnd + 1;
        y_tmp = strtod(pStart, &pEnd);
        if (*pEnd != ',')
        {
            return false;
        }
        pStart = pEnd + 1;
        z_tmp = strtod(pStart, &pEnd);
        if (*pEnd != ',')
        {
            return false;
        }
        pStart = pEnd + 1;
        quat_x_tmp = strtod(pStart, &pEnd);
        if (*pEnd != ',')
        {
            return false;
        }
        pStart = pEnd + 1;
        quat_y_tmp = strtod(pStart, &pEnd);
        if (*pEnd != ',')
        {
            return false;
        }
        pStart = pEnd + 1;
        quat_z_tmp = strtod(pStart, &pEnd);
        if (*pEnd != ',')
        {
            return false;
        }
        pStart = pEnd + 1;
        quat_w_tmp = strtod(pStart, &pEnd);
        //pthread_mutex_lock(&msg_data_mutex);
        current_pos << x_tmp, y_tmp, z_tmp;
        current_attitude = Eigen::Quaterniond(quat_w_tmp, quat_x_tmp, quat_y_tmp, quat_z_tmp);
        //pthread_mutex_unlock(&msg_data_mutex);

        /* build transformstamped msg and publish to update msf */
        geometry_msgs::TransformStamped vicon_pose;
        vicon_pose.header.seq = seq++;
        vicon_pose.header.stamp = ros::Time::now();
        vicon_pose.header.frame_id = "world";
        vicon_pose.transform.translation.x = x_tmp;
        vicon_pose.transform.translation.y = y_tmp;
        vicon_pose.transform.translation.z = z_tmp;
        vicon_pose.transform.rotation.x = quat_x_tmp;
        vicon_pose.transform.rotation.y = quat_y_tmp;
        vicon_pose.transform.rotation.z = quat_z_tmp;
        vicon_pose.transform.rotation.w = quat_w_tmp;
        vicon_publisher.publish(vicon_pose);
        //return true;
        //std::cout << "data:" << x_tmp << "," << y_tmp << "," << z_tmp << "," 
        //          << quat_x_tmp << "," << quat_y_tmp << "," << quat_z_tmp << "," << quat_w_tmp << std::endl;
    }
    
    found_idx = msg.find("cmd:");
    if (found_idx != std::string::npos) // get cmd msg
    {
        double thrust_x, thrust_y, thrust_z, yaw_setpoint;
        char *pStart = tmp_str + 4;
        char *pEnd;
        static int seq = 0;
        thrust_x = strtod(pStart, &pEnd);
        if (*pEnd != ',')
        {
            return false;
        }
        pStart = pEnd + 1;
        thrust_y = strtod(pStart, &pEnd);
        if (*pEnd != ',')
        {
            return false;
        }
        pStart = pEnd + 1;
        thrust_z = strtod(pStart, &pEnd);
        if (*pEnd != ',')
        {
            return false;
        }
        pStart = pEnd + 1;
        yaw_setpoint = strtod(pStart, &pEnd);

        Eigen::Vector3d current_rpy;
        odometry.getEulerAngles(&current_rpy);
        double current_roll = current_rpy(0);
        double current_pitch = current_rpy(1);
        double current_yaw = current_rpy(2);
        processAttitudeThrustMsg(current_roll, current_pitch, current_yaw, thrust_x, thrust_y, thrust_z, yaw_setpoint);
        //std::cout << "cmd:" << thrust_x << "," << thrust_y << "," << thrust_z << "," << yaw_setpoint << std::endl;        
    }
    return true;
}

int main(int argc, char **argv)
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

    // odometry_subscriber = nh.subscribe(odometry_topic, 1,
    //                                    odometry_cb, ros::TransportHints().tcpNoDelay());
    // attitude_thrust_subscriber = nh.subscribe(attitude_thrust_topic, 10,
    //                                    attitude_thrust_cb, ros::TransportHints().tcpNoDelay());
    rpyrate_thrust_publisher = nh.advertise<mav_msgs::RollPitchYawrateThrust>(rpyrate_thrust_topic, 10);
    vicon_publisher = nh.advertise<geometry_msgs::TransformStamped>("/vicon/huan_minidrone2_NUC/huan_minidrone2_NUC", 10);

    //pthread_mutex_init(&msg_data_mutex, NULL);
    client.setup(TCP_ADDRESS, TCP_SERVER_PORT);
    client.addRecvCallback(msgCallback);
    pthread_create(&clientThread, NULL, &ClientTask, NULL);
    ros::spin();
    return 0;
}
