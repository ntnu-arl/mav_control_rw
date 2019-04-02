#include "command_converter_2_serial.h"

static ros::Subscriber odometry_subscriber;
//static ros::Subscriber attitude_thrust_subscriber;
static ros::Publisher rpyrate_thrust_publisher;
static ros::Publisher vicon_publisher;
//static double current_roll, current_pitch, current_yaw;
static mav_msgs::EigenOdometry odometry;
//static int counter = 0;
static pthread_t clientThread;
static Eigen::Vector3d current_pos;
static Eigen::Quaterniond current_attitude;
//pthread_mutex_t odom_data_mutex;

static int uartFP;

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

void msgPoseCallback(uint8_t *data)
{
    double x_tmp, y_tmp, z_tmp, quat_x_tmp, quat_y_tmp, quat_z_tmp, quat_w_tmp;
    int32_t x_tmp_int, y_tmp_int, z_tmp_int;
    int16_t quat_x_tmp_int, quat_y_tmp_int, quat_z_tmp_int, quat_w_tmp_int;
    x_tmp_int = (int32_t)data[3] | (data[2] << 8) | (data[1] << 16) | (data[0] << 24);
    x_tmp = 0.001 * x_tmp_int;

    y_tmp_int = (int32_t)data[7] | (data[6] << 8) | (data[5] << 16) | (data[4] << 24);
    y_tmp = 0.001 * y_tmp_int;

    z_tmp_int = (int32_t)data[11] | (data[10] << 8) | (data[9] << 16) | (data[8] << 24);
    z_tmp = 0.001 * z_tmp_int;

    quat_x_tmp_int = (int16_t)data[13] | (data[12] << 8);
    quat_x_tmp = 0.001 * quat_x_tmp_int;

    quat_y_tmp_int = (int16_t)data[15] | (data[14] << 8);
    quat_y_tmp = 0.001 * quat_y_tmp_int;

    quat_z_tmp_int = (int16_t)data[17] | (data[16] << 8);
    quat_z_tmp = 0.001 * quat_z_tmp_int;

    quat_w_tmp_int = (int16_t)data[19] | (data[18] << 8);
    quat_w_tmp = 0.001 * quat_w_tmp_int;

    current_pos << x_tmp, y_tmp, z_tmp;
    current_attitude = Eigen::Quaterniond(quat_w_tmp, quat_x_tmp, quat_y_tmp, quat_z_tmp);

    /* build transformstamped msg and publish to update msf */
    geometry_msgs::TransformStamped vicon_pose;
    static int seq = 0;
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
    //vicon_publisher.publish(vicon_pose);

    ROS_INFO_STREAM_THROTTLE(1,"vicon:" << x_tmp << "," << y_tmp << "," << z_tmp << "," 
                                << quat_x_tmp << "," << quat_y_tmp << "," << quat_z_tmp << "," << quat_w_tmp);
}

void msgAttThrustCallback(uint8_t *data)
{
    double thrust_x, thrust_y, thrust_z, yaw_setpoint;
    int16_t thrust_x_tmp, thrust_y_tmp, thrust_z_tmp, yaw_setpoint_tmp;
    thrust_x_tmp = (int16_t)data[1] | (data[0] << 8);
    thrust_x = 0.001 * thrust_x_tmp;

    thrust_y_tmp = (int16_t)data[3] | (data[2] << 8);
    thrust_y = 0.001 * thrust_y_tmp;

    thrust_z_tmp = (int16_t)data[5] | (data[4] << 8);
    thrust_z = 0.001 * thrust_z_tmp;

    yaw_setpoint_tmp = (int16_t)data[7] | (data[6] << 8);
    yaw_setpoint = 0.001 * yaw_setpoint_tmp;

    ROS_INFO_STREAM_THROTTLE(1, "cmd:" << thrust_x << "," << thrust_y << "," << thrust_z << "," << yaw_setpoint);

    Eigen::Vector3d current_rpy;
    odometry.getEulerAngles(&current_rpy);
    double current_roll = current_rpy(0);
    double current_pitch = current_rpy(1);
    double current_yaw = current_rpy(2);
    processAttitudeThrustMsg(current_roll, current_pitch, current_yaw, thrust_x, thrust_y, thrust_z, yaw_setpoint);
}

void *SerialReceiveTask(void *arg)
{
    ROS_INFO("Serial Task run");
    uint8_t msg_buff[256] = {0};
    int64_t time_recv = static_cast<int64_t>(ros::Time::now().toNSec());
    int state = 0;
    int length;
    int i, j;
    int msg_cnt = 0;
    uint8_t current_byte;
    int expected_len = 0;
    while (1)
    {
        if (readSerial(&current_byte, 1) > 0)
        {
            //std::cout << current_byte;
            time_recv = static_cast<int64_t>(ros::Time::now().toNSec());
            if ((state == 0) && (msg_cnt >= 2))
            {
                msg_buff[0] = msg_buff[1];
                msg_buff[1] = current_byte;
            }
            else
            {
                msg_buff[msg_cnt++] = current_byte;
            }
            switch (state)
            {
            case 0: /* receive header */
                if (msg_cnt >= 2)
                {
                    if ((msg_buff[0] == 'S') && (msg_buff[1] == 'T'))
                    {
                        //std::cout << "state 1" << std::endl;
                        state = 1;
                    }
                }
                break;
            case 1: /* receive msg id */
                if (current_byte >= MSG_ID_MAX)
				{
                    state = 0;
                    msg_cnt = 0;					
				}
				else
				{
                    state++;
                    switch (current_byte)
                    {
                    case MSG_POSE:
                        expected_len = 20; // not counted 2 end bytes or CRC
                        break;
                    case MSG_ATT_THRUST:
                        expected_len = 8;
                        break;
                    default:
                        expected_len = 0;
                        break;
                    }
				}
				break;
            case 2:/* remain bytes */
                if (msg_cnt == expected_len + 5)// 2 start bytes + 1 msg_id byte + 2 end bytes
                {
                    //std::cout << "state 2" << std::endl;
                    if ((msg_buff[expected_len + 3] == 'E') && (msg_buff[expected_len + 4] == 'D'))
                    {
                        
                        switch (msg_buff[2])
                            {
                            case MSG_POSE:
                                //std::cout << "got pose msg" << std::endl;
                                msgPoseCallback(msg_buff + 3);
                                break;
                            case MSG_ATT_THRUST:
                                //std::cout << "got att thrust msg" << std::endl;
                                msgAttThrustCallback(msg_buff + 3);
                                break;
                            }
                    }
                    state = 0;
                    msg_cnt = 0;
                }
                break;
            }
        }
        if (static_cast<int64_t>(ros::Time::now().toNSec()) - time_recv > 1000000000)
        {
            ROS_INFO("No msg received from master last 1 sec");
            state = 0;
            msg_cnt = 0;
        }
        //usleep(200);
    }
}

void mySigintHandler(int sig)
{
    closeSerialPort(uartFP);
    ros::shutdown();
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
    //client.setup(TCP_ADDRESS, TCP_SERVER_PORT);
    //client.addRecvCallback(msgCallback);
    // setup an exit handler to close the serial port
    signal(SIGINT, mySigintHandler);
    // get the name of the serial port, and the port number from the launch file
    std::string serial_port_device_name = "/dev/ttyUSB";
    nh.getParam("serial_port_name", serial_port_device_name);
    int serial_port_device_num = 1;
    nh.getParam("serial_port_num", serial_port_device_num);
    // setup the serial port
    const char *port_name_ptr = serial_port_device_name.c_str();
    // if it fails to open
    if ((uartFP = openSerialPort("/dev/ttyUSB", 1)) == -1)
    {
        // spit an error
        ROS_ERROR("Failed to open %s%d: %s (%d)\n", port_name_ptr, serial_port_device_num, strerror(errno), errno);
        // shutdown the node
        ros::shutdown();
    }
    pthread_create(&clientThread, NULL, &SerialReceiveTask, NULL);
    
    ros::spin();

    //ros::MultiThreadedSpinner spinner();
    //spinner.spin();

    return 0;
}
