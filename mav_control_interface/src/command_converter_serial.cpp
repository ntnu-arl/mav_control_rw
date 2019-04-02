#include "command_converter_serial.h"

#define MAX_PACKET_LEN      32

static ros::Subscriber odometry_subscriber;
static ros::Subscriber odometry_quad2_subscriber;
static ros::Subscriber vicon_quad1_subscriber;
static ros::Subscriber vicon_quad2_subscriber;
static ros::Subscriber attitude_thrust_quad1_subscriber;
static ros::Subscriber attitude_thrust_quad2_subscriber;
static ros::Publisher rpyrate_thrust_publisher;
static ros::Publisher encoder_publisher;
static mav_msgs::EigenOdometry odometry;
static mav_msgs::EigenOdometry odometry_quad2;
static double joint1_x_offset_, joint1_y_offset_, joint1_z_offset_;
static double joint2_x_offset_, joint2_y_offset_, joint2_z_offset_; 
static Eigen::Vector3d joint1_offset_;
static Eigen::Vector3d joint2_offset_;
static Eigen::Vector3d quad1_pos_;
static Eigen::Quaterniond quad1_orientation_;
static Eigen::Vector3d quad2_pos_;
static Eigen::Quaterniond quad2_orientation_;
static pthread_t serverThread;
static int counter = 0;

static int uartFP;

void serialWritePacket(int8_t msg_id, uint8_t *writeBuffer, uint32_t len);

void odometry_cb(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);   
    quad1_pos_ = odometry.position_W;
    quad1_orientation_ = odometry.orientation_W_B;
    //calculate encoders of quad1
    Eigen::Vector3d joint1_pos;
    Eigen::Vector3d joint2_pos;
    Eigen::Vector3d stick_dir;
    joint1_pos = quad1_pos_ +  quad1_orientation_ * joint1_offset_;  
    joint2_pos = quad2_pos_ +  quad2_orientation_ * joint2_offset_; 
    stick_dir = joint1_pos - joint2_pos;

    geometry_msgs::Vector3 quad1_encoder;
    //calculate stick quaternion
    Eigen::Quaterniond R_L, R_1L;
    double yaw_stick = atan2(stick_dir(1), stick_dir(0));
    double pitch_stick = -atan2(stick_dir(2), sqrt(stick_dir(0)*stick_dir(0)+stick_dir(1)*stick_dir(1)));
    R_L =  Eigen::AngleAxisd(yaw_stick, Eigen::Vector3d::UnitZ())  // yaw
          * Eigen::AngleAxisd(pitch_stick, Eigen::Vector3d::UnitY());  // pitch 
           // assume roll =  0 ????
    R_1L = odometry.orientation_W_B.conjugate() * R_L;
    Eigen::Vector3d encoders;
    mav_msgs::getEulerAnglesFromQuaternion(R_1L, &encoders);
    quad1_encoder.x = encoders(2);//yaw
    quad1_encoder.y = encoders(1);//pitch
    quad1_encoder.z = encoders(0);//roll  
    encoder_publisher.publish(quad1_encoder); 
    //ROS_INFO_THROTTLE(1,"Publish enc");   
}

void attitude_thrust_quad1_cb(const mav_msgs::AttitudeThrustPtr& attitude_thrust_msg)
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
    //     ROS_INFO_STREAM("Current RPY 1:" << current_roll << "," << current_pitch << "," << current_yaw);
    //     ROS_INFO_STREAM("Attitude thrust 1:" << attitude_thrust_msg->thrust.x << "," << attitude_thrust_msg->thrust.y
    //         << "," << attitude_thrust_msg->thrust.z);
    //     ROS_INFO_STREAM("RPY 1:" << rpyrate_thrust_cmd->roll << "," << rpyrate_thrust_cmd->pitch << ","
    //         << rpyrate_thrust_cmd->yaw_rate << "," << rpyrate_thrust_cmd->thrust.z);        
    //     counter = 0;
    // }    
}
void attitude_thrust_quad2_cb(const mav_msgs::AttitudeThrustPtr& attitude_thrust_msg)
{
    double yaw;
    Eigen::Vector3d euler_cmd;

    Eigen::Quaterniond attitude_cmd(attitude_thrust_msg->attitude.w, attitude_thrust_msg->attitude.x, 
        attitude_thrust_msg->attitude.y, attitude_thrust_msg->attitude.z);
    mav_msgs::getEulerAnglesFromQuaternion(attitude_cmd, &euler_cmd);
    yaw = euler_cmd(2);

    // snprintf(str_tmp,256, "cmd:%.3f,%.3f,%.3f,%.4f", attitude_thrust_msg->thrust.x, attitude_thrust_msg->thrust.y, 
    //     attitude_thrust_msg->thrust.z, yaw);
    //snprintf(str_tmp,256, "cmd:%.3f,%.3f,%.3f,%.4f", 1.0, 2.0, 3.0, 4.0);
    uint8_t att_thrust_buff[8];
    int16_t data[4];
    data[0] = (int16_t)(attitude_thrust_msg->thrust.x * 1000);
    data[1] = (int16_t)(attitude_thrust_msg->thrust.y * 1000);
    data[2] = (int16_t)(attitude_thrust_msg->thrust.z * 1000);
    data[3] = (int16_t)(yaw * 1000);  
    int i;
    for (i=0; i<4; i++)
    {
        att_thrust_buff[2*i] = (uint8_t)(data[i] >> 8);
        att_thrust_buff[2*i + 1] = (uint8_t)(data[i]);        
    }  
    serialWritePacket(MSG_ATT_THRUST, att_thrust_buff, 8);
}
/* This callback is for testing purpose */
// void vicon_quad1_cb(const geometry_msgs::TransformStampedConstPtr& vicon_quad1)
// {
//     geometry_msgs::Vector3 trans_tmp = vicon_quad1->transform.translation;
//     geometry_msgs::Quaternion rot_tmp = vicon_quad1->transform.rotation;
//     quad1_pos_ << trans_tmp.x, trans_tmp.y, trans_tmp.z;
//     quad1_orientation_ = Eigen::Quaterniond(rot_tmp.w, rot_tmp.x, rot_tmp.y, rot_tmp.z);
//     //calculate encoders of quad1
//     Eigen::Vector3d joint1_pos;
//     Eigen::Vector3d joint2_pos;
//     Eigen::Vector3d stick_dir;
//     joint1_pos = quad1_pos_ +  quad1_orientation_ * joint1_offset_;  
//     joint2_pos = quad2_pos_ +  quad2_orientation_ * joint2_offset_; 
//     stick_dir = joint1_pos - joint2_pos;

//     geometry_msgs::Vector3 quad1_encoder;
//     //calculate stick quaternion
//     Eigen::Quaterniond R_L, R_1L;
//     double yaw_stick = atan2(stick_dir(1), stick_dir(0));
//     double pitch_stick = -atan2(stick_dir(2), sqrt(stick_dir(0)*stick_dir(0)+stick_dir(1)*stick_dir(1)));
//     R_L =  Eigen::AngleAxisd(yaw_stick, Eigen::Vector3d::UnitZ())  // yaw
//           * Eigen::AngleAxisd(pitch_stick, Eigen::Vector3d::UnitY());  // pitch 
//            // assume roll =  0 ????
//     R_1L = odometry.orientation_W_B.conjugate() * R_L;
//     Eigen::Vector3d encoders;
//     mav_msgs::getEulerAnglesFromQuaternion(R_1L, &encoders);
//     quad1_encoder.x = encoders(2);//yaw
//     quad1_encoder.y = encoders(1);//pitch
//     quad1_encoder.z = encoders(0);//roll  
//     encoder_publisher.publish(quad1_encoder);

//     // counter++;
//     // if (counter > 10)
//     // {
//     //     ROS_INFO_STREAM("stick dir:" << stick_dir.transpose() << ", yaw:" << yaw_stick << ", pitch:" << pitch_stick);
//     //     ROS_INFO_STREAM("encoders:" << encoders.transpose());     
//     //     counter = 0;
//     // }
// }

void vicon_quad2_cb(const geometry_msgs::TransformStampedConstPtr& vicon_quad2)
{
    geometry_msgs::Vector3 trans_tmp = vicon_quad2->transform.translation;
    geometry_msgs::Quaternion rot_tmp = vicon_quad2->transform.rotation;
    quad2_pos_ << trans_tmp.x, trans_tmp.y, trans_tmp.z;
    quad2_orientation_ = Eigen::Quaterniond(rot_tmp.w, rot_tmp.x, rot_tmp.y, rot_tmp.z);
    //send to quad 2 via serial
    uint8_t pose_buff[20];
    //snprintf(str_tmp,256,"data:%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f", trans_tmp.x, trans_tmp.y, trans_tmp.z, 
    //     rot_tmp.x, rot_tmp.y, rot_tmp.z, rot_tmp.w);
    //snprintf(str_tmp,256,"data:%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f", 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);    
    int32_t data_pos[3];
    int16_t data_quat[4];
    data_pos[0] = (int32_t)(trans_tmp.x * 1000);
    data_pos[1] = (int32_t)(trans_tmp.y * 1000);
    data_pos[2] = (int32_t)(trans_tmp.z * 1000);
    data_quat[0] = (int16_t)(rot_tmp.x * 1000);
    data_quat[1] = (int16_t)(rot_tmp.y * 1000);
    data_quat[2] = (int16_t)(rot_tmp.z * 1000);
    data_quat[3] = (int16_t)(rot_tmp.w * 1000);  
    int i;
    for (i=0; i<3; i++)
    {
        pose_buff[4*i] = (uint8_t)(data_pos[i] >> 24);
        pose_buff[4*i + 1] = (uint8_t)(data_pos[i] >> 16);
        pose_buff[4*i + 2] = (uint8_t)(data_pos[i] >> 8);
        pose_buff[4*i + 3] = (uint8_t)(data_pos[i]);        
    } 
    for (i=0; i<4; i++)
    {
        pose_buff[2*i + 12] = (uint8_t)(data_quat[i] >> 8);
        pose_buff[2*i + 13] = (uint8_t)(data_quat[i]);    
    }     
    serialWritePacket(MSG_POSE, pose_buff, 20);
}

// void *ServerTask(void *arg)
// {
// 	ROS_INFO("Server Task run");	
// 	server.start();
// 	server.closeServer();
// }

void mySigintHandler(int sig)
{
    closeSerialPort(uartFP);
    ros::shutdown();
}

void serialWritePacket(int8_t cmd_id, uint8_t *writeBuffer, uint32_t len)
{
  if (len <= MAX_PACKET_LEN)
  {
    uint8_t packet_buff[MAX_PACKET_LEN + 5];
    int buff_idx = 0;
    packet_buff[0] = 'S';
    packet_buff[1] = 'T';
    packet_buff[2] = cmd_id;
    memcpy(packet_buff + 3, writeBuffer, len);
    packet_buff[len + 3] = 'E';
    packet_buff[len + 4] = 'D';
    serialWrite(packet_buff, len+5);
  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "command_converter_node");
    ros::NodeHandle nh, private_nh("~");

    std::string attitude_thrust_quad1_topic;
    private_nh.param<std::string>("attitude_thrust_quad1_topic", attitude_thrust_quad1_topic, "/quad1/command/attitude_thrust");
    std::cout << "attitude thrust quad1 topic: " << attitude_thrust_quad1_topic << '\n';

    std::string attitude_thrust_quad2_topic;
    private_nh.param<std::string>("attitude_thrust_quad2_topic", attitude_thrust_quad2_topic, "/quad2/command/attitude_thrust");
    std::cout << "attitude thrust quad2 topic: " << attitude_thrust_quad2_topic << '\n';    

    std::string rpyrate_thrust_topic;
    private_nh.param<std::string>("rpyrate_thrust_topic", rpyrate_thrust_topic, "command/roll_pitch_yawrate_thrust");
    std::cout << "rpyrate thrust topic: " << rpyrate_thrust_topic << '\n';

    std::string odometry_topic;
    private_nh.param<std::string>("odometry_topic", odometry_topic, "msf_core/odometry");
    std::cout << "odometry topic: " << odometry_topic << '\n';

    std::string quad1_encoder_topic;
    private_nh.param<std::string>("quad1_encoder_topic", quad1_encoder_topic, "/quad1/encoder");
    std::cout << "quad1 encoder topic: " << quad1_encoder_topic << '\n';    

    std::string vicon_quad1_topic;
    private_nh.param<std::string>("vicon_quad1_topic", vicon_quad1_topic, "/vicon/huan_minidrone1_NUC/huan_minidrone1_NUC");
    std::cout << "vicon quad1 topic: " << vicon_quad1_topic << '\n';

    std::string vicon_quad2_topic;
    private_nh.param<std::string>("vicon_quad2_topic", vicon_quad2_topic, "/vicon/huan_minidrone2_NUC/huan_minidrone2_NUC");
    std::cout << "vicon quad2 topic: " << vicon_quad2_topic << '\n';

    private_nh.param<double>("joint1_x_offset", joint1_x_offset_, -0.17);
    std::cout << "joint 1 x offset: " << joint1_x_offset_ << '\n';

    private_nh.param<double>("joint1_y_offset", joint1_y_offset_, 0.0);
    std::cout << "joint 1 y offset: " << joint1_y_offset_ << '\n';

    private_nh.param<double>("joint1_z_offset", joint1_z_offset_, 0.0);
    std::cout << "joint 1 z offset: " << joint1_z_offset_ << '\n';

    private_nh.param<double>("joint2_x_offset", joint2_x_offset_, 0.17);
    std::cout << "joint 2 x offset: " << joint2_x_offset_ << '\n';

    private_nh.param<double>("joint2_y_offset", joint2_y_offset_, 0.0);
    std::cout << "joint 2 y offset: " << joint2_y_offset_ << '\n';

    private_nh.param<double>("joint2_z_offset", joint2_z_offset_, 0.0);
    std::cout << "joint 2 z offset: " << joint2_z_offset_ << '\n';    

    joint1_offset_ << joint1_x_offset_, joint1_y_offset_, joint1_z_offset_;
    joint2_offset_ << joint2_x_offset_, joint2_y_offset_, joint2_z_offset_;

    odometry_subscriber = nh.subscribe(odometry_topic, 10,
                                       odometry_cb, ros::TransportHints().tcpNoDelay());
    attitude_thrust_quad1_subscriber = nh.subscribe(attitude_thrust_quad1_topic, 10,
                                       attitude_thrust_quad1_cb, ros::TransportHints().tcpNoDelay());
    attitude_thrust_quad2_subscriber = nh.subscribe(attitude_thrust_quad2_topic, 10,
                                       attitude_thrust_quad2_cb, ros::TransportHints().tcpNoDelay());                                       
    rpyrate_thrust_publisher = nh.advertise<mav_msgs::RollPitchYawrateThrust>(rpyrate_thrust_topic, 10);

    /* For testing purpose */
    //vicon_quad1_subscriber = nh.subscribe(vicon_quad1_topic, 10, vicon_quad1_cb, ros::TransportHints().tcpNoDelay());
    vicon_quad2_subscriber = nh.subscribe(vicon_quad2_topic, 10, vicon_quad2_cb, ros::TransportHints().tcpNoDelay());      
    encoder_publisher = nh.advertise<geometry_msgs::Vector3>(quad1_encoder_topic, 10);
    
    // setup an exit handler to close the serial port
	signal(SIGINT, mySigintHandler);
	// get the name of the serial port, and the port number from the launch file
	std::string serial_port_device_name = "/dev/ttyUSB";
	nh.getParam("serial_port_name", serial_port_device_name);
	int serial_port_device_num = 0;
	nh.getParam("serial_port_num", serial_port_device_num);
	// setup the serial port
	const char *port_name_ptr = serial_port_device_name.c_str();
    // if it fails to open
	if ((uartFP = openSerialPort("/dev/ttyUSB", 0)) == -1) 
	{
    // spit an error
		ROS_ERROR("Failed to open %s%d: %s (%d)\n", port_name_ptr, serial_port_device_num, strerror(errno), errno);
    // shutdown the node
		ros::shutdown();
	}
	// if the serial port was setup successfully, spin ros


    ros::spin();
    return 0;
}
