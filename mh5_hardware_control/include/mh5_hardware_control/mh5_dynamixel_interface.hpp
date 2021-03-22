#include <iostream>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include "mh5_port_handler.hpp"

namespace mh5_hardware_interface
{

class MH5DynamixelInterface: public hardware_interface::RobotHW
{
public:
    MH5DynamixelInterface();
    ~MH5DynamixelInterface();
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);

protected:
    ros::NodeHandle nh_;
    
    // communication
    std::string port_;
    int baudrate_;
    bool rs485_;
    double protocol_;

    // dynamixel
    mh5_port_handler::PortHandlerMH5 *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    dynamixel::GroupSyncRead *syncRead_;

    //interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PosVelJointInterface pos_vel_joint_interface;

    int num_joints;
    std::vector<std::string>    joint_name;

    //actual servos
    std::vector<uint8_t>        servo_ids;
    std::vector<bool>           servo_present;
    std::vector<bool>           joint_direction_inverse;
    std::vector<double>         joint_offset;

    //actual states
    std::vector<double>         joint_position_state;
    std::vector<double>         joint_velocity_state;
    std::vector<double>         joint_effort_state;


    //commands
    std::vector<double>         joint_position_command;
    std::vector<double>         joint_velocity_command;

    //help methods
    bool initPort();
    bool initJoints();
    bool findServos();
    bool initServos();
    bool pingServo(const int /*index*/, const int /*num_tries*/);
    bool writeRegister(const int /*index*/, const uint16_t /*address*/, const int /*size*/, const long /*value*/, const int /*num_tries*/);
    bool setupDynamixelLoops();

};
}