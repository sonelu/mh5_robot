#include <iostream>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

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
    float protocol_;

    // dynamixel
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;

    //interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::EffortJointInterface effort_joint_interface;

    int num_joints;
    std::vector<std::string> joint_name;

    //actual states
    std::vector<double> joint_position_state;
    std::vector<double> joint_velocity_state;
    std::vector<double> joint_effort_state;

    //given setpoints
    std::vector<double> joint_effort_command;

};
}