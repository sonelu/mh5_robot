#include "dynamixel_joint.hpp"
#include "dynamixel_loop.hpp"
#include "active_joint_interface.hpp"


#pragma once

namespace mh5_hardware
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
    const char* nss_;       // c string for nh_ namespace; used for messages
    
    // communication
    std::string port_;
    int baudrate_;
    bool rs485_;
    double protocol_;

    // dynamixel
    mh5_port_handler::PortHandlerMH5 *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    mh5_hardware::PVLReader *pvlReader_;
    dynamixel::GroupSyncWrite *syncWrite_;

    //interfaces
    hardware_interface::JointStateInterface     joint_state_interface;
    hardware_interface::PosVelJointInterface    pos_vel_joint_interface;
    mh5_hardware::ActiveJointInterface          active_joint_interface;
    mh5_hardware::CommunicationStatsInterface   communication_stats_interface;

    int                         num_joints_;
    std::vector<Joint>          joints_;

    // communication statistics
    int read_total_packets_;
    int read_error_packets_;
    int write_total_packets_;
    int write_error_packets_;

    //help methods
    bool initPort();
    bool initJoints();
    bool setupDynamixelLoops();

};

} // namespace