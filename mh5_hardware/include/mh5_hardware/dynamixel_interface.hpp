#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include "port_handler.hpp"
#include "active_joint_interface.hpp"

#pragma once

namespace mh5_hardware
{


class Joint
{
public:
    Joint() {}

    void fromParam(ros::NodeHandle& hw_nh, std::string& name, mh5_port_handler::PortHandlerMH5* port, dynamixel::PacketHandler* ph);

    uint8_t id() { return id_;}
    std::string name() { return name_;}
    bool present() { return present_;}
    void setPresent(bool state) { present_ = state;}

    bool ping(const int num_tries);
    void initRegisters();

    bool writeRegister(const uint16_t address, const int size, const long value, const int num_tries);
    bool readRegister(const uint16_t address, const int size, long& value, const int num_tries);

    bool isActive(bool refresh);
    bool torqueOn();
    bool torqueOff();
    bool shouldToggleTorque() { return active_state_ != active_command_;}
    bool toggleTorque();

    void setPositionFromRaw(int32_t raw_pos) { position_state_ = (raw_pos - 2047 ) * 0.001533980787886 + offset_;}
    void setVelocityFromRaw(int32_t raw_vel) { velocity_state_ = raw_vel * 0.023980823922402;}
    void setEffortFromRaw(int32_t raw_eff) { effort_state_ = raw_eff * 0.0014;}

    int32_t getRawPositionFromCommand() {return (int32_t)((position_command_ - offset_) / 0.001533980787886 + 2047);}
    uint32_t getVelocityProfileFromCommand() { return abs((position_command_ - position_state_) / velocity_command_) * 1000; }

    const hardware_interface::JointStateHandle& getJointStateHandle() { return jointStateHandle_; }
    const hardware_interface::PosVelJointHandle& getJointPosVelHandle() { return jointPosVelHandle_; }
    const hardware_interface::JointHandle& getJointActiveHandle() { return jointActiveHandle_; }

protected:
    std::string                         name_;

    mh5_port_handler::PortHandlerMH5*   port_;
    dynamixel::PacketHandler*           ph_;

    ros::NodeHandle                     nh_;
    const char*                         nss_;       // c string for nh_ namespace; used for messages

    //actual servos
    uint8_t         id_;
    bool            present_;
    bool            inverse_;
    double          offset_;

    //actual states
    double          position_state_;
    double          velocity_state_;
    double          effort_state_;
    double          active_state_;

    //commands
    double          position_command_;
    double          velocity_command_;
    double          active_command_;

    //hardware handles
    hardware_interface::JointStateHandle    jointStateHandle_;
    hardware_interface::PosVelJointHandle   jointPosVelHandle_;
    hardware_interface::JointHandle         jointActiveHandle_;
};


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
    dynamixel::GroupSyncRead *syncRead_;
    dynamixel::GroupSyncWrite *syncWrite_;

    //interfaces
    hardware_interface::JointStateInterface     joint_state_interface;
    hardware_interface::PosVelJointInterface    pos_vel_joint_interface;
    mh5_hardware::ActiveJointInterface          active_joint_interface;

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