
#include <pluginlib/class_list_macros.hpp>

#include "mh5_hardware/i2c_interface.hpp"
// #include "mh5_hardware/active_joint_interface.hpp"
// #include "mh5_hardware/dynamixel_joint.hpp"

using namespace mh5_hardware;



MH5I2CInterface::MH5I2CInterface(){
}


MH5I2CInterface::~MH5I2CInterface(){
    ROS_INFO("Interface closed");
}


bool MH5I2CInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    nh_ = robot_hw_nh;
    nss_ = nh_.getNamespace().c_str();     // to avoid calling it all the time
    
    // init port


    // init devices

    // register handles
    // //State
    // joint_state_interface.registerHandle(joints_[i]->getJointStateHandle());
    // //Command Postion - Velocity
    // pos_vel_joint_interface.registerHandle(joints_[i]->getJointPosVelHandle());
    // //Torque activation
    // active_joint_interface.registerHandle(joints_[i]->getJointActiveHandle());

    // //Register interfaces
    // registerInterface(&joint_state_interface);
    // registerInterface(&pos_vel_joint_interface);
    // registerInterface(&active_joint_interface);
    
    //return true for successful init or ComboRobotHW initialisation will fail
    return true;
}


void MH5I2CInterface::read(const ros::Time& time, const ros::Duration& period)
{
    // read IMU

    // read ADC
}


void MH5I2CInterface::write(const ros::Time& time, const ros::Duration& period)
{

}



PLUGINLIB_EXPORT_CLASS(mh5_hardware::MH5I2CInterface, hardware_interface::RobotHW)
