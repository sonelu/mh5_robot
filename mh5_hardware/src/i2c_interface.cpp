#include <stdio.h>
#include <fcntl.h>
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
    if (!nh_.getParam("port", port_name_)) {
        ROS_ERROR("[%s] no 'port' specified", nh_.getNamespace().c_str());
        return false;
    }
    if ((port_ = open(port_name_.c_str(), O_RDWR)) < 0) {
        ROS_ERROR("[%s] failed to open port %s", nh_.getNamespace().c_str(), port_name_.c_str());
        return false;
    }
    ROS_INFO("[%s] successfully opened port %s", nh_.getNamespace().c_str(), port_name_.c_str());

    // init devices
    if (!nh_.getParam("rates/imu", imu_loop_rate_)) {
        ROS_ERROR("[%s] no 'rates/imu' specified", nh_.getNamespace().c_str());
        return false;
    }
    int    imu_id;
    if (!nh_.getParam("imu/id", imu_id)) {
        ROS_ERROR("[%s] no 'imu/id' specified", nh_.getNamespace().c_str());
        return false;
    }
    imu = new LSM6DS3(port_, (uint8_t)imu_id);
    status_t result = imu->initialize();
    if (result != IMU_SUCCESS) {
        ROS_ERROR("[%s] failed to communicate with the IMU: %i", nh_.getNamespace().c_str(), result);
        return false;
    }
    ROS_INFO("[%s] IMU initialized", nh_.getNamespace().c_str());

    hardware_interface::ImuSensorHandle imu_h("IMU", "chest", NULL, NULL,
                                              ang_vel_, NULL, lin_acc_, NULL);
    imu_sensor_interface.registerHandle(imu_h);
    registerInterface(&imu_sensor_interface);
    
    //return true for successful init or ComboRobotHW initialisation will fail
    return true;
}


void MH5I2CInterface::read(const ros::Time& time, const ros::Duration& period)
{
    // read IMU
    if (imu_loop_rate_ > 0.0 && imu_last_execution_time_ + ros::Duration(1.0/imu_loop_rate_) < time)
    {
        imu_last_execution_time_ += ros::Duration(1.0/imu_loop_rate_);
        // convert to rad/s
        ang_vel_[0] = imu->readFloatGyroX() * 0.017453293;
        ang_vel_[1] = imu->readFloatGyroY() * 0.017453293;
        ang_vel_[2] = imu->readFloatGyroZ() * 0.017453293;
        // convert to m/s^2
        lin_acc_[0] = imu->readFloatAccelX() * 9.80665;
        lin_acc_[1] = imu->readFloatAccelY() * 9.80665;
        lin_acc_[2] = imu->readFloatAccelZ() * 9.80665;
    }

    // read ADC
}


void MH5I2CInterface::write(const ros::Time& time, const ros::Duration& period)
{

}



PLUGINLIB_EXPORT_CLASS(mh5_hardware::MH5I2CInterface, hardware_interface::RobotHW)
