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

    imu_ = new LSM6DS3(port_, (uint8_t)imu_id);
    status_t result = imu_->initialize();
    if (result != IMU_SUCCESS) {
        ROS_ERROR("[%s] failed to communicate with the IMU: %i", nh_.getNamespace().c_str(), result);
        return false;
    }
    result = imu_->writeRegister(LSM6DS3_ACC_GYRO_ORIENT_CFG_G, 0);
        // LSM6DS3_ACC_GYRO_ORIENT_YZX | LSM6DS3_ACC_GYRO_SIGN_Y_G_NEGATIVE);
    if (result != IMU_SUCCESS) {
        ROS_ERROR("[%s] failed to configure IMU orientation: %i", nh_.getNamespace().c_str(), result);
        return false;
    }
    ROS_INFO("[%s] IMU initialized", nh_.getNamespace().c_str());

    nh_.getParam("imu/orientation", imu_orientation_);
    imu_h_ = hardware_interface::ImuSensorHandle("imu",
                                              "chest", &imu_orientation_[0], NULL,
                                              ang_vel_, NULL, lin_acc_, NULL);
    const double* o = imu_h_.getOrientation();
    ROS_INFO("[%s] using IMU orientation: [x=%.4lf y=%.4lf z=%.4lf w=%.4lf]", nh_.getNamespace().c_str(),
        o[0], o[1], o[2], o[3]);
    // uint8_t oreg;
    // imu_->readRegister(&oreg, LSM6DS3_ACC_GYRO_ORIENT_CFG_G);
    // ROS_INFO("[%s] IMU orientation register: %i", nh_.getNamespace().c_str(), oreg);
    nh_.getParam("imu/lpf", imu_lpf_);
    ROS_INFO("[%s] IMU using low-pass filter factor: %.2lf", nh_.getNamespace().c_str(), imu_lpf_);
    imu_sensor_interface_.registerHandle(imu_h_);
    registerInterface(&imu_sensor_interface_);
    
    //return true for successful init or ComboRobotHW initialisation will fail
    return true;
}


double MH5I2CInterface::calcLPF(double old_val, double new_val, double factor)
{
    if (old_val == 0.0) {
        return new_val;
    }
    else {
        return old_val * (1 - factor) + new_val * factor;
    }
}


void MH5I2CInterface::read(const ros::Time& time, const ros::Duration& period)
{
    // read IMU
    if (imu_loop_rate_ > 0.0 && imu_last_execution_time_ + ros::Duration(1.0/imu_loop_rate_) < time)
    {
        imu_last_execution_time_ += ros::Duration(1.0/imu_loop_rate_);

        ang_vel_[0] = calcLPF(ang_vel_[0], - imu_->readFloatGyroZ(), imu_lpf_);
        ang_vel_[1] = calcLPF(ang_vel_[1], imu_->readFloatGyroY(), imu_lpf_);
        ang_vel_[2] = calcLPF(ang_vel_[2], - imu_->readFloatGyroX(), imu_lpf_); 

        lin_acc_[0] = calcLPF(lin_acc_[0], - imu_->readFloatAccelZ(), imu_lpf_);
        lin_acc_[1] = calcLPF(lin_acc_[1], imu_->readFloatAccelY(), imu_lpf_);
        lin_acc_[2] = calcLPF(lin_acc_[2], - imu_->readFloatAccelX(), imu_lpf_);
    }

    // read ADC
}


void MH5I2CInterface::write(const ros::Time& time, const ros::Duration& period)
{

}



PLUGINLIB_EXPORT_CLASS(mh5_hardware::MH5I2CInterface, hardware_interface::RobotHW)
