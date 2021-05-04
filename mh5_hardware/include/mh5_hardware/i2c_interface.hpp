#include <hardware_interface/robot_hw.h>
#include <hardware_interface/imu_sensor_interface.h>

#include "LSM6DS3.hpp"

#pragma once

namespace mh5_hardware
{

/**
 * @brief Main class implementing the protocol required by ``ros_control`` for
 * providing access to the robot hardware connected on an I2C bus.
 * 
 * This class performs communication with the devices using ioctl.
 * 
 * The class should be instantiated by the ``pluginlib`` once the main mode is
 * started and initiates the load of the ``CombinedRobotHW`` class.
 * 
 * The class uses the information from the param server to get details about the
 * communication port configuration and the attached devices. For each device
 * interface the following parameters are read:
 * 
 * ...
 * 
 * The class registers itself with the ``pluginlib`` by calling:
 * 
 *      PLUGINLIB_EXPORT_CLASS(mh5_hardware::MH5I2CInterface, hardware_interface::RobotHW)
 */
class MH5I2CInterface: public hardware_interface::RobotHW
{
public:
    /** Construct a new MH5I2CInterface object. Default constructor 
     * to support ``pluginlib``.
     */
    MH5I2CInterface();

    /**
     * @brief Destroy the MH5I2CInterface object. Provided for 
     * ``pluginlib`` support.
     */
    ~MH5I2CInterface();

    /**
     * @brief Initializes the interface.
     * 
     * Will open the system port port and the configuration of the
     * devices associated with this interface. If either of these fails it will
     * return false.
     * 
     * @param root_nh A NodeHandle in the root of the caller namespace.
     * @param robot_hw_nh A NodeHandle in the namespace from which the RobotHW
     * should read its configuration.
     * @return true if initialization was successful
     * @return false If the initialization was unsuccessful
     */
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);

    /**
     * @brief Performs the read of values for all the devices. Devices might
     * have specific frequency preferences and would compare the time / period
     * provided with their own to decide if they indeed need to do anything.
     * 
     * @param time The current time
     * @param period The time passed since the last call to \ref read
     */
    void read(const ros::Time& time, const ros::Duration& period);

    /**
     * @brief Performs the write of values for all the devices. Devices might
     * have specific frequency preferences and would compare the time / period
     * provided with their own to decide if they indeed need to do anything.
     * 
     * @param time The current time
     * @param period The time passed since the last call to \ref read
     */
    void write(const ros::Time& time, const ros::Duration& period);

protected:
    ros::NodeHandle nh_;
    const char* nss_;       // c string for nh_ namespace; used for messages
    
    // communication
    std::string     port_name_;
    int             port_;

    // Devices
    /// IMU object
    LSM6DS3*        imu_;
    /// Stores the read velocities from the IMU converted to rad/s
    double          ang_vel_[3] = {0.0, 0.0, 0.0};
    /// Stores the read accelerations from the IMU converted in m/s^2
    double          lin_acc_[3] = {0.0, 0.0, 0.0};
    /// Low-pass filter factor for IMU
    double             imu_lpf_ = 0.1;
    /// Keeps the desired execution rate (in Hz) the for IMU
    double          imu_loop_rate_;
    /// Stores the last time the IMU read was executed
    ros::Time       imu_last_execution_time_;


    //interfaces
    std::vector<double>                        imu_orientation_ = {0.0, 0.0, 0.0, 1.0};
    hardware_interface::ImuSensorHandle        imu_h_;
    hardware_interface::ImuSensorInterface     imu_sensor_interface_;

    // TLA2024*      ADC;

    double calcLPF(double old_val, double new_val, double factor);

};

} // namespace