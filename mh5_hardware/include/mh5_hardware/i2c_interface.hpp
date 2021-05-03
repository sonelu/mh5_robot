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

    // Read Loops
    // /// @brief Sync Loop for reading the position, velocity and load
    // mh5_hardware::PVLReader *pvlReader_;
    // /// @brief Sync Loop for reading the temperature and voltage
    // mh5_hardware::TVReader  *tvReader_;

    // Write Loops
    /// @brief SyncLoop for writing the position and velocity
    // mh5_hardware::PVWriter  *pvWriter_;
    // /// @brief SyncLoop for writing the torque status command
    // mh5_hardware::TWriter   *tWriter_;


    //interfaces
    // hardware_interface::JointStateInterface     joint_state_interface;
    // hardware_interface::PosVelJointInterface    pos_vel_joint_interface;
    // mh5_hardware::ActiveJointInterface          active_joint_interface;
    // mh5_hardware::CommunicationStatsInterface   communication_stats_interface;

    // Devices
    /**
     * @brief IMU object
     */
    LSM6DS3*                imu;
    /**
     * @brief Keeps the desired execution rate (in Hz) the for IMU
     */
    double                  imu_loop_rate_;
    /**
     * @brief Stores the last time the IMU read was executed
     */
    ros::Time               imu_last_execution_time_;

    double ang_vel_[3];
    double lin_acc_[3];

    //interfaces
    hardware_interface::ImuSensorInterface     imu_sensor_interface;

    // TLA2024*      ADC;

    // /**
    //  * @brief Initializes the Dynamixel port.
    //  * 
    //  * @return true if initialization was successfull
    //  * @return false if initialization was unsuccessfull
    //  */
    // bool initPort();

    // /**
    //  * @brief Initializes the joints.
    //  * 
    //  * @return true 
    //  * @return false 
    //  */
    // bool initJoints();

    // /**
    //  * @brief Convenience function that constructs a loop, reads parameters
    //  * "rates/<loop_name>" from parameter server or, if not found, uses
    //  * a default rate for initialisation. It also calls prepare() and 
    //  * registers it communication handle (from getCommStatHandle() with the
    //  * HW communication status inteface)
    //  * 
    //  * @tparam Loop the class for the loop
    //  * @param name the name of the loop
    //  * @param default_rate the default rate to use incase no parameter is 
    //  * found in the parameter server
    //  * @return Loop* the newly created loop object
    //  */
    // template <class Loop>
    // Loop* setupLoop(std::string name, const double default_rate);

    // /**
    //  * @brief Creates and initializes all the loops used by the HW interface:
    //  * - Read: position, velocity, load (pvl_reader)
    //  * - Read: temperature, voltage (tv_reader)
    //  * - Write: position, velocity (pv_writer)
    //  * - Write: torque (t_writer)
    //  * 
    //  * @return true 
    //  */
    // bool setupDynamixelLoops();

};

} // namespace