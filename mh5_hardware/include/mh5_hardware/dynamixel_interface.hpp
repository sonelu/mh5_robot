#include "dynamixel_joint.hpp"
#include "dynamixel_loop.hpp"
#include "active_joint_interface.hpp"


#pragma once

namespace mh5_hardware
{

/**
 * @brief Main class implementing the protocol required by ``ros_control`` for
 * providing access to the robot hardware.
 * 
 * This class performs communication with the servos using Dynamixel protocol
 * and manages the state of these servos. It uses for this purpose 
 * [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) (specifically the
 * ROS implementation of it) with the only exception that for port communication
 * it uses a custom subclass of ``PortHandler`` in order to be able to configure
 * the communication port with RS485 support, because the interface board used
 * by RH5 robot uses SC16IS762 chips that control the flow in heardware, but need
 * tto be connfigured in RS485 mode via ``ioctl``.
 * 
 * The class should be instantiated by the ``pluginlib`` once the main mode is
 * started and initiates the load of the ``CombinedRobotHW`` class.
 * 
 * The class uses the information from the param server to get details about the
 * communication port configuration and the attached servos. For each dynamixel
 * interface the following parameters are read:
 * 
 * 
 * The class registers itself with the ``pluginlib`` by calling:
 * 
 *      PLUGINLIB_EXPORT_CLASS(mh5_hardware::MH5DynamixelInterface, hardware_interface::RobotHW)
 */
class MH5DynamixelInterface: public hardware_interface::RobotHW
{
public:
    /** Construct a new MH5DynamixelInterface object. Default constructor 
     * to support ``pluginlib``.
     */
    MH5DynamixelInterface();

    /**
     * @brief Destroy the MH5DynamixelInterface object. Provided for 
     * ``pluginlib`` support.
     */
    ~MH5DynamixelInterface();

    /**
     * @brief Initializes the interface.
     * 
     * Will call the protected methods initPort() and initJoints() to perform
     * the initialization of the Dynamixel port and the configuration of the
     * joints associated with this interface. If either of these fails it will
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
     * @brief Performs the read of values for all the servos. This is done
     * through the sync loops objects that have been prepared in init(). The
     * caller (the main ROS node owning the hardware) would call this method
     * at an arbitrary frequency that is dictated by it's processing needs
     * (and can be much higher than the frequency with with we need to 
     * syncronise the data with the actual servos). For this reason each sync
     * loop is responsible to keep track of it's own processing frequency and
     * skip executing if requests are too often.
     * 
     * In this particular case this method asks the following loops to run:
     * 
     * - Position, Velocity, Load (\ref pvlReader_)
     * - Temperature, Voltage (\ref tvReader_)
     * 
     * @param time The current time
     * @param period The time passed since the last call to \ref read
     */
    void read(const ros::Time& time, const ros::Duration& period);

    /**
     * @brief Performs the write of position, velocity profile and acceleration profile
     * for all servos that are marked as present. Assumes the servos have already been
     * configured with velocity profile (see Dyanamixel manual 
     * https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#what-is-the-profile).
     * Converts the values from ISO (radians for position, rad / sec for velocity)
     * to Dynamixel internal measures. Uses a Dynamixel SyncWrite to write
     * the values to all servos with one communication packet.
     * 
     * @param time The current time
     * @param period The time passed since the last call to \ref read
     */
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

    /**
     * @brief Sync Loop for reading the position, velocity and load of the
     * servos.
     */
    mh5_hardware::PVLReader *pvlReader_;
    mh5_hardware::PVWriter  *pvWriter_;

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

    /**
     * @brief Initializes the Dynamixel port.
     * 
     * @return true if initialization was successfull
     * @return false if initialization was unsuccessfull
     */
    bool initPort();

    /**
     * @brief Initializes the joints.
     * 
     * @return true 
     * @return false 
     */
    bool initJoints();
    bool setupDynamixelLoops();

};

} // namespace