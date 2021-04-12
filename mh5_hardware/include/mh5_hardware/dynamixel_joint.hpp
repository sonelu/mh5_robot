#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include "port_handler.hpp"
#include "active_joint_interface.hpp"

#pragma once

namespace mh5_hardware
{

/**
 * @brief Represents a Dynamixel servo with the registers and communication
 * methods.
 * 
 * Also has convenience methods for creating HW interfaces for access by
 * controllers.
 */
class Joint
{
public:

    /**
     * @brief Default constructor
     */
    Joint() {}

    /**
     * @brief Uses information from the paramter server to initialize the Joint.
     * 
     * It will look for the following paramters in the server, under the joint name:
     * 
     * - ``id``: the Dynamixel ID of the servo; if missing the joint will be marked 
     * as not prosent (ex. present_ = false) and this will exclude it from all
     * communication
     * 
     * - ``inverse``: indicates that the joint has position values specified CW (default)
     * are CCW see https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#drive-mode10
     * bit 0. If not present the default is ``false``
     * 
     * - ``offset``: a value [in radians] that will be added to converted raw position
     * from the hardware register to report present position of servos in radians.
     * Conversely it will be substracted from the desired command position before
     * converting to the raw position value to be stored in the servo.
     * 
     * Initializes the jointStateHandle_, jointPosVelHandle_ and jointActiveHandle_
     * attributes.
     * 
     * @param hw_nh node handle to the harware interface
     * @param name name given to this joint
     * @param port Dynamixel port used for communication; should have been
     * checked and opened prior by the HW interface
     * @param ph Dynamixel port handler for communication; should have been
     * checked and initialized priod by the HW interface
     */
    void fromParam(ros::NodeHandle& hw_nh, std::string& name, mh5_port_handler::PortHandlerMH5* port, dynamixel::PacketHandler* ph);

    /**
     * @brief Returns the Dynamixel ID of the joint
     * 
     * @return uint8_t the ID of the joint.
     */
    uint8_t id() { return id_;}

    /**
     * @brief Returns the name of the joint
     * 
     * @return std::string the name of the joint.
     */
    std::string name() { return name_;}

    /**
     * @brief Returns if the joint is present (all settings are ok and communication
     * with it was successfull).
     * 
     * @return true if the joint is physically present
     * @return false if the joint could not be detected
     */
    bool present() { return present_;}

    /**
     * @brief Updates the present flag of the joint.
     * 
     * @param state the desired state (true == present, false = not present)
     */
    void setPresent(bool state) { present_ = state;}

    /**
     * @brief Performs a Dynamixel ping to the joint. It will try up to num_tries
     * times in case there is no answer or there are communication errors.
     * 
     * @param num_tries how many tries to make if there are no answers
     * @return true if the joint has responded
     * @return false if the joint failed to respond after num_tries times
     */
    bool ping(const int num_tries);

    /**
     * @brief Hard-codes the initialization of the following registers in the
     * joint (see https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table).
     * 
     * The registers are initialized as follows:
     * 
     * Register          | Address | Value | Comments                 
     * ----------------- | ------- | ----- | -------------------------
     * return delay      | 9       | 0     | 0 us delay time          
     * drive mode        | 10      | 4     | if no "inverse" mode set 
     * drive mode        | 10      | 5     | if "inverse" mode set    
     * operating mode    | 11      | 3     | position control mode    
     * temperature limit | 31      | 75    | 75 degrees Celsius       
     * max voltage       | 32      | 135   | 13.5 V                   
     * velocity limit    | 44      | 1023  | max velocity             
     * max position      | 48      | 4095  | max value                
     * min position      | 52      | 0     | min value                
     * 
     * Other registers might be added in the future.
     */
    void initRegisters();

    /**
     * @brief Convenience method for writing a register to the servo. Depending
     * on the size parameter it will call write1ByteTxRx(), write2ByteTxRx() or
     * write4ByteTxRx().
     * 
     * @param address the address of the register to write to
     * @param size the size of the register to write to
     * @param value a value to write; it will be type casted to uint8_t, uint16_t
     * or unit32_t depending on the size parameter
     * @param num_tries number of times to try in case there are errors
     * @return true if the write was sucessful
     * @return false if there was a communication or hardware error
     */
    bool writeRegister(const uint16_t address, const int size, const long value, const int num_tries);

    /**
     * @brief Convenience method for reading a register frpm the servo. Depending
     * on the size parameter it will call read1ByteTxRx(), read2ByteTxRx() or
     * read4ByteTxRx().
     * 
     * @param address the address of the register to read from
     * @param size the size of the register to read
     * @param value a value to store the read result; it will be type casted to uint8_t, uint16_t
     * or unit32_t depending on the size parameter
     * @param num_tries number of times to try in case there are errors
     * @return true if the read was sucessful
     * @return false if there was a communication or hardware error
     */
    bool readRegister(const uint16_t address, const int size, long& value, const int num_tries);


    /**
     * @brief Reboots the device by invoking the REBOOT Dynamixel instruction
     * 
     * @param num_tries how many tries to make if there are no answers
     * @return true if the reboot was successful
     * @return false if there were communication of harware errors
     */
    bool reboot(const int num_tries);


    /**
     * @brief Returns if the joint is active (torque on).
     * 
     * @param refresh if this parameter is true it will force a re-read of the 
     * register 64 from the servo otherwise it will report the cached value
     * 
     * @return true the torque is active
     * @return false the torque is inactive
     */
    bool isActive(bool refresh = false);

    /**
     * @brief Sets torque on for the joint. Forces writing 1 in the register
     * 64 of the servo.
     * 
     * @return true if the activation was successfull
     * @return false if there was an error (communication or hardware)
     */
    bool torqueOn();

    /**
     * @brief Sets torque off for the joint. Forces writing 0 in the register
     * 64 of the servo.
     * 
     * @return true if the deactivation was successfull
     * @return false if there was an error (communication or hardware)
     */
    bool torqueOff();

    /**
     * @brief Indicates if there was a command to change the torque that
     * was not yet completed. It simply returns the active_command_flag_
     * member that should be set whenever a controllers wants to switch
     * the torque status and sets the active_command_.
     * 
     * @return true there is a command that was not syncronised to hardware
     * @return false there is no change in the status
     */
    bool shouldToggleTorque() { return active_command_flag_;}

    /**
     * @brief Resets to false the active_command_flag_. Normally 
     * used by the sync loops after successful processing of an
     * update.
     */
    void resetActiveCommandFlag() { active_command_flag_ = false;}

    /**
     * @brief Changes the torque by writing into register 64 in the hardware
     * using the active_command_ value.
     * If the change is successfull it will reset the active_command_flag_.
     * 
     * @return true successful change
     * @return false communication or harware error
     */
    bool toggleTorque();

    /**
     * @brief Indicates if there was a command to reboot the joint that
     * was not yet completed. It simply returns the reboot_command_flag_
     * member that should be set whenever a controllers wants to reboot
     * the joint.
     * 
     * @return true there is a reset that was not syncronised to hardware
     * @return false there is no change in the status
     */
    bool shouldReboot() { return reboot_command_flag_; }


    /**
     * @brief Resets to false the reboot_command_flag_. Normally 
     * used by the sync loops after successful processing of an
     * update.
     */
    void resetRebootCommandFlag() { reboot_command_flag_ = 0.0; reboot_command_flag_ = false; }

    /**
     * @brief Produces an internal format for torque status based on a desired 
     * command.
     * 
     * @return uint8_t value suitable for writing to the hardware for the
     * desired torque status.
     */
    uint8_t getRawTorqueActiveFromCommand() { return (uint8_t) active_command_;}

    /**
     * @brief Set the position_state_ (represented in radians) from a raw_pos
     * that represents the value read from the hardware. It takes into account
     * the servo's charactistics, and the offset with the formula:
     * 
     *      position_state_ = (raw_pos - 2047 ) * 0.001533980787886 + offset_
     * 
     * @param raw_pos a raw position as read from the hardware; this will 
     * already contain the "inverse" classification.
     */
    void setPositionFromRaw(int32_t raw_pos) { position_state_ = (raw_pos - 2047 ) * 0.001533980787886 + offset_;}


    /**
     * @brief Set the velocity_state_ (represented in radians/sec) from a raw_vel
     * that represents the value read from the hardware. It takes into account
     * the servo's charactistics with the formula:
     * 
     *      velocity_state_ = raw_vel * 0.023980823922402
     * 
     * @param raw_vel a raw velocity as read from the hardware; this will 
     * already contain the "inverse" classification and is also signed
     */
    void setVelocityFromRaw(int32_t raw_vel) { velocity_state_ = raw_vel * 0.023980823922402;}

    /**
     * @brief Set the effort_state_ (represented in Nm) from a raw_eff
     * that represents the value read from the hardware. It takes into account
     * the servo's charactistics with the formula:
     * 
     *      effort_state_ = raw_eff * 0.0014
     * 
     * @param raw_eff a raw effort as read from the hardware; this will 
     * already contain the "inverse" classification and is also signed
     */
    void setEffortFromRaw(int32_t raw_eff) { effort_state_ = raw_eff * 0.0014;}

    /**
     * @brief @brief Set the voltage_state_ (represented in V) 
     * from a raw_volt that represents the value read from the hardware. The method
     * simply divides with 100 and converts to double.
     * 
     * @param raw_volt the value of voltage as read in hardware
     */
    void setVoltageFromRaw(int16_t raw_volt) { voltage_state_ = raw_volt / 100.0; }


    /**
     * @brief Set the temperature_state_ (represented in degrees Celsius) 
     * from a raw_temp that represents the value read from the hardware. The method
     * simply converts to double.
     * 
     * @param raw_temp 
     */
    void setTemperatureFromRaw(int8_t raw_temp) { temperature_state_ = (double) raw_temp; }


    /**
     * @brief Produces an internal format for position based on a desired 
     * command position (expressed in radians) using the formula:
     * 
     *      result = (position_command_ - offset_) / 0.001533980787886 + 2047
     * 
     * @return int32_t a value suitable for writing to the hardware for the
     * desired position in position_command_ expressed in radians.
     */
    int32_t getRawPositionFromCommand() {return (int32_t)((position_command_ - offset_) / 0.001533980787886 + 2047);}
    
    /**
     * @brief The velocity_command_ indicates the desired velocity (in rad/s) for
     * the execution of the position commands. Since we configure the servo in time
     * profile mode, the command is translated into a desired duration for the
     * execution of the position command, that is after that stored into 
     * register 112. For this the method calculates the delta between the desired position
     * and the current position divided by the desired velocity, obtaining thus
     * the desired duration for the move. The number is then multiplied with 1000
     * as the harware expect the duration in ms. The full formula for the value
     * is:
     * 
     *      result = abs((position_command_ - position_state_) / velocity_command_) * 1000
     * 
     * @return uint32_t a value suitable for writing to the hardware profile velocity
     * for the desired position in velocity_command_ expressed in radians/s.
     */
    uint32_t getVelocityProfileFromCommand() { return abs((position_command_ - position_state_) / velocity_command_) * 1000; }

    /**
     * @brief Returns the handle to the joint position interface object for this joint
     * 
     * @return const hardware_interface::JointStateHandle& 
     */
    const hardware_interface::JointStateHandle& getJointStateHandle() { return jointStateHandle_; }

    /**
     * @brief Returns the handle to the joint position / velocity command interface object for this joint
     * 
     * @return const hardware_interface::PosVelJointHandle& 
     */
    const hardware_interface::PosVelJointHandle& getJointPosVelHandle() { return jointPosVelHandle_; }

    /**
     * @brief Returns the handle to the joint activation command interface object for this joint
     * 
     * @return const mh5_hardware::JointTorqueAndReboot& 
     */
    const mh5_hardware::JointTorqueAndReboot& getJointActiveHandle() { return jointActiveHandle_; }


protected:
    /// @brief The name of the joint
    std::string                         name_;

    /// @brief The communication port to be used
    mh5_port_handler::PortHandlerMH5*   port_;

    /// @brief Dynamixel packet handler to be used
    dynamixel::PacketHandler*           ph_;

    /// @brief The node handler of the owner (hardware interface)
    ros::NodeHandle                     nh_;

    /// @brief Name of the owner as a c_str() - for easy printing of messages
    const char*                         nss_;

    //actual servos
    /// @brief Servo ID
    uint8_t         id_;

    /// @brief Servo is present (true) or not (false)
    bool            present_;

    /// @brief Servo uses inverse rotation
    bool            inverse_;

    /// @brief Offest for servo from 0 position (center) in radians
    double          offset_;

    //actual states
    /// @brief Current position in radians
    double          position_state_;

    /// @brief Current velocity in radians/s
    double          velocity_state_;

    /// @brief Current effort in Nm
    double          effort_state_;

    /// @brief Current torque state [0.0 or 1.0]
    double          active_state_;

    /// @brief Current voltage [V]
    double          voltage_state_;

    /// @brief Current temperature deg C
    double          temperature_state_;

    //commands
    /// @brief Desired position in radians
    double          position_command_;

    /// @brief Desired velocity in radians/s
    double          velocity_command_;

    /// @brief Indicates that the controller has updated the
    /// desired poistion / velocity and is not yet syncronised.
    bool            poistion_command_flag_;

    /// @brief Desired torque state [0.0 or 1.0]
    double          active_command_;

    /// @brief Indicates that the controller has updated the
    /// desired torque state and is not yet syncronised.
    bool            active_command_flag_;

    /// @brief Controller requested a reboot and is not yet syncronised
    bool            reboot_command_flag_;

    //hardware handles
    /// @brief A handle that provides access to position, velocity and effort
    hardware_interface::JointStateHandle    jointStateHandle_;

    /// @brief A handle that provides access to desired position and desired velocity
    hardware_interface::PosVelJointHandle   jointPosVelHandle_;

    /// @brief A handle that provides access to desired torque state
    mh5_hardware::JointTorqueAndReboot       jointActiveHandle_;
};


} //namespace