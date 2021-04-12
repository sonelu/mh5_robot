#include <hardware_interface/joint_command_interface.h>
//#include <hardware_interface/internal/hardware_resource_manager.h>

#pragma once


namespace mh5_hardware
{


/**
 * @brief Extends the hardware_interface::JointHandle with a boolean flag
 * that indicates when a new command was posted. This helps the HW interface
 * decide if that value needs to be replicated to the servos or not.
 */
class JointHandleWithFlag : public hardware_interface::JointHandle
{
public:

    JointHandleWithFlag() = default;

    /**
     * @brief Construct a new JointHandleWithFlag object by extending the
     * hardware_interface::JointHandle with an additional boolean
     * flag that indicates a new command has been issued.
     * 
     * @param js the JointStateHandle that is commanded
     * @param cmd pointer to the command attribute in the HW interface
     * @param cmd_flag pointed to the bool flag in the HW interface that is
     * used to indicate that the value was changed and therefore needs to be
     * synchronized by the HW.
     */
    JointHandleWithFlag(const JointStateHandle& js, double* cmd, bool* cmd_flag)
    : hardware_interface::JointHandle(js, (cmd)),
      cmd_flag_(cmd_flag)
      {
          if (!cmd_flag_)
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command flag pointer is null.");
      }

    /**
     * @brief Overrides the hardware_interface::JointHandle setCommand()
     * method by setting the flag in the HW to true to indicate that a new
     * value was storred and therefore it needs to be synchronised after
     * calling the inherited method.
     * 
     * @param command the command set to the joint
     */
    void setCommand(double command) 
    {
        hardware_interface::JointHandle::setCommand(command);
        assert(cmd_flag_);
        *cmd_flag_ = true;
    }

private:

    /**
     * @brief Keeps the pointed to the flag in the HW that indicates when
     * value change.
     */
    bool* cmd_flag_ = {nullptr};

};

class JointTorqueAndReboot : public JointHandleWithFlag
{
public:
    JointTorqueAndReboot() = default;

    JointTorqueAndReboot(const JointStateHandle& js, double* torque, bool* torque_flag, bool* reboot_flag)
    : JointHandleWithFlag(js, torque, torque_flag), reboot_flag_(reboot_flag) 
    {
        if (!reboot_flag_)
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Rebbot flag pointer is null.");
    }

    void setReboot(bool reboot) { assert(reboot_flag_); *reboot_flag_ = reboot; }
    bool getReboot() { assert(reboot_flag_); return *reboot_flag_; }

private:

    bool* reboot_flag_ = {nullptr};

};

/**
 * @brief Joint that supports activation / deactivation
 * 
 * To keep track of updates to the HW resource we use and additional flag
 * that is set to true when a new command is issued to the servo. The
 * communication loops will use this flag to determine which servos really
 * need to be syncronised and will reset it once the synchronisation is
 * finished.
 */
class ActiveJointInterface : public hardware_interface::HardwareResourceManager<JointTorqueAndReboot> {};


}
