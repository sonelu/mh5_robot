#include <hardware_interface/joint_command_interface.h>


#pragma once


namespace mh5_hardware_interface
{

/**
 * @brief Joint that supports activation / deactivation
 * 
 */
class ActiveJointInterface : public hardware_interface::JointCommandInterface {};

}
