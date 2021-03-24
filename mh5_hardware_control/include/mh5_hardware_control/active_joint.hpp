#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <forward_command_controller/forward_joint_group_command_controller.h>
#include <trajectory_interface/quintic_spline_segment.h>

#pragma once

namespace mh5_hardware_interface
{


/**
 * @brief Adds support for the control of a joint that supports activation /
 * deactivation (ex. torque enable / disable).
 * 
 */
class ActiveJointHandle
{
    public:

        ActiveJointHandle() = default;

        ActiveJointHandle(const std::string& name, const int* act, int *cmd_act)
        : name_(name), act_(act), cmd_act_(cmd_act)
        {
            if (!act_)
                throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Present activation pointer is null.");

            if (!cmd_act_)
                throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Command activation pointer is null.");
        }

        std::string     getName()                    const { return name_;}
        int             getActive()                  const { assert(act_); return *act_;}
        const int*      getActivePtr()               const { return act_;} 
        void            setCommandActive(int cmd_act)      { assert(cmd_act_); *cmd_act_ = cmd_act;}
        int             getCommandActive()           const { assert(cmd_act_); return *cmd_act_;}
        const int*      getCommandActivePtr()        const { return cmd_act_;}

    private:
        std::string     name_;
        const int*      act_        = {nullptr};
        int*            cmd_act_    = {nullptr};
};


/**
 * @brief Joint that supports activation / deactivation
 * 
 */
class ActiveJointInterface : public hardware_interface::HardwareResourceManager<mh5_hardware_interface::ActiveJointHandle, hardware_interface::ClaimResources> {};

typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>, hardware_interface::PosVelJointInterface> BasePosVelController;
typedef forward_command_controller::ForwardJointGroupCommandController<mh5_hardware_interface::ActiveJointInterface> ActiveJointGroupController;

class JointTrajectoryController : public BasePosVelController, ActiveJointGroupController
{

};

}
