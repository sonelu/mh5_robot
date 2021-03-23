#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>

#pragma once

namespace mh5_hardware_interface
{


class JointStateHandle : public hardware_interface::JointStateHandle
{
    public:

        JointStateHandle() = default;

        JointStateHandle(const std::string& name, const double* pos, const double* vel, const double* eff, const int* act)
        : hardware_interface::JointStateHandle(name, pos, vel, eff), active_(act)
        {
            if (!act)
                throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Activation data pointer is null.");

        }

        bool isActive()    const {return active_;}


    private:
        const int* active_          = {nullptr};

};

class JointStateInterface : public hardware_interface::HardwareResourceManager<mh5_hardware_interface::JointStateHandle> {};


class ActiveJointHandle : public JointStateHandle
{
    public:

        ActiveJointHandle() = default;

        ActiveJointHandle(const mh5_hardware_interface::JointStateHandle& js, int *cmd_act)
        : mh5_hardware_interface::JointStateHandle(js), cmd_act_(cmd_act)
        {
            if (!cmd_act)
                throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command activation pointer is null.");
        }

        void setCommandActive(int cmd_act)    {assert(cmd_act_); *cmd_act_ = cmd_act;}
        int getCommandActive()                const {assert(cmd_act_); return *cmd_act_;}

    private:
        int*    cmd_act_    = {nullptr};
};

class ActiveJointInterface : public hardware_interface::HardwareResourceManager<mh5_hardware_interface::ActiveJointHandle, hardware_interface::ClaimResources> {};

}