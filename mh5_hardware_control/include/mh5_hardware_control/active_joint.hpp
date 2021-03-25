#include <hardware_interface/robot_hw.h>


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

//typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>, hardware_interface::PosVelJointInterface> PosVelJointTrajectoryController;
//typedef forward_command_controller::ForwardJointGroupCommandController<mh5_hardware_interface::ActiveJointInterface> ActiveJointGroupController;


// class JointTrajectoryController : public PosVelJointTrajectoryController
// {
//     public:
//         JointTrajectoryController();
//         ~JointTrajectoryController();
//     // public:
//     //     bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

//     //     bool initRequest(hardware_interface::RobotHW* hw, ros::NodeHandle& nh, ros::NodeHandle& pnh,
//     //         controller_interface::ControllerBase::ClaimedResources& cr) override;

//     //     void starting(const ros::Time&) override;

//     //     void update(const ros::Time&, const ros::Duration&) override;
// };

}
