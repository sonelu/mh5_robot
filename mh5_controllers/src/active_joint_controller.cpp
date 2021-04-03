#include <pluginlib/class_list_macros.hpp>
#include "mh5_controllers/active_joint_controller.hpp"
#include "mh5_controllers/ActivateJoint.h"

namespace mh5_controllers
{

bool ActiveJointController::init(mh5_hardware::ActiveJointInterface* hw, ros::NodeHandle &n)
{
    if(!n.hasParam("groups"))
    {
        ROS_INFO("[%s] no groups specified; all joints will be placed in a group called 'all'",
                  n.getNamespace().c_str());
        joints_["all"];
        for (auto & joint_name : hw->getNames()) {
            joints_["all"].push_back(hw->getHandle(joint_name));
        }
        ROS_INFO("[%s] group 'all' registered with %d items",
                  n.getNamespace().c_str(), joints_["all"].size());
    }
    else
    {
        std::vector<std::string> groups;
        n.getParam("groups", groups);
        for (auto & group : groups)
        {
            std::vector<std::string>    names;      // could be joints or subgroups
            std::vector<std::string>    joint_names;// only joint names
            std::vector<hardware_interface::JointHandle>    joint_handles;
            joints_[group];
            n.getParam(group, names);
            for (auto & name : names) {
                if (joints_.count(name))
                {
                    // handle subgroup; we copy the handles from the subgroup
                    for (auto & handle : joints_[name]) {
                        joints_[group].push_back(handle);
                    }
                }
                else {
                    joints_[group].push_back(hw->getHandle(name));
                }
            }
            ROS_INFO("[%s] group '%s' registered with %d items",
                  n.getNamespace().c_str(), group.c_str(), joints_[group].size());
        }
    }
    hw->clearClaims();

    // n_joints_ = joint_names_.size();

    // if(n_joints_ == 0) {
    //     ROS_ERROR("List of joint names is empty.");
    //     return false;
    // }

    // for(unsigned int i=0; i<n_joints_; i++)
    // {
    //     try {
    //         joints_.push_back(hw->getHandle(joint_names_[i]));
    //     }
    //     catch (const hardware_interface::HardwareInterfaceException& e) {
    //         ROS_ERROR("Exception thrown: %s", e.what());
    //         return false;
    //     }
    // }

    // commands_buffer_.writeFromNonRT(false);

    torque_srv_ = n.advertiseService("switch_torque", &ActiveJointController::torqueCB, this);

    return true;
}


bool ActiveJointController::torqueCB(mh5_controllers::ActivateJoint::Request &req, mh5_controllers::ActivateJoint::Response &res)
{
    if (joints_.count(req.name))  {
        commands_buffer_.writeFromNonRT(req);
        res.success = true;
        res.message = "Group " + req.name + " torque change buffered";
        return true;
    }

    // a little ugly to search for and 
    for (auto & group: joints_) {
        for (auto & handle : joints_[group.first]) {
            if (handle.getName() == req.name) {
                commands_buffer_.writeFromNonRT(req);
                res.success = true;
                res.message = "Joint " + req.name + " torque change buffered";
                return true;
            }
        }
    }

    // no group nor joint with that name available
    res.success = false;
    res.message = "No group of joint named " + req.name + " found";
    return false;
}


void ActiveJointController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
    ActivateJoint::Request command = *commands_buffer_.readFromRT();

    if (joints_.count(command.name))  {
        for (auto & handle : joints_[command.name])
            handle.setCommand(command.state);
        return;
    }

    for (auto & group : joints_)
        for (auto & handle : joints_[group.first])
            if (handle.getName() == command.name)
            {
                handle.setCommand(command.state);
                return;
            }
}

} // namespace

PLUGINLIB_EXPORT_CLASS(mh5_controllers::ActiveJointController, controller_interface::ControllerBase)