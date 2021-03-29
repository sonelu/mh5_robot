#include <pluginlib/class_list_macros.hpp>
#include "mh5_controllers/active_joint_controller.hpp"

namespace mh5_controllers
{

bool ActiveJointController::init(mh5_hardware::ActiveJointInterface* hw, ros::NodeHandle &n)
{
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_)) {
        ROS_ERROR("Failed to getParam '%s' (namespace: %s).", param_name, n.getNamespace());
        return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0) {
        ROS_ERROR("List of joint names is empty.");
        return false;
    }

    for(unsigned int i=0; i<n_joints_; i++)
    {
        try {
            joints_.push_back(hw->getHandle(joint_names_[i]));
        }
        catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR("Exception thrown: %s", e.what());
            return false;
        }
    }

    commands_buffer_.writeFromNonRT(0.0);

    sub_command_ = n.subscribe<std_msgs::Bool>("torque/command", 1, &ActiveJointController::commandCB, this);
    return true;
}


void ActiveJointController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
    double command = *commands_buffer_.readFromRT();
    for(unsigned int i=0; i<n_joints_; i++)
        joints_[i].setCommand(command);
}

} // namespace

PLUGINLIB_EXPORT_CLASS(mh5_controllers::ActiveJointController, controller_interface::ControllerBase)