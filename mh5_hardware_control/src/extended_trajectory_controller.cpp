#include <pluginlib/class_list_macros.hpp>

#include "mh5_hardware_control/extended_trajectory_controller.hpp"


namespace mh5_hardware_interface
{

bool ExtendedJointTrajectoryController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
    hardware_interface::PosVelJointInterface*   pos_hw = robot_hw->get<hardware_interface::PosVelJointInterface>();
    if(!pos_hw) {
        ROS_ERROR("Requires pos_velocity_interface");
        return false;
    }
    pos_controller_ = new BaseJointTrajectoryController();
    if(!pos_controller_->init(pos_hw, root_nh, controller_nh)) {
        ROS_ERROR("Failed to initialize the trajectory controller object");
        return false;
    }

    ActiveJointInterface* act_hw = robot_hw->get<ActiveJointInterface>();
    if(!act_hw) {
        ROS_ERROR("Requires torque_interface");
        return false;
    }
    act_controller_ = new ActiveJointController();
    if(!act_controller_->init(act_hw, controller_nh)) {
        ROS_ERROR("Failed to initialize the torque controller object");
        return false;
    }
    act_hw->clearClaims();

    return true;
}


void ExtendedJointTrajectoryController::update(const ros::Time& time, const ros::Duration& period)
{

}


} // namespace

PLUGINLIB_EXPORT_CLASS(mh5_hardware_interface::ExtendedJointTrajectoryController, controller_interface::ControllerBase)
