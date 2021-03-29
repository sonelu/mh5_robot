#include <pluginlib/class_list_macros.hpp>

#include "mh5_controllers/extended_trajectory_controller.hpp"


namespace mh5_controllers
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

    mh5_hardware::ActiveJointInterface* act_hw = robot_hw->get<mh5_hardware::ActiveJointInterface>();
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


void ExtendedJointTrajectoryController::starting(const ros::Time& time)
{
    pos_controller_->starting(time);
    act_controller_->starting(time);
}


void ExtendedJointTrajectoryController::stopping(const ros::Time& time)
{
    pos_controller_->stopping(time);
    act_controller_->stopping(time);
}


void ExtendedJointTrajectoryController::update(const ros::Time& time, const ros::Duration& period)
{
    pos_controller_->update(time, period);
    act_controller_->update(time, period);
}


} // namespace

PLUGINLIB_EXPORT_CLASS(mh5_controllers::ExtendedJointTrajectoryController, controller_interface::ControllerBase)
