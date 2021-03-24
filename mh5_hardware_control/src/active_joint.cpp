#include "mh5_hardware_control/active_joint.hpp"

using namespace mh5_hardware_interface;

bool JointTrajectoryController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
    if (!BasePosVelController::init(robot_hw, root_nh, controller_nh))
        return false;

    if (!ActiveJointGroupController::init(robot_hw, root_nh, controller_nh))
        return false;
}


bool JointTrajectoryController::initRequest(hardware_interface::RobotHW* hw, ros::NodeHandle& nh, ros::NodeHandle& pnh,
            controller_interface::ControllerBase::ClaimedResources& cr)
{
    return BasePosVelController::initRequest(hw, nh, pnh, cr);
}


void JointTrajectoryController::starting(const ros::Time& time)
{
    BasePosVelController::starting(time);
}


void JointTrajectoryController::update(const ros::Time& time, const ros::Duration& period)
{
    BasePosVelController::update(time, period);
}