
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <trajectory_interface/quintic_spline_segment.h>
#include <controller_interface/multi_interface_controller.h>


#include "mh5_hardware_control/active_joint_controller.hpp"

#pragma once

namespace mh5_hardware_interface
{


typedef joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::PosVelJointInterface>
        BaseJointTrajectoryController;

class ExtendedJointTrajectoryController : public controller_interface::MultiInterfaceController<hardware_interface::PosVelJointInterface, ActiveJointInterface>
{
public:
    ExtendedJointTrajectoryController()
    : controller_interface::MultiInterfaceController<hardware_interface::PosVelJointInterface, ActiveJointInterface> (true)
    {}

    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    // /** \name Real-Time Safe Functions
    // *\{*/
    // /** \brief Holds the current position. */
    // void starting(const ros::Time& time);

    // /** \brief Cancels the active action goal, if any. */
    // void stopping(const ros::Time& /*time*/);

    void update(const ros::Time& time, const ros::Duration& period);

private:
    mh5_hardware_interface::BaseJointTrajectoryController*  pos_controller_;
    mh5_hardware_interface::ActiveJointController*          act_controller_;
};


} // namespace