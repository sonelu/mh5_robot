
//#include <hardware_interface/joint_state_interface.h>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
//#include <forward_command_controller/forward_joint_group_command_controller.h>
#include <trajectory_interface/quintic_spline_segment.h>


#pragma once

namespace mh5_hardware_interface
{



class ExtendedJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
                                                                 hardware_interface::PosVelJointInterface>
{
// public:
//     ExtendedTrajectoryController();

    // /** \name Non Real-Time Safe Functions
    // *\{*/
    // bool init(HardwareInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    // /*\}*/

    // /** \name Real-Time Safe Functions
    // *\{*/
    // /** \brief Holds the current position. */
    // void starting(const ros::Time& time);

    // /** \brief Cancels the active action goal, if any. */
    // void stopping(const ros::Time& /*time*/);

    // void update(const ros::Time& time, const ros::Duration& period);
    /*\}*/
};


} // namespace