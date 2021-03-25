
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

} // namespace