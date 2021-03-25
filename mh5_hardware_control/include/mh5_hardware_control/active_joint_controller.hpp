#include <std_msgs/Bool.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller.h>
#include "mh5_hardware_control/active_joint_interface.hpp"

#pragma once

namespace mh5_hardware_interface
{


class ActiveJointController : public controller_interface::Controller<ActiveJointInterface>
{
public:

    ActiveJointController()
    : controller_interface::Controller<ActiveJointInterface> ()
    {}

    ~ActiveJointController() {sub_command_.shutdown();}

    bool init(ActiveJointInterface* hw, ros::NodeHandle &n);
    void starting(const ros::Time& time) {}
    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

    std::vector< std::string >                        joint_names_;
    std::vector< hardware_interface::JointHandle >    joints_;
    realtime_tools::RealtimeBuffer<double>            commands_buffer_;
    unsigned int                                      n_joints_;

private:

    ros::Subscriber sub_command_;

    void commandCB(const std_msgs::BoolConstPtr& msg) {
        commands_buffer_.writeFromNonRT(msg->data);
    }

};

} // namespace