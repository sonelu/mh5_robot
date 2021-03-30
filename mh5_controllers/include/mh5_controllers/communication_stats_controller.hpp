#include <diagnostic_msgs/DiagnosticArray.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_interface/controller.h>
#include <mh5_hardware/communication_stats_interface.hpp>

#pragma once

namespace mh5_controllers
{


class CommunicationStatsController : public controller_interface::Controller<mh5_hardware::CommunicationStatsInterface>
{
public:

    CommunicationStatsController() : publish_period_(0.0) {}

    bool init(mh5_hardware::CommunicationStatsInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);
    virtual void stopping(const ros::Time& /*time*/);

    // std::vector< std::string >                        joint_names_;
    // std::vector< hardware_interface::JointHandle >    joints_;
    // realtime_tools::RealtimeBuffer<double>            commands_buffer_;
    // unsigned int                                      n_joints_;

private:

    std::vector<mh5_hardware::CommunicationStatsHandle> communication_states_;
    std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> > realtime_pub_;
    ros::Time last_publish_time_;
    double publish_period_;

};

} // namespace