#include <pluginlib/class_list_macros.hpp>
//#include <diagnostic_msgs/KeyValue.h>
#include "mh5_controllers/communication_stats_controller.hpp"

using namespace mh5_controllers;


bool CommunicationStatsController::init(mh5_hardware::CommunicationStatsInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    // get all joint names from the hardware interface
    const std::vector<std::string>& loop_names = hw->getNames();

    // get publishing period
    if (!controller_nh.getParam("publish_period", publish_period_)){
      ROS_INFO("[CommunicationStatusController] Parameter 'publish_period' not set; default to 30s");
      publish_period_ = 30.0;
    }

    // realtime publisher
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray>(root_nh, "diagnostics", 4));

    // get joints and allocate message
    for (auto & loop_name : loop_names)
    {
        communication_states_.push_back(hw->getHandle(loop_name));

        diagnostic_msgs::DiagnosticStatus* s = new diagnostic_msgs::DiagnosticStatus();
        s->name = loop_name;
        // s.level = ...
        // s.message = ...
        // s.hardware_id = ...
        s->values.resize(7);
        s->values[0].key = "packets";
        s->values[1].key = "errors";
        s->values[2].key = "error_rate_perc";
        s->values[3].key = "total_packets";
        s->values[4].key = "total_errors";
        s->values[5].key = "total_error_rate_perc";
        s->values[6].key = "real_rate";

        realtime_pub_->msg_.status.push_back(*s);
    }

    return true;
  }


void CommunicationStatsController::starting(const ros::Time& time)
  {
    // initialize time
    last_publish_time_ = time;
  }


void CommunicationStatsController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
    // limit rate of publishing
    if (publish_period_ > 0.0 && last_publish_time_ + ros::Duration(publish_period_) < time)
    {
        // try to publish
        if (realtime_pub_->trylock())
        {
            // we're actually publishing, so increment time
            // last_publish_time_ = last_publish_time_ + ros::Duration(publish_period_);
            last_publish_time_ = time;

            // populate joint state message:
            // - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
            // - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
            realtime_pub_->msg_.header.stamp = time;
            
            for (unsigned i=0; i<communication_states_.size()   ; i++) {
                mh5_hardware::CommunicationStatsHandle& state = communication_states_[i];
                diagnostic_msgs::DiagnosticStatus &status = realtime_pub_->msg_.status[i];
            
                status.values[0].value = std::to_string(state.getPackets());
                status.values[1].value = std::to_string(state.getErrors());
                status.values[2].value = std::to_string(100.0 * state.getErrors() / state.getPackets());

                status.values[3].value = std::to_string(state.getTotPackets());
                status.values[4].value = std::to_string(state.getTotErrors());
                status.values[5].value = std::to_string(100.0 * state.getTotErrors() / state.getTotPackets());

                status.values[6].value = std::to_string((double)state.getPackets() / publish_period_);
            }
            realtime_pub_->unlockAndPublish();

            for (auto & state : communication_states_) {
                state.setReset(true);
            }
        } // lock
    } // publish_period
}

void CommunicationStatsController::stopping(const ros::Time& /*time*/) {}

PLUGINLIB_EXPORT_CLASS( mh5_controllers::CommunicationStatsController, controller_interface::ControllerBase)