#include <diagnostic_msgs/DiagnosticArray.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_interface/controller.h>
#include <mh5_hardware/communication_stats_interface.hpp>

#pragma once

namespace mh5_controllers
{

/**
 * @brief Publishes communication ststistics for all the Dynamixel loops registered in the
 * hardware interface. Requires mh5_hardware::CommunicationStatsInterface to access
 * the statistics for all loops. If combined HW interface is used please note that this
 * will get all the loops, across all the physical HW interfaces that the combined
 * HW interface will start.
 * 
 * The messages are publish as diagnostic_msgs::DiagnosticArray under topic "diagnostics".
 * Aggregators can be used to process thsese raw diagnostic messages and publish
 * them to a RobotMonitor.
 */

class CommunicationStatsController : public controller_interface::Controller<mh5_hardware::CommunicationStatsInterface>
{
public:
    /**
     * @brief Construct a new Communication Stats Controller object; defaults the
     * publish period to 0.0.
     */
    CommunicationStatsController() : publish_period_(0.0) {}

    /**
     * @brief Initializes the controller. Reads the parameter server 
     * "publish_period" [expressed in seconds] and uses it for sheduling
     * the publishing of the communication information. It defaults to 30s
     * if no value is avaialable. Please note that the publishing period
     * is also used to reset the short time communication statistics that
     * are provided by the mh5_hardware::CommunicationStatsInterface.
     * 
     * It will setup the realtime publisher and allocate the message structure
     * to accomodate the data from the CommunicationStatsInterface.
     * 
     * @param hw the hardware providing the loops; could be a Combined HW Interface
     * @param root_nh the top Node Handler
     * @param controller_nh the node handler of the controller; used to access
     * the parameter server
     * @return true if controller was initialized sucessfully
     */
    bool init(mh5_hardware::CommunicationStatsInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

    /**
     * @brief Resets the last_publish_time_ to the provided time.
     * 
     * @param time when the controller was started
     */
    void starting(const ros::Time& time);

    /**
     * @brief Performs the actual publishing of statistics by accesing the inteface 
     * data. It will check the last time the message was published and does not
     * do any publish if it is less than publish_period_ desired for these
     * message publishing.
     * 
     * Please note that after the massage is published it invokes the setReset(true)
     * for the CommunicationStatsInterface to reset to 0 the short-term statistics.
     */
    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

    /**
     * @brief Provided for completion of the controller interface.
     */
    virtual void stopping(const ros::Time& /*time*/);

private:

    /**
     * @brief Holds the list of handles to all the loops across all the HW interfaces
     */
    std::vector<mh5_hardware::CommunicationStatsHandle> communication_states_;

    /**
     * @brief Publisher object
     */
    std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray> > realtime_pub_;

    /**
     * @brief Keeps the last publish time. Updated every time we publish a new message
     */
    ros::Time last_publish_time_;

    /**
     * @brief The desired publishing period in seconds for the diagnostoc messages
     */
    double publish_period_;

};

} // namespace