#include <dynamixel_sdk/dynamixel_sdk.h>
#include "mh5_hardware/communication_stats_interface.hpp"
#include "mh5_hardware/port_handler.hpp"
#include "dynamixel_joint.hpp"

#pragma once 

namespace mh5_hardware
{


class CommunicationStats
{
public:

    CommunicationStats(const std::string& name, double loop_rate)
    : tot_packets_(0), tot_errors_(0), packets_(0), errors_(0), reset_(false),
      loop_rate_(loop_rate), last_execution_time_(ros::Time::now()),
      comm_stats_handle_(name, &packets_, &errors_, &tot_packets_, &tot_errors_, &reset_) {}

    ~CommunicationStats() {}

    const std::string getName() { return comm_stats_handle_.getName();}

    void resetStats()        { packets_ = 0; errors_ = 0; }
    void resetAllStats()     { resetStats(); tot_packets_ = 0; tot_errors_ = 0;}

    const CommStatsHandle& getCommStatHandle() { return comm_stats_handle_;}

    /* prepares the loop based on the joints provided */
    virtual bool prepare(std::vector<Joint>& joints) = 0 ;

    /* processing before performing the communication */
    virtual bool beforeExecute(std::vector<Joint>& joints) = 0;

    /* keeps track of communication while executing it */
    bool Execute(const ros::Time& time, const ros::Duration& period) {
        if (loop_rate_ > 0.0 && last_execution_time_ + ros::Duration(1.0/loop_rate_) < time) {
            last_execution_time_ += ros::Duration(1.0/loop_rate_);
            if (reset_) {                   // was requested to reset statistics
                resetStats();
                reset_ = false;
            }
            incPackets();
            bool result = Communicate();
            if (!result)
                incErrors();
            return result;
        }
        return true;
    }

    /* particular communication for this loop */
    virtual bool Communicate() = 0;

    /* postprocessing after communication */
    virtual bool afterExecute(std::vector<Joint>& joints) = 0;

protected:

    double      loop_rate_;
    ros::Time   last_execution_time_;

    long        packets_;
    long        errors_;
    long        tot_packets_;
    long        tot_errors_;
    bool        reset_;

    void incPackets()   { packets_++; tot_packets_++;}
    void incErrors()    { errors_++; tot_errors_++;}

    const CommStatsHandle comm_stats_handle_;

};



class GroupSyncRead : public dynamixel::GroupSyncRead, public CommunicationStats
{
public:

    GroupSyncRead(const std::string& name, double loop_rate, dynamixel::PortHandler *port, dynamixel::PacketHandler *ph, uint16_t start_address, uint16_t data_length)
    : dynamixel::GroupSyncRead(port, ph, start_address, data_length),
      CommunicationStats(name, loop_rate) {}

    bool prepare(std::vector<Joint>& joints) override;

    // SyncRead does not need any preparation before communication
    bool beforeExecute(std::vector<Joint>& joints) override {return true;}

    bool Communicate() override;
};


class PVLReader : public GroupSyncRead
{
public:
    PVLReader(const std::string& name, double loop_rate, dynamixel::PortHandler *port, dynamixel::PacketHandler *ph)
    : GroupSyncRead(name, loop_rate, port, ph, 126, 10) {}

    bool afterExecute(std::vector<Joint>& joints) override;
};

} //namespace