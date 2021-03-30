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

    CommunicationStats(const std::string& name)
    : tot_packets_(0), tot_errors_(0), packets_(0), errors_(0),
      comm_stats_handle_(name, &packets_, &errors_, &tot_packets_, &tot_errors_) {}

    ~CommunicationStats() {}

    const std::string getName() { return comm_stats_handle_.getName();}

    void resetStats()        { packets_ = 0; errors_ = 0;}
    void resetAllStats()     { resetStats(); tot_packets_ = 0; tot_errors_ = 0;}

    const CommStatsHandle& getCommStatHandle() { return comm_stats_handle_;}

    /* prepares the loop based on the joints provided */
    virtual bool prepare(std::vector<Joint>& joints) = 0 ;

    /* processing before performing the communication */
    virtual bool beforeExecute(std::vector<Joint>& joints) = 0;

    /* keeps track of communication while executing it */
    bool Execute() {
        incPackets();
        bool result = Communicate();
        if (!result)
            incErrors();
        return result;
    }

    /* particular communication for this loop */
    virtual bool Communicate() = 0;

    /* postprocessing after communication */
    virtual bool afterExecute(std::vector<Joint>& joints) = 0;


protected:

    long    packets_;
    long    errors_;
    long    tot_packets_;
    long    tot_errors_;

    void incPackets()   { packets_++; tot_packets_++;}
    void incErrors()    { errors_++; tot_errors_++;}

    const CommStatsHandle comm_stats_handle_;

};



class GroupSyncRead : public dynamixel::GroupSyncRead, public CommunicationStats
{
public:

    GroupSyncRead(std::string& name, dynamixel::PortHandler *port, dynamixel::PacketHandler *ph, uint16_t start_address, uint16_t data_length)
    : dynamixel::GroupSyncRead(port, ph, start_address, data_length),
      CommunicationStats(name) {}

    bool prepare(std::vector<Joint>& joints) override;

    // SyncRead does not need any preparation before communication
    bool beforeExecute(std::vector<Joint>& joints) override {return true;}

    bool Communicate() override;
};


class PVLReader : public GroupSyncRead
{
public:
    PVLReader(std::string& name, dynamixel::PortHandler *port, dynamixel::PacketHandler *ph)
    : GroupSyncRead(name, port, ph, 126, 10) {}

    bool afterExecute(std::vector<Joint>& joints) override;
};

} //namespace