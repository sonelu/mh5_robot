#include <dynamixel_sdk/dynamixel_sdk.h>
#include "mh5_hardware/communication_stats_interface.hpp"
#include "mh5_hardware/port_handler.hpp"

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

    void incPackets()   { packets_++; tot_packets_++;}
    void incErrors()    { errors_++; tot_errors_++;}
    void reset()        { packets_ = 0; errors_ = 0;}
    void resetAll()     { reset(); tot_packets_ = 0; tot_errors_ = 0;}

    const CommStatsHandle& getCommStatHandle() { return comm_stats_handle_;}

private:

    long    packets_;
    long    errors_;
    long    tot_packets_;
    long    tot_errors_;


    const CommStatsHandle comm_stats_handle_;

};


class GroupSyncRead : public dynamixel::GroupSyncRead, CommunicationStats
{
public:

    GroupSyncRead(std::string& name, dynamixel::PortHandler *port, dynamixel::PacketHandler *ph, uint16_t start_address, uint16_t data_length)
    : dynamixel::GroupSyncRead(port, ph, start_address, data_length),
      CommunicationStats(name) {}



};

} //namespace