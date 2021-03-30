#include <hardware_interface/internal/hardware_resource_manager.h>

#pragma once

namespace mh5_hardware
{

class CommunicationStatsHandle
{
public:

    CommunicationStatsHandle() = default;

    CommunicationStatsHandle(const std::string& name, const long *packets, const long* errors, const long* tot_packets, const long *tot_errors, bool *reset)
    : name_(name), packets_(packets), errors_(errors), tot_packets_(tot_packets), tot_errors_(tot_errors), reset_(reset)
    {
        if (!packets)
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Packets data pointer is null.");

        if (!errors)
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Errors data pointer is null.");

        if (!tot_packets)
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Total packets data pointer is null.");

        if (!tot_errors)
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Total errors data pointer is null.");

        if (!reset)
            throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name + "'. Reset data pointer is null.");
    }

    std::string getName()           const { return name_;}
    long getPackets()               const { assert(packets_); return *packets_;}
    long getErrors()                const { assert(errors_); return *errors_;}
    long getTotPackets()            const { assert(tot_packets_); return *tot_packets_;}
    long getTotErrors()             const { assert(tot_errors_); return *tot_errors_;}

    const long* getPacketsPtr()     const {return packets_;}
    const long* getErrorsPtr()      const { return errors_;}
    const long* getTotPacketsPtr()  const { return tot_packets_;}
    const long* getTotErrorsPtr()   const { return tot_errors_;}

    void setReset(bool reset)       { assert(reset_); *reset_ = reset;}

private:

    std::string  name_;
    const long*  packets_     = {nullptr};
    const long*  errors_      = {nullptr};
    const long*  tot_packets_ = {nullptr};
    const long*  tot_errors_  = {nullptr};
    bool*        reset_       = {nullptr};

};

class CommunicationStatsInterface : public hardware_interface::HardwareResourceManager<CommunicationStatsHandle> {};

} // namespace