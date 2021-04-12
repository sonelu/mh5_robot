#include <dynamixel_sdk/dynamixel_sdk.h>
#include "mh5_hardware/communication_stats_interface.hpp"
#include "mh5_hardware/port_handler.hpp"
#include "dynamixel_joint.hpp"

#pragma once 

namespace mh5_hardware
{

/**
 * @brief Class that wrapps around a Dynaxmiel GroupSync process and can be executed
 * with a given frequency. It also keeps tabs on the communication statistics:
 * total (since the start of the node) number of Dynamixel packs executed, total 
 * number of errors encountered, as well as a shorter timeframe count of packets
 * and errors that can be reset and can be used to report "recent" statistics.
 * 
 * The class can produce a CommunicationStatsHandle for the registering with a
 * controller that can publish these statistics.
 */
class LoopWithCommunicationStats
{
public:
    /**
     * @brief Construct a new Communication Stats object.
     * 
     * Initializes the communication statistics to 0 and the last_execution_time_ 
     * to the current time.
     * 
     * @param name will be the name used for the loop when registering with the
     * resource manager
     * @param loop_rate the rate (in Hz) that the loop should execute. The Execute()
     * method checks if enough time has passed since last run, otherwise it will
     * not be executed. This permits the loop to be configured to run on a much lower
     * rate than the owner loop. 
     */
    LoopWithCommunicationStats(const std::string& name, double loop_rate)
    : tot_packets_(0), tot_errors_(0), packets_(0), errors_(0), reset_(false),
      loop_rate_(loop_rate), last_execution_time_(ros::Time::now()),
      comm_stats_handle_(name, &packets_, &errors_, &tot_packets_, &tot_errors_, &reset_) {}

    /**
     * @brief Destroy the Communication Stats object.
     */
    ~LoopWithCommunicationStats() {}

    /**
     * @brief Returns the name of the loop. Used for message genration.
     * 
     * @return const std::string the name of the loop.
     */
    const std::string getName() { return comm_stats_handle_.getName();}

    /**
     * @brief Resets the recent statistics. Only the packets_ and errors_ are
     * reset to 0, the total_packets_ and total_errors_ (that keep the cummulative
     * packets since the start of the node) are not affected.
     */
    void resetStats()        { packets_ = 0; errors_ = 0; }

    /**
     * @brief Resets all statistics, including the totals.
     */
    void resetAllStats()     { resetStats(); tot_packets_ = 0; tot_errors_ = 0;}

    /**
     * @brief Returns a ``ros_control`` resource Handle to the communication 
     * statistics. Intendent to be called by the main hardware interface in
     * order to register the loop statistics as a resource with a controller
     * that will publish this statistics.
     * 
     * @return const CommunicationStatsHandle& a ``ros_control`` resource handle
     */
    const CommunicationStatsHandle& getCommStatHandle() { return comm_stats_handle_;}

    /**
     * @brief Prepare the loop (if necessary) based on the specifics of the loop
     * and the joint information. This should be called only once by the owner of
     * the loop, imidiately after the constructor. The method needs to be
     * implemented in the subclass to perform (or just return a true) whatever
     * is needed for that type of loop.
     * 
     * @param joints an array of joints that might be needed in the preparation step
     * @return true if the activity was successful
     * @return false if there was an error performing the activity
     */
    virtual bool prepare(std::vector<Joint *> joints) = 0 ;

    /**
     * @brief This is an activity that needs to be performed each time in the
     * loop just before the communication. This allows the particular implementation
     * of the loop to do activities required before the actual communication.
     * 
     * @param joints an array of joints that might be needed in this step
     * @return true if the activity was successful
     * @return false if there was an error performing the activity
     */
    virtual bool beforeCommunication(std::vector<Joint *> joints) = 0;

    /**
     * @brief Wraps the actual communication steps so that it takes into account
     * the requested processing rate and keeps track of the communication
     * statistics. If the call to Execute() is too early
     * (no enough time has passed since last run to account for the execution
     * rate) the method will simply return true.
     * 
     * If enough time has passed, the method checks first if there was a request
     * to reset the statistics then it will call resetStats(). It will then call:
     * beforeCommunication() and if this is not successfule it will stop and
     * return false. If the step above is successful it will increment the
     * packets statistics and then call Communicate() and check again the
     * result. If this is not successfull it will increment the number of
     * errors and return false. If the communication was successfull it will call
     * afterCommunication() and return the result of that processing.
     * 
     * @param time time to execute the method (typically close to now)
     * @param period the time passed since the last call to this method
     * @param joints an array of joints that need to be processed
     * @return true if the processing (including the call to Communicate() ) was
     * successfull
     * @return false the call to Communicate() was unsuccessfull
     */
    bool Execute(const ros::Time& time, const ros::Duration& period, std::vector<Joint *> joints)
    {
        if (loop_rate_ > 0.0 && last_execution_time_ + ros::Duration(1.0/loop_rate_) < time)
        {
            last_execution_time_ += ros::Duration(1.0/loop_rate_);
        
            if (reset_) {                   // was requested to reset statistics
                resetStats();
                reset_ = false;
            }
        
            if(!beforeCommunication(joints))
                return false;
            
            incPackets();
            bool result = Communicate();
            if (!result) {
                incErrors();
                return false;
            }
            return afterCommunication(joints);
        }
        return true;
    }

    /**
     * @brief Virtual method that needs to be impplemented by the subclasses
     * depending on the actual work the loop is doing (reading or writing).
     * 
     * @return true the communication was successfull
     * @return false the communication was not successfull
     */
    virtual bool Communicate() = 0;

    /**
     * @brief This is an activity that needs to be performed each time in the
     * loop just after the communication. This allows the particular implementation
     * of the loop to do activities required after the actual communication
     * (ex. for an read loop to retrieve the data from the response package and
     * store it in the joints attributes).
     * 
     * @param joints an array of joints that might be needed in this step
     * @return true if the activity was successful
     * @return false if there was an error performing the activity
     */
    virtual bool afterCommunication(std::vector<Joint *> joints) = 0;

protected:

    /**
     * @brief Keeps the desired execution rate (in Hz) the for loop
     */
    double      loop_rate_;

    /**
     * @brief Stores the last time the loop was executed.
     */
    ros::Time   last_execution_time_;

    /**
     * @brief Number of packets transmited since last reset
     */
    long        packets_;

    /**
     * @brief Number of errors encountered since last reset
     */
    long        errors_;

    /**
     * @brief Total number of packets transmitted since the start of node
     */
    long        tot_packets_;

    /**
     * @brief Total number of errors encountered since the start of node
     */
    long        tot_errors_;

    /**
     * @brief Keeps asyncronously the requests (from the controllers) to
     * reset the statistics. The Execute() method will check this and if
     * set to true it will reset the statistics.
     */
    bool        reset_;

    /**
     * @brief Convenience method to increment the number of packets and
     * total packets.
     */
    void incPackets()   { packets_++; tot_packets_++;}

    /**
     * @brief Convenience method to increment the number of errors and
     * total total.
     */
    void incErrors()    { errors_++; tot_errors_++;}

    /**
     * @brief A ``ros_control`` resource type handle for passing to the
     * resource manager and to be used by the controller that publishes the
     * statistics.
     */
    const CommunicationStatsHandle comm_stats_handle_;

};


/**
 * @brief A specialization of the loop using a Dynamixel GroupSyncRead. Intended for
 * reading data from a group of dynamixels.
 * 
 * This specialization needs a start address and a data length that the loop will
 * handle, implements the prepare() method that calls addParam() for all IDs of
 * joints that are marked as "present" and provides a specific implementation of
 * the Communicate() method.
 * 
 */
class GroupSyncRead : public dynamixel::GroupSyncRead, public LoopWithCommunicationStats
{
public:

    /**
     * @brief Construct a new GroupSyncRead object which is an extension on a
     * standard dynamixel GroupSyncRead.
     * 
     * @param name the name of the loop; used for messages and for registering resources
     * @param loop_rate the rate the loop will be expected to run
     * @param port the dynamixel::PortHandler needed for the communication
     * @param ph the dynamixel::PacketHandler needed for communication
     * @param start_address the start addres for reading the data for all servos
     * @param data_length the length of the data to be read
     */
    GroupSyncRead(const std::string& name, double loop_rate, dynamixel::PortHandler *port, dynamixel::PacketHandler *ph, uint16_t start_address, uint16_t data_length)
    : dynamixel::GroupSyncRead(port, ph, start_address, data_length),
      LoopWithCommunicationStats(name, loop_rate) {}

    /**
     * @brief Adds all the joints that are marked "present" to the processing loop
     * by invoking the addParam() methods of the dynamixel::GroupSyncRead. If there are 
     * errors there will be a warning printed.
     * 
     * @param joints a vector of joints to used in the loop
     * @return true if at least one joint has been added to the loop
     * @return false if no joints has been suucessfully added to the loop
     */
    bool prepare(std::vector<Joint *> joints) override;

    /**
     * @brief Simply returns true. SyncReads do not need any additional preparation
     * before the communication.
     * 
     * @param joints an array of joints that might be needed in this step
     * @return true always
     */
    bool beforeCommunication(std::vector<Joint *> joints) override {return true;}

    /**
     * @brief Particular implementation of the communication, specific to the
     * GroupSyncRead. Calls txrxPacket() of dynamixel::GroupSyncRead and
     * checks the communication result.
     * 
     * @return true if the communication was successful
     * @return false if there was a communication error
     */
    bool Communicate() override;
};


/**
 * @brief A specialization of the loop using a Dynamixel GroupSyncWrite. Intended for
 * writing data to a group of dynamixels.
 * 
 * This specialization needs a start address and a data length that the loop will
 * handle, implements the beforeExecute() method that calls addParam() for all IDs of
 * joints that are marked as "present" and provides a specific implementation of
 * the Communicate() method.
 * 
 */
class GroupSyncWrite : public dynamixel::GroupSyncWrite, public LoopWithCommunicationStats
{
public:

    GroupSyncWrite(const std::string& name, double loop_rate, dynamixel::PortHandler* port, dynamixel::PacketHandler* ph, uint16_t start_address, uint16_t data_length)
    : dynamixel::GroupSyncWrite(port, ph, start_address, data_length),
      LoopWithCommunicationStats(name, loop_rate) {}


    /**
     * @brief Simply returns true. SyncWrites need to pre-prepare data foar each
     * execution and this is implemented in beforeExecute().
     * 
     * @param joints an array of joints that might be needed in this step
     * @return true always
     */
    bool prepare(std::vector<Joint *> joints) { return true; }


    /**
     * @brief Simply returns true. SyncWrites do not need any activities
     * after communication.
     * 
     * @param joints an array of joints that might be needed in this step
     * @return true always
     */
    bool afterCommunication(std::vector<Joint *> joints) { return true; }


    /**
     * @brief Particular implementation of the communication, specific to the
     * GroupSyncWrite. Calls txPacket() of dynamixel::GroupSyncWrite and
     * checks the communication result.
     * 
     * @return true if the communication was successful
     * @return false if there was a communication error
     */
    bool Communicate() override;
};


/**
 * @brief Specialization of the GroupSyncRead to perform the read of the following
 * registers for XL430 Dynamixel series: present position, present velocity,
 * present load (hence the name PVL).
 */
class PVLReader : public GroupSyncRead
{
public:
    /**
     * @brief Construct a new PVLReader object. Uses 126 as the start of the address
     * and 10 as the data_lenght
     * 
     * @param name the name of the loop; used for messages and for registering resources
     * @param loop_rate the rate the loop will be expected to run
     * @param port the dynamixel::PortHandler needed for the communication
     * @param ph the dynamixel::PacketHandler needed for communication 
     */
    PVLReader(const std::string& name, double loop_rate, dynamixel::PortHandler *port, dynamixel::PacketHandler *ph)
    : GroupSyncRead(name, loop_rate, port, ph, 126, 10) {}

    /**
     * @brief Postprocessing of data after communication, specific to the position,
     * velocity and load registers. Unpacks the data from the returned response
     * and calls the joints' setPositionFromRaw(), setVelocityFromRaw(),
     * setEffortFromRaw() to update them. If there are errors there will be
     * ROS_DEBUG messages issued but the processing will not be stopped.
     * 
     * @param joints 
     * @return true 
     * @return false 
     */
    bool afterCommunication(std::vector<Joint *> joints) override;
};


/**
 * @brief Specialization of the GroupSyncRead to perform the read of the following
 * registers for XL430 Dynamixel series: present temperature, present voltage,
 * (hence the name TV).
 */
class TVReader : public GroupSyncRead
{
public:
    /**
     * @brief Construct a new TVReader object. Uses 144 as the start of the address
     * and 3 as the data_lenght
     * 
     * @param name the name of the loop; used for messages and for registering resources
     * @param loop_rate the rate the loop will be expected to run
     * @param port the dynamixel::PortHandler needed for the communication
     * @param ph the dynamixel::PacketHandler needed for communication 
     */
    TVReader(const std::string& name, double loop_rate, dynamixel::PortHandler *port, dynamixel::PacketHandler *ph)
    : GroupSyncRead(name, loop_rate, port, ph, 144, 3) {}

    /**
     * @brief Postprocessing of data after communication, specific to the temperature,
     * and voltage registers. Unpacks the data from the returned response
     * and calls the joints' setTemperatureFromRaw(), setVoltageFromRaw(),
     * to update them. If there are errors there will be
     * ROS_DEBUG messages issued but the processing will not be stopped.
     * 
     * @param joints 
     * @return true always
     */
    bool afterCommunication(std::vector<Joint *> joints) override;
};


/**
 * @brief Specialization of the GroupSyncWrite to perform the write of the following
 * registers for XL430 Dynamixel series: goal position, goal velocity (profile),
 * (hence the name PVWriter). The Joint object handles the conversion of commands
 * (position, velocity) into (position, velocity profile) needed to control
 * dynamixel XL430s in velocity profile mode.
 */
class PVWriter : public GroupSyncWrite
{
public:
    /**
     * @brief Initializes the writer object with start address 108 and 12 bytes of
     * information to be written (4 for position, 4 for velocity profile and
     * 4 for acceleration profile)
     * 
     * @param name the name of the loop
     * @param loop_rate the rate to be executed
     * @param port the Dynamixel port handle to be used for communication
     * @param ph the Dynamixel protocol handle to be used for communication
     */
    PVWriter(const std::string& name, double loop_rate, dynamixel::PortHandler *port, dynamixel::PacketHandler *ph)
    : GroupSyncWrite(name, loop_rate, port, ph, 108, 12) {}

    /**
     * @brief For each joint retrieves the desired position and velocity profile
     * (determined internally by the Joint class from the velocity command) and
     * prepares a data buffer with the 12 bytes needed to update the goal position
     * (reg 116), velocity profile (reg. 112) and acceleration profile (reg. 108).
     * Acceleration profile is hard-coded to 1/4 of the velocity profile. Only 
     * joints that are "present" are taken into account.
     * 
     * @param joints vector of joints for processing
     * @return true if there is at least one joint that has been added to the loop
     * @return false if no joints were added to the loop
     */
    bool beforeCommunication(std::vector<Joint *> joints) override;
};


/**
 * @brief Specialization of the GroupSyncWrite to perform the write of the torque
 * register for XL430 Dynamixel series. It will only syncronize devices
 * that have changed (ex. the shouldToggleTorque() returns ``true``).
 */
class TWriter : public GroupSyncWrite
{
public:
    TWriter(const std::string& name, double loop_rate, dynamixel::PortHandler *port, dynamixel::PacketHandler *ph)
    : GroupSyncWrite(name, loop_rate, port, ph, 64, 1) {}

    /**
     * @brief For each joint checks if the active state has changed (the controller
     * call to setCommand() would have activated the field active_command_flag_ that
     * can be checked by calling shouldToggleTorque()). Also, only servos that are
     * "present" are included.
     * 
     * @param joints vector of joints for processing
     * @return true if there is at least one joint that has been added to the loop
     * @return false if no joints were added to the loop
     */
    bool beforeCommunication(std::vector<Joint *> joints) override;
};

} //namespace