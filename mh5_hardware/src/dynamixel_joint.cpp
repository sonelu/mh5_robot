#include "mh5_hardware/dynamixel_joint.hpp"


using namespace mh5_hardware;

#define TRIES 5         // retry for cdynamixel communication

void Joint::fromParam(ros::NodeHandle& nh, std::string& name, mh5_port_handler::PortHandlerMH5* port, dynamixel::PacketHandler* ph )
{
    name_ = name;
    port_ = port;
    ph_ = ph;
    nh_ = nh;
    nss_ = nh.getNamespace().c_str();

    int servo_id;                    // stores from param server
    if (!nh.getParam(name + "/id", servo_id)) {
        ROS_ERROR("[%s] ID not found for joint %s; will be disabled", nss_, name_.c_str());
        present_ = false;
    }
    else {
        id_ = (uint8_t)servo_id;
        present_ = true;
        inverse_ = nh.param<bool>(name + "/inverse", false);
        offset_ = nh.param<double>(name + "/offset", 0);
    }

    // setup hardware handles
    jointStateHandle_ = hardware_interface::JointStateHandle(name_, &position_state_, &velocity_state_, &effort_state_);
    jointPosVelHandle_ = hardware_interface::PosVelJointHandle(jointStateHandle_, &position_command_, &velocity_command_);
    jointActiveHandle_ = mh5_hardware::JointTorqueAndReboot (jointStateHandle_, &active_command_, &active_command_flag_, &reboot_command_flag_);
}


bool Joint::ping(const int num_tries )
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool servo_ok = false;
    
    for (int n=0; n < num_tries; n++)
    {
        dxl_comm_result = ph_->ping(port_, id_, &dxl_error);
        
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("[%s] failed to communicate with joint %s [%d] (try %d/%d): %s", 
                      nss_, name_.c_str(), id_, n + 1, num_tries, ph_->getTxRxResult(dxl_comm_result));
            continue;
        }
        
        if (dxl_error != 0) {
            ROS_ERROR("[%s] error reported when communicating with joint %s [%d] (try %d/%d): %s", 
                      nss_, name_.c_str(), id_, n + 1, num_tries, ph_->getRxPacketError(dxl_error));
            continue;
        }
        
        ROS_INFO("[%s] joint %s [%d] detected", nss_, name_.c_str(), id_);
        servo_ok = true;
        break;
    }

    return servo_ok;
}


bool Joint::readRegister(const uint16_t address, const int size, long& value, const int num_tries)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;
    long buff = 0;                                     // buffer for reading value

    // we'll make 5 attempt in case there are communication errors
    for (int n=0; n < num_tries; n++)
    {
        switch(size) {
            case 1:
                dxl_comm_result = ph_->read1ByteTxRx(port_, id_, address, (uint8_t *)&buff, &dxl_error);
                break;
            case 2:
                dxl_comm_result = ph_->read2ByteTxRx(port_, id_, address, (uint16_t*) &buff, &dxl_error);
                break;
            case 4:
                dxl_comm_result = ph_->read4ByteTxRx(port_, id_, address, (uint32_t*) &buff, &dxl_error);
                break;
            default: {
                ROS_ERROR("[%s] Incorrect 'writeRegister' call with size %d", nss_, size);
                return false;
            }
        }

        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("[%s] readRegister communication failure (%s) for servo %s [%d], register %d (try %d/%d)",
                      nss_, ph_->getTxRxResult(dxl_comm_result), name_.c_str(), id_, address, n+1, num_tries);
            continue;
        }

        if (dxl_error != 0) {
            ROS_ERROR("[%s] readRegister packet error (%s) for servo %s [%d], register %d (try %d/%d)",
                      nss_, ph_->getRxPacketError(dxl_error), name_.c_str(), id_, address, n+1, num_tries);
            continue;
        }

        result = true;
        value = buff;
    }

    return result;
}


bool Joint::writeRegister(const uint16_t address, const int size, const long value, const int num_tries)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    bool result = false;

    // we'll make 5 attempt in case there are communication errors
    for (int n=0; n < num_tries; n++)
    {
        switch(size) {
            case 1:
                dxl_comm_result = ph_->write1ByteTxRx(port_, id_, address, (uint8_t) value, &dxl_error);
                break;
            case 2:
                dxl_comm_result = ph_->write2ByteTxRx(port_, id_, address, (uint16_t) value, &dxl_error);
                break;
            case 4:
                dxl_comm_result = ph_->write4ByteTxRx(port_, id_, address, (uint32_t) value, &dxl_error);
                break;
            default: {
                ROS_ERROR("[%s] incorrect 'writeRegister' call with size %d", nss_, size);
                return false;
            }
        }

        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_ERROR("[%s] writeRegister communication failure (%s) for servo %s [%d], register %d (try %d/%d)",
                      nss_, ph_->getTxRxResult(dxl_comm_result), name_.c_str(), id_, address, n+1, num_tries);
            continue;
        }

        if (dxl_error != 0) {
            ROS_ERROR("[%s] writeRegister packet error (%s) for servo %s [%d], register %d (try %d/%d)",
                      nss_, ph_->getRxPacketError(dxl_error), name_.c_str(), id_, address, n+1, num_tries);
            continue;
        }

        result = true;
        break;
    }

    return result;
}


bool Joint::reboot(const int num_tries)
{
    for (int n=0; n < num_tries; n++)
    {
        int dxl_comm_result = COMM_TX_FAIL;             // Communication result
        uint8_t dxl_error = 0;                          // Dynamixel error

        dxl_comm_result = ph_->reboot(port_, id_, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            // ROS_ERROR("Failed to reset device %s [%d]: %s", 
            //         name_.c_str(), id_, ph_->getTxRxResult(dxl_comm_result));
            continue;
        }
        else if (dxl_error != 0) {
            // ROS_ERROR("Failed to reset device %s [%d]: %s", 
            //         name_.c_str(), id_, ph_->getRxPacketError(dxl_error));
            continue;
        }
        else {
            ROS_INFO("Successful rebooted device %s [%d]",  name_.c_str(), id_);
            return true;
        }
    }
    ROS_ERROR("Failed to reset device %s [%d]: %s", name_.c_str(), id_);
    return false;
}


bool Joint::isActive(bool refresh)
{
    if (refresh) {
        long value;
        if(!readRegister(64, 1, value, TRIES)) {
            ROS_ERROR("[%s] failed to read torque status for %s [%d]", nss_, name_.c_str(), id_);
        }
        else {
            active_state_ = (double)value;
        }
    }
    return (active_state_ != 0);
}


bool Joint::torqueOn()
{
    return writeRegister(64, 1, 1, TRIES);
}


bool Joint::torqueOff()
{
    return writeRegister(64, 1, 0, TRIES);
}


bool Joint::toggleTorque()
{
    if(writeRegister(64, 1, (long)active_command_, TRIES)) {
        active_state_ = active_command_;
        return true;
    }
    else
        return false;
}


void Joint::initRegisters()
{
    writeRegister(9, 1, 0, TRIES);       // return delay
    writeRegister(11, 1, 3, TRIES);      // operating mode
    writeRegister(31, 1, 75, TRIES);     // temperature limit
    writeRegister(32, 2, 135, TRIES);    // max voltage
    writeRegister(44, 4, 1023, TRIES);   // velocity limit
    writeRegister(48, 4, 4095, TRIES);   // max position
    writeRegister(52, 4, 0, TRIES);      // min position

    // direction
    if (inverse_)
        writeRegister(10, 1, 5, TRIES);  // inverse; time profile
    else
        writeRegister(10, 1, 4, TRIES);  // direct; time profile

    // PID and FF
    writeRegister(80, 2, 4000, TRIES);      // Position D Gain
    writeRegister(82, 2, 0, TRIES);         // Position I Gain
    writeRegister(84, 2, 1280, TRIES);       // Position P Gain
    writeRegister(88, 2, 0, TRIES);         // FF 2nd Gain
    writeRegister(90, 2, 0, TRIES);         // FF 1st Gain

    // initilizes the active members to avoid issues later when the syncs start
    active_command_ = 0.0;
    active_state_ = 0.0;
    active_command_flag_ = false;
    reboot_command_flag_ = false;
}