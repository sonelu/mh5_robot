
#include <pluginlib/class_list_macros.hpp>

#include "mh5_hardware_control/dynamixel_interface.hpp"
#include "mh5_hardware_control/active_joint_interface.hpp"

using namespace mh5_hardware_interface;

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
    jointActiveHandle_ = hardware_interface::JointHandle (jointStateHandle_, &active_command_);
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
            ROS_ERROR("[%s] readRegister communication failure for servo %s [%d], register %d (try %d/%d)",
                      nss_, name_.c_str(), id_, address, n, num_tries);
            continue;
        }

        if (dxl_error != 0) {
            ROS_ERROR("[%s] readRegister packet error for servo %s [%d], register %d (try %d/%d)",
                      nss_, name_.c_str(), id_, address, n, num_tries);
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
            ROS_ERROR("[%s] writeRegister communication failure for servo %s [%d], register %d (try %d/%d)",
                      nss_, name_.c_str(), id_, address, n, num_tries);
            continue;
        }

        if (dxl_error != 0) {
            ROS_ERROR("[%s] writeRegister packet error for servo %s [%d], register %d (try %d/%d)",
                      nss_, name_.c_str(), id_, address, n, num_tries);
            continue;
        }

        result = true;
        break;
    }

    return result;
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
    writeRegister(48, 4, 4095, TRIES);   // max poisiton
    writeRegister(52, 4, 0, TRIES);      // min position

    // direction
    if (inverse_)
        writeRegister(10, 1, 5, TRIES);  // inverse; time profile
    else
        writeRegister(10, 1, 4, TRIES);  // direct; time profile
}








MH5DynamixelInterface::MH5DynamixelInterface(){
}


MH5DynamixelInterface::~MH5DynamixelInterface(){
    ROS_INFO("Interface closed");
}


/**
 * @brief Initializes the Dyanmixel inteface.
 * 
 * @param root_nh top node handle owning the control
 * @param robot_hw_nh node handle to hardware owning this interface
 * @return true if all asctions have been successful
 * @return false if any of the action is unsucessful
 */
bool MH5DynamixelInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
    nh_ = robot_hw_nh;
    nss_ = nh_.getNamespace().c_str();     // to avoid calling it all the time

    // reset statistics
    read_total_packets_ = 0;
    read_error_packets_ = 0;
    write_total_packets_ = 0;
    write_error_packets_ = 0;
    
    if (!initPort()) return false;

    if (!initJoints()) return false;

    // if (!findServos())
    //     return false;

    // if (!initServos())
    //     return false;

    if (!setupDynamixelLoops())
        return false;

    //Register handles
    for(int i=0; i<num_joints_; i++){
        //State
        //hardware_interface::JointStateHandle jointStateHandle(joint_name[i], &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
        joint_state_interface.registerHandle(joints_[i].getJointStateHandle());
        //Control
        // hardware_interface::PosVelJointHandle jointPosVelHandle(jointStateHandle, &joint_position_command[i], &joint_velocity_command[i]);
        pos_vel_joint_interface.registerHandle(joints_[i].getJointPosVelHandle());
        //Torque activation
        // hardware_interface::JointHandle jointActiveHandle(jointStateHandle, /*&joint_active_state[i], */&joint_active_command[i]);
        active_joint_interface.registerHandle(joints_[i].getJointActiveHandle());
    }

    //Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&pos_vel_joint_interface);
    registerInterface(&active_joint_interface);
    
    //return true for successful init or ComboRobotHW initialisation will fail
    return true;
}

/**
 * @brief Initializes the Dynamixel port
 * 
 * @return true if all went ok
 * @return false if configuration information is missing or unable to open and
 *               configure the port
 */
bool MH5DynamixelInterface::initPort()
{
    // get the serial port configuration
    if (!nh_.getParam("port", port_)) {
        ROS_ERROR("[%s] no 'port' specified", nh_.getNamespace().c_str());
        return false;
    }
    if (!nh_.param("baudrate", baudrate_, 1000000))
        ROS_INFO("[%s] no 'baudrate' specified; defaulting to 1000000 bps", nh_.getNamespace().c_str());

    nh_.param("rs485", rs485_, false);

    nh_.param("protocol", protocol_, 2.0); 

    // open the serial port
    portHandler_ = new mh5_port_handler::PortHandlerMH5(port_.c_str());
    if (! portHandler_->openPort()) {
        ROS_ERROR("[%s] failed to open port %s", nh_.getNamespace().c_str(), port_.c_str());
        return false;
    }
    ROS_INFO("[%s] successfully opened port %s", nh_.getNamespace().c_str(), port_.c_str());

    if (!portHandler_->setBaudRate(baudrate_)) {
        ROS_ERROR("[%s] failed to set baud rate %i bps on port %s", nh_.getNamespace().c_str(), baudrate_, port_.c_str());
        return false;
    }
    ROS_INFO("[%s] successfully set baud rate %i bps on port %s", nh_.getNamespace().c_str(), baudrate_, port_.c_str());
    
    if (rs485_) {
        if (!portHandler_->setRS485() ) {
            ROS_ERROR("[%s] failed to configure RS485 on port %s", nh_.getNamespace().c_str(), port_.c_str());
        return false;
        }
        ROS_INFO("[%s] successfully configured RS485 on port %s", nh_.getNamespace().c_str(), port_.c_str());
    }

    packetHandler_ = dynamixel::PacketHandler::getPacketHandler((float)protocol_);
    ROS_INFO("[%s] Dynamixel protocol %3.1f initialzed", nh_.getNamespace().c_str(), protocol_);

    return true;
}

/**
 * @brief Initializes the joint information and the associated structures.
 * 
 * @return true if al is ok
 * @return false if something is missing (ex. no 'joints' paramter is defined)
 */
bool MH5DynamixelInterface::initJoints()
{
    //get joint names and num of joint
    std::vector<std::string>    joint_names;
    if (!nh_.getParam("joints", joint_names)) {
        ROS_ERROR("[%s] no 'joints' defined", nh_.getNamespace().c_str());
        return false;
    }
    num_joints_ = joint_names.size();
    if (num_joints_ == 0) {
        ROS_ERROR("[%s] 'joints' is empty", nh_.getNamespace().c_str());
        return false;
    }

    joints_.resize(num_joints_);
    for (int i=0; i < num_joints_; i++)
    {
        Joint& j = joints_[i];
        // get param setting
        j.fromParam(nh_, joint_names[i], portHandler_, packetHandler_);
        // see if avaialble
        if(!j.ping(5)) {
            ROS_ERROR("[%s] joint %s [%d] will be disabled (failed to communicate 5 times)", 
                        nss_, j.name().c_str(), j.id());
            j.setPresent(false);
        }
        // initialize; if torque is on, turn it off beffore setting the registers
        if(j.isActive(true)) {
            ROS_INFO("[%s] torqe is enabled for %s [%d]; it will be disabled to allow configuration of servos",
                     nh_.getNamespace().c_str(), j.name().c_str(), j.id());
            if(!j.torqueOff()) {
                ROS_ERROR("[%s] failed to reset torque status for %s [%d]",
                          nh_.getNamespace().c_str(), j.name().c_str(), j.id());
                continue;
            }
        }
        j.initRegisters();
        ROS_INFO("[%s] joint %s [%d] initialized", nh_.getNamespace().c_str(),
                 j.name().c_str(), j.id());
    }

    return true;
}

// /**
//  * @brief Checks if the servo is avaialable. Updates the `servo_present` vector.
//  * 
//  * @return true always
//  */
// bool MH5DynamixelInterface::findServos()
// {
//     for (int i=0; i < num_joints; i++)
//         if (servo_present[i]) 
//         {
//             if (pingServo(i, 5)) 
//                 servo_present[i] = true;
//             else {
//                 ROS_ERROR("[%s] joint %s [%d] will be disabled (failed to communicate 5 times)", 
//                             nh_.getNamespace().c_str(),
//                             joint_name[i].c_str(), servo_ids[i]);
//                 servo_present[i] = false;
//             }
//         }

//     return true;
// }


// /**
//  * @brief Pings the dyanmixel ID on the bus. Tries several times in case there
//  *        are communication problems.
//  * 
//  * @param index the index of the joint for which to ping the Dynamixel 
//  * @param num_tries how many tries to do in case there are no answers
//  * @return true if the Dynamixel has answered
//  * @return false if the Dynamixel failed to answer after `num_tries`
//  */
// bool MH5DynamixelInterface::pingServo(const int index, const int num_tries)
// {    
//     int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//     uint8_t dxl_error = 0;                          // Dynamixel error
//     bool servo_ok = false;
    
//     for (int n=0; n < num_tries; n++)
//     {
//         dxl_comm_result = packetHandler_->ping(portHandler_, servo_ids[index], &dxl_error);
        
//         if (dxl_comm_result != COMM_SUCCESS) {
//             ROS_ERROR("[%s] failed to communicate with joint %s [%d] (try %d/%d): %s", 
//                       nh_.getNamespace().c_str(),
//                       joint_name[index].c_str(),
//                       servo_ids[index],
//                       n + 1,
//                       num_tries,
//                       packetHandler_->getTxRxResult(dxl_comm_result));
//             continue;
//         }
        
//         if (dxl_error != 0) {
//             ROS_ERROR("[%s] error reported when communicating with joint %s [%d] (try %d/%d): %s", 
//                       nh_.getNamespace().c_str(),
//                       joint_name[index].c_str(),
//                       servo_ids[index], 
//                       n + 1,
//                       num_tries,
//                       packetHandler_->getRxPacketError(dxl_error));
//             continue;
//         }
        
//         ROS_INFO("[%s] joint %s [%d] detected", 
//                  nh_.getNamespace().c_str(),
//                  joint_name[index].c_str(),
//                  servo_ids[index]);
//         servo_ok = true;
//         break;
//     }
//     return servo_ok;
// }


// /**
//  * @brief Initializes the robots' registers. If during initialization there are
//  *        errors (ex. failed to update registers more than 5 times) the servo
//  *        will be marked as `false` in `servo_present` vector.
//  * 
//  * @return true always
//  */
// bool MH5DynamixelInterface::initServos()
// {
//     for (int i=0; i < num_joints; i++)

//         if (servo_present[i]) {
//             // read torque enable
//             long enable;
//             if(!readRegister(i, 64, 1, enable, TRIES)) {
//                 ROS_ERROR("[%s] failed to read torque status for %s [%d]",
//                       nh_.getNamespace().c_str(),
//                       joint_name[i].c_str(),
//                       servo_ids[i]);
//                 continue;
//             }
//             else if (enable == 1) {
//                 ROS_INFO("[%s] torqe is enabled for %s [%d]; it will be disabled to allow configuration of servos",
//                          nh_.getNamespace().c_str(),
//                          joint_name[i].c_str(),
//                          servo_ids[i]);
//                 if (!writeRegister(i, 64, 1, 0,TRIES)) {
//                     ROS_ERROR("[%s] failed to reset torque status for %s [%d]",
//                                nh_.getNamespace().c_str(),
//                                joint_name[i].c_str(),
//                                servo_ids[i]);
//                     continue;
//                 }
//                 else {
//                     ROS_INFO("[%s] sucessfully reset torque status for %s [%d]",
//                               nh_.getNamespace().c_str(),
//                               joint_name[i].c_str(),
//                               servo_ids[i]);
//                 }
//             }
//             joint_active_state[i] = 0;
//             // common
//             writeRegister(i, 9, 1, 0, TRIES);       // return delay
//             writeRegister(i, 11, 1, 3, TRIES);      // operating mode
//             writeRegister(i, 31, 1, 75, TRIES);     // temperature limit
//             writeRegister(i, 32, 2, 135, TRIES);    // max voltage
//             writeRegister(i, 44, 4, 1023, TRIES);   // velocity limit
//             writeRegister(i, 48, 4, 4095, TRIES);   // max poisiton
//             writeRegister(i, 52, 4, 0, TRIES);      // min position

//             // direction
//             if (joint_direction_inverse[i])
//                 writeRegister(i, 10, 1, 5, TRIES);  // inverse; time profile
//             else
//                 writeRegister(i, 10, 1, 4, TRIES);  // direct; time profile

//             ROS_INFO("[%s] joint %s [%d] initialized",
//                      nh_.getNamespace().c_str(),
//                      joint_name[i].c_str(), servo_ids[i]);
//         }

//     return true;
// }




bool MH5DynamixelInterface::setupDynamixelLoops()
{
    bool params_added = false;                        // addParam result
    // start address = 126 (Present Load)
    // data length = 10 (Present Load, Present Velocity, Present Position)
    syncRead_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, 126, 10);

    for (int i=0; i < num_joints_; i++) {
        if (joints_[i].present()) {
            if(!syncRead_->addParam(joints_[i].id()))
                ROS_WARN("Failed to add servo ID %d to SyncReadLoop", joints_[i].id());
            else
                params_added = true;
        }
    }

    if (!params_added) {
        ROS_WARN("No servos active for SyncReadLoop");
    }

    syncWrite_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, 108, 12);

    return true;
}


/**
 * @brief Performs the read of position, velocity, load for all servos that
 * are marked as present and converts the values to ISO (radians for position,
 * rad / sec for velocity and Nm for load). Uses a Dynamixel SyncRead to read
 * the values from all servis with one communication packet.
 * 
 * @param time The current time
 * @param period The time passed since the last call to \ref read
 */
void MH5DynamixelInterface::read(const ros::Time& time, const ros::Duration& period)
{
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    uint8_t dxl_error = 0;                            // Dynamixel error
    bool dxl_getdata_result = false;                  // GetParam result

    //call SyncRead
    dxl_comm_result = syncRead_->txRxPacket();
    read_total_packets_ += 1;
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_DEBUG("[%s] SyncRead communication failed: %s",
                 nh_.getNamespace().c_str(),
                 packetHandler_->getTxRxResult(dxl_comm_result));
        read_error_packets_ += 1;
        return;
    }

    // process each servo
    for(int i=0;i < num_joints_;i++)
    {
        Joint& j = joints_[i];
        uint8_t id = j.id();     // to avoid callling it all the time...
        //only present servos
        if (!j.present())
            continue;
        // check no errors
        if (syncRead_->getError(id, &dxl_error)) {
            ROS_DEBUG("[%s] SyncRead error getting ID %d: %s",
                      nss_, id, packetHandler_->getRxPacketError(dxl_error));
            continue;
        }
        //position
        dxl_getdata_result = syncRead_->isAvailable(id, 132, 4);
        if (!dxl_getdata_result)
            ROS_DEBUG("[%s] SyncRead getting position for ID %d failed", nss_, id);
        else {
            int32_t position = syncRead_->getData(id, 132, 4);
            // convert to radians
            // for XL430 a value of 2048 = pi > factor = pi / 2048
            j.setPositionFromRaw(position);
            //joint_position_state[i] = (position - 2047 ) * 0.001533980787886 + joint_offset[i];
        }
        //velocity
        dxl_getdata_result = syncRead_->isAvailable(id, 128, 4);
        if (!dxl_getdata_result)
            ROS_DEBUG("[%s] SyncRead getting velocity for ID %d failed", nss_, j.id());
        else {
            int32_t velocity = syncRead_->getData(id, 128, 4);
            // convert to radians / sec
            // 1 tick = 0.229 rev / min 
            // (see https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#velocity-limit44)
            // factor = 0.229 * 2pi / 60 [rad/sec]
            // if (velocity > 1023)
            //     velocity -= 4294967296;
            //joint_velocity_state[i] = velocity * 0.023980823922402;
            j.setVelocityFromRaw(velocity);
        }
        //load
        dxl_getdata_result = syncRead_->isAvailable(id, 126, 2);
        if (!dxl_getdata_result)
            ROS_DEBUG("[%s] SyncRead getting load for ID %d failed", nss_, id);
        else {
            int16_t load = syncRead_->getData(id, 126, 2);
            // convert to Nm
            // 1 tick = 0.1% of max torque
            // max torque = 1.4 [Nm]
            // (see https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#present-load126)
            // if (load > 1000)
            //     load -= 65536;
            //joint_effort_state[i] = load * 0.0014;
            j.setEffortFromRaw(load);
        }
    }
}


/**
 * @brief Performs the write of position, velocity profile and acceleration profile
 * for all servos that are marked as present. Assumes the servos have already been
 * configured with velocity profile (see Dyanamixel manual 
 * https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#what-is-the-profile).
 * Converts the values from ISO (radians for position, rad / sec for velocity)
 * to Dynamixel internal measures. Uses a Dynamixel SyncWrite to write
 * the values to all servos with one communication packet.
 * 
 * @param time The current time
 * @param period The time passed since the last call to \ref read
 */
void MH5DynamixelInterface::write(const ros::Time& time, const ros::Duration& period)
{
    // buffer for Dynamixel values
    uint8_t command[12];

    bool dxl_addparam_result = false;                 // addParam result
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool param_added = false;                         // at least one param added

    syncWrite_->clearParam();
    
    for (int i=0; i < num_joints_; i++)
    {
        Joint& j = joints_[i];
        uint8_t id = j.id();
        if (j.present())
        {
            //convert from radians and adjust for offset and dynamixel center
            // int32_t p = (int32_t)((joint_position_command[i] - joint_offset[i]) / 0.001533980787886 + 2047);
            int32_t p = j.getRawPositionFromCommand();
            // velocity profile = duration of the move [ms]
            // uint32_t vp = abs((joint_position_command[i] - joint_position_state[i]) / joint_velocity_command[i]) * 1000;
            uint32_t vp = j.getVelocityProfileFromCommand();
            // acceleration profile is 25% of velocity profile
            uint32_t ap = vp / 4;
            // platform-independent handling of byte order
            // acceleration; register 108
            command[0] = DXL_LOBYTE(DXL_LOWORD(ap));
            command[1] = DXL_HIBYTE(DXL_LOWORD(ap));
            command[2] = DXL_LOBYTE(DXL_HIWORD(ap));
            command[3] = DXL_HIBYTE(DXL_HIWORD(ap));
            // velocity profile ; register 112
            command[4] = DXL_LOBYTE(DXL_LOWORD(vp));
            command[5] = DXL_HIBYTE(DXL_LOWORD(vp));
            command[6] = DXL_LOBYTE(DXL_HIWORD(vp));
            command[7] = DXL_HIBYTE(DXL_HIWORD(vp));
            // position; register 116
            command[8] = DXL_LOBYTE(DXL_LOWORD(p));
            command[9] = DXL_HIBYTE(DXL_LOWORD(p));
            command[10] = DXL_LOBYTE(DXL_HIWORD(p));
            command[11] = DXL_HIBYTE(DXL_HIWORD(p));
            // addParam
            dxl_addparam_result = syncWrite_->addParam(id, command);
            if (dxl_addparam_result != true) {
                ROS_ERROR("[%s] failed to addParam for sync write for %s [%d]", nss_, j.name(), id);
                continue;
            }
            else
                param_added = true;
        }
    }

    if (param_added) {
        dxl_comm_result = syncWrite_->txPacket();
        write_total_packets_ += 1;
        if (dxl_comm_result != COMM_SUCCESS) {
            ROS_DEBUG("[%s] sync read failed: %s", nss_, packetHandler_->getTxRxResult(dxl_comm_result));
            write_error_packets_ += 1;
        }
    }

    // torqe activation
    for (int i=0; i < num_joints_; i++)
    {
        Joint& j = joints_[i];
        if (j.present())
        {
            if (j.shouldToggleTorque()) {
                if(!j.toggleTorque())
                    ROS_ERROR("[%s] failed to change torque for %s [%d] to %d",
                              nss_, j.name().c_str(), j.id(), (int)j.isActive(false));
                else {
                    ROS_INFO("[%s] successfully changed torque for %s [%d] to %d",
                              nss_, j.name().c_str(), j.id(), (int)j.isActive(false));
                }
            }
        }
    }

}



PLUGINLIB_EXPORT_CLASS(mh5_hardware_interface::MH5DynamixelInterface, hardware_interface::RobotHW)
