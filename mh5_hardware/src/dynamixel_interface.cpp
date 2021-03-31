
#include <pluginlib/class_list_macros.hpp>

#include "mh5_hardware/dynamixel_interface.hpp"
#include "mh5_hardware/active_joint_interface.hpp"
#include "mh5_hardware/dynamixel_joint.hpp"

using namespace mh5_hardware;



MH5DynamixelInterface::MH5DynamixelInterface(){
}


MH5DynamixelInterface::~MH5DynamixelInterface(){
    ROS_INFO("Interface closed");
}


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

    if (!setupDynamixelLoops()) return false;

    //Register handles
    for(int i=0; i<num_joints_; i++){
        //State
        joint_state_interface.registerHandle(joints_[i].getJointStateHandle());
        //Command Postion - Velocity
        pos_vel_joint_interface.registerHandle(joints_[i].getJointPosVelHandle());
        //Torque activation
        active_joint_interface.registerHandle(joints_[i].getJointActiveHandle());
    }

    //Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&pos_vel_joint_interface);
    registerInterface(&active_joint_interface);
    
    //return true for successful init or ComboRobotHW initialisation will fail
    return true;
}


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


bool MH5DynamixelInterface::setupDynamixelLoops()
{
    std::string loopName;
    double rate;
    std::string ns = nh_.getNamespace();

    // Position, Velocity, Load (PVL) Reader
    loopName = "pvl_reader";
    if(!nh_.getParam("rates/"+loopName, rate)) {
        ROS_INFO("[%s] no '%s' available; default to 100.0", nss_, ("rates/"+loopName).c_str());
        rate = 100.0;
    }
    else
        ROS_INFO("[%s] loop %s initialized at %.1f", nss_, loopName.c_str(), rate);
    pvlReader_ = new mh5_hardware::PVLReader(ns+"_"+loopName, rate, portHandler_, packetHandler_);
    pvlReader_->prepare(joints_);
    communication_stats_interface.registerHandle(pvlReader_->getCommStatHandle());

    // Temperature, Voltage (TV) Reader

    // Positon, Velocity (PV) Writer
    syncWrite_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, 108, 12);

    // PID Writer
    // TODO

    //Register interfaces
    registerInterface(&communication_stats_interface);

    return true;
}



void MH5DynamixelInterface::read(const ros::Time& time, const ros::Duration& period)
{
    pvlReader_->Execute(time, period);
    pvlReader_->afterExecute(joints_);
}


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
            int32_t p = j.getRawPositionFromCommand();
            uint32_t vp = j.getVelocityProfileFromCommand();
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



PLUGINLIB_EXPORT_CLASS(mh5_hardware::MH5DynamixelInterface, hardware_interface::RobotHW)
