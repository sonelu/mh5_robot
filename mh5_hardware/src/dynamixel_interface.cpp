
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
    
    if (!initPort()) return false;

    if (!initJoints()) return false;

    if (!setupDynamixelLoops()) return false;

    //Register handles
    for(int i=0; i<num_joints_; i++){
        //State
        joint_state_interface.registerHandle(joints_[i]->getJointStateHandle());
        //Command Postion - Velocity
        pos_vel_joint_interface.registerHandle(joints_[i]->getJointPosVelHandle());
        //Torque activation
        active_joint_interface.registerHandle(joints_[i]->getJointActiveHandle());
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
        Joint* j = new Joint();
        // get param setting
        j->fromParam(nh_, joint_names[i], portHandler_, packetHandler_);
        // see if avaialble
        if(!j->ping(5)) {
            ROS_ERROR("[%s] joint %s [%d] will be disabled (failed to communicate 5 times)", 
                        nss_, j->name().c_str(), j->id());
            j->setPresent(false);
        }
        // because both axles' torque needs to be turned off in order to update
        // the registers; we first scan all the servos and disable torque if
        // necessary
        if(j->isActive(true)) {
            ROS_INFO("[%s] torqe is enabled for %s [%d]; it will be disabled to allow configuration of servos",
                     nh_.getNamespace().c_str(), j->name().c_str(), j->id());
            if(!j->torqueOff()) {
                ROS_ERROR("[%s] failed to reset torque status for %s [%d]",
                          nh_.getNamespace().c_str(), j->name().c_str(), j->id());
                continue;
            }
        }
        joints_[i] = j;
    }

    // now we can initialize the registers
    for (int i=0; i < num_joints_; i++) 
    {
        joints_[i]->initRegisters();
        ROS_INFO("[%s] joint %s [%d] initialized", nh_.getNamespace().c_str(),
                 joints_[i]->name().c_str(), joints_[i]->id());
    }

    return true;
}


template <class Loop>
Loop* MH5DynamixelInterface::setupLoop(std::string name, const double default_rate)
{
    double rate;
    if (!nh_.getParam("rates/"+name, rate)) 
    {
        ROS_INFO("[%s] loop %s no 'rates/%s' available, default to %.1f Hz", nss_, name.c_str(),
                  name.c_str(), default_rate);
        rate = default_rate;
    }
    else
        ROS_INFO("[%s] loop %s initialized at %.1f Hz", nss_, name.c_str(), rate);
    Loop* loop = new Loop(nh_.getNamespace()+"_"+name, rate, portHandler_, packetHandler_);
    loop->prepare(joints_);
    communication_stats_interface.registerHandle(loop->getCommStatHandle());

    return loop;
}


bool MH5DynamixelInterface::setupDynamixelLoops()
{
    // Position, Velocity, Load (PVL) Reader
    pvlReader_ = setupLoop<mh5_hardware::PVLReader>("pvl_reader", 100.0);

    // Temperature, Voltage (TV) Reader
    tvReader_ = setupLoop<mh5_hardware::TVReader>("tv_reader", 1.0);

    // Positon, Velocity (PV) Writer
    pvWriter_ = setupLoop<mh5_hardware::PVWriter>("pv_writer", 100.0);

    // Torque (T) Writer
    tWriter_ = setupLoop<mh5_hardware::TWriter>("t_writer", 2.0);

    // PID Writer
    // TODO

    //Register interfaces
    registerInterface(&communication_stats_interface);

    return true;
}



void MH5DynamixelInterface::read(const ros::Time& time, const ros::Duration& period)
{
    pvlReader_->Execute(time, period, joints_);
    tvReader_->Execute(time, period, joints_);
}


void MH5DynamixelInterface::write(const ros::Time& time, const ros::Duration& period)
{
    pvWriter_->Execute(time, period, joints_);
    tWriter_->Execute(time, period, joints_);
}



PLUGINLIB_EXPORT_CLASS(mh5_hardware::MH5DynamixelInterface, hardware_interface::RobotHW)
