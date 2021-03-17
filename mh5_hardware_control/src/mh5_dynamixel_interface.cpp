#include <mh5_hardware_control/mh5_dynamixel_interface.hpp>
#include <mh5_hardware_control/mh5_port_handler.hpp>

namespace mh5_hardware_interface
{

MH5DynamixelInterface::MH5DynamixelInterface(){

}

MH5DynamixelInterface::~MH5DynamixelInterface(){

}

bool MH5DynamixelInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh){
    
    nh_ = robot_hw_nh;
    
    if (!initPort())
        return false;

    if (!initServos())
        return false;

    if (!setupDynamixelLoops())
        return false;

    //Register handles
    for(int i=0; i<num_joints; i++){
        //State
        hardware_interface::JointStateHandle jointStateHandle(joint_name[i], &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
        joint_state_interface.registerHandle(jointStateHandle);
        //Effort
        hardware_interface::PosVelJointHandle jointPosVelHandle(jointStateHandle, &joint_position_command[i], &joint_velocity_command[i]);
        pos_vel_joint_interface.registerHandle(jointPosVelHandle);
    }

    //Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&pos_vel_joint_interface);
    
    //return true for successful init or ComboRobotHW initialisation will fail
    return true;
}


bool MH5DynamixelInterface::initPort() {
    // get the serial port configuration
    if (!nh_.getParam("port", port_)) {
        ROS_ERROR("[%s] no 'port' specified", nh_.getNamespace().c_str());
        return false;
    }
    if (!nh_.getParam("baudrate", baudrate_)) {
        ROS_ERROR("[%s] no 'baudrate' specified", nh_.getNamespace().c_str());
        return false;
    }
    if (!nh_.getParam("rs485", rs485_)) {
        rs485_ = false;
    }
    if (!nh_.getParam("protocol", protocol_)) {
        ROS_ERROR("[%s] no 'protcol' specified", nh_.getNamespace().c_str());
        return false;
    }

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

    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_);

    return true;
}


bool MH5DynamixelInterface::initServos() {

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    
    //get joint names and num of joint
    if (!nh_.getParam("joints", joint_name)) {
        ROS_ERROR("[%s] no 'joints' defined", nh_.getNamespace().c_str());
        return false;
    }
    num_joints = joint_name.size();
    if (num_joints == 0) {
        ROS_ERROR("[%s] 'joints' is empty", nh_.getNamespace().c_str());
        return false;
    }

    //resize vectors
    servo_ids.resize(num_joints);
    servo_present.resize(num_joints);
    joint_position_state.resize(num_joints);
    joint_velocity_state.resize(num_joints);
    joint_effort_state.resize(num_joints);
    joint_position_command.resize(num_joints);
    joint_velocity_command.resize(num_joints);

    // read servo configuration
    for (int i=0; i < num_joints; i++) {
        
        int servo_id;

        if (!nh_.getParam(joint_name[i] + "/id", servo_id)) {
            ROS_ERROR("[%s] ID not found for joint %s; will be disabled",
                      nh_.getNamespace().c_str(),
                      joint_name[i].c_str());
            servo_present[i] = false;
            continue;
        }
        
        servo_ids[i] = (uint8_t)servo_id;
        bool servo_ok = false;
        // ping the servo; will try 5 times in case random errors are appearing
        for (int n=0; n < 5; n++) {
            dxl_comm_result = packetHandler_->ping(portHandler_, servo_ids[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                ROS_ERROR("[%s] failed to communicate with joint %s [%d] (try %d/5): %s", 
                        nh_.getNamespace().c_str(),
                        joint_name[i].c_str(), servo_ids[i],
                        n + 1,
                        packetHandler_->getTxRxResult(dxl_comm_result));
                continue;
            }
            if (dxl_error != 0) {
                ROS_ERROR("[%s] error reported when communicating with joint %s [%d] (try %d/5): %s", 
                        nh_.getNamespace().c_str(),
                        joint_name[i].c_str(), servo_ids[i], 
                        n + 1,
                        packetHandler_->getRxPacketError(dxl_error));
                continue;
            }
            ROS_INFO("[%s] joint %s [%d] detected", 
                    nh_.getNamespace().c_str(),
                    joint_name[i].c_str(),
                    servo_ids[i]);
            servo_ok = true;
            break;
        }
        if (servo_ok) 
            servo_present[i] = true;
        else {
            ROS_ERROR("[%s] joint %s [%d] will be disabled (failed to communicate 5 times)", 
                        nh_.getNamespace().c_str(),
                        joint_name[i].c_str(), servo_ids[i]);
            servo_present[i] = false;
        }

    }
    return true;
}


bool MH5DynamixelInterface::setupDynamixelLoops() {
    bool params_added = false;                        // addParam result
    // start address = 126 (Present Load)
    // data length = 10 (Present Load, Present Velocity, Present Position)
    syncRead_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, 126, 10);

    for (int i=0; i < num_joints; i++) {
        if (servo_present[i]) {
            if(!syncRead_->addParam(servo_ids[i]))
                ROS_WARN("Failed to add servo ID %d to SyncReadLoop", servo_ids[i]);
            else
                params_added = true;
        }
    }

    if (!params_added) {
        ROS_WARN("No servos active for SyncReadLoop");
    }

    return true;
}


void MH5DynamixelInterface::read(const ros::Time& time, const ros::Duration& period){
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    uint8_t dxl_error = 0;                            // Dynamixel error
    bool dxl_getdata_result = false;                  // GetParam result

    //call SyncRead
    dxl_comm_result = syncRead_->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        ROS_DEBUG("[%s] SyncRead communication failed: %s",
                 nh_.getNamespace().c_str(),
                 packetHandler_->getTxRxResult(dxl_comm_result));
        return;
    }

    // process each servo
    for(int i=0;i < num_joints;i++) {
        //only present servos
        if (!servo_present[i])
            continue;
        // check no errors
        if (syncRead_->getError(servo_ids[i], &dxl_error)) {
            ROS_DEBUG("[%s] SyncRead error getting ID %d: %s",
                      nh_.getNamespace().c_str(),
                      servo_ids[i],
                      packetHandler_->getRxPacketError(dxl_error));
            continue;
        }
        //position
        dxl_getdata_result = syncRead_->isAvailable(servo_ids[i], 132, 4);
        if (!dxl_getdata_result)
            ROS_DEBUG("[%s] SyncRead getting position for ID %d failed",
                      nh_.getNamespace().c_str(),
                      servo_ids[i]);
        else {
            int32_t position = syncRead_->getData(servo_ids[i], 132, 4);
            // convert to radians
            joint_position_state[i] = (position - 2047) * 0.001534355386369;
        }
        //velocity
        dxl_getdata_result = syncRead_->isAvailable(servo_ids[i], 128, 4);
        if (!dxl_getdata_result)
            ROS_DEBUG("[%s] SyncRead getting velocity for ID %d failed",
                      nh_.getNamespace().c_str(),
                      servo_ids[i]);
        else {
            int32_t velocity = syncRead_->getData(servo_ids[i], 128, 4);
            // convert to radians / sec
            if (velocity > 1023)
                velocity -= 4294967296;
            joint_velocity_state[i] = velocity * 0.023980823922402;
        }
        //load
        dxl_getdata_result = syncRead_->isAvailable(servo_ids[i], 126, 2);
        if (!dxl_getdata_result)
            ROS_DEBUG("[%s] SyncRead getting load for ID %d failed",
                      nh_.getNamespace().c_str(),
                      servo_ids[i]);
        else {
            int16_t load = syncRead_->getData(servo_ids[i], 126, 2);
            // convert to Nm
            if (load > 1000)
                load -= 65536;
            joint_effort_state[i] = load * 0.0014;
        }
    }

}

void MH5DynamixelInterface::write(const ros::Time& time, const ros::Duration& period){
   for(int i=0;i < num_joints;i++){
       //robot.writeJoints(joint_effort_command[i]);
   }
}

}
PLUGINLIB_EXPORT_CLASS(mh5_hardware_interface::MH5DynamixelInterface, hardware_interface::RobotHW)