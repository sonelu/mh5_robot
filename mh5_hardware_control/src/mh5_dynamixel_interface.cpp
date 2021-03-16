#include <mh5_hardware_control/mh5_dynamixel_interface.hpp>
#include <mh5_hardware_control/mh5_port_handler.hpp>

namespace mh5_hardware_interface
{

MH5DynamixelInterface::MH5DynamixelInterface(){

}

MH5DynamixelInterface::~MH5DynamixelInterface(){

}

bool MH5DynamixelInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh){
    
    if (!initPort(robot_hw_nh))
        return false;

    if (!initServos(robot_hw_nh))
        return false;
 
    //Register handles
    for(int i=0; i<num_joints; i++){
        // only register succesfully detected joints
        if (servo_present[i]) {
            //State
            hardware_interface::JointStateHandle jointStateHandle(joint_name[i], &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
            joint_state_interface.registerHandle(jointStateHandle);
            //Effort
            hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command[i]);
            effort_joint_interface.registerHandle(jointEffortHandle);
        }
    }

    //Register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&effort_joint_interface);
    
    //return true for successful init or ComboRobotHW initialisation will fail
    return true;
}


bool MH5DynamixelInterface::initPort(ros::NodeHandle& robot_hw_nh) {
    // get the serial port configuration
    if (!robot_hw_nh.getParam("port", port_)) {
        ROS_ERROR("%s no 'port' specified", robot_hw_nh.getNamespace().c_str());
        return false;
    }
    if (!robot_hw_nh.getParam("baudrate", baudrate_)) {
        ROS_ERROR("%s no 'baudrate' specified", robot_hw_nh.getNamespace().c_str());
        return false;
    }
    if (!robot_hw_nh.getParam("rs485", rs485_)) {
        rs485_ = false;
    }
    if (!robot_hw_nh.getParam("protocol", protocol_)) {
        ROS_ERROR("%s no 'protcol' specified", robot_hw_nh.getNamespace().c_str());
        return false;
    }

    // open the serial port
    portHandler_ = new mh5_port_handler::PortHandlerMH5(port_.c_str());
    if (! portHandler_->openPort()) {
        ROS_ERROR("%s failed to open port %s", robot_hw_nh.getNamespace().c_str(), port_.c_str());
        return false;
    }
    ROS_INFO("%s successfully opened port %s", robot_hw_nh.getNamespace().c_str(), port_.c_str());

    if (!portHandler_->setBaudRate(baudrate_)) {
        ROS_ERROR("%s failed to set baud rate %i bps on port %s", robot_hw_nh.getNamespace().c_str(), baudrate_, port_.c_str());
        return false;
    }
    ROS_INFO("%s successfully set baud rate %i bps on port %s", robot_hw_nh.getNamespace().c_str(), baudrate_, port_.c_str());
    
    if (rs485_) {
        if (!portHandler_->setRS485() ) {
            ROS_ERROR("%s failed to configure RS485 on port %s", robot_hw_nh.getNamespace().c_str(), port_.c_str());
        return false;
        }
        ROS_INFO("%s successfully configured RS485 on port %s", robot_hw_nh.getNamespace().c_str(), port_.c_str());
    }

    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_);

    return true;
}


bool MH5DynamixelInterface::initServos(ros::NodeHandle& robot_hw_nh) {

    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error
    
    //get joint names and num of joint
    if (!robot_hw_nh.getParam("joints", joint_name)) {
        ROS_ERROR("%s no 'joints' defined", robot_hw_nh.getNamespace().c_str());
        return false;
    }
    num_joints = joint_name.size();
    if (num_joints == 0) {
        ROS_ERROR("%s 'joints' is empty", robot_hw_nh.getNamespace().c_str());
        return false;
    }

    //resize vectors
    servo_ids.resize(num_joints);
    servo_present.resize(num_joints);
    joint_position_state.resize(num_joints);
    joint_velocity_state.resize(num_joints);
    joint_effort_state.resize(num_joints);
    joint_effort_command.resize(num_joints);

    // read servo configuration
    for (int i=0; i < num_joints; i++) {
        int servo_id;
        if (! robot_hw_nh.getParam(joint_name[i] + "/id", servo_id)) {
            ROS_ERROR("ID not found for joint %s; will be disabled", joint_name[i].c_str());
            servo_present[i] = false;
        }
        else {
            servo_ids[i] = (uint8_t)servo_id;
            // ping the servo
            dxl_comm_result = packetHandler_->ping(portHandler_, servo_ids[i], &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                ROS_ERROR("failed to communicate with joint %s [%d]: %s; will be disabled", 
                          joint_name[i].c_str(), servo_ids[i], 
                          packetHandler_->getTxRxResult(dxl_comm_result));
                servo_present[i] = false;
            }
            else if (dxl_error != 0) {
                ROS_ERROR("error reported when communicating with joint %s [%d]: %s; will be disabled", 
                          joint_name[i].c_str(), servo_ids[i], 
                          packetHandler_->getRxPacketError(dxl_error));
                servo_present[i] = false;
            }
            else {
                ROS_INFO("joint %s [%d] detected", joint_name[i].c_str(), servo_ids[i]);
                servo_present[i] = true;
            }
        }
    }

    return true;
}

void MH5DynamixelInterface::read(const ros::Time& time, const ros::Duration& period){
        for(int i=0;i < num_joints;i++){
            //joint_position_state[i] = robot.readJoints(i);
        }
}

void MH5DynamixelInterface::write(const ros::Time& time, const ros::Duration& period){
   for(int i=0;i < num_joints;i++){
       //robot.writeJoints(joint_effort_command[i]);
   }
}

}
PLUGINLIB_EXPORT_CLASS(mh5_hardware_interface::MH5DynamixelInterface, hardware_interface::RobotHW)