#include <mh5_hardware_control/mh5_dynamixel_interface.hpp>

namespace mh5_hardware_interface
{

MH5DynamixelInterface::MH5DynamixelInterface(){

}

MH5DynamixelInterface::~MH5DynamixelInterface(){

}

bool MH5DynamixelInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh){
    
    if (!initPort(robot_hw_nh))
        return false;

    
    //get joint names and num of joint
    robot_hw_nh.getParam("joints", joint_name);
    num_joints = joint_name.size();

    //resize vectors
    joint_position_state.resize(num_joints);
    joint_velocity_state.resize(num_joints);
    joint_effort_state.resize(num_joints);
    joint_effort_command.resize(num_joints);
 
    //Register handles
    for(int i=0; i<num_joints; i++){
        //State
        hardware_interface::JointStateHandle jointStateHandle(joint_name[i], &joint_position_state[i], &joint_velocity_state[i], &joint_effort_state[i]);
        joint_state_interface.registerHandle(jointStateHandle);
        

        //Effort
        hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command[i]);
        effort_joint_interface.registerHandle(jointEffortHandle);
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
        ROS_ERROR("[%s] no 'port' specified", robot_hw_nh.getNamespace().c_str());
        return false;
    }
    if (!robot_hw_nh.getParam("baudrate", baudrate_)) {
        ROS_ERROR("[%s] no 'baudrate' specified", robot_hw_nh.getNamespace().c_str());
        return false;
    }
    if (!robot_hw_nh.getParam("rs485", rs485_)) {
        rs485_ = false;
    }
    if (!robot_hw_nh.getParam("protocol", protocol_)) {
        ROS_ERROR("[%s] no 'protcol' specified", robot_hw_nh.getNamespace().c_str());
        return false;
    }

    // // open the serial port
    // portHandler_ = dynamixel::PortHandler::getPortHandler(port_.c_str());
    // if (! portHandler_->openPort()) {
    //     ROS_ERROR("[%s] failed to open port [%s]", robot_hw_nh.getNamespace().c_str(), port_.c_str());
    //     return false;
    // }
    // ROS_INFO("[%s] successfully opened port [%s]", robot_hw_nh.getNamespace().c_str(), port_.c_str());

    // if (!portHandler_->setBaudRate(baudrate_)) {
    //     ROS_ERROR("[%s] failed to set baud rate [%i] on port [%s]", robot_hw_nh.getNamespace().c_str(), baudrate_, port_.c_str());
    //     return false;
    // }
    // ROS_INFO("[%s] successfully set baud rate [%i] on port [%s]", robot_hw_nh.getNamespace().c_str(), baudrate_, port_.c_str());
    
    // if (rs485_) {

    // }

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