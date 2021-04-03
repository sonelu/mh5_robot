#include <std_srvs/Trigger.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller.h>
#include <mh5_hardware/active_joint_interface.hpp>
#include "mh5_controllers/ActivateJoint.h"

#pragma once

namespace mh5_controllers
{

/**
 * @brief Controller that can swithc on or off the torque on a group of Dynamixel
 * servos.
 * 
 * Requires mh5_harware::ActiveJointInterfaces to be registered with the hardware
 * interface. Reads "joints" parameter from the param server, which should contain
 * a list of joint names that are controlled by this controllers.
 * 
 * Advertises 2 services under the namespace where launched:
 * 
 * - "torque/on" of type std_srvs::Trigger; this will buffer the request to
 * turn torque on for all joints associated with the controller
 * - "torque/off" of type std_srvs::Trigger; this will buffer the request to
 * turn torque off for all joints associated with the controller
 * 
 * The controller is intended to be as easy to use as possible from the 
 * command line such that no parameters are needed to be passed when calling
 * it:
 * 
 *      rosservice call /left_arm/torque/on
 * 
 * Will simply turn on the torque on all the servos associated with the 
 * "left_arm". Due to the way the servos are grouped in MH5 robot (head,
 * left_arm, right_arm, left_leg, right_leg) there is little need for activating
 * / deactivating torques at a more granular level.
 */
class ActiveJointController : public controller_interface::Controller<mh5_hardware::ActiveJointInterface>
{
public:

    /**
     * @brief Construct a new Active Joint Controller object using a 
     * mh5_hardware::ActiveJointInterface interface.
     */
    ActiveJointController()
    : controller_interface::Controller<mh5_hardware::ActiveJointInterface> ()
    {}

    /**
     * @brief Destroy the Active Joint Controller object. Shuts also down the 
     * two ROS servers.
     */
    ~ActiveJointController() {torque_srv_.shutdown(); }

    /**
     * @brief Initializes the controller by reading the joint list from the
     * parameter server under "joints". It expects a list of joint names
     * that are already registered with the hardware manager. If the parameter
     * is not provided or none of the joints listed are accessible through
     * the hardware interface the initialization will fail. It also trigger
     * a torque off request.
     * 
     * This function also advertises the two ROS services:
     * - torque/on
     * - torque/off
     * That will be shown under the current namespace of the initiator.
     * 
     * @param hw the hardware interface that will provide the access to the
     * repoces
     * @param n the nodehandle of the initiator controller 
     * @return true if there is at least one joint that has been successfully
     * identified and registered with this controller
     * @return false if either no "joints" parameter was available in the
     * param server or no joints has been successfully retrieved from the
     * hardware interface.
     */
    bool init(mh5_hardware::ActiveJointInterface* hw, ros::NodeHandle &n);

    /**
     * @brief Does nothing in this case. Used for completing the controller
     * interface.
     * @param time 
     */
    void starting(const ros::Time& time) {}

    /**
     * @brief Does the actual update of the joints' torque activation member.
     * Please note that this controller only sets the field as provided by
     * the mh5_hardware::ActiveJointInterface and it is not actually triggering
     * any communication with the actual servos. It is the hardware interface
     * respoonsibility to replicate this requests to the device.
     */
    void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);

    /**
     * @brief Map group->list of joint handles
     */
    std::map<std::string, std::vector< hardware_interface::JointHandle >>   joints_;

    /**
     * @brief Holds commands to be processed during the update() processings.
     * The service callbacks only store "true" or "false" in this buffer
     * depending on the command processed.
     */
    realtime_tools::RealtimeBuffer<ActivateJoint::Request> commands_buffer_;

    // /**
    //  * @brief Convenience, keeps the number of joints for loops, etc.
    //  */
    // unsigned int                                      n_joints_;

private:

    /**
     * @brief ROS Service that responds to the "switch_torque" calls.
     */
    ros::ServiceServer torque_srv_;

    /**
     * @brief Callback for processing "switch_torque" calls. Checks if the requested
     * group exists or if there is a joint by that name
     * 
     * @param req the service request; group/joint name  + desired state
     * @param res the service response; if things are successful + detailed message
     * @return true always
     */
    bool torqueCB(mh5_controllers::ActivateJoint::Request &req, mh5_controllers::ActivateJoint::Response &res);


};

} // namespace