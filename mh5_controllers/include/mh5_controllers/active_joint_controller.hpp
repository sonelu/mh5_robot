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
 * interface. Reads "groups" parameter from the param server, which should contain
 * a list of groups that can be toggled in the same time. It is possible to 
 * nest groups in each other as long as they build on each other.
 * 
 * Advertises a service ``/torque_control/switch_torque`` of type 
 * ``mh5_controllers/ActivateJoint``. The name passed in calls to this 
 * service can be individual joints or groups of joints.
 * 
 *      rosservice call /torque_control/swtich_torque "{name: "head_p", state: true}"
 * 
 * of for a group:
 * 
 *      rosservice call /torque_control/swtich_torque "{name: "head", state: true}"
 * 
 * Will simply turn on  or off the torque on all the servos associated with the 
 * group.
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
     *  ROS service.
     */
    ~ActiveJointController() {torque_srv_.shutdown(); }

    /**
     * @brief Initializes the controller by reading the joint list from the
     * parameter server under "groups". If no parameter is provided it will
     * create a group "all" and assign all avaialable resources to this
     * group. If groups are defined then they should be first listed in the
     * "groups" parameter, then each one of them should be listed separately
     * with the joints, or subgroups that are included. If subgroups are used
     * they have to be fully defined first, befire they are used in a superior
     * group. 
     * 
     * This function also advertises the ROS service: /[controller name]/switch_torque
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

private:

    /**
     * @brief Map group->list of joint handles
     */
    std::map<std::string, std::vector< mh5_hardware::JointTorqueAndReboot >>   joints_;

    /**
     * @brief Holds torque activation commands to be processed during the 
     * update() processings.
     * The service callbacks only store "true" or "false" in this buffer
     * depending on the command processed.
     */
    realtime_tools::RealtimeBuffer<ActivateJoint::Request> torque_commands_buffer_;

    /**
     * @brief Holds reboot commands to be processed during the 
     * update() processings.
     * The service callbacks only store "true" or "false" in this buffer
     * depending on the command processed.
     */
    realtime_tools::RealtimeBuffer<ActivateJoint::Request> reboot_commands_buffer_;

    /**
     * @brief ROS Service that responds to the "switch_torque" calls.
     */
    ros::ServiceServer torque_srv_;

    /**
     * @brief ROS Service that responds to the "reboot" calls.
     */
    ros::ServiceServer reboot_srv_;

    /**
     * @brief Callback for processing "switch_torque" calls. Checks if the requested
     * group exists or if there is a joint by that name
     * 
     * @param req the service request; group/joint name  + desired state
     * @param res the service response; if things are successful + detailed message
     * @return true always
     */
    bool torqueCB(mh5_controllers::ActivateJoint::Request &req, mh5_controllers::ActivateJoint::Response &res);

   /**
     * @brief Callback for processing "reboot" calls. Checks if the requested
     * group exists or if there is a joint by that name
     * 
     * @param req the service request; group/joint name  + desired state
     * @param res the service response; if things are successful + detailed message
     * @return true always
     */
    bool rebootCB(mh5_controllers::ActivateJoint::Request &req, mh5_controllers::ActivateJoint::Response &res);
};

} // namespace