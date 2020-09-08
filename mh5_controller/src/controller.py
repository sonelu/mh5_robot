#!/usr/bin/env python3

import yaml
import rospy
import actionlib

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Temperature, BatteryState
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryResult, \
                             FollowJointTrajectoryFeedback, \
                             FollowJointTrajectoryGoal
from diagnostic_msgs.msg import DiagnosticArray

from mh5_controller.srv import ChangeTorque, ChangeTorqueResponse
from mh5_controller.srv import DynamixelCommand, DynamixelCommandResponse

from bus import DynamixelBus
from device import PVE


class DynamixelController():
    """Manges exactly one Dynamixel bus and peforms periodically read/write
    to all the devices that are connected to that bus.
    """
    def __init__(self):
        # load config file
        self.__init_from_config()
        # setup stuff
        self.main_rate = self.get_param('~rate', 100)
        self.rate = rospy.Rate(self.main_rate)
        self.execs = 0
        # publishers
        self.pub_stat = rospy.Publisher('communication_statistics', DiagnosticArray, queue_size=1)
        self.pub_pve = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.pub_temp = rospy.Publisher('temperature', Temperature, queue_size=25)
        self.pub_volt = rospy.Publisher('voltage', BatteryState, queue_size=25)
        # follow joint trajectory action server
        self.fjt_server = actionlib.SimpleActionServer(
            'follow_joint_trajectory', FollowJointTrajectoryAction,
            self.do_follow_joint_trajectory, False)
        # services
        self.torque_srv = rospy.Service('change_torque', ChangeTorque, self.do_change_torque)
        self.dxl_srv = rospy.Service('dynamixel_command', DynamixelCommand, self.do_dyamixel_command)

    def __init_from_config(self):
        """Reads the configuration file and sets up the controller."""
        config_file = self.get_param('~config_file')
        with open(config_file, 'r') as f:
            config_items = yaml.load(f, Loader=yaml.FullLoader)
        # buses & devices
        self.buses = {}
        self.devices = {}
        for bus_name, bus_info in config_items['buses'].items():
            new_bus = DynamixelBus(name=bus_name, **bus_info)
            self.buses[bus_name] = new_bus
            for device in new_bus.devices.values():
                self.devices[device.name] = device
        # inits
        self.inits = config_items.get('inits', {})
        # groups
        self.groups = {}
        if 'groups' in config_items:
            for group_name, group_items in config_items['groups'].items():
                joints = []
                for item in group_items:
                    if item in self.devices:
                        joints.append(item)
                    elif item in self.groups:
                        for joint in self.groups[item]:
                            if joint not in joints:
                                joints.append(joint)
                self.groups[group_name] = joints
                rospy.loginfo(f'Adding group {group_name} with joints {joints}')

    def start(self):
        # open ports
        for bus in self.buses.values():
            bus.open()
        # torque off; just to make sure
        for device in self.devices.values():
            device.torque_change(state=False)
        # initialize devices
        for dev_name, device in self.devices.items():
            if device.inits:
                rospy.loginfo(f'Initializing {dev_name}')
                regs = {}
                for init in device.inits:
                    if init not in self.inits:
                        rospy.loginfo(
                            f'Init {init} specified for device {dev_name} '
                            f'does not exist. Will be ignored.')
                    else:
                        regs.update(self.inits[init])
                if regs:
                    for reg_num, values in regs.items():
                        device.write(reg_num, values[0], values[1])
        # start syncs
        for bus in self.buses.values():
            bus.start_sync()
        # start publishers
        rate = self.get_param('~pub_stats_rate', 0.5)
        rospy.loginfo(f'Scheduling communication_statistics publisher at {rate:.1f}Hz')
        self.pub_stat_timer = rospy.Timer(rospy.Duration(1.0/rate), self.publish_communication_statistics)
        rate = self.get_param('~pub_pve_rate', 0.5)
        rospy.loginfo(f'Scheduling joint_states publisher at {rate:.1f}Hz')
        self.pub_pve_timer = rospy.Timer(rospy.Duration(1.0/rate), self.publish_joint_states)
        rate = self.get_param('~pub_temp_rate', 0.5)
        rospy.loginfo(f'Scheduling temperature publisher at {rate:.1f}Hz')
        self.pub_temp_timer = rospy.Timer(rospy.Duration(1.0/rate), self.publish_temperature)
        rate = self.get_param('~pub_volt_rate', 0.5)
        rospy.loginfo(f'Scheduling voltage publisher at {rate:.1f}Hz')
        self.pub_volt_timer = rospy.Timer(rospy.Duration(1.0/rate), self.publish_voltage)
        # start servers

        # start action servers
        self.fjt_server.start()

    def get_param(self, param, default=None):
        if rospy.has_param(param):
            return rospy.get_param(param)
        if default is not None:
            rospy.loginfo(f'Default value {default} will be used for '
                          f'ROS parameter {param}')
            return default
        rospy.logfatal(f'Parameter {param} not supplied '
                       'and no default available')
        exit()

    def publish_communication_statistics(self, event):
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()
        for bus in self.buses.values():
            msg.status.append(bus.pve_reader.stats_as_msg())
            msg.status.append(bus.tv_reader.stats_as_msg())
            msg.status.append(bus.pva_writer.stats_as_msg())
        self.pub_stat.publish(msg)

    def publish_joint_states(self, event):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        for device_name, device in self.devices.items():
            msg.name.append(device_name)
            # position in radians; factor = 2 * pi / 4095
            raw_pos = device.current.pos + device.offset
            rad_pos = (raw_pos - 2047) * 0.001534355386369
            msg.position.append(rad_pos)
            # angular velocity in rad /s; factor = 2 * pi * 0.229 / 60
            raw_vel = device.current.vel
            if raw_vel > 1023:
                raw_vel = raw_vel - 4294967296
            rad_vel = raw_vel * 0.023980823922402
            msg.velocity.append(rad_vel)
            # aprox effort in N*m; factor = 1/1000 * 1.4 (stall torque)
            # _very_ aproximate!
            raw_eff = device.current.eff
            if raw_eff > 1000:
                raw_eff = raw_eff - 65536
            nm_eff = raw_eff * 0.0014
            msg.effort.append(nm_eff)
        # publish one message for all joints
        self.pub_pve.publish(msg)

    def publish_temperature(self, event):
        for device_name, device in self.devices.items():
            msg = Temperature()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = device_name
            msg.temperature = device.temperature
            self.pub_temp.publish(msg)

    def publish_voltage(self, event):
        MAX_VOLTAGE = 3 * 4.2 - 0.1     # voltage for 100% charge battery
        MIN_VOLTAGE = 3 * 3.0 - 0.1     # voltage for 0% charge battery
        RANGE_VOLTAGE = MAX_VOLTAGE - MIN_VOLTAGE
        for device_name, device in self.devices.items():
            msg = BatteryState()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = device_name
            msg.voltage = device.voltage / 10.0
            msg.percentage = (msg.voltage - MIN_VOLTAGE) / RANGE_VOLTAGE * 100.0
            self.pub_volt.publish(msg)

    def do_change_torque(self, request):
        """Call back for ChangeTorque server.
        Toggles torque for the indicated joints. It will return the joints
        processed and the result of the transaction. It will also mark every joint
        with the state of the torque in the ``torque_active`` dictionary key.
        """
        # remove any duplicates
        joints = set(request.joints)
        for group in request.groups:
            if group in self.groups:
                joints.update(self.groups[group])
        results = []
        if joints:
            joints_list = list(joints)
            for joint_name in joints_list:
                res = self.devices[joint_name].torque_change(request.state)
                results.append(res)
        return ChangeTorqueResponse(joints_list, results)

    def do_dyamixel_command(self, request):
        """Call back for DynamixelCommand server.
        Supports direct commands to dynamixel devices.
        """
        # remove any duplicates
        joints = set(request.joints)
        for group in request.groups:
            if group in self.groups:
                joints.update(self.groups[group])
        joints_list = list(joints)
        results = []
        print(joints_list)
        if not joints_list:
            return DynamixelCommandResponse('No joints provided', [], [])
        if request.command == 'reboot':
            for joint_name in joints_list:
                if joint_name in self.devices:
                    res = self.devices[joint_name].reboot()
                    results.append(res)
            return DynamixelCommandResponse('Reboot executed', joints_list, results)

    def do_follow_joint_trajectory(self, request):
        """Call-back for follow_joint_trajectory server."""
        rospy.loginfo(f'Goal received: {request}')
        # if not self.torque_active:
        #     result = FollowJointTrajectoryResult(-1, 'torque not active')
        #     self.fjt_server.set_aborted(result)
        #     return
        # torque active, execute the request
        # extract the information from the request
        trajectory = request.trajectory
        # check joints are known
        joints = [joint for joint in trajectory.joint_names if joint not in self.devices]
        if joints:
            result = FollowJointTrajectoryResult(
                FollowJointTrajectoryResult.INVALID_JOINTS,
                f'unknown joints: {joints}')
            self.fjt_server.set_aborted(result)
            return
        # check joints are with torque on
        joints = [joint for joint in trajectory.joint_names if not self.devices[joint].torque_active]
        if joints:
            result = FollowJointTrajectoryResult(
                FollowJointTrajectoryResult.INVALID_JOINTS,
                f'joints not active: {joints}')
            self.fjt_server.set_aborted(result)
            return
        
        last_time = 0.0         # keeps track of the time in request
        for pose in trajectory.points:
            frame_time = pose.time_from_start.to_sec()
            frame_duration = frame_time - last_time
            for index, joint_name in enumerate(trajectory.joint_names):
                device = self.devices[joint_name]
                rad_pos = pose.positions[index]
                # convert to 0-4095
                raw_pos = rad_pos / 0.001534355386369 + 2047 - device.offset
                # for velocity we use the profile_velocity
                cur_pos = device.current.pos
                raw_vel = abs(raw_pos - cur_pos) / 7.8165333 / frame_duration
                raw_acc = raw_vel / 4.0
                device.goal = PVE(int(raw_pos), int(raw_vel), int(raw_acc))
                # wait for the
            last_time = frame_time
            # need to adjust the duration here to be more acurate
            rospy.sleep(frame_duration)
            # feedback
            feedback = FollowJointTrajectoryFeedback()
            feedback.joint_names = trajectory.joint_names
            feedback.desired = pose
            feedback.actual.positions = [(self.devices[device].current.pos - 2047) * 0.001534355386369 for device in trajectory.joint_names]
            # feedback.error.positions = feedback.desired.positions - feedback.actual.positions
            self.fjt_server.publish_feedback(feedback)
        result = FollowJointTrajectoryResult(
            FollowJointTrajectoryResult.SUCCESSFUL, 'reached that goal')
        self.fjt_server.set_succeeded(result)

    def finish(self):
        # torque off; just to make sure
        for device in self.devices.values():
            device.torque_change(state=False)
        # stop publishers
        self.pub_pve_timer.shutdown()
        self.pub_stat_timer.shutdown()
        self.pub_temp_timer.shutdown()
        self.pub_volt_timer.shutdown()
        for bus in self.buses.values():
            bus.close()


if __name__ == '__main__':

    rospy.init_node('dynamixel_controller', log_level=rospy.INFO)
    controller = DynamixelController()
    controller.start()

    while not rospy.is_shutdown():

        controller.rate.sleep()

    controller.finish()
