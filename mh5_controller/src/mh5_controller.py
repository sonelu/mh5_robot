#!/usr/bin/env python

import yaml
from collections import namedtuple
from threading import Lock

import dynamixel_sdk as dyn
from serial import rs485

import rospy
import actionlib

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Temperature, BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryResult, \
                             FollowJointTrajectoryActionGoal
from mh5_robot.srv import ChangeTorque, ChangeTorqueResponse



class Communicator():
    """A base class for handling communication on the Dyamixel bus. It keeps
    track of the communication statistics (although the subclasses must update
    this information according to the tasks performed) and can package it in
    a ``DiagnosticStatus`` message.

    Parameters
    ----------

    name: str
        The name of the communication object. Used in messages to indicate the
        source.

    port: PortHandler
        The port that is used for communication. Should have already been
        opened and setup by the user.


    ph: PacketHandler
        The Dynamixel packet handler used for communication. Should havce
        already been setup by the user.

    devices: dict
        A dictionary with the devices. The key is the device (joint) name and
        the value is the information for that joint. See the YML configuration
        file for what is the expected structure.

    run_every: int
        A number that indicates the periodicity of actually performing the
        communication. All communication objects will be called by the
        controller on the rate that the controller is configured (ex. 100Hz).
        But each communicator can be configured to actually perform the actions
        much rare, like in the case of reading the voltage and temperature,
        where we only need to do it every second or so. The ``run`` method
        only increments the executions and calls the actual ``communicate``
        method only when the number of executions hit ``run_every``. This way,
        even if the main loop runs at 100Hz, by setting the ``run_every`` to
        100 a pparticular communication task will be executed only once a
        second.
    """
    def __init__(self, name, bus, devices, run_every):
        self.name = name
        self.bus = bus
        self.devices = devices
        self.run_every = run_every
        # keeps track for statistics
        self.errors = 0
        self.packets = 0
        self.cum_errors = 0
        self.cum_packets = 0
        self.execs = 0
        # for calculating the actual rate
        self.last_time = rospy.rostime.get_rostime()
        self.comms = 0
        self.real_rate = 0.0

    def setup(self):
        """Prepares the communicator. Must be implemented by the subclass."""
        pass

    def run(self, event):
        """Runs a cycle. It increments the counts of the executions and if
        matches ``run_every`` then it invokes the actual communication method.
        """
        self.execs += 1
        if self.execs >= self.run_every:
            self.communicate()
            self.comms +=1
            durr = (event.current_real - self.last_time).to_sec()
            if durr >= 2.0 and self.comms >= 10:
                self.real_rate = self.comms / durr
                self.last_time = event.current_real
                self.comms = 0
            self.execs = 0

    def communicate(self):
        """Performs a communication. Must be implemented by subclass."""
        pass

    def stats_as_msg(self, reset=True):
        """Returns a DiagnosticStatus message with information from statistics.
        """
        msg = DiagnosticStatus()
        msg.name = self.name
        # we might need to add the hardware ID to differentiate between robots?
        msg.values.append(KeyValue('errors', str(self.errors)))
        msg.values.append(KeyValue('packets', str(self.packets)))
        rate = self.errors / self.packets if self.packets != 0 else 0.0
        msg.values.append(KeyValue('error_rate_perc', f'{rate * 100:.2f}'))
        self.cum_errors += self.errors
        self.cum_packets += self.packets
        msg.values.append(KeyValue('cum_errors', str(self.cum_errors)))
        msg.values.append(KeyValue('cum_packets', str(self.cum_packets)))
        rate = self.cum_errors / self.cum_packets if self.cum_packets != 0 else 0.0
        msg.values.append(KeyValue('cum_error_rate_perc', f'{rate * 100:.2f}'))
        msg.values.append(KeyValue('real_rate', f'{self.real_rate:.2f}'))
        if reset:
            self.errors = 0
            self.packets = 0
        return msg


class PVEReader(Communicator):
    """A SyncRead communication that reads postition, velocity and effort of
    Dynamnixel registers.
    """
    def __init__(self, bus, run_every):
        super().__init__(name=f'{bus.name}_pvl_reader',
                         bus=bus,
                         devices=bus.devices,
                         run_every=run_every)
        self.gsr = dyn.GroupSyncRead(bus.dyn_port, bus.dyn_ph, 126, 10)
        for device in self.devices.values():
            self.gsr.addParam(device.dev_id)
        # self.pub = rospy.Publisher('joint_state', JointState, queue_size=1)

    def communicate(self):
        # We will ignore the result of the txRxPacket because it is a stacked
        # result of multiple packet read and is not representative. We will
        # instead use the information per device to keep track of statistics
        with self.bus.lock:
            _ = self.gsr.txRxPacket()
        for device in self.devices.values():
            dev_id = device.dev_id
            self.packets += 1
            # position
            if not self.gsr.isAvailable(dev_id, 132, 4):
                self.errors += 1
                rospy.logdebug(f'{self.name}: failed to get data for device {dev_id}')
            else:
                # sometimes there are some random errors and they throw the
                # whole thread off; we just log it and move on
                try:
                    raw_pos = self.gsr.getData(dev_id, 132, 4)
                    raw_vel = self.gsr.getData(dev_id, 128, 4)
                    raw_eff = self.gsr.getData(dev_id, 126, 2)
                    device.current = PVE(raw_pos, raw_vel, raw_eff)
                except IndexError as e:
                    rospy.loginfo(f'An IndexError exception was raised in the PVE Reader for {self.name}')
                    rospy.loginfo(f'{e}')


class TVReader(Communicator):
    """A SyncRead communication that reads temperature and voltage of
    Dynamnixel registers.
    """
    def __init__(self, bus, run_every):
        super().__init__(name=f'{bus.name}_tv_reader',
                         bus=bus,
                         devices=bus.devices,
                         run_every=run_every)
        self.gsr = dyn.GroupSyncRead(bus.dyn_port, bus.dyn_ph, 144, 3)
        for device in self.devices.values():
            self.gsr.addParam(device.dev_id)

    def communicate(self):
        with self.bus.lock:
            _ = self.gsr.txRxPacket()
        for device in self.devices.values():
            dev_id = device.dev_id
            self.packets += 1
            if not self.gsr.isAvailable(dev_id, 144, 2):
                self.errors += 1
                rospy.logdebug(f'{self.name}: failed to get data for device {dev_id}')
            else:
                device.temperature = self.gsr.getData(dev_id, 146, 1)
                device.voltage = self.gsr.getData(dev_id, 144, 2)


PVE = namedtuple('PVE', ['pos', 'vel', 'eff'])
"""A representaion of position, velocity, efort."""

class DynamixelDevice():
    """A reduced representation of a dynamixel servo. We only include the
    data that we need here."""
    def __init__(self, name, bus, dev_id, inits, **kwargs):
        self.name = name
        self.bus = bus
        self.dev_id = dev_id
        self.inits = inits
        self.current = PVE(0, 0, 0)
        self.goal = PVE(0, 0, 0)
        self.torque_active = False
        self.temperature = 0
        self.voltage = 0
    
    def write(self, reg_num, reg_len, value):
        """Writes a register in a dynamixel. Should only be used for
        initialization purposes."""
        ph = self.bus.dyn_ph
        port = self.bus.dyn_port
        if reg_len == 1:
            func = ph.write1ByteTxRx
        elif reg_len == 2:
            func = ph.write2ByteTxRx
        elif reg_len == 4:
            func = ph.write4ByteTxRx
        else:
            rospy.logerr(f'Invalid register length received: {reg_len}')
            return
        with self.bus.lock:
            res, err = func(port, self.dev_id, reg_num, value)
        if res != 0:
            rospy.loginfo(
                f'Error writing register {reg_num} '
                f'of device {self.dev_id} '
                f'with value {value}')
            rospy.loginfo(f'-- result: {res}')
            rospy.loginfo(f'-- error returned: {ph.getTxRxResult(err)} ({err})')
        return res

    def torque_change(self, state=False):
        """Changes torque for the given devices. It is the responsiblity of
        the caller to make sure that the devices are existing in the controller's
        devices."""
        res = self.write(reg_num=64, reg_len=1, value=int(state))
        if res == 0:
            self.torque_active = state
        return res

class DynamixelBus():
    """A dynamixel bus with settings and devices."""
    def __init__(self, name, port, baud_rate, devices, **kwargs):
        self.name = name
        self.port = port
        self.baud_rate = baud_rate
        self.devices = {}
        self.kwargs = kwargs
        for device_name, device_info in devices.items():
            new_device = DynamixelDevice(name=device_name, bus=self, **device_info)
            self.devices[device_name] = new_device
        self.lock = Lock()

    def open(self):
        self.dyn_port = dyn.PortHandler(self.port)
        rospy.loginfo(f'Opening port {self.port}')
        self.dyn_port.openPort()
        self.dyn_port.setBaudRate(self.baud_rate)
        self.dyn_port.ser.rs485_mode = rs485.RS485Settings()
        self.dyn_ph = dyn.PacketHandler(2.0)
        # setup the communicators
        self.sync_rate = self.kwargs.get('rate', 20.0)
        rate = self.kwargs.get('read_pve', 1.0)
        rospy.loginfo(f'Setting up PVE reader for {self.name} at {self.sync_rate/rate:.1f}Hz')
        self.pve_reader = PVEReader(self, rate)
        rate = self.kwargs.get('read_tv', 100.0)
        rospy.loginfo(f'Setting up TV reader for {self.name} at {self.sync_rate/rate:.1f}Hz')
        self.tv_reader = TVReader(self, rate)

    def start_sync(self):
        rospy.loginfo(f'Starting Sync loop for {self.name} at {self.sync_rate:.1f}Hz')
        self.syncloop = rospy.Timer(rospy.Duration(1.0/self.sync_rate), self.run)

    def run(self, event=None):
        self.pve_reader.run(event)
        self.tv_reader.run(event)

    def close(self):
        rospy.loginfo(f'Stopping SyncLoop for {self.name}')
        self.syncloop.shutdown()
        rospy.loginfo(f'Closing port {self.port}')
        self.dyn_port.closePort()


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
        self.pub_pve = rospy.Publisher('joint_state', JointState, queue_size=1)
        self.pub_temp = rospy.Publisher('temperature', Temperature, queue_size=25)
        self.pub_volt = rospy.Publisher('voltage', BatteryState, queue_size=25)
        # follow joint trajectory action server
        self.fjt_server = actionlib.SimpleActionServer(
            'follow_joint_trajectory', FollowJointTrajectoryAction,
            self.do_follow_joint_trajectory, False)
        # services
        self.torque_srv = rospy.Service('change_torque', ChangeTorque, self.do_change_torque)

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
            rospy.loginfo(f'Initializing {dev_name}')
            if device.inits:
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
        self.pub_stat_timer = rospy.Timer(rospy.Duration(2.0), self.publish_communication_statistics)
        self.pub_pve_timer = rospy.Timer(rospy.Duration(1/20.0), self.publish_joint_states)
        self.pub_temp_timer = rospy.Timer(rospy.Duration(2.0), self.publish_temperature)
        self.pub_volt_timer = rospy.Timer(rospy.Duration(2.0), self.publish_voltage)
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
        self.pub_stat.publish(msg)

    def publish_joint_states(self, event):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        for device_name, device in self.devices.items():
            msg.name.append(device_name)
            # position in radians; factor = 2 * pi / 4095
            raw_pos = device.current.pos
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

    def do_follow_joint_trajectory(self, request):
        """Call-back for follow_joint_trajectory server."""
        rospy.loginfo(f'Goal received: {request}')
        if not self.torque_active:
            result = FollowJointTrajectoryResult(-1, 'torque not active')
            self.fjt_server.set_aborted(result)
            return
        # torque active, execute the request
        # extract the information from the request
        trajectory = request.goal.trajectory
        last_time = 0.0         # keeps track of the time in request
        for point in trajectory.points:
            frame_time = point.time_from_start.to_sec()
            frame_duration = frame_time - last_time
            for index, joint_name in enumerate(trajectory.joint_names):
                if joint_name in self.devices:
                    rad_pos = point.positions[index]
                    # convert to 0-4095
                    raw_pos = rad_pos / 0.001534355386369 + 2047
                    # for velocity we use the profile_velocity setup in time based profile
                    raw_vel = frame_duration * 1000      # dynamixel is in us
                    raw_acc = raw_vel / 4.0
                    self.devices[joint_name]['goal'] = {'position': raw_pos,
                                                        'velocity': raw_vel,
                                                        'acceleration': raw_acc}
                # wait for the
                rospy.sleep(frame_duration)
                last_time = frame_time
        result = FollowJointTrajectoryResult(0, 'reached that goal')
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
