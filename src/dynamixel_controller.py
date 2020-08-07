#!/usr/bin/env python

import yaml

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
    def __init__(self, name, port, ph, devices, run_every):
        self.name = name
        self.port = port
        self.ph = ph
        self.devices = devices
        self.run_every = run_every
        # keeps track for statistics
        self.errors = 0
        self.packets = 0
        self.cum_errors = 0
        self.cum_packets = 0
        self.execs = 0

    def setup(self):
        """Prepares the communicator. Must be implemented by the subclass."""
        pass

    def run(self):
        """Runs a cycle. It increments the counts of the executions and if
        matches ``run_every`` then it invokes the actual communication method.
        """
        self.execs += 1
        if self.execs >= self.run_every:
            self.communicate()
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
        if reset:
            self.errors = 0
            self.packets = 0
        return msg


class PVLReader(Communicator):
    """A SyncRead communication that reads postition, velocity and load of
    Dynamnixel registers.
    """
    def __init__(self, controller, run_every):
        super().__init__(name=f'{controller.bus_name}_pvl_reader',
                         port=controller.port,
                         ph=controller.ph,
                         devices=controller.devices,
                         run_every=run_every)
        self.gsr = dyn.GroupSyncRead(controller.port, controller.ph, 126, 10)
        for device_info in controller.devices.values():
            self.gsr.addParam(device_info['id'])
        self.pub = rospy.Publisher('joint_state', JointState, queue_size=1)

    def communicate(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        # We will ignore the result of the txRxPacket because it is a stacked
        # result of multiple packet read and is not representative. We will
        # instead use the information per device to keep track of statistics
        _ = self.gsr.txRxPacket()

        for name, dev_info in self.devices.items():
            dev_id = dev_info['id']
            self.packets += 1
            # position
            if not self.gsr.isAvailable(dev_id, 132, 4):
                self.errors += 1
            else:
                msg.name.append(name)
                # position in radians; factor = 2 * pi / 4095
                raw_pos = self.gsr.getData(dev_id, 132, 4)
                rad_pos = (raw_pos - 2047) * 0.001534355386369
                msg.position.append(rad_pos)
                # angular velocity in rad /s; factor = 2 * pi * 0.229 / 60
                raw_vel = self.gsr.getData(dev_id, 128, 4)
                if raw_vel > 1023:
                    raw_vel = raw_vel - 4294967296
                rad_vel = raw_vel * 0.023980823922402
                msg.velocity.append(rad_vel)       
                # aprox effort in N*m; factor = 1/1000 * 1.4 (stall torque)
                # _very_ aproximate!
                raw_eff = self.gsr.getData(dev_id, 126, 2)
                if raw_eff > 1000:
                    raw_eff = raw_eff - 65536
                nm_eff = raw_eff * 0.0014
                msg.effort.append(nm_eff)
                # save the values in the devices dictionary
                self.devices[name]['present'] = {'position': raw_pos,
                                                 'velocity': raw_vel,
                                                 'effort': raw_eff}
        # publish one message for all joints
        self.pub.publish(msg)


class TVReader(Communicator):
    """A SyncRead communication that reads temperature and voltage of
    Dynamnixel registers.
    """

    MAX_VOLTAGE = 3 * 4.2      # voltage for 100% charge battery
    MIN_VOLTAGE = 3 * 3.0      # voltage for 0% charge battery
    RANGE_VOLTAGE = MAX_VOLTAGE - MIN_VOLTAGE

    def __init__(self, controller, run_every):
        super().__init__(name=f'{controller.bus_name}_tv_reader',
                         port=controller.port,
                         ph=controller.ph,
                         devices=controller.devices,
                         run_every=run_every)
        self.gsr = dyn.GroupSyncRead(controller.port, controller.ph, 144, 3)
        for device_info in controller.devices.values():
            self.gsr.addParam(device_info['id'])
        self.t_pub = rospy.Publisher('temperature', Temperature, queue_size=1)
        self.v_pub = rospy.Publisher('voltage', BatteryState, queue_size=1)

    def communicate(self):
        t_msg = Temperature()
        v_msg = BatteryState()
        # We will ignore the result of the txRxPacket because it is a stacked
        # result of multiple packet read and is not representative. We will
        # instead use the information per device to keep track of statistics
        _ = self.gsr.txRxPacket()
        time = rospy.Time.now()
        t_msg.header.stamp = time
        v_msg.header.stamp = time

        for name, dev_info in self.devices.items():
            dev_id = dev_info['id']
            self.packets += 1
            if not self.gsr.isAvailable(dev_id, 144, 2):
                self.errors += 1
            else:
                # temperature
                t_msg.header.frame_id = name
                # temperature is already in degrees celsius
                raw_temp = self.gsr.getData(dev_id, 146, 1)
                t_msg.temperature = raw_temp
                self.t_pub.publish(t_msg)
                # voltage
                v_msg.header.frame_id = name
                raw_voltage = self.gsr.getData(dev_id, 144, 2)
                v_voltage = raw_voltage / 10.0
                v_msg.voltage = v_voltage
                v_msg.percentage = (v_voltage - self.MIN_VOLTAGE) / self.RANGE_VOLTAGE * 100.0
                self.v_pub.publish(v_msg)


class DynamixelController():
    """Manges exactly one Dynamixel bus and peforms periodically read/write
    to all the devices that are connected to that bus.
    """

    def __init__(self):
        # load config file
        config_file = self.get_param('~config_file')
        with open(config_file, 'r') as f:
            config_items = yaml.load(f, Loader=yaml.FullLoader)
        # extract specfic bus
        self.bus_name = self.get_param('~bus')
        bus = self.get_key(self.bus_name, config_items)
        # bus parameters
        port_path = self.get_key('port', bus)
        baud_rate = self.get_key('baud_rate', bus, default=1000000)
        # devices
        self.devices = self.get_key('devices', bus)
        # main rate
        self.main_rate = self.get_param('~rate', 100)
        self.rate = rospy.Rate(self.main_rate)
        self.execs = 0
        # inits
        self.inits = self.get_key('inits', config_items, {})
        # setup stuff
        self.port = dyn.PortHandler(port_path)
        rospy.loginfo(f'Opening port {port_path}')
        self.port.openPort()
        self.port.setBaudRate(baud_rate)
        self.port.ser.rs485_mode = rs485.RS485Settings()
        self.ph = dyn.PacketHandler(2.0)
        # communicators
        self.pvl_reader = PVLReader(self, self.get_key('read_pvl', bus, 1.0))
        self.tv_reader = TVReader(self, self.get_key('read_tv', bus, 100.0))
        # follow joint trajectory server
        self.fjt_server = actionlib.SimpleActionServer(
            'follow_joint_trajectory', FollowJointTrajectoryAction,
            self.do_follow_joint_trajectory, False)
        # stats publisher
        self.pub_stat = rospy.Publisher(
            'communication_statistics', DiagnosticArray, queue_size=5)
        self.torque_active = False

    def send_all(self, reg_number, reg_len, value):
        """Sends a write message to all devices."""
        if reg_len == 1:
            func = self.ph.write1ByteTxRx
        elif reg_len == 2:
            func = self.ph.write2ByteTxRx
        elif reg_len == 4:
            func = self.ph.write4ByteTxRx
        else:
            rospy.logerr(f'Invalid register length received: {reg_len}')
            return
        for dev_info in self.devices.values():
            res, err = func(self.port, dev_info['id'], reg_number, value)
            if res != 0:
                rospy.loginfo(
                    f'Error writing register {reg_number} '
                    f'of device {dev_info["id"]} '
                    f'with value {value}')
                rospy.loginfo(f'Error returned: {self.ph.getTxRxResult(err)}')

    def torque_off(self):
        self.send_all(reg_number=64, reg_len=1, value=0)

    def torque_on(self):
        self.send_all(reg_number=64, reg_len=1, value=0)
        # update all goal info to the latest current:
        for device in self.devices:
            present = device['present']
            device['goal'] = {'position': present['position'],
                              'velocity': 0,
                              'acceleration': 0}

    def start(self):
        # torque off; just to make sure
        self.torque_off()
        self.torque_active = False
        # initialize devices
        for name, dev_info in self.devices.items():
            rospy.loginfo(f'Initializing {name}')
            if 'inits' in dev_info:
                for init in dev_info['inits']:
                    if init not in self.inits:
                        rospy.loginfo(
                            f'Init {init} specified for device {name} '
                            f'does not exist. Will be ignored.')
                    else:
                        for reg_num, values in self.inits[init].items():
                            self.send_all(
                                reg_number=reg_num,
                                reg_len=values[0],
                                value=values[1])
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

    def get_key(self, key, params, default=None):
        if key in params:
            return params[key]
        if default is not None:
            rospy.loginfo(f'Default value {default} will be used for '
                          f'config parameter {key}')
            return default
        rospy.logfatal(f'Parameter {key} mising in configuration file '
                       'and no default value available')
        exit()

    def run(self):
        """Runs one pass. Keeps track of executions to produce the statistics.
        """
        # runs the communications
        self.pvl_reader.run()
        self.tv_reader.run()
        # stats
        self.execs += 1
        if self.execs >= self.main_rate:
            msg = DiagnosticArray()
            msg.header.stamp = rospy.Time.now()
            msg.status.append(self.pvl_reader.stats_as_msg())
            msg.status.append(self.tv_reader.stats_as_msg())
            self.pub_stat.publish(msg)
            self.execs = 0

    def finish(self):
        if self.torque_active:
            rospy.loginfo('Turning torque off...')
            self.torque_off()
        rospy.loginfo(f'Closing port {self.port.port_name}')
        self.port.closePort()

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


if __name__ == '__main__':

    rospy.init_node('dynamixel_controller')
    controller = DynamixelController()
    controller.start()

    while not rospy.is_shutdown():

        controller.run()
        controller.rate.sleep()

    controller.finish()
