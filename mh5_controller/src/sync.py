import rospy

import dynamixel_sdk as dyn

from device import PVE

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class Sync():
    """A base class for handling communication on the Dyamixel bus. It keeps
    track of the communication statistics (although the subclasses must update
    this information according to the tasks performed) and can package it in
    a ``DiagnosticStatus`` message.

    Parameters
    ----------

    name: str
        The name of the sync object. Used in messages to indicate the
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
        sync. All sync objects will be called by the
        controller on the rate that the controller is configured (ex. 100Hz).
        But each sync can be configured to actually perform the actions
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
            self.comms += 1
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


class PVEReader(Sync):
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
            self.gsr.txRxPacket()
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


class TVReader(Sync):
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
            self.gsr.txRxPacket()
        for device in self.devices.values():
            dev_id = device.dev_id
            self.packets += 1
            if not self.gsr.isAvailable(dev_id, 144, 2):
                self.errors += 1
                rospy.logdebug(f'{self.name}: failed to get data for device {dev_id}')
            else:
                device.temperature = self.gsr.getData(dev_id, 146, 1)
                device.voltage = self.gsr.getData(dev_id, 144, 2)

class PVAWriter(Sync):
    """A SyncWrite communication that writes postition, velocity profile and 
    acceleration profile of Dynamnixel registers.
    """
    def __init__(self, bus, run_every):
        super().__init__(name=f'{bus.name}_pva_writer',
                         bus=bus,
                         devices=bus.devices,
                         run_every=run_every)
        self.gsw = dyn.GroupSyncWrite(bus.dyn_port, bus.dyn_ph, 108, 12)

    def communicate(self):
        # we only process the devices that have torque active
        self.gsw.clearParam()
        has_devices = False
        for device in self.devices.values():
            if device.torque_active:
                data = device.goal.eff.to_bytes(4, byteorder='little') + \
                       device.goal.vel.to_bytes(4, byteorder='little') + \
                       device.goal.pos.to_bytes(4, byteorder='little')
                result = self.gsw.addParam(device.dev_id, data)
                if not result:
                    rospy.loginfo(f'{self.name}: Failed to setup SyncWrite for device {device.dev_id}')
                has_devices = True
        # if no devices are active, skip the loop
        if not has_devices:
            return
        # We will ignore the result of the txRxPacket because it is a stacked
        # result of multiple packet read and is not representative. We will
        # instead use the information per device to keep track of statistics
        with self.bus.lock:
            result = self.gsw.txPacket()
        self.packets += 1
        if result != 0:
            self.errors += 1
            rospy.logdebug(f'{self.name}: Failed to transmit data ')
            rospy.logdebug(f'cerr={self.gsw.ph.getTxRxResult(result)}')
