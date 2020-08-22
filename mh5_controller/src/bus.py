
import rospy
from threading import Lock
import dynamixel_sdk as dyn
from serial import rs485

from device import DynamixelDevice
from sync import PVEReader, TVReader, PVAWriter


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
        rate = self.kwargs.get('write_pva', 1.0)
        rospy.loginfo(f'Setting up PVA writer for {self.name} at {self.sync_rate/rate:0.1f}Hz')
        self.pva_writer = PVAWriter(self, rate)

    def start_sync(self):
        rospy.loginfo(f'Starting Sync loop for {self.name} at {self.sync_rate:.1f}Hz')
        self.syncloop = rospy.Timer(rospy.Duration(1.0/self.sync_rate), self.run)

    def run(self, event=None):
        self.pve_reader.run(event)
        self.tv_reader.run(event)
        self.pva_writer.run(event)

    def close(self):
        rospy.loginfo(f'Stopping SyncLoop for {self.name}')
        self.syncloop.shutdown()
        rospy.loginfo(f'Closing port {self.port}')
        self.dyn_port.closePort()
