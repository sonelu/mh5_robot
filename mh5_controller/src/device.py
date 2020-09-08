import rospy

from collections import namedtuple

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
        self.offset = kwargs.get('offset', 0)

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
        if state:
            # if we turn on the torque we set the goal to be the
            # current possition and some default profile to avoid
            # the servo from going sudenly to 0 when the SyncWrite starts
            # replicating the goal
            self.goal = PVE(self.current.pos, 50, 25)
        res = self.write(reg_num=64, reg_len=1, value=int(state))
        if res == 0:
            self.torque_active = state
        return res
