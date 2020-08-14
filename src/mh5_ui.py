#!/usr/bin/env python

import statistics
import subprocess

import rospy
from sensor_msgs.msg import BatteryState, JointState, Temperature
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from mh5_robot.srv import ChangeTorque, ChangeTorqueResponse

from snack import Grid, GridForm, Label, Listbox, Scale, SnackScreen, Textbox


class MainUI():
    
    def __init__(self):
        # setup the graphics
        self.screen = SnackScreen()
        # for convenience
        self.w = self.screen.width
        self.h = self.screen.height
        # main window
        self.views = {}
        self.current_view = None
        self.done = False

    def add_view(self, view, hot_key, default_view=False):
        self.views[hot_key] = view
        if default_view:
            self.default_view = hot_key

    def change_view(self, hotkey):
        if self.current_view:
            self.current_view.finish()
            self.screen.popWindow()
        self.current_view = self.views[hotkey]
        self.current_view.setup()
        for key in self.views.keys():
            self.current_view.grid.addHotKey(key)
        self.current_view.grid.addHotKey('q')

    def run(self):
        self.change_view(self.default_view)
        while not self.done:
            key = self.current_view.run()
            if key == 'q':
                self.done = True
            if key in self.views:
                self.change_view(key)

        # finish the loop
        self.screen.popWindow()
        self.screen.finish()


class View():
    """Base class for a view."""
    def __init__(self, screen, timer, title):
        self.screen = screen
        self.timer = timer
        self.title = title

    def setup(self):
        """Must be caleed by the MainUi before starting the view. This
        creates all the objects of the UI and initializes them. Sets-up
        a ``GridForm`` of size 1x1 and calls ``create_content`` to fill
        the specific content of the view. Subclasses must implement this
        method. It also revisters the hot keys as are reported by the
        ``hotkeys`` property that must be subclassed if the view needs to
        handle keys."""
        self.grid = GridForm(self.screen, self.title, 1, 1)
        self.content = self.create_content()
        self.grid.add(self.content, 0, 0)
        for key in self.hotkeys:
            self.grid.addHotKey(key)
        if self.timer:
            self.grid.setTimer(self.timer)

    def create_content(self):
        """Should be impelemented in subclasses to produce the desired
        view output."""
        return Textbox(width=20, height=4,
                       text='Default text for the main UI', wrap=1)

    @property
    def hotkeys(self):
        """Returns the keys this view handles. If implemented by subclasses
        then also ``process_hotkey`` should be implemented."""
        return []

    def update_content(self):
        """Handles updates to the content of the view. Normally these are
        triggered by the elpsed timer set up by the ``timer`` property. Should
        be implemented in the subclass according to the desired behaviour."""
        pass

    def process_hotkey(self, key):
        """Processes the declared hotkeys. Should be implemented in subclass."""
        pass

    def run(self):
        """Performs a ``run`` of the grid. First calls the ``update_content``
        to trigger updates to the interface and refreshes the screen. After
        running a ``grid.run()`` it will ask the ``process_key`` method to
        process the hotkey pressed (if any) after whicg it returns the hot key
        to the caller program (tipically the MainUI) so that the loop there
        can process it's own hot keys."""
        self.update_content()
        self.screen.refresh()
        key = self.grid.run()
        self.process_hotkey(key)
        return key

    def finish(self):
        """Provides a way for thge view to clear resources before being
        switched from. For intance views that are displaying information
        from ROS topics have the change to unsubscribe from the topics here.
        """
        pass


class JointView(View):

    def __init__(self, screen, timer, title='Joint State'):
        super().__init__(screen, timer, title)

        self.joint_names = [
            'head_p', 'head_y',
            'r_sho_p', 'r_sho_r', 'r_elb_y', 'r_elb_p',
            'r_hip_y', 'r_hip_p', 'r_hip_r', 'r_kne_p', 'r_ank_p', 'r_ank_r',
            'l_sho_p', 'l_sho_r', 'l_elb_y', 'l_elb_p',
            'l_hip_y', 'l_hip_p', 'l_hip_r', 'l_kne_p', 'l_ank_p', 'l_ank_r']
        self.joint_values = {}
        self.mode = 'r'         # radians

    def create_content(self):
        self.js_subsr = rospy.Subscriber('joint_state', JointState, self.joint_values_call_back)
        self.jt_subsr = rospy.Subscriber('temperature', Temperature, self.joint_temperature_call_back)
        self.jv_subsr = rospy.Subscriber('voltage', BatteryState, self.joint_voltage_call_back)

        lb = Listbox(height=22, width=36)
        for pos in range(22):
            lb.append(f'Text for position {pos:2d}', pos)
        return lb

    def update_content(self):
        for index, joint_name in enumerate(self.joint_names):
            values = self.joint_values.get(joint_name, {})

            if self.mode == 't':
                temp = values.get('temp', 0)
                volt = values.get('volt', 0)
                self.content.replace(f'{joint_name:12s}   {temp:5.1f}°C   {volt:5.1f}V', index)
                continue

            if self.mode == 'd':
                pos = values.get('pos', 0) * 57.29578
                vel = values.get('vel', 0) * 57.29578
                eff = values.get('eff', 0)
                self.content.replace(f'{joint_name:12s} {pos:5.1f}   {vel:5.1f}   {eff:5.1f}', index)
                continue

            if self.mode == 'r':
                pos = values.get('pos', 0)
                vel = values.get('vel', 0)
                eff = values.get('eff', 0)
                self.content.replace(f'{joint_name:12s} {pos:5.2f}   {vel:5.2f}   {eff:5.2f}', index)

    @property
    def hotkeys(self):
        return ['d', 'r', 't']

    def process_hotkey(self, key):
        if key in self.hotkeys:
            self.mode = key

    def joint_values_call_back(self, msg):
        for index, name in enumerate(msg.name):
            if name not in self.joint_values:
                self.joint_values[name] = {}
            self.joint_values[name]['pos'] = msg.position[index]
            self.joint_values[name]['vel'] = msg.velocity[index]
            self.joint_values[name]['eff'] = msg.effort[index]

    def joint_temperature_call_back(self, msg):
        name = msg.header.frame_id
        if name not in self.joint_values:
            self.joint_values[name] = {}
        self.joint_values[name]['temp'] = msg.temperature

    def joint_voltage_call_back(self, msg):
        name = msg.header.frame_id
        if name not in self.joint_values:
            self.joint_values[name] = {}
        self.joint_values[name]['volt'] = msg.voltage

    def finish(self):
        self.js_subsr.unregister()
        self.jt_subsr.unregister()
        self.jv_subsr.unregister()


class NameValueScale():

    def __init__(self, name, unit, grid, row, widths, min_val, max_val):
        self.unit = unit
        self.name = Textbox(widths[0], 1, name)
        grid.setField(self.name, 0, row)
        self.value = Textbox(widths[1], 1, '')
        grid.setField(self.value, 1, row)
        self.min_val = min_val
        self.max_val = max_val
        self.range_val = self.max_val - self.min_val
        self.scale = Scale(widths[2], int(self.range_val * 100))
        grid.setField(self.scale, 2, row)

    def update_value(self, value):
        self.value.setText(f'{value:4.1f}{self.unit}')
        self.scale.set(int((value -self.min_val)*100))

class NameStatValue():

    def __init__(self, name, unit, grid, row, widths):
        self.unit = unit
        self.name = Textbox(widths[0], 1, name)
        grid.setField(self.name, 0, row)
        self.stat = Textbox(widths[1], 1, '')
        grid.setField(self.stat, 1, row)
        self.value = Textbox(widths[2], 1, '')
        grid.setField(self.value, 2, row)

    def update_value(self, stat, value=''):
        self.stat.setText(f'{stat:>4s}{self.unit}')
        self.value.setText(value)


class RobotStatusView(View):

    def __init__(self, screen, timer, title='Robot Status'):
        super().__init__(screen, timer, title)

    def create_content(self):
        grid = Grid(3,19)
        w = [16, 6, 14]         # widths for columns
        row = 0                 # current row
        # Voltage
        grid.setField(Label('Voltage'), 0, row)
        row += 1
        self.battery = NameValueScale('Battery', 'V', grid, row, w, 9.0, 12.6)
        self.battery.update_value(12.2)
        row += 1
        self.rpi5v = NameStatValue('5V rail', 'V', grid, row, w)
        self.rpi5v.update_value('5.0')
        row += 1
        self.rpi3v = NameStatValue('3.3V rail', 'V', grid, row, w)
        self.rpi3v.update_value('3.2')
        row += 1
        grid.setField(Label(''), 0, row)
        # Temperature
        row += 1
        grid.setField(Label('Temperature'), 0, row)
        row += 1
        self.temp = NameValueScale('RPi temperature', '°', grid, row, w, 25.0, 85.0)
        self.temp.update_value(45.0)
        row += 1
        self.fan = NameStatValue('Fan', '', grid, row, w)
        self.fan.update_value('Off')
        row += 1
        grid.setField(Label(''), 0, row)
        # CPU
        row += 1
        grid.setField(Label('CPU'), 0, row)
        row += 1
        self.cpu_1m = NameValueScale('Load [1m]', '', grid, row, w, 0.0, 4.0)
        self.cpu_1m.update_value(2.5)
        row += 1
        self.cpu_5m = NameValueScale('Load [5m]', '', grid, row, w, 0.0, 4.0)
        self.cpu_5m.update_value(2.5)
        row += 1
        self.cpu_15m = NameValueScale('Load [15m]', '', grid, row, w, 0.0, 4.0)
        self.cpu_15m.update_value(2.5)
        row += 1
        max_mem_str = self.shell_cmd('cat /proc/meminfo | grep MemTotal')
        self.max_mem = int(max_mem_str.split()[1]) / 1000000.0
        self.cpu_mem = NameValueScale('Memory', '', grid, row, w, 0.0, self.max_mem)
        self.cpu_mem.update_value(0.0)
        row += 1
        grid.setField(Label(''), 0, row)
        # WiFi
        row += 1
        grid.setField(Label('Wi-Fi'), 0, row)
        row += 1
        # get the interface name from AP config
        ap_config = self.shell_cmd('cat /etc/hostapd/hostapd.conf | grep interface')
        self.ap_interf = ap_config.strip().split('=')[1]
        self.ap_stat = NameStatValue('AP', '', grid, row, w)
        self.ap_stat.update_value('', '')
        row += 1
        nets = self.shell_cmd('ls /sys/class/net/ | grep wl*').split('\n')
        nets.remove(self.ap_interf)
        if nets:
            self.wf_interf = nets[0]
        else:
            self.wf_interf = ''
        self.wf_stat = NameStatValue('WAN', '', grid, row, w)
        self.wf_stat.update_value('On', '')
        row += 1
        self.eth_stat = NameStatValue('Eth', '', grid, row, w)
        self.eth_stat.update_value('', '')
        # row += 1
        # grid.setField(Label(''), 0, row)
        return grid

    def shell_cmd(self, command):
        comm= subprocess.run(command, shell=True, stdout=subprocess.PIPE,
            encoding='utf-8')
        if comm.returncode == 0:
            return comm.stdout.strip()
        else:
            return ''

    def get_interf_status(self, interf):
        inet_line = self.shell_cmd(f'ifconfig {interf} | grep "inet "')
        if inet_line:
            return 'On', inet_line.split(' ')[1]
        else:
            return 'Off', ''

    def update_content(self):
        # voltage
        volt = self.shell_cmd('cat /sys/class/i2c-dev/i2c-1/device/1-0048/in4_input')
        value = int(volt)/1000.0 if volt else 0
        self.rpi3v.update_value(f'{value:4.1f}')
        volt = self.shell_cmd('cat /sys/class/i2c-dev/i2c-1/device/1-0048/in5_input')
        value = int(volt)/500.0 if volt else 0
        self.rpi5v.update_value(f'{value:4.1f}')
        volt = self.shell_cmd('cat /sys/class/i2c-dev/i2c-1/device/1-0048/in6_input')
        value = int(volt)/250.0 if volt else 0
        self.battery.update_value(value)
        # temperature
        temp_str = self.shell_cmd('cat /sys/class/thermal/thermal_zone0/temp')
        self.temp.update_value(int(temp_str)/1000.0)
        fan_str = self.shell_cmd('cat /sys/class/thermal/cooling_device0/cur_state')
        if fan_str == '1':
            self.fan.update_value('On')
        else:
            self.fan.update_value('Off')
        # CPU
        load_str = self.shell_cmd('cat /proc/loadavg').split()
        self.cpu_1m.update_value(float(load_str[0]))
        self.cpu_5m.update_value(float(load_str[1]))
        self.cpu_15m.update_value(float(load_str[2]))        
        mem_str = self.shell_cmd('cat /proc/meminfo | grep MemFree')
        mem = int(mem_str.split()[1]) / 1000000.0
        self.cpu_mem.update_value(self.max_mem - mem)
        # wi-fi
        if self.ap_interf:
            stat, ip_addr = self.get_interf_status(self.ap_interf)
            self.ap_stat.update_value(stat, ip_addr)
        else:
            self.ap_stat.update_value('N/A')

        if self.wf_interf:
            stat, ip_addr = self.get_interf_status(self.wf_interf)
            self.wf_stat.update_value(stat, ip_addr)
        else:
            self.wf_stat.update_value('N/A')

        stat, ip_addr = self.get_interf_status('eth0')
        self.eth_stat.update_value(stat, ip_addr)


class CommStatusView(View):

    def __init__(self, screen, timer, title='Comm Status'):
        super().__init__(screen, timer, title)
        self.stats = {}

    def comms_call_back(self, msg):
        for status in msg.status:
            name = status.name
            perc = 0.0
            packs = 0
            for value in status.values:
                if value.key == 'cum_error_rate_perc':
                    perc = float(value.value)
                if value.key == 'cum_packets':
                    packs = int(int(value.value)/1000)
            self.stats[name] = {'perc': perc, 'packs': packs}

    def create_content(self):
        lb = Listbox(height=18, width=36)
        lb.append('Name                  Packs[k] %Err ', 0)
        for pos in range(1,18):
            lb.append('', pos)
        self.comm_subsr = rospy.Subscriber('communication_statistics', DiagnosticArray, self.comms_call_back)
        return lb

    def update_content(self):
        for index, name in enumerate(self.stats):
            stats = self.stats[name]
            packs = stats['packs']
            perc = stats['perc']
            self.content.replace(f'{name:23s} {packs:6d} {perc:4.1f}%', index + 1)

    def finish(self):
        self.comm_subsr.unregister()


class Menu(View):

    def __init__(self, screen, title):
        super().__init__(screen, 0, title)
        self.navigation = []
        self.current = 'main'
        self.redraw = False
        self.change_torque = rospy.ServiceProxy('change_torque', ChangeTorque)
        self.menus = {
            'main': [
                ('System', self.navigate, ('system',)),
                ('Robot', self.navigate, ('robot',)),
                ('Actions', self.navigate, ('actions',))
            ],
            'system': [
                ('Close ROS', self.close_ros, ()),
                ('Shutdown robot', self.shutdown_robot, ())
            ],
            'robot': [
                ('Torque enable', self.navigate, ('torque_enable',)),
                ('Torque disable', self.navigate, ('torque_disable',))
            ],
            'torque_enable': [
                ('Torque enable head', self.do_change_torque, (True, 'head'))
            ],
            'torque_disable': [
                ('Torque disable head', self.do_change_torque, (False, 'head'))
            ],
            'actions': [
                ('Stand up', self.action_stand_up, ()),
                ('Sit down', self.action_sit_down, ())
            ]
        }

    def create_content(self):
        lb = Listbox(height=18, width=24)
        for index, (menu_item, _, _) in enumerate(self.menus[self.current]):
            lb.append(menu_item, index)
        return lb

    def update_content(self):
        if self.redraw:
            self.content.clear()
            for index, (menu_item, _, _) in enumerate(self.menus[self.current]):
                self.content.append(menu_item, index)
            self.redraw = False

    @property
    def hotkeys(self):
        return ['ESC', 'ENTER']

    def process_hotkey(self, key):
        if key == 'ENTER':
            sel = self.content.current()
            func = self.menus[self.current][sel][1]
            args = self.menus[self.current][sel][2]
            func(*args)
            return None
        if key == 'ESC':
            if self.navigation:
                menu = self.navigation.pop()
                self.current = menu
                self.redraw = True
            return None

    def navigate(self, menu):
        self.navigation.append(self.current)
        self.current = menu
        self.redraw = True

    def close_ros(self):
        pass

    def shutdown_robot(self):
        pass

    def torque_enable(self):
        pass

    def torque_disable(self):
        pass

    def action_stand_up(self):
        pass

    def action_sit_down(self):
        pass

    def do_change_torque(self, state, group):
        result = self.change_torque(state, [], [group])

if __name__ == '__main__':

    rospy.init_node('mh5_edge_ui')

    ui = MainUI()
    ui.add_view(RobotStatusView(ui.screen, 1000, 'MH5 Status'), 's', default_view=True)
    ui.add_view(JointView(ui.screen, 100, 'Joint Status'), 'j')
    ui.add_view(CommStatusView(ui.screen, 100, 'Comm Status'), 'c')
    ui.add_view(Menu(ui.screen, 'MH5 Main Menu'), 'm')
    ui.run()
