#!/usr/bin/env python

import statistics
import subprocess

import rospy
from sensor_msgs.msg import BatteryState, JointState, Temperature
from snack import Grid, GridForm, Label, Listbox, Scale, SnackScreen, Textbox


class MainUI():
    
    def __init__(self, update_frequency=10):
        # refresh period in mili-seconds
        self.period = int(1000.0 / update_frequency)
        # setup the graphics
        self.screen = SnackScreen()
        # for convenience
        self.w = self.screen.width
        self.h = self.screen.height
        # main window
        self.view = RobotStatusView(self.screen, self.period, self.hot_keys, 'Robot Status')
        # self.grid = GridForm(self.screen, 'Title', 1, 1)
        # self.content = Textbox(20, 2,
        #                        text='Default text for the main UI', wrap=1)
        # self.grid.add(self.content, 0, 0)
        # self.grid.addHotKey('q')
        # self.grid.setTimer(self.period)
        # self.shortcuts = {'q': self.set_done}
        # self.content_update_callback = None
        # self.status = Grid(2,2)
        # self.voltages = {}          # voltages reported by servos
        # self.battery = 12.0         # will be updated by node
        # self.batt_label = Label('')
        # self.batt_scale = Scale(18, self.RANGE_VOLTAGE)
        # self.update_battery_status(12.0)
        # self.temperature = 55.0     # will be updated by node
        # self.temp_label = Label('')
        # self.temp_scale = Scale(18, self.RANGE_TEMPERATURE)
        # self.update_temperature_status(55.0)
        # self.status.setField(self.batt_label, 0, 0)
        # self.status.setField(self.batt_scale, 0, 1)
        # self.status.setField(self.temp_label, 1, 0)
        # self.status.setField(self.temp_scale, 1, 1)
        # subscribe to get temp and voltage updates
        # self.temp_subcr = rospy.Subscriber('temperature', Temperature, self.new_temperature, queue_size=5)
        # self.volt_subsr = rospy.Subscriber('voltage', BatteryState, self.new_voltage, queue_size=5)
        self.done = False

    def change_view(self, new_view):
        self.view.finish()
        self.screen.popWindow()
        self.view = new_view

    @property
    def hot_keys(self):
        return ['q']

    # def update_battery_status(self, battery_value):
    #     self.battery = battery_value
    #     self.batt_label.setText(f'Battery:   ({battery_value:4.1f}V)')
    #     self.batt_scale.set(max(0, battery_value - self.MIN_VOLTAGE))

    # def update_temperature_status(self, temperature_value):
    #     self.temperature = temperature_value
    #     self.temp_label.setText(f'Temp.  :   ({temperature_value:4.1f}℃')
    #     self.temp_scale.set(max(0, temperature_value - self.MIN_TEMPERATURE))

    # def new_voltage(self, msg):
    #     name = msg.header.frame_id
    #     voltage = msg.voltage
    #     self.voltages[name] = voltage

    # def new_temperature(self, msg):
    #     pass

    def run(self):
        while not self.done:
            key = self.view.run()
            if key == 'q':
                self.done = True

        # finish the loop
        self.screen.popWindow()
        self.screen.finish()


class View():

    def __init__(self, screen, timer, master_hotkeys, title):
        self.screen = screen
        self.grid = GridForm(screen, title, 1, 1)
        self.content = self.create_content()
        self.grid.add(self.content, 0, 0)
        for key in master_hotkeys:
            self.grid.addHotKey(key)
        for key in self.hotkeys:
            self.grid.addHotKey(key)
        self.grid.setTimer(timer)


    def create_content(self):
        return Textbox(width=20, height=4,
                       text='Default text for the main UI', wrap=1)

    @property
    def hotkeys(self):
        return []

    def update_content(self):
        pass

    def process_hotkey(self, key):
        pass

    def run(self):
        self.update_content()
        self.screen.refresh()
        key = self.grid.run()
        self.process_hotkey(key)
        return key

    def finish(self):
        pass


class JointView(View):

    def __init__(self, screen, timer, master_hotkeys, title='Joint State'):
        super().__init__(screen, timer, master_hotkeys, title)

        self.joint_names = [
            'head_p', 'head_y',
            'r_sho_p', 'r_sho_r', 'r_elb_y', 'r_elb_p',
            'r_hip_y', 'r_hip_p', 'r_hip_r', 'r_kne_p', 'r_ank_p', 'r_ank_r',
            'l_sho_p', 'l_sho_r', 'l_elb_y', 'l_elb_p',
            'l_hip_y', 'l_hip_p', 'l_hip_r', 'l_kne_p', 'l_ank_p', 'l_ank_r']
        self.joint_values = {}
        self.mode = 'r'         # radians
        self.js_subsr = rospy.Subscriber('joint_state', JointState, self.joint_values_call_back)
        self.jt_subsr = rospy.Subscriber('temperature', Temperature, self.joint_temperature_call_back)
        self.jv_subsr = rospy.Subscriber('voltage', BatteryState, self.joint_voltage_call_back)

    def create_content(self):
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

    def __init__(self, screen, timer, master_hotkeys, title='Joint State'):
        super().__init__(screen, timer, master_hotkeys, title)
        self.grid.setTimer(1000)

    def create_content(self):
        grid = Grid(3,17)
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
        self.cpu_load = NameValueScale('Load', '', grid, row, w, 0.0, 4.0)
        self.cpu_load.update_value(2.5)
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
        # temperature
        temp_str = self.shell_cmd('cat /sys/class/thermal/thermal_zone0/temp')
        self.temp.update_value(int(temp_str)/1000.0)
        fan_str = self.shell_cmd('cat /sys/class/thermal/cooling_device0/cur_state')
        if fan_str == '1':
            self.fan.update_value('On')
        else:
            self.fan.update_value('Off')
        # CPU
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


if __name__ == '__main__':

    rospy.init_node('mh5_edge_ui')
    if rospy.has_param('~rate'):
        rate = rospy.get_param('~rate')
    else:
        rate = 10

    ui = MainUI(rate)
    ui.run()
