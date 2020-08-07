#!/usr/bin/env python

from snack import SnackScreen, GridForm, Textbox, Listbox, Scale, Label
import rospy
from sensor_msgs.msg import Temperature, BatteryState
from sensor_msgs.msg import JointState
import statistics


class MainUI():

    MAX_VOLTAGE = 3 * 4.2      # voltage for 100% charge battery
    MIN_VOLTAGE = 3 * 3.0      # voltage for 0% charge battery
    RANGE_VOLTAGE = MAX_VOLTAGE - MIN_VOLTAGE

    MIN_TEMPERATURE = 25.0       # 25 deg Celsius
    MAX_TEMPERATURE = 80.0       # 80 deg Celsius
    RANGE_TEMPERATURE = MAX_TEMPERATURE - MIN_TEMPERATURE
    
    def __init__(self, update_frequency=10):
        # refresh period in mili-seconds
        self.period = int(1000.0 / update_frequency)
        # setup the graphics
        self.screen = SnackScreen()
        # for convenience
        self.w = self.screen.width
        self.h = self.screen.height
        # main window
        self.view = JointView(self.screen, self.period, self.hot_keys)
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


if __name__ == '__main__':

    rospy.init_node('mh5_edge_ui')
    if rospy.has_param('~rate'):
        rate = rospy.get_param('~rate')
    else:
        rate = 10

    ui = MainUI(rate)
    ui.run()
