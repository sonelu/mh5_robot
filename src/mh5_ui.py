from snack import SnackScreen, Grid, Textbox, Listbox, Scale, Label
import rospy
from sensor_msgs.msg import Temperature, BatteryState
import statistics


class MainUI():

    MAX_VOLTAGE = 3 * 4.2      # voltage for 100% charge battery
    MIN_VOLTAGE = 3 * 3.0      # voltage for 0% charge battery
    RANGE_VOLTAGE = MAX_VOLTAGE - MIN_VOLTAGE

    MIN_TEMPERATURE = 25.0       # 25 deg Celsius
    MAX_TEMPERATURE = 80.0       # 80 deg Celsius
    RANGE_TEMPERATURE = MAX_TEMPERATURE - MIN_TEMPERATURE
    
    def __init__(self, controller):
        # setup the graphics
        self.screen = SnackScreen()
        self.controller = controller
        # main window
        self.grid = Grid(1, 2)
        self.status = Grid(2,2)
        self.voltages = {}          # voltages reported by servos
        self.battery = 12.0         # will be updated by node
        self.batt_label = Label('')
        self.batt_scale = Scale(18, self.RANGE_VOLTAGE)
        self.update_battery_status(12.0)
        self.temperature = 55.0     # will be updated by node
        self.temp_label = Label()
        self.temp_scale = Scale(18, self.RANGE_TEMPERATURE)
        self.update_temperature_status(55.0)
        self.status.setField(self.batt_label, 0, 0)
        self.status.setField(self.batt_scale, 0, 1)
        self.status.setField(self.temp_label, 1, 0)
        self.status.setField(self.temp_scale, 1, 1)
        self.set_content()
        # subscribe to get temp and voltage updates
        self.temp_subcr = rospy.Subscriber('temperature', Temperature, new_temperature, queue_size=5)
        self.volt_subsr = rospy.Subscriber('voltage', BatteryState, new_voltage, queue_size=5)


    def set_content(self, content=None, title='<No Title>', shortcuts=None, content_update_callback=None):
        self.screen.popWindow()
        if content:
            self.content = content
        else:
            self.content = Textbox(
                width=self.screen.width - 2, 
                height=self.screen.height - 6,
                text='Default text for the main UI',
                wrap=1)
        if shortcuts:
            self.shortcuts = shortcuts
        else:
            self.shortcuts = '[h]elp'
        self.content_update_callback = content_update_callback
        # update the view
        self.grid.setField(self.content, 0, 0)
        self.grid.setField(self.status, 0, 1)
        self.screen.gridWrappedWindow(self.g, title)

    def update_battery_status(self, battery_value):
        self.battery = battery_value
        self.batt_label.setText(f'Battery:   ({battery_value:4.1f}V)')
        self.batt_scale.set(max(0, battery_value - self.MIN_VOLTAGE))

    def update_temperature_status(self, temperature_value):
        self.temperature = temperature_value
        self.temp_label.setText(f'Temp.  :   ({temperature_value:4.1f}℃')
        self.temp_scale.set(max(0, temperature_value - self.MIN_TEMPERATURE))

    def new_voltage(self, msg):
        name = msg.header.frame_id
        voltage = msg.voltage
        self.voltages[name] = voltage

    def new_temperature(self, msg):
        pass

    def refresh(self):
        # update the voltage
        voltage = statistics.median(self.voltages.values())
        self.update_battery_status(voltage)
        # update the temperature

        # update the main view
        if self.content_update_callback
            self.content_update_callback()

        self.screen.refresh()

    def run(self):
        while True:
            self.refresh()
            ret = self.grid.run()       # will only stay until timer kicks in
            if ret in 
