import time
import struct
from random import random, randint
from collections import deque
from math import log10, cos
from statistics import mean, median, mode
import posix_ipc
import json

# Kivy
from kivy.app import App
from kivy.config import Config
from kivy.metrics import dp
from kivy.core.window import Window
from kivy.clock import Clock
from kivy.base import runTouchApp
from kivy.lang import Builder
from kivy.properties import ListProperty, NumericProperty, StringProperty, ObjectProperty
from kivy.graphics import Color, Ellipse, Line
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.settings import SettingsWithSidebar, SettingsWithSpinner
from kivy.gesture import Gesture, GestureDatabase
from glock import check


# Message queue parameters
RXQUEUE_NAME  = "/rf22b_server_tx"
MAX_RXMSG_SIZE = 255

wssettings = json.dumps([
    {'type': 'title',
     'title': 'Home Weather Station'},
    {'type': 'bool',
     'title': 'Small mode',
     'desc': 'Reduce app window size',
     'section': 'general',
     'key': 'small_mode'},
     {'type': 'string',
     'title': 'RX message queue',
     'desc': 'The message queue name for receiving weather data',
     'section': 'general',
     'key': 'rxqueue_name'},    
    {'type': 'numeric',
     'title': 'Wind direction steps',
     'desc': 'Number of steps for wind direction angle (integer)',
     'section': 'calibration',
     'key': 'wind_direction_steps'},
    {'type': 'numeric',
     'title': 'North direction index',
     'desc': 'Index 1..<Wind direction steps> for the North direction (integer)',
     'section': 'calibration',
     'key': 'north_index'},
    {'type': 'options',
     'title': 'An options setting',
     'desc': 'Options description text',
     'section': 'example',
     'key': 'optionsexample',
     'options': ['option1', 'option2', 'option3']}])


class ClientMq(object):

    # The message queue
    mqRX = None

    def __init__(self, rxqueue_name):
        # Create the message queue
        # The umask is not relevant for reading, when the queue has been created as RW!
        #old_umask = os.umask(0)
        #os.umask(old_umask)
        if hasattr(posix_ipc, 'MessageQueue'):
            try:
                self.mqRX = posix_ipc.MessageQueue(rxqueue_name, posix_ipc.O_RDONLY | posix_ipc.O_CREAT)
                # Request notifications
                #mqRX.request_notification((self.process_notification, mqRX))
            except:
                self.mqRX = None
                pass
        else:
            self.mqRX = None


    def process_notification(self,mq):

        s, p = mq.receive()
        sd = s.decode()

        # Re-register for notifications
        mq.request_notification((self.process_notification, mq))

        #print("Message received: %s (%d) - %s\n" % (s, p, sd))

    def read_weather_data(self, timeout_sec=None):
       
        # The local dictionary for storing the most recent weather data
        _weather_data = {}

        # Return if no RX message queue
        if self.mqRX is None:
            return None

        # Receive from the MessageQueue     
        try:
            msg, pri = self.mqRX.receive(timeout=timeout_sec)
        except posix_ipc.BusyError:
            return None

        # Unpack header info    
        _unpacked_msgheader = struct.unpack('HHHHHHBBBBBb', msg[:18])
        _weather_data["Header"] = list(_unpacked_msgheader)

        # Show raw data
        #print(F"Message ({p:d}):")
        #print(unpacked_msgheader)
        #print(":".join("{:1s}".format(chr(c)) for c in msg[18:38]))

        # Decode the 20 bytes weather data (from the RF22B/Arduino)
        if _weather_data["Header"][11] == 20:
            ii = 18
            lng = 0
            while ii < 38:
                if chr(msg[ii]) == 'N':
                    val = msg[ii+1]
                    lng = 1

                    #print(F"{chr(msg[ii]):1s}: {val:1d}")
                    _weather_data[F"{chr(msg[ii]):1s}"] = val
                
                elif chr(msg[ii]) in ['S', 'T', 'P', 'H']:
                    if chr(msg[ii]) is 'P':
                        val_d = bytearray(msg[ii+1:ii+6])
                        lng = 5
                    else:
                        val_d = bytearray(msg[ii+1:ii+4])                    
                        lng = 3

                    if val_d[0] == 0x20:
                        val_d[0] = 0x30

                    val = 0
                    for dd in range(lng):
                        val += (val_d[lng-dd-1]-0x30)*10**(dd-1)   

                    #print(F"{chr(msg[ii]):1s}: {val:.1f}")
                    _weather_data[F"{chr(msg[ii]):1s}"] = val

                ii += (lng+1)

        # Initial message with start-up info (string)
        else:
            _weather_data["IniMsg"] = "".join("{:1s}".format(chr(c)) for c in msg[18:(18+_weather_data["Header"][11])])

        return _weather_data

class MyScreenManager(ScreenManager):    

    screen_traces = ObjectProperty(None)

    # The dictionary storing the most recent weather data read from the message queue 
    weather_data = {'Header': tuple(), 'IniMsg': None, 'N': 0,'S':0, 'T':0, 'P':0,'H':0}
 
    # Dictionary of deques with weather data for the past ~15 minutes in steps corresponding to the received rate (~15*12 values)
    weather_data_trace15 = {}
    weather_data_trace15["Time"] = deque([],180)
    weather_data_trace15["Rssi"] = deque([],180)
    weather_data_trace15["N"] = deque([],180)
    weather_data_trace15["T"] = deque([],180)
    weather_data_trace15["S"] = deque([],180)
    weather_data_trace15["P"] = deque([],180)
    weather_data_trace15["H"] = deque([],180)

    # Dictionary of deques with weather data for the past 24 hours in 15 minutes steps (24*4 values)
    weather_data_trace24 = {}
    weather_data_trace24["Time"] = deque([],96)
    weather_data_trace24["Rssi"] = deque([],96)
    weather_data_trace24["N"] = deque([],96)
    weather_data_trace24["T"] = deque([],96)
    weather_data_trace24["S"] = deque([],96)
    weather_data_trace24["P"] = deque([],96)
    weather_data_trace24["H"] = deque([],96)

    # Settings (configurable, see app.on_config_change(...))
    smallMode    = False
    windDirSteps = 16 #1..16
    northIndex   = 16
    rxqueueName  = RXQUEUE_NAME

    # The message queue
    _msg_queue = None

    # Displayed info and their color
    #(access with root.manager.* from the kv file)
    _wind_direction = NumericProperty(22.5)
    _wind_speed = NumericProperty(0.11)
    _wind_speed_trace24 = ListProperty([])
    # HSV: Hue = degrees/360
    #     Red: 0 and 60 degrees.
    #     Yellow: 61 and 120 degrees.
    #     Green: 121 and 180 degrees.
    #     Cyan: 181 and 240 degrees.
    #     Blue: 241 and 300 degrees.
    #     Magenta: 301 and 360 degrees.
    _wind_speed_hue = NumericProperty(160)
    _wind_speed_color = ListProperty([0,1,0])
    _air_temperature = NumericProperty(23.5)
    _air_temperature_color = ListProperty([0,1,0])
    _air_pressure = NumericProperty(1013.25)
    _air_pressure_color = ListProperty([0,1,0])
    _air_relhumidity = NumericProperty(50.0)
    _air_relhumidity_color = ListProperty([0,1,0])
    _rssi_dBm = NumericProperty(-66)

    # Time stamps (floating seconds)
    _start_secs = 0
    _crt_secs = 0

    # Current time as string
    _crt_time_str = StringProperty(time.asctime(time.localtime(time.time())))


    def __init__(self, **kwargs):
        super(MyScreenManager, self).__init__(**kwargs)
        _update_mainscreen_sch = Clock.schedule_interval(self.update_mainscreen_info, 5.0)

        # Create the receive message queue
        # _msg_queue.mqRX is set to None when no MessageQueue is available (e.g. MacOS)!
        self._msg_queue = ClientMq(self.rxqueueName)

    def update_mainscreen_info(self, *args):

        # Current time as string
        self._crt_time_str = time.asctime(time.localtime(time.time()))

        # Get real data from the remote weather station (via RF22B and local MessageQueue)
        if self._msg_queue.mqRX is not None:

            _weather_data = self._msg_queue.read_weather_data(1.0)
            if _weather_data is not None:

                # Data received: update info
                if _weather_data["Header"][11] == 20:
                    self.weather_data["IniMsg"] = None

                    # Adjust North direction based on calibration data
                    _weather_data["N"]     = _weather_data["N"] - self.northIndex
                    if _weather_data["N"] <= 0:
                        _weather_data["N"] += self.windDirSteps 
                    self._wind_direction   = (360/self.windDirSteps) * _weather_data["N"]

                    self._air_temperature  = _weather_data["T"]           
                    self._wind_speed       = _weather_data["S"]
                    self._air_pressure     = _weather_data["P"]
                    self._air_relhumidity  = _weather_data["H"]
                
                else:
                    self.weather_data["IniMsg"] = _weather_data["IniMsg"]

                # Header info
                _weather_data["Header"][5] += 1900
                _weather_data["Header"][10] -= 256
                self.weather_data["Header"] = _weather_data["Header"]

                # RSSI
                self._rssi_dBm = _weather_data["Header"][10]

            else:
                # Data not received: keep last reading
                # Header info
                struc_t = time.localtime(time.time())
                self.weather_data["Header"] = (
                    struc_t.tm_sec,
                    struc_t.tm_min,
                    struc_t.tm_hour,
                    struc_t.tm_mday,
                    struc_t.tm_mon,
                    struc_t.tm_year,
                    0,0,0,0, self._rssi_dBm, 20)
                self.weather_data["IniMsg"] = 'No Data'


        # Generate random test data        
        else:
            
            if random() > 0.5:
                self._air_temperature += 2*random()
            else:
                self._air_temperature -= 2*random()

            if random() > 0.5:
                self._air_pressure += 2*random()
            else:
                self._air_pressure -= 2*random()

            if random() > 0.5:
                self._air_relhumidity += random()
            else:
                self._air_relhumidity -= random()

            if random() > 0.5:
                self._wind_speed = 25*random()

            if random() > 0.5:
                self._wind_direction = (360/self.windDirSteps) * randint(1, self.windDirSteps)
        
            if random() > 0.5:
                self._rssi_dBm = -(60+42*random())

            # Header info
            struc_t = time.localtime(time.time())
            self.weather_data["Header"] = (
                struc_t.tm_sec,
                struc_t.tm_min,
                struc_t.tm_hour,
                struc_t.tm_mday,
                struc_t.tm_mon,
                struc_t.tm_year,
                0,0,0,0,self._rssi_dBm, 20)
            self.weather_data["IniMsg"] = None


        # Store last weather data reading   
        self.weather_data["N"] = self._wind_direction  
        self.weather_data["T"] = self._air_temperature
        self.weather_data["S"] = self._wind_speed
        self.weather_data["P"] = self._air_pressure
        self.weather_data["H"] = self._air_relhumidity

        print(self.weather_data)

        # Process and store weather data       
        if self.weather_data["IniMsg"] is None:
            
            if self.procstore_weather_data() is not None:

                # Update the 24 hours traces info every ~15 minutes
                self.get_screen('traces24').update_trace_plots(self.weather_data_trace24)

                print(self.weather_data_trace24)

            # Update the 15 minutes traces info
            self.get_screen('traces15').update_trace_plots(self.weather_data_trace15)

        # Set the info display colors depending on their value/range
        if self._air_temperature < -10:
            self._air_temperature_color = [0,0,1]
        elif self._air_temperature < 0 and self._air_temperature > -10:
            self._air_temperature_color = [0,0.5,0.5]
        elif self._air_temperature > 0 and self._air_temperature < 20:
            self._air_temperature_color = [0,1,0]
        elif self._air_temperature > 20 and self._air_temperature < 35:
            self._air_temperature_color = [0.5,0.5,0]
        elif self._air_temperature > 35:
            self._air_temperature_color = [1,0,0]

        self._wind_speed_hue = (160-200*log10(1+self._wind_speed/5))/360
        #self._wind_speed_color = Color(self._wind_speed_hue, 1, 1, mode='hsv'))

    # Process and store weather data
    def procstore_weather_data(self):

        # Update trace15 deques
        self._crtTime = (
            self.weather_data["Header"][5], 
            self.weather_data["Header"][4], 
            self.weather_data["Header"][3], 
            self.weather_data["Header"][2], 
            self.weather_data["Header"][1], 
            self.weather_data["Header"][0], 
            0, 0, 0)
        self._crt_secs = time.mktime(self._crtTime)

        self.weather_data_trace15["Time"].append(self._crtTime)
        self.weather_data_trace15["Rssi"].append(self.weather_data["Header"][10])
        self.weather_data_trace15["N"].append(self.weather_data["N"])
        self.weather_data_trace15["T"].append(self.weather_data["T"])
        self.weather_data_trace15["S"].append(self.weather_data["S"])
        self.weather_data_trace15["P"].append(self.weather_data["P"])
        self.weather_data_trace15["H"].append(self.weather_data["H"])

        if len(self.weather_data_trace15["Time"]) < 2:
            self._start_secs = time.mktime((
                self.weather_data["Header"][5], 
                self.weather_data["Header"][4], 
                self.weather_data["Header"][3], 
                self.weather_data["Header"][2], 
                self.weather_data["Header"][1], 
                self.weather_data["Header"][0], 
                0, 0, 0))
            return None

        # At the end of each ~15 minutes time window:
        # - Calculate average values over the past ~15 minutes (all values in the trace15 deque)
        # - Clear trace15 deque and append the current value
        # - Append dictionary with average values to the trace24 deques
        _TimeAvg = None
        if self._crt_secs - self._start_secs >= 900:

            # Update the trace24 deques with the average values of the past ~15 minutes
            # The wind direction 'average' is the most frequent direction
            # The time stamp 'average' is the mid-time
            struc_t = time.localtime(self._start_secs+450)
            _TimeAvg = (struc_t.tm_year,
                struc_t.tm_mon,
                struc_t.tm_mday,
                struc_t.tm_hour,
                struc_t.tm_min,
                struc_t.tm_sec,
                0,0,0)
            self.weather_data_trace24["Time"].append(_TimeAvg)
            self.weather_data_trace24["N"].append(mode(self.weather_data_trace15["N"]))
            self.weather_data_trace24["T"].append(mean(self.weather_data_trace15["T"]))
            self.weather_data_trace24["S"].append(mean(self.weather_data_trace15["S"]))
            self.weather_data_trace24["P"].append(mean(self.weather_data_trace15["P"]))
            self.weather_data_trace24["H"].append(mean(self.weather_data_trace15["H"]))
            self.weather_data_trace24["Rssi"].append(mean(self.weather_data_trace15["Rssi"]))

            # Store first values of the new 15 minutes window in the trace15 deques
            for k in self.weather_data_trace15:
                self.weather_data_trace15[k].clear()

            self._crtTime = (
                self.weather_data["Header"][5], 
                self.weather_data["Header"][4], 
                self.weather_data["Header"][3], 
                self.weather_data["Header"][2], 
                self.weather_data["Header"][1], 
                self.weather_data["Header"][0], 
                0, 0, 0)
            self._start_secs = time.mktime(self._crtTime)

            self.weather_data_trace15["Time"].append(self._crtTime)
            self.weather_data_trace15["N"].append(self.weather_data["N"])
            self.weather_data_trace15["T"].append(self.weather_data["T"])
            self.weather_data_trace15["S"].append(self.weather_data["S"])
            self.weather_data_trace15["P"].append(self.weather_data["P"])
            self.weather_data_trace15["H"].append(self.weather_data["H"])
            self.weather_data_trace15["Rssi"].append(self.weather_data["Header"][10])

        # Current time
        #time_secs = time.mktime(time.localtime(time.time()))

        return _TimeAvg
 
    def north_check(self):
        '''Check the North direction index vs. number of wind direction steps'''
        if self.northIndex > self.windDirSteps:
            self.northIndex = self.windDirSteps

    #def display_settings(self, settings):
    #    self.back_screen_name = self.current
    #    self.current = 'lock'


class TracesScreen(Screen):
    '''Display traces of weather data'''

    # The plot areas
    widget_air_temp   = ObjectProperty(None)
    widget_wind_speed = ObjectProperty(None)
    widget_air_press  = ObjectProperty(None)
    widget_air_relhum = ObjectProperty(None)
    progress_bar      = ObjectProperty(None)

    # The plot traces
    _start_time_str         = StringProperty(time.asctime(time.localtime(time.time())))
    _end_time_str           = StringProperty(time.asctime(time.localtime(time.time())))
    _wind_direction_points  = ListProperty([])
    _air_temperature_points = ListProperty([])
    _wind_speed_points      = ListProperty([])
    _air_pressure_points    = ListProperty([])
    _air_relhumidity_points = ListProperty([])
    _rssi_points            = ListProperty([])

    # Sets the transparency of the background color for each trace widget
    _widget_visible = 0.0

    # The padding_x value for the child widgets
    _x_padding = dp(30) 

    # The x axis offset (common for all traces)
    _x_offset = dp(100) 

    def update_trace_plots(self, weather_data_trace):
        '''Update the plots for all traces'''

        # The x axis step (common for all traces)     
        self._x_step   = (self.widget_air_temp.width - self._x_offset - dp(20))/weather_data_trace["Time"].maxlen

        # The time labels
        self._start_time_str = time.asctime(weather_data_trace["Time"][0])    
        self._end_time_str   = time.asctime(weather_data_trace["Time"][-1])

        # The trace data values
        for k in weather_data_trace:
            if k is not "Time":
                if k is "N":
                    self._update_wind_direction_plot(weather_data_trace[k])
                elif k is "T":
                    self._update_air_temperature_plot(weather_data_trace[k])
                elif k is "S":
                    self._update_wind_speed_plot(weather_data_trace[k])
                elif k is "P":
                    self._update_air_pressure_plot(weather_data_trace[k])
                elif k is "H":
                    self._update_air_relhumidity_plot(weather_data_trace[k])
                elif k is "Rssi":
                    self._update_rssi_plot(weather_data_trace[k])
            
        # Update the progress bar
        self.progress_bar.min = 0
        self.progress_bar.max = weather_data_trace["Time"].maxlen
        self.progress_bar.value += 1
                    
    def _update_wind_direction_plot(self, y_values):  
        '''Update the wind direction trace plot'''

        # TODO
        self._wind_direction_points = [] 

    def _update_rssi_plot(self, y_values):  
        '''Update the RSSI trace plot'''

        # TODO
        self._rssi_points = [] 


    def _update_air_temperature_plot(self, y_values):  
        '''Update the air temperature trace plot'''

        # The x-axis
        _x_off = self.widget_air_temp.pos[0] + self._x_offset
        x_axis = [_x_off + i*self._x_step for i in range(len(y_values))]
        
        # The y offset and scale
        _y_sc = self.widget_air_temp.height/60 
        _y_off = self.widget_air_temp.pos[1] + self.widget_air_temp.height/3 
         
        # The points to plot
        points = [0 for i in range(2*len(y_values))]
        points[0::2] = x_axis
        points[1::2] = [_y_off + _y_sc*y for y in y_values]
        self._air_temperature_points = points

    def _update_wind_speed_plot(self, y_values):  
        '''Update the wind speed trace plot'''

        # The x-axis
        _x_off = self.widget_wind_speed.pos[0] + self._x_offset
        x_axis = [_x_off + i*self._x_step for i in range(len(y_values))]
        
        # The y offset
        _y_sc = self.widget_wind_speed.height/35.0
        _y_off = self.widget_wind_speed.pos[1] + dp(2)   

        # The points to plot
        points = [0 for i in range(2*len(y_values))]
        points[0::2] = x_axis
        points[1::2] = [_y_off + _y_sc*y for y in y_values]
        self._wind_speed_points = points

    def _update_air_pressure_plot(self, y_values):  
        '''Update the air pressure trace plot'''

        # The x-axis
        _x_off = self.widget_air_press.pos[0] + self._x_offset
        x_axis = [_x_off + i*self._x_step for i in range(len(y_values))]
        
        # The y offset and scale
        # The atm unit is roughly equivalent to the mean sea-level atmospheric pressure on Earth, 
        # that is, the Earth's atmospheric pressure at sea level is approximately 1 atm = 1013.25 mbar
        _y_sc = self.widget_wind_speed.height/20.0
        _y_off = self.widget_air_press.pos[1] + 0.5*self.widget_air_press.height - _y_sc*1013.25
          

        # The points to plot
        points = [0 for i in range(2*len(y_values))]
        points[0::2] = x_axis
        points[1::2] = [_y_off + _y_sc*y for y in y_values]
        self._air_pressure_points = points 

    def _update_air_relhumidity_plot(self, y_values):  
        '''Update the air relative humidity trace plot'''

        # The x-axis
        _x_off = self.widget_air_relhum.pos[0] + self._x_offset
        x_axis = [_x_off + i*self._x_step for i in range(len(y_values))]
        
        # The y offset and scale
        _y_sc = self.widget_wind_speed.height/40
        _y_off = self.widget_air_relhum.pos[1] + 0.25*self.widget_air_relhum.height - _y_sc*50.0 

        # The points to plot
        points = [0 for i in range(2*len(y_values))]
        points[0::2] = x_axis
        points[1::2] = [_y_off + _y_sc*y for y in y_values]
        self._air_relhumidity_points = points 


class RoundedButton(Button):
    '''Generate a rounded button with custom color'''
    background_color = [1,0,0,0]
    color_rgb = ListProperty([1., 0., 0.])


class MainScreen(Screen):
    '''Display live weather data'''
    widget_main   = ObjectProperty(None)

    # Sets the transparency of the background color for each widget
    _widget_visible = 0.0

    # The padding_x value for the child widgets
    _x_padding = dp(30) 


class LockScreen(Screen):
    '''Screen for configuration settings'''
    widget_lock   = ObjectProperty(None)

    # Sets the transparency of the background color for each widget
    _widget_visible = 0.0

    # The padding_x value for the child widgets
    _x_padding = dp(30) 

    def __init__(self, *args, **kwargs):
        super(LockScreen, self).__init__()
        self.gdb = GestureDatabase()

        self.gdb.add_gesture(check)

    def on_touch_down(self, touch):
        if touch.x > self.center_x + dp(300) or touch.x < self.center_x - dp(300):
            return super(LockScreen, self).on_touch_down(touch)

        # Start collecting points in touch.ud
        # Create a line to display the points
        if self.collide_point(*touch.pos):
            touch.grab(self)
            ud = touch.ud
            ud['group'] = g = str(touch.uid)
            with self.canvas:
                Color(0.5, 0.5, 1)
                d = 60.
                Ellipse(pos=(touch.x - d / 2, touch.y - d / 2), size=(d, d), group=g)
                ud['line'] = Line(points=(touch.x, touch.y), width=10, group=g)
                ret = True
        else:
            ret = super(LockScreen, self).on_touch_down(touch)

        return ret
 
    def on_touch_move(self, touch):
        # Store points of the touch movement
        #if touch.grab_current is not self:
        #    return
        try:
            touch.ud['line'].points += [touch.x, touch.y]
        except KeyError:
            pass
 
    def on_touch_up(self, touch):
        # Touch is over
        ret = False
        if touch.grab_current is self:
         
            # Add gesture to database
            try:
                g = Gesture()
                g.add_stroke(list(zip(touch.ud['line'].points[::2],
                                            touch.ud['line'].points[1::2])))
                g.normalize()
                g.name = 'try'

                # Delete trace line
                self.canvas.remove_group(touch.ud['group'])

                # Compare gesture to my_gestures.py
                #print("gesture representation:", self.gdb.gesture_to_str(g))
                #print("check:", g.get_score(check))

                # use database to find the more alike gesture, if any
                g2 = self.gdb.find(g, minscore=0.90)

                #print(g2)
                if g2 and g2[1] == check:
                    self.manager.app.open_settings()

                ret = True

            except KeyError:
                ret = super(LockScreen, self).on_touch_up(touch)

            touch.ungrab(self)

        return ret


# Build the GUI
root_widget = Builder.load_file('screens.kv')

class HomeWeatherStationApp(App):
    '''The Kivy App'''

    def __init__(self, **kwargs):
        super(HomeWeatherStationApp, self).__init__(**kwargs)
        #Window.bind(on_close=self.on_stop)
        #Window.size = (dp(800), dp(480))
        #Window.resizable = '0'
        #Window.borderless = True
        #Window.fullscreen = True
        Config.set('kivy', 'exit_on_escape', 1)
        Config.set('graphics', 'borderless', 1)
        #Config.set('graphics', 'height', dp(480))
        #Config.set('graphics', 'width', dp(800))
        Config.set('graphics', 'resizable', 0)

    def build(self):
        self.title = 'Home Weather Station V0'

        self.settings_cls = SettingsWithSidebar
        self.use_kivy_settings = True
        
        self.manager = root_widget
        root_widget.app = self

        return root_widget

    def build_config(self, config):
        config.setdefaults('general', {'small_mode': False, 'rxqueue_name': RXQUEUE_NAME})
        config.setdefaults('calibration', {'north_index': 16, 'wind_direction_steps': 16})
        config.setdefaults('example',{'optionsexample': 'option2'})

    def build_settings(self, settings):
        settings.add_json_panel('WS',
                                self.config,
                                data=wssettings)

    def on_config_change(self, config, section,
                         key, value):
        #print(config, section, key, value)
        if config is self.config:
            token = (section, key)
            if token == ('general', 'small_mode'):
                self.manager.smallMode = value
                if value is '1':
                    #Window.resizable = '1'
                    Window.size = (dp(400), dp(240))
                else:
                    #Window.resizable = '0'
                    Window.size = (dp(800), dp(480))
            elif token == ('calibration', 'north_index'):
                self.manager.northIndex = int(value)
                self.manager.north_check()
            elif token == ('calibration', 'wind_direction_steps'):
                self.manager.windDirSteps = int(value)
                self.manager.north_check()

    def on_start(self):
        #print("\non_start:")
        return True

    def on_pause(self):
        return True

    def on_resume(self):
        pass

    def on_stop(self):
        #print("\non_stop:")
        return True
    

HomeWeatherStationApp().run()
