'''
Home Weather Staion code for Raspberry Pi

Kivy based UI to display weather data (live and traces). The Kivy widgets are defined in screens.kv
Uses a POSIX IPC Meesage Queue client to receive weather data from a server (see raspi/rfm/server_mq.cpp) running on the same machine

Author: Istvan Z. Kovacs, 2020-2021
'''

# System
import time
import struct
from random import random, randint
from collections import deque
from math import log10, cos
from statistics import mean, median, mode, StatisticsError
import json
from colorsys import hsv_to_rgb

# Kivy
import kivy
kivy.require('2.0.0')
from kivy.app import App
from kivy.core.window import Window
from kivy.metrics import dp
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

# Set window (before it's created)
from kivy.config import Config
Config.set('kivy', 'exit_on_escape', 1)
Config.set('graphics', 'borderless', 1)
Config.set('graphics', 'height', '480')
Config.set('graphics', 'width', '800')
Config.set('graphics', 'window_state', 'maximized')
Config.set('graphics', 'fullscreen', 'auto')
Config.set('graphics', 'resizable', False)
Config.set('graphics', 'show_cursor', 0)
Window.set_title('Home Weather Station V1')

# The POSIX IPC Meesage Queue client
from client_mq import ClientMQ

# Message queue parameters
# Message queue read interval (seconds)
RXQUEUE_TIME  = 15 
# Message queue name to connect to
RXQUEUE_NAME  = "/rf69_server_tx"

# The Adafruit IO client
# Note: This requires an external aiocfg.txt configuration file:
# username,aio_key
# WS,ws
# North,Temperature,Windspeed,Pressure,Humidity,Rssi
# 57.0547,9.9198,44
from client_io import AdafruitClientIO

# Wind direction steps and North index/step
DIR_STEPS      = 16
NORTHDIR_INDX  = 16

# Debug messages
# TODO: Use logger
DEBUG = 0

wssettings = json.dumps([
    {'type': 'title',
     'title': 'General'},
    {'type': 'bool',
     'title': 'Small mode',
     'desc': 'Reduce app window size',
     'section': 'general',
     'key': 'small_mode'},
    {'type': 'title',
     'title': 'READ ONLY at runtime!'},
     {'type': 'string',
     'title': 'RX message queue',
     'desc': 'The message queue name for receiving weather data',
     'section': 'general',
     'key': 'rxqueue_name'},    
    {'type': 'numeric',
     'title': 'RX message queue read time interval',
     'desc': 'The interval for reading from the meesage queue (seconds)',
     'section': 'general',
     'key': 'rxqueue_timeintv'},
    {'type': 'title',
     'title': 'Calibration'},
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
    {'type': 'title',
     'title': 'Options'},
    {'type': 'options',
     'title': 'An options setting',
     'desc': 'Options description text',
     'section': 'example',
     'key': 'optionsexample',
     'options': ['option1', 'option2', 'option3']}])


class MyScreenManager(ScreenManager):    

    screen_traces = ObjectProperty(None)

    # Settings (configurable, see app.on_config_change(...))
    smallMode    = False
    windDirSteps = DIR_STEPS
    northIndex   = NORTHDIR_INDX
    # Note: these parameters cannot be changed during runtime!
    rxqueueName  = RXQUEUE_NAME
    rxtimeIntv   = RXQUEUE_TIME

    # The dictionary storing the most recent weather data read from the message queue 
    weather_data = {'Header': tuple(), 'IniMsg': None, 'N': 0,'S':0, 'T':0, 'P':0,'H':0}
 
    # Dictionary of deques with weather data for the past ~15 minutes in steps corresponding to the received rate (15*60/rxtimeIntv values)
    weather_data_trace15 = {}

    # Dictionary of deques with weather data for the past 24 hours in 15 minutes steps (24*4 values)
    weather_data_trace24 = {}

    # The message queue
    _msg_queue = None

    # The client IO
    _client_io = None

    # Displayed info and their color
    #(access with root.manager.* from the kv file)
    _wind_direction = NumericProperty(22.5)
    _wind_speed = NumericProperty(0)
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

    # Info string
    _crt_msg_str   = StringProperty('None')
    _crt_msg_color = ListProperty([0,1,0])
    _crt_msg_vis   = NumericProperty(0)

    def __init__(self, **kwargs):
        super(MyScreenManager, self).__init__(**kwargs)

    def init(self):

        # The clock schedule    
        _update_mainscreen_sch = Clock.schedule_interval(self.update_mainscreen_info, self.rxtimeIntv)

        # Dictionary of deques with weather data for the past ~15 minutes in steps corresponding to the received rate (15*60/rxtimeIntv values)
        self.weather_data_trace15["Time"] = deque([],int(900/self.rxtimeIntv))
        self.weather_data_trace15["Rssi"] = deque([],int(900/self.rxtimeIntv))
        self.weather_data_trace15["N"]    = deque([],int(900/self.rxtimeIntv))
        self.weather_data_trace15["T"]    = deque([],int(900/self.rxtimeIntv))
        self.weather_data_trace15["S"]    = deque([],int(900/self.rxtimeIntv))
        self.weather_data_trace15["P"]    = deque([],int(900/self.rxtimeIntv))
        self.weather_data_trace15["H"]    = deque([],int(900/self.rxtimeIntv))

        # Dictionary of deques with weather data for the past 24 hours in 15 minutes steps (24*4 values)
        self.weather_data_trace24["Time"] = deque([],96)
        self.weather_data_trace24["Rssi"] = deque([],96)
        self.weather_data_trace24["N"] = deque([],96)
        self.weather_data_trace24["T"] = deque([],96)
        self.weather_data_trace24["S"] = deque([],96)
        self.weather_data_trace24["P"] = deque([],96)
        self.weather_data_trace24["H"] = deque([],96)

        # Create the receive message queue
        # _msg_queue.mqRX is set to None when no MessageQueue is available (e.g. MacOS)!
        self._msg_queue = ClientMQ(self.rxqueueName)

        # Create IO client
        # The feeds are updated only with the set period
        self._client_io = AdafruitClientIO(update_sec=60)


    def update_mainscreen_info(self, *args):

        # Current time as string
        self._crt_time_str = time.asctime(time.localtime(time.time()))

        # Get real data from the remote weather station (via local MessageQueue)
        if self._msg_queue.mqRX is not None:

            _weather_data = self._msg_queue.read_weather_data(1.0)
            if _weather_data is not None:

                # Data received: update info
                if _weather_data["Header"][14] == 20:
                    self.weather_data["IniMsg"] = None

                    # Adjust North direction based on calibration data
                    _weather_data["N"]     -= self.northIndex
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
                self.weather_data["Header"] = _weather_data["Header"]

                # RSSI
                _weather_data["Header"][13] -= 256
                self._rssi_dBm = _weather_data["Header"][13]

                # Info
                self._crt_msg_str = "RXMQ:OK. Data:OK"
                self._crt_msg_vis = 0.7
                if self._rssi_dBm > -80:
                    self._crt_msg_color = [0,1,0]
                if self._rssi_dBm <= -80 and self._rssi_dBm > -85:
                    self._crt_msg_color = [0.5,1,0]
                elif self._rssi_dBm <= -85 and self._rssi_dBm > -90:
                    self._crt_msg_color = [1,1,0]
                elif self._rssi_dBm <= -90 and self._rssi_dBm > -95:
                    self._crt_msg_color = [1,0.5,0]
                elif self._rssi_dBm <= -95 and self._rssi_dBm > -100:
                    self._crt_msg_color = [1,0.5,0.5]    
                elif self._rssi_dBm <= -100:
                    self._crt_msg_color = [1,0,0]    

            else:
                # Data not received: keep last reading
                # Header info
                struc_t = time.localtime(time.time())
                self.weather_data["Header"] = [
                    struc_t.tm_sec,
                    struc_t.tm_min,
                    struc_t.tm_hour,
                    struc_t.tm_mday,
                    struc_t.tm_mon,
                    struc_t.tm_year,
                    struc_t.tm_wday,
                    struc_t.tm_yday,
                    struc_t.tm_isdst,
                    0,0,0,0, 
                    self._rssi_dBm, 
                    20]
                self.weather_data["IniMsg"] = 'No Data'

                # Info
                self._crt_msg_str = "RXMQ:OK. Data:N/A"
                self._crt_msg_vis = 0.7
                self._crt_msg_color = [1,0.5,0.5]  

        # Generate random test data        
        else:
            
            if random() > 0.5:
                self._air_temperature += 2*random()
            else:
                self._air_temperature -= 2*random()

            if random() > 0.5:
                self._air_pressure += 5*random()
            else:
                self._air_pressure -= 5*random()

            if random() > 0.5:
                self._air_relhumidity += random()
            else:
                self._air_relhumidity -= random()

            if random() > 0.5:
                self._wind_speed = 35*random()

            if random() > 0.5:
                self._wind_direction = (360/self.windDirSteps) * randint(1, self.windDirSteps)
        
            if random() > 0.5:
                self._rssi_dBm = -(60+42*random())

            # Header info
            struc_t = time.localtime(time.time())
            self.weather_data["Header"] = [
                struc_t.tm_sec,
                struc_t.tm_min,
                struc_t.tm_hour,
                struc_t.tm_mday,
                struc_t.tm_mon,
                struc_t.tm_year,
                struc_t.tm_wday,
                struc_t.tm_yday,
                struc_t.tm_isdst,
                0,0,0,0,
                self._rssi_dBm, 
                20]
            self.weather_data["IniMsg"] = None

            # Info
            self._crt_msg_str = "RXMQ:N/A. Data:RND"
            self._crt_msg_vis = 0.7
            self._crt_msg_color = [1,1,0]  


        # Store last weather data reading   
        self.weather_data["N"] = self._wind_direction  
        self.weather_data["T"] = self._air_temperature
        self.weather_data["S"] = self._wind_speed
        self.weather_data["P"] = self._air_pressure
        self.weather_data["H"] = self._air_relhumidity
        self.weather_data["Rssi"] = self._rssi_dBm

        # Send data with default metadata attached
        # The feeds are updated only with the set period
        self._client_io.send_data_all_feeds(self.weather_data)
 
        if DEBUG:
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
            self._air_temperature_color = [0,1,1]
        elif self._air_temperature > 0 and self._air_temperature < 20:
            self._air_temperature_color = [0,1,0]
        elif self._air_temperature > 20 and self._air_temperature < 35:
            self._air_temperature_color = [1,1,0]
        elif self._air_temperature > 35:
            self._air_temperature_color = [1,0,0]

        self._wind_speed_hue = (160-170*log10(1+self._wind_speed/5))/360
        self._wind_speed_color = hsv_to_rgb(self._wind_speed_hue, 1, 1)

    # Process and store weather data
    def procstore_weather_data(self):

        # Update current time info
        struc_t = time.localtime(time.time())
        _crtTime = (
            self.weather_data["Header"][5], 
            self.weather_data["Header"][4], 
            self.weather_data["Header"][3], 
            self.weather_data["Header"][2], 
            self.weather_data["Header"][1], 
            self.weather_data["Header"][0], 
            self.weather_data["Header"][6], 
            self.weather_data["Header"][7], 
            self.weather_data["Header"][8])
        self._crt_secs = time.mktime(_crtTime)

        # Update trace15 deques
        self.weather_data_trace15["Time"].append(_crtTime)
        self.weather_data_trace15["Rssi"].append(self.weather_data["Rssi"])
        self.weather_data_trace15["N"].append(self.weather_data["N"])
        self.weather_data_trace15["T"].append(self.weather_data["T"])
        self.weather_data_trace15["S"].append(self.weather_data["S"])
        self.weather_data_trace15["P"].append(self.weather_data["P"])
        self.weather_data_trace15["H"].append(self.weather_data["H"])

        if len(self.weather_data_trace15["Time"]) < 2:
            self._start_secs = time.mktime(_crtTime)
            return None


        # At the end of each ~15 minutes time window:
        # - Calculate average values over the past ~15 minutes (all values in the trace15 deques)
        # - Append to trace24 deques the average values of the past 15 minutes
        _TimeAvg = None
        if self._crt_secs - self._start_secs >= 900:

            # Update the trace24 deques with the average values of the past ~15 minutes
            # The wind direction 'average' is the most frequent direction
            # The time stamp 'average' is the mid-time
            struc_t = time.localtime(self._start_secs+450)
            _TimeAvg = (
                struc_t.tm_year,
                struc_t.tm_mon,
                struc_t.tm_mday,
                struc_t.tm_hour,
                struc_t.tm_min,
                struc_t.tm_sec,
                struc_t.tm_wday, 
                struc_t.tm_yday, 
                struc_t.tm_isdst)
            self.weather_data_trace24["Time"].append(_TimeAvg)
            try:
                self.weather_data_trace24["N"].append(mode(self.weather_data_trace15["N"]))
            except StatisticsError:
                self.weather_data_trace24["N"].append(self.weather_data["N"])
                pass
            self.weather_data_trace24["T"].append(mean(self.weather_data_trace15["T"]))
            self.weather_data_trace24["S"].append(mean(self.weather_data_trace15["S"]))
            self.weather_data_trace24["P"].append(mean(self.weather_data_trace15["P"]))
            self.weather_data_trace24["H"].append(mean(self.weather_data_trace15["H"]))
            self.weather_data_trace24["Rssi"].append(mean(self.weather_data_trace15["Rssi"]))

            # Update start time info
            self._start_secs = time.mktime(_crtTime)

        return _TimeAvg
 
    def north_check(self):
        '''Check the North direction index vs. number of wind direction steps'''
        if self.northIndex > self.windDirSteps:
            self.northIndex = self.windDirSteps


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

    # The default plot step
    _x_step = dp(100)

    def on_enter(self, *args):
        self.widget_air_temp.canvas.ask_update()
        self.widget_wind_speed.canvas.ask_update()
        self.widget_air_press.canvas.ask_update()
        self.widget_air_relhum.canvas.ask_update()

    def update_trace_plots(self, weather_data_trace):
        '''Update the plots for all traces'''

        # The x axis step (common for all traces)     
        self._x_step   = (self.widget_air_temp.width - self.widget_air_temp.pos[0] - self._x_offset)/weather_data_trace["Time"].maxlen

        #print(self.widget_air_temp.pos,self.widget_air_temp.width,self.widget_air_temp.height)

        # The time labels
        self._start_time_str = time.asctime(weather_data_trace["Time"][0])    
        self._end_time_str   = time.asctime(weather_data_trace["Time"][-1])

        # The trace data values
        for k in weather_data_trace:
            if k != "Time":
                if k == "N":
                    self._update_wind_direction_plot(weather_data_trace[k])
                elif k == "T":
                    self._update_air_temperature_plot(weather_data_trace[k])
                elif k == "S":
                    self._update_wind_speed_plot(weather_data_trace[k])
                elif k == "P":
                    self._update_air_pressure_plot(weather_data_trace[k])
                elif k == "H":
                    self._update_air_relhumidity_plot(weather_data_trace[k])
                elif k == "Rssi":
                    self._update_rssi_plot(weather_data_trace[k])
            
        # Update the progress bar
        #self.progress_bar.min = 0
        #self.progress_bar.max = weather_data_trace["Time"].maxlen
        #self.progress_bar.value += 1
                    
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
        _y_sc = self.widget_wind_speed.height/20.0
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
        _y_sc = self.widget_air_press.height/50.0
        _y_off = self.widget_air_press.pos[1] + 0.25*self.widget_air_press.height - _y_sc*990.0
          

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
        _y_sc = self.widget_wind_speed.height/50
        _y_off = self.widget_air_relhum.pos[1] + 0.25*self.widget_air_relhum.height - _y_sc*60.0 

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
    
    # Note:  Many programs set the title just after mapping the window which means the value Openbox sees as it is determining which rules to apply 
    # is sometimes empty or something like "Untitled". 
    # The _OB_APP_TITLE property will show the value that Openbox used, not the current title.
    # (from http://openbox.org/wiki/Help:Applications)
    # => The self.title only sets the WM_NAME property of openbox!
    # => The _OB_APP_TITLE is not set!
 
    def __init__(self, **kwargs):
        super(HomeWeatherStationApp, self).__init__(**kwargs)
        #Window.bind(on_close=self.on_stop)

    def build(self):
        self.title = 'Home Weather Station V1'

        # Settings
        self.settings_cls = SettingsWithSidebar
        self.use_kivy_settings = True
        
        # Book keeping
        self.manager = root_widget
        root_widget.app = self

        # Set these values in the config file
        #self.config.set('general', 'small_mode', False)
        #self.config.set('general', 'rxqueue_name', RXQUEUE_NAME)
        #self.config.set('example','optionsexample', 'option1')
        #self.config.write()

        # Read these config parameter values when App starts
        self.manager.northIndex   = int(self.config.get('calibration','north_index',fallback=NORTHDIR_INDX))
        self.manager.windDirSteps = int(self.config.get('calibration','wind_direction_steps',fallback=DIR_STEPS))
        # Note: these parameter values cannot be changed during runtime!
        self.manager.rxqueueName = self.config.get('general','rxqueue_name',fallback=RXQUEUE_NAME)
        self.manager.rxtimeIntv  = float(self.config.get('general','rxqueue_timeintv',fallback=RXQUEUE_TIME))

        # Init and apply config parameter values
        self.manager.init()

        return root_widget

    def build_config(self, config):
        config.setdefaults('general', {'small_mode': False, 'rxqueue_name': RXQUEUE_NAME, 'rxqueue_timeintv': RXQUEUE_TIME})
        config.setdefaults('calibration', {'north_index': NORTHDIR_INDX , 'wind_direction_steps': DIR_STEPS})
        config.setdefaults('example',{'optionsexample': 'option1'})

        #print(f"build_config: {config.get('general', 'small_mode')}")

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
                if int(value)==0:
                    #Window.fullscreen = True
                    Window.maximize()
                    Window.borderless = True
                    Window.resizable = False
                    #Window.size = (self._win_width, self._win_height)
                else:
                    #Window.fullscreen = False
                    Window.borderless = False
                    Window.resizable = True
                    Window.size = (self._win_width/2, self._win_height/2)
            elif token == ('calibration', 'north_index'):
                self.manager.northIndex = int(value)
                self.manager.north_check()
            elif token == ('calibration', 'wind_direction_steps'):
                self.manager.windDirSteps = int(value)
                self.manager.north_check()

    def on_start(self):
        #print("\non_start:")
        self._win_width = int(Config.get('graphics', 'width'))
        self._win_height = int(Config.get('graphics', 'height'))
        #print(self._win_width, self._win_height)
        

    def on_pause(self):
        return True

    def on_resume(self):
        pass

    def on_stop(self):
        #print("\non_stop:")
        #if self.manager._msg_queue.mqRX is not None:
        #    self.manager._msg_queue.mqRX.close()
        pass
    
HomeWeatherStationApp().run()
