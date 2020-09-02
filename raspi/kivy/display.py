import time
import struct
from random import random
from collections import deque
from math import log10, cos
import posix_ipc

# Kivy
from kivy.app import App
from kivy.metrics import dp
from kivy.clock import Clock
from kivy.base import runTouchApp
from kivy.lang import Builder
from kivy.properties import ListProperty, NumericProperty
from kivy.uix.button import Button
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.core.window import Window
Window.size = (dp(800), dp(480))
Window.resizable = '0'
Window.borderless = True
#Window.fullscreen = True

# Message queue parameters
RXQUEUE_NAME  = "/rf22b_server_tx"
MAX_RXMSG_SIZE = 255

class ClientMq(object):

    # Create the message queue
    # The umask is not relevant for reading, when the queue has been created as RW!
    #old_umask = os.umask(0)
    #os.umask(old_umask)
    if hasattr(posix_ipc, 'MessageQueue'):
        try:
            mqRX = posix_ipc.MessageQueue(RXQUEUE_NAME, posix_ipc.O_RDONLY | posix_ipc.O_CREAT)
            # Request notifications
            #mqRX.request_notification((self.process_notification, mqRX))
        except:
            mqRX = None
            pass
    else:
        mqRX = None


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
        _weather_data["Header"] = _unpacked_msgheader

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

    # How to update the values
    _update_display_rnd  = False
    _update_display_msgq = True

    # Displayed info and their color
    #(access with root.manager.* from the kv file)
    _wind_direction = NumericProperty(22.5)
    _wind_speed = NumericProperty(0.11)
    # HSV: Hue = degrees/360
    #     Red: 0 and 60 degrees.
    #     Yellow: 61 and 120 degrees.
    #     Green: 121 and 180 degrees.
    #     Cyan: 181 and 240 degrees.
    #     Blue: 241 and 300 degrees.
    #     Magenta: 301 and 360 degrees.
    _wind_speed_hue = NumericProperty(160)
    _air_temperature = NumericProperty(23.5)
    _air_temperature_color = ListProperty([0,1,0])
    _air_pressure = NumericProperty(1000.8)
    _air_relhumidity = NumericProperty(64.5)
    _rssi_dBm = NumericProperty(-66)

    # The dictionary storing the most recent weather data read from the message queue 
    weather_data = {'Header': tuple(), 'IniMsg': None, 'N': _wind_direction,'S':_wind_speed, 'T':_air_temperature, 'P':_air_pressure,'H':_air_relhumidity}
 
    # Deque of dictionaries with weather data for the past 15min in steps corresponding to the received rate
    weather_data_trace15 = deque([])

    # Deque of dictionaries with weather data for the past 24h in 15min steps
    weather_data_trace24 = deque([])

    # Create the receive message queue
    if _update_display_msgq:
        _msg_queue = ClientMq()
        if _msg_queue.mqRX is None:
            _update_display_msgq = False
            _update_display_rnd  = True
            _msg_queue = None
    else:
        _msg_queue = None

    def __init__(self, **kwargs):
        super(MyScreenManager, self).__init__(**kwargs)
        Clock.schedule_interval(self.update_display_info, 5.0)

    def update_display_info(self, *args):

        # Generate random test data
        if self._update_display_rnd:
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
                self._wind_direction = 360*random()
        
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

        # Real data from the weather station
        elif self._update_display_msgq:
            _weather_data = self._msg_queue.read_weather_data(1.0)
            if _weather_data is not None:

                if _weather_data["Header"][11] == 20:
                    self.weather_data["IniMsg"] = None

                    # TODO: adjust North direction based on calibration data
                    self._wind_direction   = _weather_data["N"]*22.5

                    self._air_temperature  = _weather_data["T"]           
                    self._wind_speed       = _weather_data["S"]
                    self._air_pressure     = _weather_data["P"]
                    self._air_relhumidity  = _weather_data["H"]
                
                else:
                    self.weather_data["IniMsg"] = _weather_data["IniMsg"]

                # TODO: convert tuple and extract time stamp, rssi, etc.
                _weather_data["Header"][5] += 1900
                _weather_data["Header"][10] -= 256
                self.weather_data["Header"] = _weather_data["Header"]
                    

        # Store last weather data reading   
        self.weather_data["N"] = self._wind_direction  
        self.weather_data["T"] = self._air_temperature
        self.weather_data["S"] = self._wind_speed
        self.weather_data["P"] = self._air_pressure
        self.weather_data["H"] = self._air_relhumidity

        print(self.weather_data)

        # Process and store weather data
        if self.weather_data["IniMsg"] is None:
            self.procstore_weather_data()

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

        self._wind_speed_hue = (160-205*log10(1+self._wind_speed/5))/360

    # Process and store weather data
    def procstore_weather_data(self):

        # Init trace15 deque
        if len(self.weather_data_trace15) < 2:
            self.weather_data_trace15.append(self.weather_data)
            return

        # Check past 15min time window (current - oldest)
        t_crt_secs = time.mktime((
            self.weather_data["Header"][5], 
            self.weather_data["Header"][4], 
            self.weather_data["Header"][3], 
            self.weather_data["Header"][2], 
            self.weather_data["Header"][1], 
            self.weather_data["Header"][0], 
            0, 0, 0))

        _oldest_weather_data = self.weather_data_trace15[0]
        t_old_secs = time.mktime(( 
            _oldest_weather_data["Header"][5], 
            _oldest_weather_data["Header"][4], 
            _oldest_weather_data["Header"][3], 
            _oldest_weather_data["Header"][2], 
            _oldest_weather_data["Header"][1], 
            _oldest_weather_data["Header"][0], 
            0, 0, 0))

        # At the end of the 15min time window:
        # - Calculate average values over the past 15min (all values in the trace15 deque)
        # - Clear trace15 deque and append the current value
        # - Append dictionary with average values to the trace24 deque
        if t_crt_secs - t_old_secs > 900:

            _avg15_weather_data = {'TimeAvg':tuple(), 'Rssi':0, 'N':0, 'S':0, 'T':0, 'P':0, 'H':0}
            for wd in self.weather_data_trace15:
                _avg15_weather_data["Rssi"] += wd["Header"][10] 
                _avg15_weather_data["T"] += wd["T"]
                _avg15_weather_data["S"] += wd["S"]
                _avg15_weather_data["P"] += wd["P"]
                _avg15_weather_data["H"] += wd["H"]

            # The time of the average values
            struc_t = time.localtime(t_old_secs+450)
            _avg15_weather_data["TimeAvg"] = (
                struc_t.tm_year,
                struc_t.tm_mon,
                struc_t.tm_mday,
                struc_t.tm_hour,
                struc_t.tm_min,
                struc_t.tm_sec)

            # TODO: Estimate an average direction
            _avg15_weather_data["N"] = self.weather_data_trace15[:-1]["N"]

            # The average values
            _avg15_weather_data["Rssi"] /= len(self.weather_data_trace15)
            _avg15_weather_data["T"] /= len(self.weather_data_trace15)
            _avg15_weather_data["S"] /= len(self.weather_data_trace15)
            _avg15_weather_data["P"] /= len(self.weather_data_trace15)
            _avg15_weather_data["H"] /= len(self.weather_data_trace15)

            # Update deques
            self.weather_data_trace15.clear()
            self.weather_data_trace15.append(self.weather_data)
            self.weather_data_trace24.append(_avg15_weather_data)

            # Keep only the last 24h data (24*4 values)
            if len(self.weather_data_trace24) > 96:
                self.weather_data_trace24.popleft()

        # Current time
        #time_secs = time.mktime(time.localtime(time.time()))
        #self.weather_data["Header"]



class MainScreen(Screen):
    pass

class TracesScreen(Screen):
    dt = NumericProperty(0)
    _air_temperature_points = ListProperty([])

    def __init__(self, **kwargs):
        super(TracesScreen, self).__init__(**kwargs)
        Clock.schedule_interval(self.update_points_animation, 5.0)

    def update_points_animation(self, dt):
        cy = self.height * 0.6
        cx = self.width * 0.1
        w = self.width * 0.9
        step = 50
        points = []
        self.dt += dt
        for i in range(int(w / step)):
            x = i * step
            points.append(cx + x)
            points.append(cy + cos(x / w * 8. + self.dt) * self.height * 0.1)
        self._air_temperature_points = points

class LockScreen(Screen):
    pass

class RoundedButton(Button):
    background_color = [1,0,0,0]
    color_rgb = ListProperty([1., 0., 0.])


root_widget = Builder.load_file('screens.kv')

class HomeWeatherStationApp(App):
    def build(self):
        self.title = 'Home Weather Station V0'
        return root_widget

HomeWeatherStationApp().run()
