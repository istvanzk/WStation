#import time
import struct
from random import random
from math import log10
import posix_ipc

# Kivy
from kivy.app import App
from kivy.clock import Clock
from kivy.base import runTouchApp
from kivy.lang import Builder
from kivy.properties import ListProperty, NumericProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.core.window import Window
Window.size = (800, 480)
Window.resizable = '0'
Window.borderless = True
#Window.fullscreen = True

# Debug messages
DEBUG_MSG = False

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

        return _weather_data

class MyScreenManager(ScreenManager):    

    # How to update the values
    _update_display_rnd  = False
    _update_display_msgq = True

    # Displayed info and their color
    #(access with root.manager.* from the kv file)
    _wind_direction = NumericProperty(1)
    _wind_speed = NumericProperty(0.5)
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

    # The dictionary storing the most recent weather data read from the message queue
    weather_data = {'N': _wind_direction,'S':_wind_speed, 'T':_air_temperature, 'P':_air_pressure,'H':_air_relhumidity}

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
        Clock.schedule_interval(self.update_display_info, 1.0)

    def update_display_info(self, *args):

        # Update the values
        # Random: display testing
        # MsgQueue: normal operation
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
                self._wind_direction= 360*random()
        
        elif self._update_display_msgq:
            self.weather_data = self._msg_queue.read_weather_data(11.0)
            if self.weather_data is not None:
                # TODO: adjust North direction based on calibration data
                self._wind_direction  = self.weather_data['N']*22.5
                self._air_temperature = self.weather_data['T']
                self._wind_speed      = self.weather_data['S']
                self._air_pressure    = self.weather_data['P']
                self._air_relhumidity = self.weather_data['H']


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


class FirstScreen(Screen):
    pass

class SecondScreen(Screen):
    pass


root_widget = Builder.load_file('screens.kv')


class HomeWeatherStationApp(App):
    def build(self):
        self.title = 'Home Weather Station V0'
        return root_widget

HomeWeatherStationApp().run()
