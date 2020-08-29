#import time
import struct
from random import random
from math import log10

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


# Message queue parameters
RXQUEUE_NAME  = "/rf22b_server_tx"
MAX_RXMSG_SIZE = 255

class ClientMq():

    # Create the message queue
    # The umask is not relevant for reading, when the queue has been created as RW!
    #old_umask = os.umask(0)
    #os.umask(old_umask)
    import posix_ipc
    mqRX = posix_ipc.MessageQueue(RXQUEUE_NAME, posix_ipc.O_RDONLY)

    # Request notifications
    #mqRX.request_notification((self.process_notification, mqRX))

    def process_notification(self,mq):

        s, p = mq.receive()
        sd = s.decode()

        # Re-register for notifications
        mq.request_notification((self.process_notification, mq))

        print("Message received: %s (%d) - %s\n" % (s, p, sd))

    def read_msg(self):
        s, p = self.mqRX.receive()
        unpacked_s = struct.unpack('HHHHHHBBBBBb', s[:18])
        #sd = s.decode()
        print("Message#%d:(%d):" % (m, p))
        print(unpacked_s)
        print(":".join("{:02x}".format(ord(c)) for c in s[18:38].decode()))


class FirstScreen(Screen):
    wind_direction = NumericProperty(0.0)
    wind_speed = NumericProperty(0.5)
    # HSV: Hue = degrees/360
    #     Red: 0 and 60 degrees.
    #     Yellow: 61 and 120 degrees.
    #     Green: 121 and 180 degrees.
    #     Cyan: 181 and 240 degrees.
    #     Blue: 241 and 300 degrees.
    #     Magenta: 301 and 360 degrees.
    wind_speed_hue = NumericProperty(160)
    air_temperature = NumericProperty(20.5)
    air_temperature_color = ListProperty([0,1,0])
    air_pressure = NumericProperty(1008.5)
    air_relhumidity = NumericProperty(61.5)

    _update_display_ev = None

    def __init__(self, **kwargs):
        super(FirstScreen, self).__init__(**kwargs)
        Clock.schedule_interval(self.update_display_info, 1)

    def update_display_info(self, *args):
        if random() > 0.5:
            self.air_temperature += 2*random()
        else:
            self.air_temperature -= 2*random()

        if self.air_temperature < -10:
            self.air_temperature_color = [0,0,1]
        elif self.air_temperature < 0 and self.air_temperature > -10:
            self.air_temperature_color = [0,0,0.5]
        elif self.air_temperature > 0 and self.air_temperature < 20:
            self.air_temperature_color = [0,1,0]
        elif self.air_temperature > 20 and self.air_temperature < 35:
            self.air_temperature_color = [0.5,0,0]
        elif self.air_temperature > 35:
            self.air_temperature_color = [1,0,0]

        if random() > 0.5:
            self.air_pressure += 5*random()
        else:
            self.air_pressure -= 5*random()

        if random() > 0.5:
            self.air_relhumidity += random()
        else:
            self.air_relhumidity -= random()

        if random() > 0.5:
            self.wind_speed = 35*random()
        self.wind_speed_hue = (160-205*log10(1+self.wind_speed/5))/360

        if random() > 0.5:
            self.wind_direction= 360*random()

class SecondScreen(Screen):
    pass

class MyScreenManager(ScreenManager):    
    pass

root_widget = Builder.load_file('screens.kv')


class HomeWeatherStationApp(App):
    def build(self):
        self.title = 'Home Weather Station V0'
        return root_widget

HomeWeatherStationApp().run()
