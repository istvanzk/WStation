
from kivy.app import App
from kivy.clock import Clock
from kivy.base import runTouchApp
from kivy.lang import Builder
from kivy.properties import ListProperty, NumericProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout

from kivy.uix.screenmanager import ScreenManager, Screen, FadeTransition, SlideTransition

from kivy.core.window import Window
Window.size = (800, 480)
Window.resizable = '0'
Window.borderless = True

import time
from random import random

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
    air_temperature = NumericProperty(20.5)
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
