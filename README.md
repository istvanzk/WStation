## Home Weather Station built with Arduino Pro Mini and Raspberry PI (3 A+)

**Note: This repo contains work-in-progress code and schematics - August 2021 (v2.0)**

**Dependencies**

- [OBH Nordica eHome wind meter system 4931][13] (or similar set of sensor)
- [Arduino Pro Mini 328 3.3V/8MHz][8]
- [Raspberry Pi 3 A+][9] and the [7″ touchscreen monitor][11]
- The [forked & modified RadioHead library][1] with support of [RFM22B][2] and [RFM69HCW][10] based radio modules for Raspberry Pi. 
- The original [RadioHead Packet Radio library][3] for embedded microprocessors maintained by Mike McCauley. 
- The original [BCM2835][12] library for Raspbery Pi maintained by Mike McCauley. 
- Arduino libraries for communication with the [Barometric Pressure Sensor Breakout - BMP180][4] and [Humidity and Temperature Sensor Breakout - SHT15][5]
- Python module for [POSIX IPC Semaphores, Shared Memory and Message Queues][6]
- Pyhton library [Kivy][7]

**GUI (v1.0)**
- The main screen - updated every 45 seconds
![Main screen](/docs/mainscreen.jpg)

- The trace sscreen - for last 15 minutes
![Traces screen](/docs/tracescreen.jpg)


**Structure (v1.0)**

*arduino*
- *KiCad*: KiCad circuit diagrams
- *WStation*: code
  - *sensrfm*: client code for collecting weather sensor data and sending it via RFM69HCW or RFM22B radio
  - other folders: test code for sensors and radio

*raspi*
- *KiCad*: KiCad circuit diagrams
- *kivy*: Kivy GUI code (client_mq.py, display.py and screens.kv)
- *rfm*: C code for reading the weather data transmissions via the RFM69HCW or RFM22B radio and sending them via POSIX MessageQueue to the Kivy GUI (server_mq.cpp)

*docs*
- Schematics and pictures


[1]: https://github.com/istvanzk/RadioHead
[2]: https://www.hoperf.com/modules/rf_transceiver/RFM22BW.html
[3]: http://www.airspayce.com/mikem/arduino/RadioHead/
[4]: https://github.com/sparkfun/BMP180_Breakout_Arduino_Library
[5]: https://github.com/sparkfun/SHT15_Breakout
[6]: https://github.com/osvenskan/posix_ipc 
[7]: https://kivy.org
[8]: https://www.arduino.cc/en/Main/ArduinoBoardProMini
[9]: https://www.raspberrypi.org/blog/new-product-raspberry-pi-3-model-a
[10]: https://www.hoperf.com/modules/rf_transceiver/RFM69HCW.html
[11]: https://www.raspberrypi.org/products/raspberry-pi-touch-display/
[12]: http://www.airspayce.com/mikem/bcm2835/
[13]: https://github.com/istvanzk/WStation/blob/master/docs/OBHeHome4931.png

