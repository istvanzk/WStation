## Home Weather Station built with Arduino Pro Mini and Raspberry PI (3 A+)

**Note: This repo contains work-in-progress code and schematics - October 2020**

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


**Structure (draft)**

*arduino*
- *KiCad*:
- *WStation*:

*raspi*
- *KiCad*:
- *kivy*:
- *rfm*:

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

