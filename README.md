## Weather Station built with Arduino Pro Mini and Raspberry PI (3 A+)

**Note: This repo contains work-in-progress code and schematics - November 2019**

**Dependencies**

- Old OBH Nordica eHome wind sensor system, or similar
- [Arduino Pro Mini 328 3.3V/8MHz][8]
- [Raspberry Pi 3 A+][9]
- The [forked & modified RadioHead library][1] with support for [RFM22B][2] based radio modules on Raspberry Pi. 
The original [RadioHead Packet Radio library][3] for embedded microprocessors is maintained by Mike McCauley. 
- Arduino libraries for communication with the [Barometric Pressure Sensor Breakout - BMP180][4] and [Humidity and Temperature Sensor Breakout - SHT15][5]
- Python module for [POSIX IPC Semaphores, Shared Memory and Message Queues][6]
- Pyhton library [Kivy][7] for user interfaces
- ...


**Structure (draft)**

*arduino*
- *KiCad*:
- *WStation*:

*raspi*
- *KiCad*:
- *kivy*:
- *rfm22b*:

*docs*
- Schematics and pictures


[1]: https://github.com/istvanzk/RadioHead
[2]: https://www.sparkfun.com/products/12030
[3]: http://www.airspayce.com/mikem/arduino/RadioHead/
[4]: https://github.com/sparkfun/BMP180_Breakout_Arduino_Library
[5]: https://github.com/sparkfun/SHT15_Breakout
[6]: https://github.com/osvenskan/posix_ipc 
[7]: https://kivy.org
[8]: https://www.arduino.cc/en/Main/ArduinoBoardProMini
[9]: https://www.raspberrypi.org/blog/new-product-raspberry-pi-3-model-a


