## Arduino code

The *\*_test* folders contain scripts used to test the individual components connected to the [Arduino board Pro Mini][1].
The final code is in the folder *ws_final*.

### BOM 
[Arduino board Pro Mini][1]

[RFM22B-S2][2]

[SparkFun Barometric Pressure Sensor Breakout - BMP180][3]

[SparkFun Humidity and Temperature Sensor Breakout - SHT15][4]

[Octal Bus Transceiver Tri-State - 74HC245][5]

### Connections

**The [Arduino board Pro Mini][1] connections (see [full schematics][6] for details)**
```cpp
D2/INT0 --- nIRQ (RFM2B)
D3 ---  /G (74HC245)

D4 --- R1_PU +  B1 (74HC245)
D5 --- R2_PU + B2 (74HC245)
D6 --- R1_PD + B3 (74HC245)
D7 --- R2_PD + B4 (74HC245)
D8 --- R3_PD + B5 (74HC245)
D9 --- R4_PD + B6 (74HC245)

D10/SS   --- nSEL (RFM22B)
D11/MOSI --- SDI  (RFM22B)
D12/MISO --- SDO  (RFM22B)
D13/SCK  --- SCK  (RFM22B)

AN0 (DIN)  --- R6 --- A8 (74HC245)
AN1 (DOUT) --- R5_PD + B7 (74HC245)
AN2 --- R5 + JP1_4 (Yellow)
AN3 --- nc

AN4/SDA --- JP4_2 --- SDA (BMP180)
AN5/SCL --- JP4_3 --- SCL (BMP180)

AN6 --- JP5_2 --- DATA (SHT15)
AN7 --- JP5_3 --- SCK (SHT15)

GND --- JP3_1
VCC/3.3V --- JP3_2
```


**The [74HC245][5] connections (see [full schematics][6] for details)**
```cpp
DIR --- GND
/G --- D3 (AT328)
B0 --- R1_PU + D4 (AT328)
B1 --- R2_PU + D5 (AT328)
B2 --- R1_PD + D6 (AT328)
B3 --- R2_PD + D7 (AT328)
B4 --- R3_PD + D8 (AT328)
B5 --- R4_PD + D9 (AT328)
B6 --- R5_PD + AN1 (AT328)
B7 --- R6_PD + JP2_1 (ReedRelay)
A0 --- JP1_2 (Gray1)
A1 --- JP1_1 (Blue2)
A2 --- R1 --- JP1_8
A3 --- R2 --- JP1_7
A4 --- R3 --- JP1_6
A5 --- R4 --- JP1_5
A6 --- R5 --- JP1_4
A7 --- R6 --- AN0 (AT328)
```

[1]: https://www.arduino.cc/en/Main/ArduinoBoardProMini
[2]: https://www.sparkfun.com/products/retired/10154
[3]: https://github.com/sparkfun/BMP180_Breakout_Arduino_Library
[4]: https://github.com/sparkfun/SHT15_Breakout
[5]: http://www.futurlec.com/74HC/74HC245.shtml
[6]: https://github.com/istvanzk/WStation/docs/arduino_wstation.pdf
