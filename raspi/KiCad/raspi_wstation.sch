EESchema Schematic File Version 4
LIBS:raspi_wstation-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Weather Station - Raspberry Pi A+"
Date "2020-02-24"
Rev "1.0"
Comp ""
Comment1 "This is the Raspberry Pi A+ part of the WS project"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_02x20_Odd_Even J0
U 1 1 5E53CFBA
P 5400 3900
F 0 "J0" H 5450 4900 50  0000 C CNN
F 1 "Raspberry Pi GPIO" H 5425 2775 50  0000 C CNN
F 2 "" H 5400 3900 50  0001 C CNN
F 3 "~" H 5400 3900 50  0001 C CNN
	1    5400 3900
	1    0    0    -1  
$EndComp
$Comp
L rfm22b:RFM22B Breakout1
U 1 1 5E549B5B
P 3600 3375
F 0 "Breakout1" H 3900 4000 60  0000 C CNN
F 1 "RFM22B" H 3900 3900 60  0000 C CNN
F 2 "" H 3600 3375 60  0000 C CNN
F 3 "" H 3600 3375 60  0000 C CNN
	1    3600 3375
	-1   0    0    -1  
$EndComp
$Comp
L rtc_DS3231:DS3231_BreakOut Breakout2
U 1 1 5E550939
P 4125 4575
F 0 "Breakout2" H 3950 5100 50  0000 L CNN
F 1 "RTC_DS3231" H 3900 5025 50  0000 L CNN
F 2 "DS3231 Breakout (Adafruit)" H 4125 4725 50  0001 C CNN
F 3 "" H 4125 4575 50  0001 C CNN
	1    4125 4575
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5200 2550 5200 3000
Wire Wire Line
	6600 2900 6600 2775
Wire Wire Line
	4975 2775 4975 3300
Wire Wire Line
	4975 3300 5200 3300
Wire Wire Line
	6400 2900 6400 2850
Wire Wire Line
	4900 2850 4900 3500
Wire Wire Line
	4900 3500 5200 3500
$Comp
L onoff_shim:OnOff_SHIM Breakout0
U 1 1 5E542556
P 6600 3300
F 0 "Breakout0" V 5900 3500 50  0000 R CNN
F 1 "OnOff_SHIM" V 6000 3525 50  0000 R CNN
F 2 "ON-OFF Shim (Pimoroni)" H 6900 2950 50  0001 C CNN
F 3 "" H 6600 3300 50  0001 C CNN
	1    6600 3300
	0    1    1    0   
$EndComp
Wire Wire Line
	5200 2550 6900 2550
Wire Wire Line
	6900 2550 6900 2900
Wire Wire Line
	4975 2775 6600 2775
Wire Wire Line
	4900 2850 6400 2850
Wire Wire Line
	6700 4250 6700 4375
Wire Wire Line
	6700 4375 6200 4375
Wire Wire Line
	6200 4375 6200 3200
Wire Wire Line
	6200 3200 5700 3200
Wire Wire Line
	6800 4250 6900 4250
Wire Wire Line
	6800 4250 6800 4475
Wire Wire Line
	6800 4475 6125 4475
Wire Wire Line
	6125 4475 6125 3000
Wire Wire Line
	6125 3000 5700 3000
Connection ~ 6800 4250
Wire Wire Line
	5700 3100 5700 3000
Connection ~ 5700 3000
Wire Wire Line
	5200 3100 5050 3100
Wire Wire Line
	5050 3100 5050 4525
Wire Wire Line
	5200 3200 5125 3200
Wire Wire Line
	5125 3200 5125 4425
Wire Wire Line
	5125 4425 4525 4425
Wire Wire Line
	5200 3800 4975 3800
Wire Wire Line
	4975 3800 4975 4225
Wire Wire Line
	4975 4225 4525 4225
$Comp
L Switch:SW_Push_Dual SW0
U 1 1 5E56054A
P 8025 3250
F 0 "SW0" V 7979 3398 50  0000 L CNN
F 1 "ON-OFF" V 8070 3398 50  0000 L CNN
F 2 "" H 8025 3450 50  0001 C CNN
F 3 "~" H 8025 3450 50  0001 C CNN
	1    8025 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	7200 3350 7575 3350
Wire Wire Line
	7575 3350 7575 3050
Wire Wire Line
	7825 3450 8025 3450
$Comp
L Connector:USB_B_Micro J1
U 1 1 5E564C14
P 7850 3900
F 0 "J1" H 7620 3889 50  0000 R CNN
F 1 "USB_B_Micro on OnOff SHIM" H 7620 3798 50  0000 R CNN
F 2 "" H 8000 3850 50  0001 C CNN
F 3 "~" H 8000 3850 50  0001 C CNN
	1    7850 3900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7575 3050 7825 3050
Connection ~ 7825 3450
Wire Wire Line
	7200 3450 7825 3450
Connection ~ 7825 3050
Wire Wire Line
	7825 3050 8025 3050
Wire Wire Line
	7200 3700 7550 3700
Wire Wire Line
	7325 3800 7325 4375
Wire Wire Line
	7325 4375 7850 4375
Wire Wire Line
	7850 4375 7850 4300
Wire Wire Line
	7325 3800 7200 3800
Wire Wire Line
	5200 4100 4600 4100
Wire Wire Line
	4600 4100 4600 3275
Wire Wire Line
	4600 3275 4100 3275
Wire Wire Line
	5200 3900 4700 3900
Wire Wire Line
	4700 3900 4700 3175
Wire Wire Line
	4700 3175 4100 3175
Wire Wire Line
	5200 4000 4825 4000
Wire Wire Line
	4825 4000 4825 3075
Wire Wire Line
	4825 3075 4100 3075
Wire Wire Line
	5050 4525 4525 4525
Wire Wire Line
	4100 3425 4175 3425
Wire Wire Line
	4175 3425 4175 3875
Wire Wire Line
	4175 3875 3700 3875
Connection ~ 3600 3875
Wire Wire Line
	3600 3875 3500 3875
Connection ~ 3700 3875
Wire Wire Line
	3700 3875 3600 3875
Text GLabel 4250 3575 2    50   Input ~ 0
CE0
Wire Wire Line
	4250 3575 4100 3575
Text GLabel 4250 3675 2    50   Input ~ 0
nIRQ
Wire Wire Line
	4250 3675 4100 3675
Text GLabel 5775 4000 2    50   Input ~ 0
nIRQ
Text GLabel 5775 4100 2    50   Input ~ 0
CE0
Wire Wire Line
	5775 4100 5700 4100
Wire Wire Line
	5775 4000 5700 4000
Text Label 2650 3450 0    50   ~ 0
TX_RFM22B
Text Label 2650 3550 0    50   ~ 0
RX_RFM22B
Text Label 3000 3650 0    50   ~ 0
nc
Wire Wire Line
	3100 3075 2850 3075
Wire Wire Line
	2850 3075 2850 2350
Text Label 2875 2425 0    50   ~ 0
16.4cm@434MHz
Wire Wire Line
	5200 2550 3600 2550
Connection ~ 5200 2550
Wire Wire Line
	3600 2550 3600 2875
Text Label 4550 2525 0    50   ~ 0
3V3
$Comp
L power:GND #PWR?
U 1 1 5E5AEE58
P 3600 4025
F 0 "#PWR?" H 3600 3775 50  0001 C CNN
F 1 "GND" H 3605 3852 50  0000 C CNN
F 2 "" H 3600 4025 50  0001 C CNN
F 3 "" H 3600 4025 50  0001 C CNN
	1    3600 4025
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 4025 3600 3875
$Comp
L power:GND #PWR?
U 1 1 5E5B0BAA
P 7850 4525
F 0 "#PWR?" H 7850 4275 50  0001 C CNN
F 1 "GND" H 7855 4352 50  0000 C CNN
F 2 "" H 7850 4525 50  0001 C CNN
F 3 "" H 7850 4525 50  0001 C CNN
	1    7850 4525
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 4375 7850 4525
Connection ~ 7850 4375
Text Label 6350 4325 0    50   ~ 0
na
Text Label 6450 4325 0    50   ~ 0
na
Text Label 6550 4325 0    50   ~ 0
na
Text Label 6650 2875 0    50   ~ 0
na
Text Label 6750 2875 0    50   ~ 0
na
$EndSCHEMATC
