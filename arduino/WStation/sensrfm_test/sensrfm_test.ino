/* Wind direction + speed sensor (OBH eHome Wind Meter), SFE_BMP180, SHT15 and RFM22B test sketch

With all Serial.print()'s:
Sketch uses 20,042 bytes (65%) of program storage space. Maximum is 30,720 bytes.
Global variables use 1,018 bytes (49%) of dynamic memory, leaving 1,030 bytes for local variables. Maximum is 2,048 bytes.

Without Serial.print()'s:
Sketch uses 16,898 bytes (55%) of program storage space. Maximum is 30,720 bytes.
Global variables use 1,001 bytes (48%) of dynamic memory, leaving 1,047 bytes for local variables. Maximum is 2,048 bytes.

BMP180 Connections:
GND  -> GND
Vcc  -> Vcc
DATA -> A4/SDA
SCK  -> A5/SCL

SHT15 Connections:
GND  -> GND
Vcc  -> Vcc
DATA -> A2
SCK  -> A3

OBH eHome Wind Meter Connections:
D3    	-> Enable 74HC245
D4-D9 	-> Ports B0-B7 (input) 74HC245, IR LEDs Pw
A0		-> Wind speed Reed relay Output
A1 (DO)	-> Wind direction IR Sensor Pw
A6 		-> Wind direction IR Sensor Output

RFM-22B Connections:
                GND----------GND-\ (ground in)
                             SDN-/ (shutdown in)
                3V3----------VCC   (3.3V in)
interrupt 0 pin D2-----------NIRQ  (interrupt request out)
         SS pin D10----------NSEL  (chip select in)
        SCK pin D13----------SCK   (SPI clock in)
       MOSI pin D11----------SDI   (SPI Data in)
       MISO pin D12----------SDO   (SPI data out)
						  /--GPIO0 (GPIO0 out to control transmitter antenna TX_ANT)
						  \--TX_ANT (TX antenna control in) RFM22B only
						  /--GPIO1 (GPIO1 out to control receiver antenna RX_ANT)
						  \--RX_ANT (RX antenna control in) RFM22B only


Requires:
Sparkfun BMP180 Arduino Library: https://github.com/sparkfun/BMP180_Breakout
SparkFun_SHT1X Arduino Library: https://github.com/sparkfun/SHT15_Breakout/
RadioHead RH_RF22 Arduino Library: http://www.airspayce.com/mikem/arduino/RadioHead/index.html


Output:

BMP data:
 temperature: 19.11 C, 66.39 F
 absolute pressure: 1017.10 mb, 30.04 inHg
 relative (sea-level) pressure: 1021.93 mb, 30.18 inHg
 computed altitude: 40 meters, 131 feet
SHT data:
 temperature: 18.94 C, 66.13 F
 humidity: 67.77%
Wind data:
 direction: 14
 speed: 0.13

*/

#include <Arduino.h>
#include <stdlib.h>
#include <SPI.h>
#include <SFE_BMP180.h>
#include <SHT1X.h>
#include <Wire.h>

#define RH_RF22_MAX_MESSAGE_LEN 20 //Max=255
#include <RH_RF22.h>
#include <RHReliableDatagram.h>

// Enable Serial.print()' s
#define SERIAL_PRINT

// Directions [deg from North]
#define REF_ANGLE -22.5
#define DELTA_ANGLE 22.5

// Reference altitude of Frydendal, Aalborg in meters
#define ALTITUDE 40.0

// RFM22B communication addresses
#define RF_GROUP_ID   22 // All devices
#define RF_GATEWAY_ID 1  // Server ID (where to send packets)
#define RF_NODE_ID    10 // Client ID (device sending the packets)

/*
const char s_N[] PROGMEM = "N";
const char s_NNE[] PROGMEM = "NNE";
const char s_NE[] PROGMEM = "NE";
const char s_NEE[] PROGMEM = "NEE";
const char s_E[] PROGMEM = "E";

const char s_SEE[] PROGMEM = "SEE";
const char s_SE[] PROGMEM = "SE";
const char s_SSE[] PROGMEM = "SSE";
const char s_S[] PROGMEM = "S";

const char s_SSW[] PROGMEM = "SSW";
const char s_SW[] PROGMEM = "SW";
const char s_SWW[] PROGMEM = "SWW";
const char s_W[] PROGMEM = "W";

const char s_NWW[] PROGMEM = "NWW";
const char s_NW[] PROGMEM = "NW";
const char s_NNW[] PROGMEM = "NNW";

const char* const WindDir_string_table[] PROGMEM=
{
	s_NNE,
	s_NE,
	s_NEE,
	s_E,
	s_SEE,
	s_SE,
	s_SSE,
	s_S,
	s_SSW,
	s_SW,
	s_SWW,
	s_W,
	s_NWW,
	s_NW,
	s_NNW,
	s_N,
};
 */

// SFE_BMP180 object "pressure":
SFE_BMP180 pressure;

// SHT15 object
SHT1x sht15(A2, A3); //Data, SCK

// RFM22B radio driver
RH_RF22 driver;
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, RF_NODE_ID);
// The RH_ASK driver uses a timer-driven interrupt to generate 8 interrupts per bit period. By default it takes over Timer 1.
// You can force it to use Timer 2 instead by enabling the define RH_ASK_ARDUINO_USE_TIMER2 near the top of RH_ASK.cpp

uint8_t tx_data[RH_RF22_MAX_MESSAGE_LEN+1];
// Dont put this on the stack:
uint8_t rx_buf[RH_RF22_MAX_MESSAGE_LEN+1];

// Hwd status flags
boolean bmp180_OK = false;
boolean sht15_OK  = false;
boolean rfm22_OK  = false;
boolean windM_OK  = false;

// Wind Meter Control pins
int windIntfEnable   = 3;  //active low; enables the 74HC245 3-state buffers
int windSpeedSens    = A0; //use as digital input
int windDirGrp2468Pw = 5;  //1D2,4,6,8; active low
int windDirGrp1357Pw = 4;  //1D1,3,5,7; active low
int windDirGrp12Pw   = 9;  //1D1+2; active high
int windDirGrp34Pw   = 8;  //1D3+4; active high
int windDirGrp56Pw   = 7;  //1D5+6; active high
int windDirGrp78Pw   = 6;  //1D7+8; active high
int windDirSensPw    = A1; //use as digital output; active high
int windDirSens      = A6; //use as analog input

// Wind Meter variables
int windDirGrp4Seq[2] = {windDirGrp1357Pw, windDirGrp2468Pw};
int windDirGrp2Seq[4] = {windDirGrp12Pw, windDirGrp34Pw, windDirGrp56Pw, windDirGrp78Pw};

float AnemometerSpeed = 0.0;
float AnemometerScaleMPS = 0.33; // Windspeed [meters/second] for 1 pulse (revolution) every second
volatile unsigned long AnemometerPeriodTotal = 0;
volatile unsigned long AnemometerPeriodReadingCount = 0;
volatile unsigned long lastAnemometerPeriodReadingCount = 11111;
volatile uint8_t zeroSpeedCount = 0;
volatile unsigned long GustPeriod = 65535;
volatile unsigned long lastAnemometerEvent = 0;



void setup()
{
  // Hwd status flags
  bmp180_OK = false;
  sht15_OK  = false;
  rfm22_OK  = false;
  windM_OK  = false;

#if defined(SERIAL_PRINT)
  Serial.begin(9600);
  Serial.println(F("REBOOT"));
#endif

  // Initialize RFM22B
  // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36, 8dBm Tx power
  if (manager.init())
  {
#if defined(SERIAL_PRINT)
    Serial.println(F("RFM22B initialized"));
#endif
    rfm22_OK = true;
  }

  // Initialize the BMP180 sensor (it is important to get calibration values stored on the device).
  if (pressure.begin())
  {
#if defined(SERIAL_PRINT)
    Serial.println(F("BMP180 initialized"));
#endif
    bmp180_OK = true;
  }

#if defined(SERIAL_PRINT)
  Serial.println(F("SHT15 init"));
#endif
  sht15_OK = true;

   // Configure pin change interrupt (wind speed meter)
  set_pci();

  // Initialize Wind Meter ports
  pinMode(windIntfEnable, OUTPUT);
  digitalWrite(windIntfEnable, HIGH);

  pinMode(windSpeedSens, INPUT);
  digitalWrite(windSpeedSens, LOW);

  for( uint8_t grp4 = 0; grp4 < 2; grp4++ )
  {
	pinMode(windDirGrp4Seq[grp4], OUTPUT);
  	digitalWrite(windDirGrp4Seq[grp4], HIGH);
  }
  for( uint8_t grp2 = 0; grp2 < 4; grp2++ )
  {
  	pinMode(windDirGrp2Seq[grp2], OUTPUT);
  	digitalWrite(windDirGrp2Seq[grp2], LOW);
  }

  pinMode(windDirSensPw, OUTPUT);
  digitalWrite(windDirSensPw, LOW);

  analogReference(DEFAULT);

#if defined(SERIAL_PRINT)
  Serial.println(F("WIND meter initialized"));
#endif
  windM_OK = true;

}

void loop()
{
  // Wind Meter variables
  int windDirSensValue = 1023;
  //float dir_angle = 0.0;
  uint8_t dir_val = 0;
  boolean dir_flag = false;

  // BMP180+SHT15 variables
  char status;
  double T = 0, P = 0, p0 = 0, a = 0;
  float tempC = 0, tempF = 0, humidity = 0;

  char buf[10];

  // Read barometric sensor
  if (bmp180_OK)
  {
	  status = pressure.startTemperature();
	  if (status != 0)
	  {
		// Wait for the measurement to complete:
		delay(status);

		// Retrieve the completed temperature measurement:
		// Note that the measurement is stored in the variable T.
		// Function returns 1 if successful, 0 if failure.

		status = pressure.getTemperature(T);
		if (status != 0)
		{
		  // Start a pressure measurement:
		  // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
		  // If request is successful, the number of ms to wait is returned.
		  // If request is unsuccessful, 0 is returned.

		  status = pressure.startPressure(3);
		  if (status != 0)
		  {
			// Wait for the measurement to complete:
			delay(status);

			// Retrieve the completed pressure measurement:
			// Note that the measurement is stored in the variable P.
			// Note also that the function requires the previous temperature measurement (T).
			// (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
			// Function returns 1 if successful, 0 if failure.

			status = pressure.getPressure(P,T);
			if (status != 0)
			{
			  // The pressure sensor returns abolute pressure, which varies with altitude.
			  // To remove the effects of altitude, use the sealevel function and your current altitude.
			  // This number is commonly used in weather reports.
			  // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
			  // Result: p0 = sea-level compensated pressure in mb
			  p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)

			  // On the other hand, if you want to determine your altitude from the pressure reading,
			  // use the altitude function along with a baseline pressure (sea-level or other).
			  // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
			  // Result: a = altitude in m.
			  a = pressure.altitude(P,p0);

			}
#if defined(SERIAL_PRINT)
			else Serial.println(F("error retrieving BMP pressure measurement\n"));
#endif
		  }
#if defined(SERIAL_PRINT)
		  else Serial.println(F("error starting BMP pressure measurement\n"));
#endif
		}
#if defined(SERIAL_PRINT)
		else Serial.println(F("error retrieving BMP temperature measurement\n"));
#endif
	  }
#if defined(SERIAL_PRINT)
	  else Serial.println(F("error starting BMP temperature measurement\n"));
#endif
  }

  // Read humidity sensor
  if (sht15_OK)
  {
	  tempC = sht15.readTemperatureC();
	  //tempF = sht15.readTemperatureF();
	  humidity = sht15.readHumidity();
  }


  // Read wind meter sensors
  if (windM_OK)
  {
	  // Enable Wind Meter interface
	  digitalWrite(windIntfEnable, LOW);

	  // Power on the direction sensor
	  digitalWrite(windDirSensPw, HIGH);

	  // Scan sequentially Grp1357 and Grp2468
	  for( uint8_t grp4 = 0; grp4 < 2; grp4++ )
	  {
		// Enable
		digitalWrite(windDirGrp4Seq[grp4], LOW);

		// Scan sequentially Grp12, Grp34, Grp56 and Grp78
		for( uint8_t grp2 = 0; grp2 < 4; grp2++ )
		{
			// Power on
			digitalWrite(windDirGrp2Seq[grp2], HIGH);

			// Read wind direction
			windDirSensValue = analogRead(windDirSens);

			// Power off
			digitalWrite(windDirGrp2Seq[grp2], LOW);

#if defined(SERIAL_PRINT)
			Serial.print(F("grp4:grp2 = "));
			Serial.print(grp4);
			Serial.print(F(":"));
			Serial.println(grp2);
			Serial.print(F("AN6: "));
			Serial.println(windDirSensValue);
#endif

			// Check sensor reading value
			if ( windDirSensValue < 900 )
			{
				if( dir_flag )
				{
					if( dir_val == 0 && grp2 == 3)
					{
						dir_val = 8 + grp4 + (grp2<<1);
					} else
					{
						dir_val = (dir_val>>1) + grp4 + (grp2<<1);
					}
				} else
				{
					dir_val = (grp4<<1) + (grp2<<2);
				}
				dir_flag = true;
				break; //for grp2
			}
		}

		// Disable
		digitalWrite(windDirGrp4Seq[grp4], HIGH);

	  }

	  // Calculate direction angle
	  //dir_angle = -REF_ANGLE;
	  if( dir_val>0 )
	  {
		//dir_angle = dir_angle + 360 - DELTA_ANGLE*dir_val;
		dir_val = 16 - dir_val + 1;
	  } else
	  {
		dir_val = 1;
	  }

	  // Power off the direction sensor
	  digitalWrite(windDirSensPw, LOW);

	  // Disable Wind Meter interface
	  // When disabled no speed and direction measurements are possible!
	  //digitalWrite(windIntfEnable, HIGH);
  }


  // Construct RFM22B message of 20 Bytes
  tx_data[0] = 0x4E; // N
  tx_data[1] = dir_val;

  tx_data[2] = 0x53; // S
  dtostrf(AnemometerSpeed,4,1,buf);
  tx_data[3] = buf[0]; // 3Bytes= XY.Z
  tx_data[4] = buf[1];
  tx_data[5] = buf[3];

  tx_data[6] = 0x54; // T
  dtostrf((T + tempC)/2,4,1,buf);
  tx_data[7] = buf[0]; // 3Bytes= XY.Z
  tx_data[8] = buf[1];
  tx_data[9] = buf[3];

  tx_data[10] = 0x50; // P
  dtostrf(P,6,1,buf);
  tx_data[11] = buf[0]; // 5Bytes= UVWXY.Z
  tx_data[12] = buf[1];
  tx_data[13] = buf[2];
  tx_data[14] = buf[3];
  tx_data[15] = buf[5];

  tx_data[16] = 0x48; // H
  dtostrf(humidity,4,1,buf);
  tx_data[17] = buf[0]; // 3Bytes= XY.Z
  tx_data[18] = buf[1];
  tx_data[19] = buf[3];

  // Message end char
  //tx_data[20] = 0x0;


#if defined(SERIAL_PRINT)
  // Print BMP180 info
  Serial.println(F("BMP data:"));

  Serial.print(F(" temperature: "));
  Serial.print(T,2);
  Serial.print(F(" C, "));
  Serial.print((9.0/5.0)*T+32.0,2);
  Serial.println(F(" F"));

  Serial.print(F(" absolute pressure: "));
  Serial.print(P,2);
  Serial.print(F(" mb, "));
  Serial.print(P*0.0295333727,2);
  Serial.println(F(" inHg"));

  Serial.print(F(" relative (sea-level) pressure: "));
  Serial.print(p0,2);
  Serial.print(F(" mb, "));
  Serial.print(p0*0.0295333727,2);
  Serial.println(F(" inHg"));

  Serial.print(F(" computed altitude: "));
  Serial.print(a,0);
  Serial.print(F(" meters, "));
  Serial.print(a*3.28084,0);
  Serial.println(F(" feet"));

  // Print SHT15 info
  Serial.println(F("SHT data:"));
  Serial.print(F(" temperature: "));
  Serial.print(tempC);
  Serial.println(F(" C, "));
  //Serial.print(tempF);
  //Serial.println(F(" F"));
  Serial.print(F(" humidity: "));
  Serial.print(humidity);
  Serial.println(F("%"));


  // Print Wind Meter info
  Serial.println(F("Wind data:"));
  Serial.print(F(" direction: "));
  Serial.println(dir_val);
  Serial.print(F(" speed: "));
  Serial.println(AnemometerSpeed);

  // Print RFM22 TX data
  Serial.print(F("TXdata: "));
  for(uint8_t bb=0; bb < RH_RF22_MAX_MESSAGE_LEN; bb++)
  {
	  Serial.print(tx_data[bb], HEX);
	  Serial.print(":");
  }
  Serial.println();
#endif


  // Send a RFM22B message
  if (rfm22_OK)
  {

      if (manager.sendtoWait(tx_data, RH_RF22_MAX_MESSAGE_LEN, RF_GATEWAY_ID))
#if defined(SERIAL_PRINT)
        Serial.print(F("sendtoWait ACK: "));
      else
        Serial.print(F("sendtoWait failed: "));

      Serial.println(manager.headerId(), HEX);
      Serial.println();
#endif
/*
	  if (manager.sendtoWait(tx_data, RH_RF22_MAX_MESSAGE_LEN, RF_GATEWAY_ID))
	  {
		// Now wait for a reply from the server
		uint8_t len = sizeof(rx_buf);
		uint8_t from;
		if (manager.recvfromAckTimeout(rx_buf, &len, 500, &from))
		{
#if defined(SERIAL_PRINT)
		  Serial.print(F("Rx reply from : 0x"));
		  Serial.print(from, HEX);
		  Serial.print(F(": "));
		  Serial.println((char*)rx_buf);
#endif
		}
#if defined(SERIAL_PRINT)
		else Serial.println(F("No reply, is rf22_reliable_datagram_server running?"));
#endif
	  }
#if defined(SERIAL_PRINT)
	  else Serial.println(F("sendtoWait failed"));
#endif
*/
  }

  delay(2000);
}


void set_pci()
{
	cli();

	// https://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/
	// https://www.avrprogrammers.com/howto/int-pin-change
	// Enables Port C (PCINT8 - PCINT14) Pin Change Interrupts
	PCICR |= 0b00000010;
	// Mask PCINT8 (A0)
	PCMSK1 |= 0b00000001;

	// https://arduinodiy.wordpress.com/2012/02/28/timer-interrupts/
	// Set compare match register for ~0.5sec increments
	TCCR1A = B00000000;
	TCCR1B = B00000000;
	OCR1A = 2*3906;// =  0.5*8*10^6/1024 - 1 (must be <65536)
	// Turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS10 & CS12 bits for 1024 prescaler: 1024 * 65535 * 1/(8*10^6) = 8.388480 sec
	TCCR1B |= (1 << CS10) | (1 << CS12);
	// Enable timer compare ISR
	TIMSK1 |= (1 << OCIE1A);
	// Enable the overflow ISR
	//TIMSK1 |= (1 << TOIE1);
	// Reset the counter
	TCNT1 = 0;

	sei();
}


// Interrupt handler for Port C, PCINT8 - PCINT14
ISR(PCINT1_vect)
{
	// Count only at LOW to HIGH transition (pulse start)
	if( digitalRead(A0) == HIGH ) {

		// Based on https://github.com/rpurser47/weatherstation/blob/master/weatherstation.ino
		// Activated by the magnet in the anemometer (1 ticks per rotation)

		// Grab current time
		unsigned long timeAnemometerEvent = millis();
		unsigned long period = 0;

		// If there's never been an event before (first time through), then just capture it
		if(lastAnemometerEvent != 0) {
			// Calculate time since last event (account for overflow, ~50days)
			if(timeAnemometerEvent < lastAnemometerEvent) {
				period = timeAnemometerEvent + 4294967295 - lastAnemometerEvent;
			}
			else {
				period = timeAnemometerEvent - lastAnemometerEvent;
			}
			// Ignore switch-bounce glitches less than 10ms after initial edge
			if(period < 10) {
				return;
			}
			// If the period is the shortest (and therefore fastest windspeed) seen, capture it
			if(period < GustPeriod) {
				GustPeriod = period;
			}

			// Update counters
			AnemometerPeriodTotal += period;
			AnemometerPeriodReadingCount++;
		}

		// Set up for next event
		lastAnemometerEvent = timeAnemometerEvent;
    }
}


// Timer1 comparator interrupt routine (~0.5sec period)
ISR(TIMER1_COMPA_vect)
{
	if(AnemometerPeriodTotal > 0) {
		AnemometerSpeed =  AnemometerScaleMPS * 1000.0 * float(AnemometerPeriodReadingCount) / float(AnemometerPeriodTotal);
	}
	if(AnemometerPeriodReadingCount == lastAnemometerPeriodReadingCount)
	{
		zeroSpeedCount++;
		if(zeroSpeedCount > 10)
		{
			zeroSpeedCount = 0;
			AnemometerSpeed = 0.0;
			lastAnemometerEvent = 0;
			AnemometerPeriodTotal = 0;
			AnemometerPeriodReadingCount = 0;
			lastAnemometerPeriodReadingCount = 1111;
			GustPeriod = 65535;
		}
	}
	lastAnemometerPeriodReadingCount = AnemometerPeriodReadingCount;

}

