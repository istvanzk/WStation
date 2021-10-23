/* Home Weather Staion code for Arduino
 
Collects measurements from:
  * wind direction & speed (OBH eHome Wind Meter)
  * pressure (BMP180)
  * temperature & humidity (SHT15)
Transmits combined data using a RH69HCW or RFM22B radio module

Author: Istvan Z. Kovacs, 2019-2021

------------------------------------
Connections:

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
A0		  -> Wind speed Reed relay Output
A1 (DO)	-> Wind direction IR Sensor Pw
A7 		  -> Wind direction IR Sensor Output

RFM-69 Connections:
              Arduino      RFM69HCW
                GND----------GND   (ground in)
                3V3----------VCC   (3.3V in)
interrupt 0 pin D2-----------DIO0  (interrupt request out)
         SS pin D10----------NSS   (chip select in)
        SCK pin D13----------SCK   (SPI clock in)
       MOSI pin D11----------SDI   (SPI Data in)
       MISO pin D12----------SDO   (SPI data out)

RFM-22B Connections:
              Arduino       RFM22B
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

------------------------------------
Requires:

Sparkfun BMP180 Arduino Library: https://github.com/sparkfun/BMP180_Breakout
SparkFun_SHT1X Arduino Library: https://github.com/sparkfun/SHT15_Breakout/
RadioHead Arduino Library: http://www.airspayce.com/mikem/arduino/RadioHead/index.html

------------------------------------
Output (example):

Wind data:
 direction: 14
 speed: 0.13
BMP data:
 temperature: 19.11 C, 66.39 F
 absolute pressure: 1017.10 mb, 30.04 inHg
 relative (sea-level) pressure: 1021.93 mb, 30.18 inHg
 computed altitude: 40 meters, 131 feet
SHT data:
 temperature: 18.94 C, 66.13 F
 humidity: 67.77%

------------------------------------
Notes:

The RadioHead/RH_ASK driver uses a timer-driven interrupt to generate 8 interrupts per bit period. By default it takes over Timer 1.
This sketch uses Timer 2, therefore the RH_ASK timer does not need to be changed.

RFM-22B version, with no debug output
  Sketch uses 18368 bytes (59%) of program storage space. Maximum is 30720 bytes.
  Global variables use 1027 bytes (50%) of dynamic memory, leaving 1021 bytes for local variables. Maximum is 2048 bytes.

RFM-22B version, with DEBUG_LEV2 output:
  Sketch uses 21990 bytes (71%) of program storage space. Maximum is 30720 bytes.
  Global variables use 1065 bytes (52%) of dynamic memory, leaving 983 bytes for local variables. Maximum is 2048 bytes.

RFM-69 version, with DEBUG_LEV1 output:
  Sketch uses 20618 bytes (67%) of program storage space. Maximum is 30720 bytes.
  Global variables use 1076 bytes (52%) of dynamic memory, leaving 972 bytes for local variables. Maximum is 2048 bytes.

RFM-69 version, with DEBUG_LEV2 output:
  Sketch uses 21286 bytes (69%) of program storage space. Maximum is 30720 bytes.
  Global variables use 1076 bytes (52%) of dynamic memory, leaving 972 bytes for local variables. Maximum is 2048 bytes.

*/

#include <Arduino.h>
#include <stdlib.h>
#include <SPI.h>
#include <SFE_BMP180.h>
#include <SHT1X.h>
#include <Wire.h>

/// High level configurations

// Select the radio to use.
// RFM69HCW based module
#define RHRF69
// OR
// RFM22B based module
//#define RHRF22

// Select debug info level for Serial.print()
#define DEBUG_LEV1
//#define DEBUG_LEV2


// Loop delay value (milliseconds)
// Short delays are good for testing
// More accurate temperature readings are possible only with long delays
// see http://cdn.sparkfun.com/datasheets/Sensors/Pressure/BMP180.pdf
#define LOOP_DELAY 27000

// Reference altitude (m) of Home Weather Station
#define ALTITUDE 44.0

// Anemometer scaling (meters per revolution), for sensor with 1 pulse per revolution
#define ANEMOMETER_MPR 0.345708 


/// Low level configurations

// RFM69/22B communication & addresses
#define RH_RFM_MESSAGE_LEN 20
#define RF_FREQUENCY  434.0
#define RF_GROUP_ID   22 // All devices
#define RF_GATEWAY_ID 1  // Server ID (where to send packets)
#define RF_NODE_ID    10 // Client ID (device sending the packets)


// Implementation specific parameters. 
// DO NOT change these unless you absolutely need to and you know what you are doing!

#if defined(RHRF69)
// This is a high power radio module (see RH_RF69.h line 71)
#define RFM69_HW
#include <RH_RF69.h>
#elif defined(RHRF22)
// RF22B messge length (bytes). Includes payload only. The same length must be set in RH_RF22.h !
#define RH_RF22_MAX_MESSAGE_LEN 20 
#include <RH_RF22.h>
#endif
#include <RHReliableDatagram.h>

// SFE_BMP180 object "pressure":
SFE_BMP180 pressure;

// SHT15 object
SHT1x sht15(A2, A3); //Data, SCK

// RFM69/22B radio driver
#if defined(RHRF69)
RH_RF69 rfmdrv;
#define RF_TXPOW 15
#elif defined(RHRF22)
RH_RF22 rfmdrv;
#define RF_TXPOW RH_RF22_TXPOW_11DBM
#endif

// RadioHead class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(rfmdrv, RF_NODE_ID);

// Local data buffer
uint8_t tx_data[RH_RFM_MESSAGE_LEN];

// Hwd status flags
boolean rfm_OK    = false;
boolean windM_OK  = false;
boolean bmp180_OK = false;
boolean sht15_OK  = false;


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
int windDirSens      = A7; //use as analog input

// Wind Meter variables
int windDirGrp4Seq[2] = {windDirGrp1357Pw, windDirGrp2468Pw};
int windDirGrp2Seq[4] = {windDirGrp12Pw, windDirGrp34Pw, windDirGrp56Pw, windDirGrp78Pw};

float AnemometerSpeed = 0.0;
volatile unsigned long AnemometerPeriodTotal = 0;
volatile unsigned long AnemometerPeriodReadingCount = 0;
volatile unsigned long lastAnemometerPeriodReadingCount = 0;
volatile uint8_t zeroSpeedCount = 0;
volatile uint8_t ovfSpeedCount = 0;
volatile unsigned long GustPeriod = 65535;
volatile unsigned long lastAnemometerEvent = 42;



void setup()
{
  // Hwd status flags
  rfm_OK    = false;
  bmp180_OK = false;
  sht15_OK  = false;
  windM_OK  = false;

#if defined(DEBUG_LEV1) || defined(DEBUG_LEV2)
  Serial.begin(19200);
  Serial.println(F("REBOOT"));
#endif

  // Initialize RFM
  // Defaults after init are:
  //  RF69: 434.0MHz, AFC BW == RX BW == 500KHz, modulation GFSK_Rb250Fd250, 13dBm Tx power
  //  RF22B: 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36, 8dBm Tx power
  if (manager.init())
  {
    // Adjust transmission/reception parameters
#if defined(RHRF69)
    rfmdrv.setModemConfig(RH_RF69::FSK_Rb2_4Fd4_8);
    rfmdrv.setFrequency(RF_FREQUENCY);
#elif defined(RHRF22)
    rfmdrv.setFrequency(RF_FREQUENCY,0.05);
#endif
    rfmdrv.setTxPower(RF_TXPOW);

#if defined(DEBUG_LEV1) || defined(DEBUG_LEV2)
    Serial.println(F("RFM initialized"));
#endif
    rfm_OK = true;
  }

  // Initialize the BMP180 sensor (it is important to get calibration values stored on the device).
  if (pressure.begin())
  {
#if defined(DEBUG_LEV1) || defined(DEBUG_LEV2)
    Serial.println(F("BMP180 initialized"));
#endif
    bmp180_OK = true;
  }

#if defined(DEBUG_LEV1) || defined(DEBUG_LEV2)
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
 
#if defined(DEBUG_LEV1) || defined(DEBUG_LEV2)
  Serial.println(F("WIND meter initialized"));
#endif
  windM_OK = true;

}

void loop()
{
  // Wind Meter variables
  int windDirSensValue = 1023;
  noInterrupts();
  float windSpeed = AnemometerSpeed;
  interrupts();

  //float dir_angle = 0.0;
  uint8_t dir_val = 0;
  boolean dir_flag = false;

  // BMP180+SHT15 variables
  char status;
  double T = 0, P = 0, p0 = 0, a = 0;
  float tempC = 0, humidity = 0;

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
    			  p0 = pressure.sealevel(P,ALTITUDE);
    
    			  // On the other hand, if you want to determine your altitude from the pressure reading,
    			  // use the altitude function along with a baseline pressure (sea-level or other).
    			  // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
    			  // Result: a = altitude in m.
    			  a = pressure.altitude(P,p0);
    
    			}
#if defined(DEBUG_LEV2)
    			else Serial.println(F("error retrieving BMP pressure measurement\n"));
#endif
	  	  }
#if defined(DEBUG_LEV2)
		    else Serial.println(F("error starting BMP pressure measurement\n"));
#endif
		  }
#if defined(DEBUG_LEV2)
		  else Serial.println(F("error retrieving BMP temperature measurement\n"));
#endif
	  }
#if defined(DEBUG_LEV2)
	  else Serial.println(F("error starting BMP temperature measurement\n"));
#endif
  }

  // Read humidity sensor
  if (sht15_OK)
  {
	  tempC = sht15.readTemperatureC();
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

  			// Read wind direction (avg 5 measurements)
        delay(10);
  			windDirSensValue = analogRead(windDirSens);
        delay(10);
        windDirSensValue += analogRead(windDirSens);
        delay(10);
        windDirSensValue += analogRead(windDirSens);
        delay(10);
        windDirSensValue += analogRead(windDirSens);
        delay(10);
        windDirSensValue += analogRead(windDirSens);
        windDirSensValue = windDirSensValue/5;
     
  			// Power off
  			digitalWrite(windDirGrp2Seq[grp2], LOW);

#if defined(DEBUG_LEV2)
        Serial.print(F("grp4:grp2 = "));
        Serial.print(grp4);
        Serial.print(F(":"));
        Serial.println(grp2);
        Serial.print(F("AN7: "));
        Serial.println(windDirSensValue);
#endif       
  			// Check sensor reading value
  			if ( windDirSensValue < 500 )
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

	  // Power off the direction sensor
	  //digitalWrite(windDirSensPw, LOW);

#if defined(DEBUG_LEV2)
        Serial.print(F("dir_val: "));
        Serial.println(dir_val);        
#endif       

	  // Disable Wind Meter interface
	  // When disabled no speed and direction measurements are possible!
	  //digitalWrite(windIntfEnable, HIGH);

    // Calculate direction angle index
    if( dir_val>0 )
    {
      dir_val = 16 - dir_val + 1;
    } else
    {
      dir_val = 1;
    }

  }



  // Construct RFM message payload of 20 Bytes
  tx_data[0] = 0x4E; // N
  tx_data[1] = dir_val;

  tx_data[2] = 0x53; // S
  dtostrf(windSpeed,4,1,buf);
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
  tx_data[11] = buf[0]; // 5Bytes= VWXY.Z
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


#if defined(DEBUG_LEV1) || defined(DEBUG_LEV2)
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
  Serial.println(F(" C"));
  Serial.print(F(" humidity: "));
  Serial.print(humidity);
  Serial.println(F("%"));


  // Print Wind Meter info
  Serial.println(F("Wind data:"));
  Serial.print(F(" direction: "));
  Serial.println(dir_val);
  Serial.print(F(" speed: "));
  Serial.println(windSpeed);
#if defined(DEBUG_LEV2)  
  Serial.print(F(" last time: "));  
  Serial.println(lastAnemometerEvent);
  //Serial.print(F(" period count: "));
  //Serial.println(AnemometerPeriodReadingCount);
#endif

  // Print RFM TX data
  Serial.print(F("TXdata ("));
  Serial.print(RH_RFM_MESSAGE_LEN);
  Serial.print(F(" bytes): "));
  for(uint8_t bb=0; bb < RH_RFM_MESSAGE_LEN-1; bb++)
  {
	  Serial.print(tx_data[bb], HEX);
	  Serial.print(":");
  }
  Serial.print(tx_data[RH_RFM_MESSAGE_LEN-1], HEX);
  Serial.println();
#endif


  // Send a RFM message
  if (rfm_OK)
  {
      if (manager.sendtoWait(tx_data, RH_RFM_MESSAGE_LEN, RF_GATEWAY_ID))
      {
#if defined(DEBUG_LEV2)
        Serial.print(F("sendtoWait ACK: "));
#endif
      }
      else
      {
#if defined(DEBUG_LEV2)
        Serial.print(F("sendtoWait failed: "));
#endif        
      }
#if defined(DEBUG_LEV2)      
      Serial.println(manager.headerId(), HEX);
      Serial.println();
#endif
  }
  else
  {
#if defined(DEBUG_LEV1) || defined(DEBUG_LEV2)     
      Serial.println(F("Data not sent on RFM!"));
      Serial.println();
#endif        
  }

  delay(LOOP_DELAY);
}


void set_pci()
{
	noInterrupts();

  // Clear all interrupt flags
  PCIFR = B00000000; 
	// Enable Port C (PCINT8 - PCINT14) Pin Change Interrupts
	PCICR |= B00000010;
	// Mask PCINT8 (A0)
	PCMSK1 |= B00000001;

  // Set up TIMER2
  TCCR2A = B00000000;
  TCCR2B = B00000000; 
  // Set CS20 & CS22 bits for 1024 prescaler: 1024 * 256 * 1/(8*10^6) = 32.768 msec
  TCCR2B |= (1 << CS20) | (1 << CS22);
  // Enable the overflow ISR
  TIMSK2 |= (1 << TOIE2); 
  // Reset the counter
  TCNT2 = 0;

  interrupts();
}


// Interrupt handler for Port C, PCINT8 - PCINT14
ISR(PCINT1_vect)
{
	// Count only at LOW to HIGH transition (pulse start)
  // http://gammon.com.au/interrupts#reply6
  if( (PINC & (1 << PINC0)) == 1 ){

		// Based on https://github.com/rpurser47/weatherstation/blob/master/weatherstation.ino
		// Activated by the magnet in the anemometer, 1 tick/pulse per rotation

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
			// Ignore switch-bounce glitches less than 10ms after initial edge (max 35m/s, 125km/h)
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


// Timer2 overflow interrupt routine (0.032768sec period)
ISR(TIMER2_OVF_vect)
{
  TCNT2 = 0;
  
  // Estimate wind speed approx. every 1 second
  ovfSpeedCount++;
  if (ovfSpeedCount < 30) 
    return;

  // Estimate wind speed and reset counters
  ovfSpeedCount = 0;
  
  noInterrupts();

	if(AnemometerPeriodTotal > 0) {
		AnemometerSpeed =  ANEMOMETER_MPR * 1000.0 * float(AnemometerPeriodReadingCount) / float(AnemometerPeriodTotal);
    AnemometerPeriodTotal = 0;
		AnemometerPeriodReadingCount = 0;
	}
 
	if(AnemometerPeriodReadingCount == lastAnemometerPeriodReadingCount && lastAnemometerPeriodReadingCount > 0)
	{
		zeroSpeedCount++;
		if(zeroSpeedCount > 15)
		{
			zeroSpeedCount = 0;
			AnemometerSpeed = 0.0;
			lastAnemometerEvent = 0;
			AnemometerPeriodTotal = 0;
			AnemometerPeriodReadingCount = 0;
			GustPeriod = 65535;
		}
	}
  
  lastAnemometerPeriodReadingCount = AnemometerPeriodReadingCount;
  
  interrupts();
}
