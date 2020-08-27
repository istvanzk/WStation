/* Home Weather Sation code for Arduino
 
Collects measurements of:
  * wind direction & speed (OBH eHome Wind Meter)
  * pressure (BMP180)
  * temperature & humidity (SHT15)
and transmits combined data using RFM22B radio module.

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
A0		-> Wind speed Reed relay Output
A1 (DO)	-> Wind direction IR Sensor Pw
A7 		-> Wind direction IR Sensor Output

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

Notes:

The RH_ASK driver uses a timer-driven interrupt to generate 8 interrupts per bit period. By default it takes over Timer 1.
This sketch uses Timer 2, therefore the RH_ASK timer does not need to be changed.

Without Serial.print()'s:
  Sketch uses 18368 bytes (59%) of program storage space. Maximum is 30720 bytes.
  Global variables use 1027 bytes (50%) of dynamic memory, leaving 1021 bytes for local variables. Maximum is 2048 bytes.

With all Serial.print()'s:
  Sketch uses 21806 bytes (70%) of program storage space. Maximum is 30720 bytes.
  Global variables use 1043 bytes (50%) of dynamic memory, leaving 1005 bytes for local variables. Maximum is 2048 bytes.

*/

#include <Arduino.h>
#include <stdlib.h>
#include <SPI.h>
#include <SFE_BMP180.h>
#include <SHT1X.h>
#include <Wire.h>

// Enable Serial.print() for debugging
#define SERIAL_PRINT

// Loop delay value (milliseconds)
#define LOOP_DELAY 5000

// Reference altitude (m) of Home Weather Station
#define ALTITUDE 43.0

// Anemometer scaling (meters per revolution), for sensor with 1 pulse per revolution
#define ANEMOMETER_MPR 0.345708 

// RFM22B communication addresses
#define RF_GROUP_ID   22 // All devices
#define RF_GATEWAY_ID 1  // Server ID (where to send packets)
#define RF_NODE_ID    10 // Client ID (device sending the packets)


// Implementation specific parameters. 
// DO NOT change these unless you absolutely need to and you know what you are doing!

// RF22B messge length (bytes). The same length must be set in RH_RF22.h !
#define RH_RF22_MAX_MESSAGE_LEN 20 
#include <RH_RF22.h>
#include <RHReliableDatagram.h>

// SFE_BMP180 object "pressure":
SFE_BMP180 pressure;

// SHT15 object
SHT1x sht15(A2, A3); //Data, SCK

// RFM22B radio driver
RH_RF22 driver;
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, RF_NODE_ID);

uint8_t tx_data[RH_RF22_MAX_MESSAGE_LEN+1];
// Dont put this on the stack:
uint8_t rx_buf[RH_RF22_MAX_MESSAGE_LEN+1];

// Hwd status flags
boolean windM_OK  = false;
boolean bmp180_OK = false;
boolean sht15_OK  = false;
boolean rfm22_OK  = false;
boolean rfm22_sentOK = false;

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
        T -= 1;
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
	  tempC = sht15.readTemperatureC() - 1;
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

#if defined(SERIAL_PRINT)
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

#if defined(SERIAL_PRINT)
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
  Serial.print(F(" last time: "));  
  Serial.println(lastAnemometerEvent);
  Serial.print(F(" period count: "));
  //Serial.println(AnemometerPeriodReadingCount);

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
      {
        rfm22_sentOK = true;
#if defined(SERIAL_PRINT)
        Serial.print(F("sendtoWait ACK: "));
#endif
      }
      else
      {
        rfm22_sentOK = false;
#if defined(SERIAL_PRINT)
        Serial.print(F("sendtoWait failed: "));
#endif        
      }
#if defined(SERIAL_PRINT)      
      Serial.println(manager.headerId(), HEX);
      Serial.println();
#endif
  }

  delay(LOOP_DELAY);
}


void set_pci()
{
	cli();

	// Enable Port C (PCINT8 - PCINT14) Pin Change Interrupts
	PCICR |= B00000010;
	// Mask PCINT8 (A0)
	PCMSK1 |= B00000001;

  // Set up TIMER2
  TCCR2A = B00000000;
  TCCR2B = B00000000; 
  // Set CS20 & CS22 bits for 1024 prescaler: 1024 * 256 * 1/(8*10^6) = 0.032768 msec
  TCCR2B |= (1 << CS20) | (1 << CS22);
  // Enable the overflow ISR
  TIMSK2 |= (1 << TOIE2); 
  // Reset the counter
  TCNT2 = 0;

  sei();
}


// Interrupt handler for Port C, PCINT8 - PCINT14
ISR(PCINT1_vect)
{
	// Count only at LOW to HIGH transition (pulse start)
  if( (PINC & (1 << PINC0)) == 1 ){

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
			// Ignore switch-bounce glitches less than 50ms after initial edge
			if(period < 50) {
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
  
  // Estimate wind speed only ~1 per second
  ovfSpeedCount++;
  if (ovfSpeedCount < 30) 
    return;

  // Estimate wind speed  
  ovfSpeedCount = 0;
	if(AnemometerPeriodTotal > 0) {
		AnemometerSpeed =  ANEMOMETER_MPR * 1000.0 * float(AnemometerPeriodReadingCount) / float(AnemometerPeriodTotal);
	}
 
	if(AnemometerPeriodReadingCount == lastAnemometerPeriodReadingCount && lastAnemometerPeriodReadingCount > 0)
	{
		zeroSpeedCount++;
		if(zeroSpeedCount > 10)
		{
			zeroSpeedCount = 0;
			AnemometerSpeed = 0.0;
			lastAnemometerEvent = 0;
			AnemometerPeriodTotal = 0;
			AnemometerPeriodReadingCount = 0;
			lastAnemometerPeriodReadingCount = 0;
			GustPeriod = 65535;
		}
	}
  else
  	lastAnemometerPeriodReadingCount = AnemometerPeriodReadingCount;
  
}