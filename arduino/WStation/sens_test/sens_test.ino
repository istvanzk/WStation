/* Wind direction + speed sensor (OBH eHome Wind Meter), SFE_BMP180 and SHT15 test sketch
 
With all Serial.print()'s: 
Sketch uses 11,466 bytes (37%) of program storage space. Maximum is 30,720 bytes.
Global variables use 963 bytes (47%) of dynamic memory, leaving 1,085 bytes for local variables. Maximum is 2,048 bytes.

Without Serial.print()'s:
Sketch uses 7,680 bytes (25%) of program storage space. Maximum is 30,720 bytes.
Global variables use 755 bytes (36%) of dynamic memory, leaving 1,293 bytes for local variables. Maximum is 2,048 bytes.


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

Requires:
Sparkfun BMP180 Arduino Library: https://github.com/sparkfun/BMP180_Breakout
SparkFun_SHT1X Arduino Library: https://github.com/sparkfun/SHT15_Breakout/


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
#include <SFE_BMP180.h>
#include <SHT1X.h>
#include <Wire.h>

// SFE_BMP180 object "pressure":
SFE_BMP180 pressure;

// SHT1x object, "sht15"
SHT1x sht15(A2, A3); //Data, SCK

// Reference altitude of Frydendal, Aalborg in meters
#define ALTITUDE 40.0 


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
volatile unsigned long int AnemometerPeriodTotal = 0;
volatile unsigned long int AnemometerPeriodReadingCount = 0;
volatile unsigned long int lastAnemometerPeriodReadingCount = 11111;
volatile unsigned int zeroSpeedCount = 0;
volatile unsigned int GustPeriod = 65535;
volatile unsigned int lastAnemometerEvent = 0;



void setup()
{
    	
  Serial.begin(9600);
  Serial.println("REBOOT");

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
  
  Serial.println("WIND sensor ports init");


  // Initialize the BMP180 sensor (it is important to get calibration values stored on the device).
  if (pressure.begin())
    Serial.println("BMP180 init");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }
  
  Serial.println("SHT15 init");
  
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
  float tempC = 0;
  float tempF = 0;
  float humidity = 0;
  

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
        else Serial.println("error retrieving BMP pressure measurement\n");
      }
      else Serial.println("error starting BMP pressure measurement\n");
    }
    else Serial.println("error retrieving BMP temperature measurement\n");
  }
  else Serial.println("error starting BMP temperature measurement\n");

  // Read values from the SHT15 sensor
  tempC = sht15.readTemperatureC();
  tempF = sht15.readTemperatureF();
  humidity = sht15.readHumidity();  
  
    	
  // Enable Wind Meter interface
  digitalWrite(windIntfEnable, LOW);
  //delay(100);
  
  
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
		delay(1);
		
		// Read wind direction	
		windDirSensValue = analogRead(windDirSens);	

		// Power off
		digitalWrite(windDirGrp2Seq[grp2], LOW);    
		
		//Serial.print("grp4:grp2 = ");
		//Serial.print(grp4);
		//Serial.print(":");		
		//Serial.println(grp2);
		//Serial.print("AN6: ");
        //Serial.println(windDirSensValue);  
		
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


  // Write to output	

/*  
  // Print BMP180 info	
  Serial.println("BMP data:");
    
  Serial.print(" temperature: ");
  Serial.print(T,2);
  Serial.print(" C, ");
  Serial.print((9.0/5.0)*T+32.0,2);
  Serial.println(" F");
  
  Serial.print(" absolute pressure: ");
  Serial.print(P,2);
  Serial.print(" mb, ");
  Serial.print(P*0.0295333727,2);
  Serial.println(" inHg");
  
  Serial.print(" relative (sea-level) pressure: ");
  Serial.print(p0,2);
  Serial.print(" mb, ");
  Serial.print(p0*0.0295333727,2);
  Serial.println(" inHg");
  
  Serial.print(" computed altitude: ");
  Serial.print(a,0);
  Serial.print(" meters, ");
  Serial.print(a*3.28084,0);
  Serial.println(" feet");

  // Print SHT15 info	
  Serial.println("SHT data:");
  
  Serial.print(" temperature: ");
  Serial.print(tempC);
  Serial.print(" C, ");
  Serial.print(tempF);
  Serial.println(" F");
  Serial.print(" humidity: ");
  Serial.print(humidity); 
  Serial.println("%");
  

  // Print Wind Meter info 
  Serial.println("Wind data:");
    
  Serial.print(" direction: ");
  Serial.println(dir_val);
  Serial.print(" speed: ");
  Serial.println(AnemometerSpeed);
*/

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
		unsigned int timeAnemometerEvent = millis(); 
		unsigned int period = 0;
	 
		// If there's never been an event before (first time through), then just capture it
		if(lastAnemometerEvent != 0) {
			// Calculate time since last event (account for overflow)
			if(timeAnemometerEvent < lastAnemometerEvent) {
				period = timeAnemometerEvent + 65535 - lastAnemometerEvent;
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
		
		//Serial.print("ISR: ");	
		//Serial.print(lastAnemometerEvent);
		//Serial.print(", ");
		//Serial.print(AnemometerPeriodReadingCount);
		//Serial.print(", ");
		//Serial.println(AnemometerPeriodTotal);
			
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
