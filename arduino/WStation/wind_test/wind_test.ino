/* Wind direction & speed sensor test sketch
 
Sketch uses 5,146 bytes (16%) of program storage space. Maximum is 30,720 bytes.
Global variables use 282 bytes (13%) of dynamic memory, leaving 1,766 bytes for local variables. Maximum is 2,048 bytes. 

Connections:

Requires:

Output:

*/

#include <Arduino.h>

// Directions [deg from North]
#define REF_ANGLE -22.5
#define DELTA_ANGLE 22.5  

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


// Control pins
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

  set_pci();
  	
  Serial.begin(9600);
  Serial.println("REBOOT");

  // Initialize ports
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
  
  Serial.println("WIND sensor pins init");
  
}

void loop()
{
  int windDirSensValue = 1023;
  //float dir_angle = 0.0;
  uint8_t dir_val = 0;
  boolean dir_flag = false;
  	
  // Enable wind sensor interface
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



  // Disable wind sensor interface
  // When disabled no speed and direction measurements are possible!
  //digitalWrite(windIntfEnable, HIGH);


  // Write to output	
  //Serial.print("AN6: ");
  //Serial.println(windDirSensValue);  
  Serial.print("dir_val = ");
  Serial.println(dir_val);
  //Serial.print("Angle: ");
  //Serial.println(dir_angle);

  Serial.print("Speed: ");
  Serial.println(AnemometerSpeed);


  delay(1000);
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
