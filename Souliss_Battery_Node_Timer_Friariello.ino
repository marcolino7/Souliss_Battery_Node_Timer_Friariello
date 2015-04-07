/**************************************************************************
	Souliss - Sleeping
	
	A battery operated node (Peer) is activated on press of a pushbutton and/or on
	time base, each time the node wake-ups it sends data to the gateway (that is always
	powered on). An additional pin is used to keep the node active for all the time
	required to complete the network configuration.
	
	To have low battery consumption you need a device without voltage regulator and other
	hardware that continuously get power even when the microcontroller is sleeping.  
	
	Run this code on one of the following boards:
	  - Arduino with Nordic nRF24L01 or nRF24L01+
	  
	As option you can run the same code on the following, just changing the
	relevant configuration file at begin of the sketch
	  -	Chibiduino v1
	  - Moteino
	  
***************************************************************************/

#define	VNET_DEBUG_INSKETCH
#define VNET_DEBUG  		1

#define	MaCaco_DEBUG_INSKETCH
#define MaCaco_DEBUG  		1

#define	SLEEPING_INSKETCH
#define wakePin 		2
#define	wakePinINT		0
#define	wakeupTime		0x0004
#define	wakeupCycles	5		

// Configure the framework
#include "bconf/DevDuino_v2.h"				//Use DevDuino Version 2 Board
#include "conf/nRF24L01.h"
#include "conf/Sleep.h"
#include "conf/DynamicAddressing.h"			// Use dynamic addressing

// Include framework code and libraries
#include <SPI.h>
#include "Souliss.h"

//Define the SLOT
#define BATT_LEVEL		0		//Battery Charge Left
#define BATT_VOLT		2		//Battery Voltage
#define TEMPERATURE		4		//Temperature

//Define the PIN to keep node asleep during registration
#define	STAYUP_PIN		3
#define	StayUp()		digitalRead(STAYUP_PIN)

//Setting Deadband to 0 to all Analog Slot
#define	Read_AnalogIn(slot)		Souliss_Logic_T51(memory_map, slot, 0, &data_changed)
#define	Logic_Voltage(slot)		Souliss_Logic_T55(memory_map, slot, 0, &data_changed)
#define	Logic_Temperature(slot)	Souliss_Logic_T52(memory_map, slot, 0, &data_changed)

//Variables
float temp=0;
float vcc_f=0;
float batterycharge=0;


void setup()
{	
	Serial.begin(115200);
	delay(1000);
	Serial.print("DevDuino INIT-");
	Serial.println(millis());
	delay(250);

	Initialize();

	// Set an analog value to measure the battery voltage
	Set_AnalogIn(BATT_LEVEL);
	Set_Voltage(BATT_VOLT);
	Set_Temperature(TEMPERATURE);

	// This board request an address to the gateway at runtime, no need
	// to configure any parameter here.
	SetDynamicAddressing();
	Serial.println("SetDynamicAddressing");
	GetAddress();	
	Serial.println("GetAddress");
	/*****
		The default sleep time is about 30 minutes, you can change this from the
		base/Sleeping.h file, or using the following defines on top of the sketch
		
		#define	SLEEPING_INSKETCH
		#define wakePin 		2
		#define	wakePinINT		0
		#define	wakeupTime		0x00FF
		#define	wakeupCycles	5		

		The time between two wake-ups is equal to wakeupTime*8seconds, using 0x00FF
		the node wakes every 34 minutes. The wakeupCycles is the number of times that the
		node should execute isTimeToSleep() before put itself back in sleep.
	*****/
	
	sleepInit(SLEEPMODE_TIMER);
	
	// Use an external input to force the node in asleep, keep it activated till you
	// configuration (node address and typicals) are completed and you are able to
	// interact with your node from the user interface.
	// Once the setup is completed, release this pin and let the node sleep
	pinMode(STAYUP_PIN, INPUT);

}

void loop()
{   
	// If the node wake-ups then this statement is executed
	if(wasSleeping() || StayUp())
	{	
		
		// Here we start to play
		EXECUTEFAST() {						
			UPDATEFAST();	

			// Here we process the communication
			FAST_PeerComms();		

			// Check every 510ms how many cycles asleep as expired,
			// using default values, the nodes stays asleep 2.5 seconds
			// every 34 minutes
			FAST_510ms() {

				
				getBatteryInfo();
				Serial.print("Send Batt Level:");
				Serial.println(batterycharge);
				Serial.print("Send Batt voltage:");
				Serial.println(vcc_f);
								
				ImportAnalog(BATT_LEVEL, &batterycharge);
				Read_AnalogIn(BATT_LEVEL);

				ImportAnalog(BATT_VOLT, &vcc_f);
				Logic_Voltage(BATT_VOLT);
				
				getTemperature();
				Serial.print("Send Temperature:");
				Serial.println(temp);

				ImportAnalog(TEMPERATURE, &temp);
				Logic_Temperature(TEMPERATURE);
			
				// Back to sleep
				if(isTimeToSleep() && !StayUp())	
				{
					/**************
					Add below the code required to sleep custom devices connected, like:
						- Sensor,
						- Voltage regulator,
						- ...
					**************/
					
					// Sleep microcontroller and radio
					Serial.println("Back to Sleep");
					delay(500);
					sleepNow();

					data_changed=1;
					
					/**************
					Add below the code required to wakeup custom devices connected, like:
						- Sensor,
						- Voltage regulator,
						- ...
					**************/
				}	
			} 
			
		}
	}	
} 

void getTemperature(){
	temp = analogRead(0)*5/1024.0;
	temp = temp - 0.5;
	temp = temp / 0.01;
}

void getBatteryInfo(){
	// Read the input voltage at microcontroller
	long vcc = readVcc();
	vcc_f = (float) vcc;
				
	// Estimate the battery charge, assuming that you are powering with 2 AA
	// alkaline
	float mbatt = 3*(vcc_f/1000) - 7.5;
	batterycharge = 0.66 + 0.022*mbatt + (float)(0.0074*pow(mbatt,3)) + (float)(0.0088*pow(mbatt,9));
				
	// Cut if out of range
	if(batterycharge > 1) batterycharge = 1;
	if(batterycharge < 0) batterycharge = 0;
				
	// Get it in percentage
	batterycharge*=100;
}

long readVcc() {
	
	// Read 1.1V reference against AVcc
	// set the reference to Vcc and the measurement to the internal 1.1V reference
	#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
		ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
		ADMUX = _BV(MUX5) | _BV(MUX0);
	#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
		ADMUX = _BV(MUX3) | _BV(MUX2);
	#else
		ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#endif  

	delay(75); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Start conversion
	while (bit_is_set(ADCSRA,ADSC)); // measuring

	uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
	uint8_t high = ADCH; // unlocks both

	long result = (high<<8) | low;

	result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	return result; // Vcc in millivolts
}