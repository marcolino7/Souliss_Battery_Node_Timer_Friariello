
#define NRF24PINS_INSKETCH
#define	NRF24_RADIOEN		8				// Chip Enable Pin
#define	NRF24_SPICS			7				// SPI Chip Select Pin


#define	SLEEPING_INSKETCH
#define wakePin 		2
#define	wakePinINT		0
#define	wakeupTime		0x0002
#define	wakeupCycles	5		


#include "bconf/StandardArduino.h"			// Use a standard Arduino
#include "conf/nRF24L01.h"
#include "conf/Sleep.h"

#include "Souliss.h"
#include <SPI.h>

//Souliss Addressing
#define network_address         0x6506        // Local address
#define network_my_subnet       0xFF00
#define network_my_supern       0x6501        // RF24 gateway address

#define destination_node		0x6501

//Slot
#define BATT_LEVEL	0

#define NODEADBAND	  0

void setup(){
	Serial.begin(115200);
	delay(1000);
	Serial.print("DevDuino INIT-");
	Serial.println(millis());
	delay(250);

	Souliss_SetAddress(network_address,network_my_subnet,network_my_supern);
	Set_AnalogIn(BATT_LEVEL);	//T55 per misurare il livello di batteria
	HardcodedChannel(destination_node);
	sleepInit(SLEEPMODE_TIMER);

	Serial.print("WaitSubscription-");
	Serial.println(millis());
	WaitSubscription();

	// Stay asleep for a minute and process the communication, in this type the Gateway
	// can retrieve typicals or other informations
	Serial.print("aMinuteToSleep-");
	Serial.println(millis());
	aMinuteToSleep();
	
	
	Serial.print("DevDuino START-");
	Serial.println(millis());
	delay(250);
}

void loop()
{   
	// If the node wake-ups then this statement is executed
	if(wasSleeping())
	{	
		Serial.println("Woked UP");
		delay(150);
		// Here we start to play
		EXECUTEFAST() {
			//Serial.println("EXECUTEFAST");
			UPDATEFAST();
			//Serial.println("UPDATEFAST");

			// Here we process the communication
			FAST_PeerComms();		

			// Check every 510ms how many cycles asleep as expired,
			// using default values, the nodes stays asleep 2.5 seconds
			// every 34 minutes
			FAST_510ms() {

				//Serial.println("FAST_510ms");
					long vcc = readVcc();
					float vcc_f = (float) vcc;
					float vcc_to_send = vcc_f/1000;
				
				ImportAnalog(BATT_LEVEL, &vcc_to_send);
				Read_AnalogIn(BATT_LEVEL);
			
				Serial.print("BATT_LEVEL: ");
				Serial.println(mOutput(BATT_LEVEL));
				Serial.print("BATT_LEVEL+1: ");
				Serial.println(mOutput(BATT_LEVEL+1));
				

				// Back to sleep
				if(isTimeToSleep())	
				{
					/**************
					Add below the code required to sleep custom devices connected, like:
						- Sensor,
						- Voltage regulator,
						- ...
					**************/
					Serial.println("Back To Sleep");
					delay(150);
					// Sleep microcontroller and radio
					sleepNow();
					
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

/*void loop(){
	Serial.println("LOOP");
			delay(150);
	if(wasSleeping()){
			Serial.println("Interrupt Call");
			delay(150);
					// Here we start to play
		EXECUTEFAST() {			
			Serial.println("EXECUTEFAST");
			UPDATEFAST();
			Serial.println("UPDATEFAST");

			// Here we process the communication
				FAST_PeerComms();		

			// Check every 510ms how many cycles asleep as expired,
			// using default values, the nodes stays asleep 2.5 seconds
			// every 34 minutes
				FAST_510ms() {
					Serial.println("FAST_510ms");
					long vcc = readVcc();
					float vcc_f = (float) vcc;
					float vcc_to_send = vcc_f/1000;
					Souliss_ImportAnalog(memory_map, BATT_LEVEL, &vcc_to_send);
					Serial.print("BATT_LEVEL: ");
					Serial.println(mOutput(BATT_LEVEL));
					Souliss_Logic_T55(memory_map, BATT_LEVEL, NODEADBAND, &data_changed);
				} 
		}
	}

	// Back to sleep
	if(isTimeToSleep())	{
					
		// Sleep microcontroller and radio
		Serial.println("Back To Sleep");
		delay(150);

		sleepNow();
	}	

}*/

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