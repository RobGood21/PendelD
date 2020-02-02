/*
	Name:       PendelD.ino
	Created:	2020
	Author:     Rob Antonisse

	Arduino programma voor een pendelautomaat voor digitale locomotieven

*/

//libraries
#include <EEPROM.h>


//declarations


//temps
volatile unsigned long time;
volatile unsigned long countpuls;


void setup() {
	Serial.begin(9600);


	DDRB |= (1 << 3); //set PIN11 as output for DCC 

	//interrupt register settings
	//TCCR2A – Timer/Counter Control Register A
	TCCR2A = 0x00; //clear register
	TCCR2A |= (1 << 6);//Toggle OC2A on Compare Match
	TCCR2A |= (1 << 1); //CTC mode clear timer at compare match
	//TCCR2B – Timer / Counter Control Register B
	TCCR2B = 2; //set register timer 2 prescaler 8
	//TCCR2B|=(1 << 0); //set timer 

	//OCR2A – Output Compare Register A tijdsduur 0~255
	OCR2A = 115; //geeft een puls van 58us

	TIMSK2 |= (1 << 1);
}

ISR(TIMER2_COMPA_vect) {
	GPIOR0 ^= (1 << 0);
	if (bitRead(GPIOR0, 0) == false) { //full bit

		//bepaal volgende bit


		GPIOR0 ^= (1 << 1);
		//if (bitRead(GPIOR0, 1) == true) {

			if (GPIOR0 & (1 << 1)){  //bitwise testing bit 1 in GPIOR0
			OCR2A = 115;
		}
		else {
			OCR2A = 230;
		}
	}


//countpuls++;

//if (millis() - time > 1000) { //count 10sec
//	Serial.println(countpuls);
//	time = millis();
//	countpuls = 0;
//}
}

void loop()
{


}
