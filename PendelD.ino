/*
	Name:       PendelD.ino
	Created:	2020
	Author:     Rob Antonisse

	Arduino programma voor een pendelautomaat voor digitale locomotieven

*/

//libraries
#include <EEPROM.h>
#define TrueBit OCR2A = 115
#define FalseBit OCR2A = 230

//declarations
byte DCC_fase;
byte count_preample;
byte DCC_data[5]; //bevat te verzenden DCC bytes, current DCC commands
byte DCC_count; //aantal bytes current DCC command

//eventeel nog een buffer om meerdere DCC commandoos te hebben bij bv. 2 locs of accessoire sturing


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
		switch (DCC_fase) {
		case 0: //niks doen alleen 1 bits zenden 	
			TrueBit;
			break;
		case 1: //preample zenden
			count_preample++;
			if (count_preample > 13) {
				count_preample = 0;
				DCC_fase = 2;
				FalseBit; 
			}
			break;
		case 2: //data zenden
			break;
		}


		GPIOR0 ^= (1 << 1);
		//if (bitRead(GPIOR0, 1) == true) {

		if (GPIOR0 & (1 << 1)) {  //bitwise testing bit 1 in GPIOR0
			TrueBit;
		}
		else {
			FalseBit;
		}
	}
}

void loop()
{


}
