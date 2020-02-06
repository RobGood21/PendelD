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

//declarations, count tellers, 

byte count_preample;
byte count_byte;
byte count_bit;
unsigned int count_slow;
byte dcc_fase;
byte dcc_data[5]; //bevat te verzenden DCC bytes, current DCC commando
byte dcc_aantalBytes; //aantal bytes current van het DCC commando
byte sw_status; //laatste stand van switches

//eventeel nog een buffer om meerdere DCC commandoos te hebben bij bv. 2 locs of accessoire sturing


//temps
volatile unsigned long time;
volatile unsigned long countpuls;
unsigned long time_slow;


void setup() {
	Serial.begin(9600);

	//poorten
	DDRB |= (1 << 3); //set PIN11 as output for DCC 
	PORTC |= (15 << 0); //set pin A0, A1 pull up
	DDRB |= (1 << 0); //PIN8 as output enable DCC
	


	sw_status = 0xFF;

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


	DCC_command();

	PORTB |= (1 << 0); //set pin8 high
}

ISR(TIMER2_COMPA_vect) {
	GPIOR0 ^= (1 << 0);
	//if (bitRead(GPIOR0, 0) == false) { //full bit

	if (~GPIOR0 & (1 << 0)) {
		//bepaal volgende bit
		switch (dcc_fase) {
		case 0: //niks doen alleen 1 bits zenden 	
			TrueBit;
			break;

		case 1: //preample zenden
			count_preample++;
			if (count_preample > 13) {
				count_preample = 0;
				dcc_fase = 2;
				FalseBit;
				count_bit = 7;
				count_byte = 0;
			}
			break;
		case 2: //send dcc_data
			//MSB first; LSB last
			if (count_bit < 8) {
				if (dcc_data[count_byte] & (1 << count_bit)) { //als het [countbit] van het byte[countbyte] waar is dan>> 
					TrueBit;
				}
				else {
					FalseBit;
				}
				count_bit--;
			}
			else { //count_bit 8 or more
				count_bit = 7;
				if (count_byte < dcc_aantalBytes) {
					count_byte ++; //next byte
					FalseBit;
				}
				else { //command send reset	

					dcc_fase = 0; 
					TrueBit;
				}
			}
			break;

		case 10:
			//testing
			FalseBit;
			break;
		}
	}
}

void DCC_command() {
	dcc_aantalBytes = 3;
	//dcc_data
	dcc_data[0] = B00000110; //adres 6
	dcc_data[1] = B01110001;
	dcc_data[2] = dcc_data[0] ^ dcc_data[1];

}

void SW_exe() {
	byte poort; byte changed;
	poort = PINC;
	changed = poort ^ sw_status;
	for (byte i; i < 4; i++) {
		if (changed & (1 << i) & ~poort & (1 << i)) {
			SW_on(i);
		}
	}
	sw_status = poort;

	//DCC
	//DCC_command();
	dcc_fase = 1;
	count_preample = 0; //niet nodig?
}
void SW_on(byte sw) {
	Serial.println(sw);

	switch (sw) {
	case 0:
		PINB |= (1 << 0);
		break;
	case 1:
		///GPIOR0 ^= (1 << 1);
		//if (GPIOR0 & (1 << 1)) {/
			dcc_data[1] = B01111111;
			dcc_data[2] = dcc_data[0] ^ dcc_data[1];		
		break;
	case 2:
		dcc_data[1] = B01110001;
		dcc_data[2] = dcc_data[0] ^ dcc_data[1];
		break;
	case 3:
		break;
	}
}
void loop() {

	if (millis() - time_slow > 20) {
		time_slow = millis();
		SW_exe();
	}

	/*
	
	count_slow++;
	if (count_slow == 0) {
		//slow events
		SW_exe();
	}
*/
}
