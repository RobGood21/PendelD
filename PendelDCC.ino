/*
	Name:       PendelD.ino
	Created:	2020
	Author:     Rob Antonisse

	Arduino programma voor een pendelautomaat voor digitale DCC locomotieven

*/

//libraries
#include <EEPROM.h>
#define TrueBit OCR2A = 115
#define FalseBit OCR2A = 230

//declarations, count tellers, 

byte count_preample;
byte count_byte;
byte count_bit;
byte count_command;
int count_slow;

byte dcc_fase;
byte dcc_data[5]; //bevat te verzenden DCC bytes, current DCC commando
byte dcc_aantalBytes; //aantal bytes current van het DCC commando
byte sw_status; //laatste stand van switches

//eventeel nog een buffer om meerdere DCC commandoos te hebben bij bv. 2 locs of accessoire sturing
byte dcc_buf1[5];
byte dcc_buf2[5];
byte dcc_buf3[5]; //max 3 bytes 
byte dcc_bufCount[5];//how many times command must be send
byte dcc_bufAantalBytes[5];
byte dcc_bufActive; // bit 0= buffer 0 // bit4 = buffer 4


byte loc_adress = 6; //moet van EEPROM komen
byte loc_speed;
byte loc_function=B10000000; //moet van EEPROM komen


//temps
volatile unsigned long time;
volatile unsigned long countpuls;
unsigned long time_slow;
byte functies;
byte newlocadres=10;


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
	   	 
	PORTB |= (1 << 0); //set pin8 high
	functies = B10000000;
	//DCC_command();

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
			if (count_preample > 22) {
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
					count_byte++; //next byte
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

void PRG_locadres() { //set new locadres
	/*
	zo gaat ut niet zie de CV instellingen moeten exclusief minimaal 2x worden verstuurd, 
	Beste een 'aparte' buffer nemen voor de CV bv. buffer[2] en een flag zetten in een register als programmode... 
	dan alleen de cv versturen...
	
	
	
	*/




	//1110CCVV 0 VVVVVVVV 0 DDDDDDDD

	//find free buffer
	for (byte i = 2; i < 5; i ++) { 
		if(bitRead(dcc_bufActive,i)==false){
		//if (dcc_bufActive & ~(1 << i)) {

			Serial.println(dcc_bufActive);

			dcc_bufActive |= (1 << i); //claim buffer

			Serial.println(dcc_bufActive);

			dcc_buf1[i] = B11101100;
			dcc_buf2[i] = B00000000; //CV1
			dcc_buf3[i] = B00001010; // newlocadres;
			dcc_bufCount[i] = 4;
			dcc_bufAantalBytes[i] = 3;
			i = 10;
		}
	}
}

void DCC_command() { //nieuwe
	count_command++;
	if (count_command > 4)count_command = 0;
	switch (count_command) {
	case 0: //speed and direction
		dcc_data[0] = loc_adress;
		dcc_data[1] = loc_speed;
		dcc_aantalBytes = 2;
		break;
	case 1: //Functions
		dcc_data[0] = loc_adress;
		dcc_data[1] = loc_function;
		dcc_aantalBytes = 2;
		break;

	default: //case others (accessoires or so)
		if (dcc_bufActive & (1 << count_command) ) {
			if (dcc_bufCount[count_command] == 0) { //disable command
				dcc_bufActive &=~(1 << count_command); 
			}
			else { //make command				
				dcc_data[0] = dcc_buf1[count_command];
				dcc_data[1] = dcc_buf2[count_command];
				dcc_data[2] = dcc_buf3[count_command];
				dcc_aantalBytes = dcc_bufAantalBytes[count_command];
				dcc_bufCount[count_command]--;
			}			
		}
		else { //Send idle command
			
			dcc_data[0] = 0xFF;
			dcc_data[1] = 0x00;
			dcc_aantalBytes = 2;
		}
		break;
	}
	switch (dcc_aantalBytes) {
	case 2:
		dcc_data[2] = dcc_data[0] ^ dcc_data[1];
		break;
	case 3:
		dcc_data[3] = dcc_data[0] ^ dcc_data[1] ^ dcc_data[2];
		break;
	}
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
	DCC_command();
	dcc_fase = 1;
	count_preample = 0; //niet nodig?

}
void SW_on(byte sw) { //nieuw
	switch (sw) {
	case 0:
		PINB |= (1 << 0); //no DCC
		break;
	case 1:
		loc_speed = B01101110; //drive
		break;
	case 2:
		loc_speed = B01100000; //stop
		break;
	case 3:
		PRG_locadres();
		//loc_function ^= (1 << 4); //headlights
		break;
	}

}

void loop() {

	//slow events timer
	count_slow++;
	if (count_slow > 10000) { //makes frequency of command sending important 5000 is too fast
		count_slow = 0;
		//slow events
		SW_exe();
	}

}
