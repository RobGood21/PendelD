/*
	Name:       PendelD.ino
	Created:	2020
	Author:     Rob Antonisse

	Arduino programma voor een pendelautomaat voor digitale DCC locomotieven

*/

//libraries
#include <EEPROM.h>

//display 
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//teksten
#define cd display.clearDisplay()
#define regel1 DSP_settxt(0, 2, 2) //parameter eerste regel groot
#define regel2 DSP_settxt(0, 23, 2) //parameter tweede regel groot
#define regel3 DSP_settxt(10,23,2) //value tweede regel groot
#define regel1s DSP_settxt(0, 2, 1) //value eerste regel klein
#define regel2s DSP_settxt(0, 0, 1) //X Y size X=0 Y=0 geen cursor verplaatsing

//#define txt_dcc "DCC adres " 
//#define txt_lok "loco " 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define TrueBit OCR2A = 115
#define FalseBit OCR2A = 230

//declarations, count tellers, 

byte count_preample;
byte count_byte;
byte count_bit;
byte count_cv;
byte count_repeat;
int count_slow;

byte dcc_fase;
byte dcc_data[5]; //bevat te verzenden DCC bytes, current DCC commando
byte dcc_aantalBytes; //aantal bytes current van het DCC commando
byte sw_statusC; //laatste stand van switches op C port
byte sw_statusD; //D port

byte dcc_cv[3];

byte loc_adres[2]; //moet van EEPROM komen
byte loc_speed;
byte loc_function = B10000000; //moet van EEPROM komen

byte PRG_fase;
byte PRG_qnty = 12; //aantal programmeerfases 

//temps
volatile unsigned long time;
volatile unsigned long countpuls;
unsigned long time_slow;
byte functies;
byte newlocadres = 10;

void setup() {
	Serial.begin(9600);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

	//poorten
	DDRB |= (1 << 3); //set PIN11 as output for DCC 
	PORTC |= (15 << 0); //set pin A0 A1 pull up *****
	DDRD &= ~(1 << 7);
	PORTD |= (1 << 7); //set PIN7 pullup register

	DDRB |= (1 << 0); //PIN8 as output enable DCC

	sw_statusC = 0xFF;
	sw_statusD = 0xFF;

	//interrupt register settings
	//TCCR2A – Timer/Counter Control Register A
	TCCR2A = 0x00; //clear register
	TCCR2A |= (1 << 6);//Toggle OC2A on Compare Match
	TCCR2A |= (1 << 1); //CTC mode clear timer at compare match
	TCCR2B = 2; //set register timer 2 prescaler 8
	TIMSK2 |= (1 << 1);

	PORTB |= (1 << 0); //set pin8 high
	functies = B10000000;
	//DCC_command();
	//delay(1000);
	DSP_start();
	//******display start

	//display.clearDisplay();
	//display.setTextSize(1);
	//display.setTextColor(WHITE);
	//display.setCursor(10, 10);
	//display.println("www.wisselmotor.nl");
	//display.display();

	//Use the drawLine(x1, y1, x2, y2, color) method to create a line.The(x1, y1) coordinates 
	//indicate the start of the line, and the(x2, y2) coordinates indicates where the line ends.

	//delay(1000);
	//display.drawLine(10, 20, 115, 20, WHITE);
	//display.display();
	//delay(1000);
	//The drawRect(x, y, width, height, color) provides an easy way to draw a rectangle.
	//The(x, y) coordinates indicate the top left corner of the rectangle.Then, 
	//you need to specify the width, height and color:
	//display.drawRect(5, 25, 100, 20, WHITE);
	//display.display();
	//***display end

	MEM_read();
	GPIOR0 = B10000000; //set start conditions
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
		}
	}
}
void MEM_read() {
	loc_adres[0] = EEPROM.read(100);
	loc_adres[1] = EEPROM.read(101);
	if (loc_adres[0] == 0xFF) {
		loc_adres[0] = 0x03;
		EEPROM.update(100, loc_adres[0]);
	}
	if (loc_adres[1] == 0xFF) {
		loc_adres[1] = 0x04;
		EEPROM.update(101, loc_adres[1]);
	}
}
void MEM_update() { //sets new value
	switch (PRG_fase) {
	case 0: //loc1 adres
		EEPROM.update(100, loc_adres[0]);
		break;
	case 1: //loc2 adres
		EEPROM.update(101, loc_adres[1]);
		break;
	case 2: //prograM CV1 to all 127 adresses new adres GPIOR0 bit7 true loc1,  false loc2
		DCC_write();
		break;
	}
}
void MEM_cancel() { //cancels, recalls value
	switch (PRG_fase) {
	case 0: //loc1 adres
		loc_adres[0] = EEPROM.read(100);
		break;
	case 1: //loc2 adres
		loc_adres[1] = EEPROM.read(101);
		break;
	}

}
void DCC_write() {
	//writes loc adress in locomotive
	Serial.println("ädres inschrijven");

}

void PRG_locadres(byte newadres, byte all) {
	//sets adres of loc// all= true sets all 127 loc adresses to new adres, if adres is unknown
	//and programs EEPROM 

}
void PRG_cv(byte oldadres, byte cv, byte value) { //CV programming
	//1110CCVV 0 VVVVVVVV 0 DDDDDDDD
	//old adres alleen bij first zetten van 
	cv = cv - 1;
	GPIOR0 |= (1 << 2);
	dcc_data[0] = oldadres; // B00000000; //
	//dcc_data[0] = 0x00; //broadcast adres??
	dcc_data[1] = B11101100; //instruction write CV
	dcc_data[2] = cv;
	dcc_data[3] = value; //adres 6
	dcc_data[4] = dcc_data[0] ^ dcc_data[1] ^ dcc_data[2] ^ dcc_data[3];
	dcc_aantalBytes = 4;
	count_repeat = 4;
}
void PRG_dec() {
	switch (PRG_fase) {
	case 0: ////dcc loc 1 adres
		loc_adres[0]--;
		if (loc_adres[0] < 1)loc_adres[0] = 127;
		break;
	case 1: ////dcc loc 2 adres
		loc_adres[1]--;
		if (loc_adres[1] < 1)loc_adres[1] = 127;
		break;
	case 2: //write DCC
		GPIOR0 ^= (1 << 7);
		break;
	}
	DSP_prg();
}
void PRG_inc() {
	switch (PRG_fase) {
	case 0: ////dcc loc 1 adres
		loc_adres[0]++;
		if (loc_adres[0] > 127)loc_adres[0] = 1;
		break;
	case 1: ////dcc loc 2 adres
		loc_adres[1]++;
		if (loc_adres[1] > 127)loc_adres[1] = 1;
		break;
	case 2: //write DCC
		GPIOR0 ^= (1 << 7);
		break;
	}
	DSP_prg();
}
void DCC_command() { //nieuwe
	if (GPIOR0 & (1 << 2)) { //Send CV or basic accessoire
		count_repeat--;
		if (count_repeat > 4) GPIOR0 &= ~(1 << 2); //end CV transmit
	}
	else { //send loc data
		GPIOR0 ^= (1 << 0);
		if (GPIOR0 & (1 << 0)) { //drive
			dcc_data[0] = loc_adres[0];
			dcc_data[1] = loc_speed;
			dcc_aantalBytes = 2;
		}
		else { //function
			dcc_data[0] = loc_adres[0];
			dcc_data[1] = loc_function;
			dcc_aantalBytes = 2;
		}
		dcc_data[2] = dcc_data[0] ^ dcc_data[1];
	}
}
void SW_exe() {
	byte poort; byte changed;
	GPIOR0 ^= (1 << 4);

	if (GPIOR0 & (1 << 4)) {
		poort = PIND;
		changed = poort ^ sw_statusD;
		for (byte i = 7; i > 6; i--) {
			if (changed & (1 << i) & ~poort & (1 << i)) {
				SW_on(7 - i + 4);
			}
		}
		sw_statusD = poort;
	}
	else {
		poort = PINC;
		changed = poort ^ sw_statusC;
		for (byte i = 0; i < 4; i++) {
			if (changed & (1 << i) & ~poort & (1 << i)) {
				SW_on(i);
			}
		}
		sw_statusC = poort;
	}
}
void SW_double() { //called from SW_exe when sw2 and sw3 is pressed simultanus
	GPIOR0 ^= (1 << 5);
	//instellen display prg fase
	if (GPIOR0 & (1 << 5)) { //programmode
		DSP_prg();
	}
	else { //bedrijf
		DSP_start();
	}
}
void SW_on(byte sw) {
	switch (sw) {
	case 2:
		if (~sw_statusC & (1 << 3)) {
			SW_double();
			return;
		}
		break;
	case 3:
		if (~sw_statusC & (1 << 2)) {
			SW_double();
			return;
		}
		break;
	case 4:
		PINB |= (1 << 0); //no DCC
		break;
	case 5:
		break;
	case 6:
		break;
	case 7:
		break;
	}

	if (sw < 4) {
		if (GPIOR0 & (1 << 5)) { //programmode
			SW_PRG(sw);
		}
		else {
			SW_pendel(sw);
		}
	}
}
void SW_PRG(byte sw) {
	if (GPIOR0 & (1 << 6)) { //value instellen mode
		switch (sw) {
		case 0:
			PRG_dec();
			break;
		case 1:
			PRG_inc();
			break;
		case 2:
			GPIOR0 &= ~(1 << 6);
			MEM_update();
			break;
		case 3:
			GPIOR0 &= ~(1 << 6);
			MEM_cancel();
			break;
		}
	}
	else { //parameter kiezen
		switch (sw) {
		case 0:
			PRG_fase--;
			if (PRG_fase > PRG_qnty)PRG_fase = PRG_qnty;
			break;
		case 1:
			PRG_fase++;
			if (PRG_fase > PRG_qnty)PRG_fase = 0;
			break;
		case 2:
			GPIOR0 |= (1 << 6);
			break;
		case 3:
			GPIOR0 &= ~(1 << 5); //Pendel mode
			DSP_start();
			return;
			break;
		}
	}
DSP_prg();
}
void SW_pendel(byte sw) { //nieuw
	//Serial.println(sw);
	switch (sw) {
	case 0:
		loc_speed = B01101110; //drive	
		break;
	case 1:
		loc_speed = B01100000; //stop
		break;
	case 2:
		loc_function ^= (1 << 4); //headlights
		break;
	case 3:
		//PRG_cv(10,1,6);
		//
		//display.clearDisplay();
		//DSP_buttons(0);
		loc_function ^= (1 << 1); //cabin
		break;
	}
}
void DSP_start() {
	display.clearDisplay();
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(10, 0);
	display.print(F("www.wisselmotor.nl"));
	display.drawLine(1, 10, 127, 10, WHITE);
	display.setTextSize(2);
	display.setCursor(3, 24);
	display.print("PenDel DCC");
	DSP_buttons(0);
	display.display();
}
void DSP_prg() {
	if (GPIOR0 & (1 << 6)) { //value 
		switch (PRG_fase) {
		case 0: //dcc loc 1 adres
			TXT_pv(true, 1);
			regel3; display.print(loc_adres[0]);
			break;
		case 1: //dcc loc 2 adres
			TXT_pv(true, 2);
			regel3; display.print(loc_adres[1]);
			break;
		case 2: //Write adres in loc and accessoires
			cd; regel1s; TXT(10); TXT(1); regel2; TXT(2);
			if (GPIOR0 & (1 << 7)) {
				TXT(101);
			}
			else {
				TXT(102);
			}
			DSP_buttons(10);
			break;

		case 4:
			break;

			//enz...
		}
	}
	else { //parameter
		switch (PRG_fase) {
		case 0: //dcc loc 1 adres
			TXT_pv(false, 1);
			break;
		case 1: //dcc loc 2 adres
			TXT_pv(false, 2);
			break;
		case 2:
			cd;
			regel1; TXT(10); TXT(0);
			regel2; TXT(1); //Write DCC adres
			DSP_buttons(10);
			break;

		case 3:

			break;
		case 4:
			break;
		}
	}
	display.display();
}
void DSP_buttons(byte mode) {
	//sets mode in display
	display.drawRect(0, 50, 128, 14, WHITE);
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(3, 54);
	switch (mode) {
	case 0:
		TXT(20);
		break;
	case 10:
		TXT(21);
		break;

	default:
		break;
	}
}
void DSP_settxt(byte X, byte Y, byte size) {
	display.setTextSize(size);
	display.setTextColor(WHITE);
	if (X + Y > 0) display.setCursor(X, Y);
}
void TXT(byte t) {
	switch (t) {
	case 0:
		display.println("");
		break;
	case 1:
		display.print("DCC adres ");
		break;
	case 2:
		display.print("Loco ");
		break;
	case 10:
		display.print("Write ");
		break;
	case 20:
		display.print("Start  loco   FL  F1");
		break;
	case 21:
		display.print(" -     +     V     X");
		break;
	case 30:
		break;
	case 100:
		display.print("0 ");
		break;
	case 101:
		display.print("1 ");
		break;
	case 102:
		display.print("2 ");
		break;
	}
}
void TXT_pv(boolean small, byte val) {
	display.clearDisplay();
	if (small == true) {
		regel1s; TXT(1);
		regel2s; TXT(2); display.println(val);
	}
	else {
		regel1; TXT(1);
		display.println("");
		regel2; TXT(2); display.println(val);
	}
	DSP_buttons(10);
}
void loop() {	//slow events timer
	count_slow++;
	if (count_slow > 10000) { //10000 makes frequency of command sending important 5000 is too fast
		count_slow = 0;
		//slow events
		SW_exe(); //switches
		//start DCC command transmit
		DCC_command();
		dcc_fase = 1;
	}
}