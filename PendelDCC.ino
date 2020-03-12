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
//#define SCREEN_WIDTH 128 // OLED display width, in pixels
//#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//constanten
#define prgmax 3 //aantal programma fases(+1), verhogen bij toevoegen prgfase

//teksten
#define cd display.clearDisplay()
#define regel1 DSP_settxt(0, 2, 2) //parameter eerste regel groot
#define regel2 DSP_settxt(0, 23, 2) //parameter tweede regel groot
#define regel3 DSP_settxt(10,23,2) //value tweede regel groot
#define regel1s DSP_settxt(0, 2, 1) //value eerste regel klein
#define regel2s DSP_settxt(0, 0, 1) //X Y size X=0 Y=0 geen cursor verplaatsing

//#define txt_dcc "DCC adres " 
//#define txt_lok "loco " 

Adafruit_SSD1306 display(128, 64, &Wire, -1);

#define TrueBit OCR2A = 115
#define FalseBit OCR2A = 230

//declarations, count tellers, 

byte count_preample;
byte count_byte;
byte count_bit;
byte count_cv;
byte count_repeat;
int count_slow;
byte count_wa; //write adres

byte dcc_fase;
byte dcc_data[5]; //bevat te verzenden DCC bytes, current DCC commando
byte dcc_aantalBytes; //aantal bytes current van het DCC commando
byte sw_statusC; //laatste stand van switches op C port
byte sw_statusD; //D port

byte dcc_cv[3];

byte loc_reg[2];
byte loc_adres[3]; //moet van EEPROM komen, 3e adres is voor programmeerttoepassing
byte loc_speed[2]; //holds the speed bytes
byte loc_speedcount[2]; //holds the 28 steps
byte loc_function[2]; //moet van EEPROM komen

byte dcc_wissels; //dcc basis adres wissel decoder 
byte dcc_seinen; //dcc basis adres sein decoder

byte PRG_fase;
byte PRG_level; //hoe diep in het programmeer proces
byte PRG_typeDCC;
byte PRG_typeTest;
byte PRG_value; //ingestelde waarde op PRG_level 2

byte PRG_cvs[2]; //0=CV 1=waarde

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
	loc_function[0] = 128;
	loc_function[1] = 128;
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

	dcc_wissels = EEPROM.read(102);
	if (dcc_wissels == 0xFF) {
		dcc_wissels = 1; //default =1
		EEPROM.update(102, dcc_wissels);
	}
	dcc_seinen = EEPROM.read(103); //heeft 8 dcc kanalen(16 leds)
	if (dcc_seinen == 0xFF) {
		dcc_seinen = 254;
		EEPROM.update(103, dcc_seinen);
	}
	//default=0xFF 255

}
void MEM_update() { //sets new values/ sends CV 
	switch (PRG_fase) {
	case 0: //loc1 adres
		switch (PRG_typeDCC) {
		case 0: //loc1
			EEPROM.update(100, loc_adres[0]);
			break;
		case 1: //loc2
			EEPROM.update(101, loc_adres[1]);
			break;
		case 2://wissels
			EEPROM.update(102, dcc_wissels);
			break;
		case 3: //seinen
			EEPROM.update(103, dcc_seinen);
			break;
		}
		break;
		//****************************PROGRAM fase 2

	case 2: //prograM CV1 to all 127 adresses new adres GPIOR0 bit7 true loc1,  false loc2
		switch (PRG_value) {
		case 0: //loco 1
			loc_adres[2] = loc_adres[0];
			break;
		case 1: //loco 2
			loc_adres[2] = loc_adres[1];
			break;
		}

		GPIOR1 |= (1 << 1); //blocks display updates 
		count_wa = 0;
		GPIOR1 |= (1 << 0); //enable dcc adress write to loc//DCC_write();			
		break;
		//*****************************Programfase 3
	case 3: //program CV
		//PRG_value 0=loco1; 1=loco2; 2=accessoire
		//CVs[0]=CV; CVs[1]=Value
		switch (PRG_value) {
		case 0:
			//Serial.println("CV schrijven");
			PRG_cv(loc_adres[0], PRG_cvs[0], PRG_cvs[1]);
			break;
		case 1:
			PRG_cv(loc_adres[1], PRG_cvs[0], PRG_cvs[1]);
			break;
		case 2: //voor accesoire nog maken. andere structuur

			break;
		}
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
	if (~GPIOR0 & (1 << 2)) {
		count_wa++;
		if (count_wa & (1 << 7)) {
			DCC_endwrite();
		}
		else {
			PRG_cv(count_wa, 1, loc_adres[2]); //merk op loc adres 3 =s het adres van dan actief loc
			display.drawPixel(count_wa, 40, WHITE);
			display.drawPixel(count_wa, 41, WHITE);
			display.display();
		}
	}
}
void DCC_endwrite() {
	GPIOR1 &= ~(1 << 0);
	Serial.println("adres aangepast");
	GPIOR1 &= ~(1 << 1);

	DSP_prg();
}
byte LOC_calc(byte num,byte loc) {
	byte speed;
	if (num == 0) {
		speed = B00010000; //direct stop
	}
	else {
		speed = (num / 2) + 2;
		if (num % 2 != 0) speed |= (1 << 4);// check if number is even
	}
	//richting loco
	speed |= (1 << 6);
	if (loc_reg[loc] & (1 << 0))speed |= (1 << 5);
	return speed;
}

void PRG_locadres(byte newadres, byte all) {
	//sets adres of loc// all= true sets all 127 loc adresses to new adres, if adres is unknown
	//and programs EEPROM 

}
void PRG_cv(byte adres, byte cv, byte value) { //CV programming
	//1110CCVV 0 VVVVVVVV 0 DDDDDDDD
	//old adres alleen bij first zetten van 
	cv = cv - 1;
	GPIOR0 |= (1 << 2);
	dcc_data[0] = adres; // B00000000; //
	//dcc_data[0] = 0x00; //broadcast adres??
	dcc_data[1] = B11101100; //instruction write CV
	dcc_data[2] = cv;
	dcc_data[3] = value; //adres 6
	dcc_data[4] = dcc_data[0] ^ dcc_data[1] ^ dcc_data[2] ^ dcc_data[3];
	dcc_aantalBytes = 4;
	count_repeat = 4;
}
void PRG_dec() {
	//Serial.println(PRG_level);
	switch (PRG_fase) {
	case 0: //INstellen DCC adres
		switch (PRG_level) {
		case 2:  //dec voor welk loc, wissels of seinen
			PRG_typeDCC--;
			if (PRG_typeDCC > 3)PRG_typeDCC = 3;
			break;
		case 3://waarde DCC adres aanpassen
			switch (PRG_typeDCC) {
			case 0:
				loc_adres[0]--;
				if (loc_adres[0] == 0)loc_adres[0] = 127;
				break;
			case 1:
				loc_adres[1]--;
				if (loc_adres[1] == 0)loc_adres[1] = 127;
				break;
			case 2:
				dcc_wissels--;
				if (dcc_wissels == 0)dcc_wissels = 255;
				break;
			case 3:
				dcc_seinen--;
				if (dcc_seinen == 0)dcc_seinen = 254;
				break;
			}
			break;
		}

	case 1: //Testen
		switch (PRG_level) {
		case 2: //kiezen loc, wissels, seinen of melders
			PRG_typeTest--;
			if (PRG_typeTest > 4)PRG_typeTest = 4;
			break;
		case 3:
			switch (PRG_typeTest) {
			case 0: //speed loc 1
				if (loc_speedcount[0] > 0 )loc_speedcount[0]--;
				loc_speed[0] = LOC_calc(loc_speedcount[0],0);
				break;
			case 1://loc 2

				break;
			case 2: //test wissels
				break;
			case 3: //test seinen
				break;
			case 4://test melders
				break;
			}
			break;
		}
		break;

	case 2: //write DCC
		PRG_value--;
		if (PRG_value > 1) PRG_value = 1; //instellen dcc accessoires komen ook hier nu maar 2 loc1 en loc2
		//GPIOR0 ^= (1 << 7);
		break;
	case 3: //write CV
		switch (PRG_level) {
		case 2: //parameter
			PRG_value--;
			if (PRG_value > 2) PRG_value = 2; //instellen dcc accessoire 2 loc1 en loc2		
			break;
		case 3: //CV
			PRG_cvs[0]--;
			if (PRG_cvs[0] < 1) PRG_cvs[0] = 255;
			break;
		case 4://Value
			PRG_cvs[1]--;
			if (PRG_cvs[1] < 1) PRG_cvs[1] = 255;
			break;
		}
		break;
	}
	//DSP_prg();
}
void PRG_inc() {
	switch (PRG_fase) {
	case 0: //inc DCC van loc, wissels of seinen
		switch (PRG_level) {
		case 2:
			PRG_typeDCC++;
			if (PRG_typeDCC > 3)PRG_typeDCC = 0;
			break;
		case 3:
			switch (PRG_typeDCC) {
			case 0:
				loc_adres[0]++;
				if (loc_adres[0] == 0)loc_adres[0] = 1;
				break;
			case 1:
				loc_adres[1]++;
				if (loc_adres[1] == 0)loc_adres[1] = 1;
				break;
			case 2:
				dcc_wissels++;
				if (dcc_wissels == 0)dcc_wissels = 1;
				break;
			case 3:
				dcc_seinen++;
				if (dcc_seinen > 254)dcc_seinen = 1;
				break;
			}
			break;
			//break;
		}
		break;

	case 1: //Testen
		switch (PRG_level) {
		case 2: //loc, wissels of seinen kiezen
			PRG_typeTest++;
			if (PRG_typeTest > 4)PRG_typeTest = 0;
			break;
		case 3: //testen 
			switch (PRG_typeTest) {
			case 0: //speed loc 1
				if(loc_speedcount[0] < 27) loc_speedcount[0]++;
				
				loc_speed[0] = LOC_calc(loc_speedcount[0],0);
				break;
			case 1: //speed loc 2
				break;
			case 2: //wissels test

				break;
			case 3: //seinen test
				break;
			case 4: //melders test
				break;
			}

			break;
		}

		break;
	case 2: //write DCC
		PRG_value++;
		if (PRG_value > 1) PRG_value = 0; //instellen dcc accessoires komen ook hier nu maar 2 loc1 en loc2
		//GPIOR0 ^= (1 << 7);
		break;
	case 3: //write CV
		switch (PRG_level) {
		case 2:
			PRG_value++;
			if (PRG_value > 2) PRG_value = 0; //instellen dcc accessoire 2 loc1 en loc2		
			break;
		case 3:
			PRG_cvs[0]++;
			if (PRG_cvs[0] == 0) PRG_cvs[0] = 1;
			break;
		case 4://Value
			PRG_cvs[1]++;
			if (PRG_cvs[1] == 0) PRG_cvs[1] = 1;
			break;
		}
		break;
	}
	//DSP_prg();
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
			dcc_data[1] = loc_speed[0];
			dcc_aantalBytes = 2;
		}
		else { //function
			dcc_data[0] = loc_adres[0];
			dcc_data[1] = loc_function[0];
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
	if (PRG_level == 0) {
		GPIOR0 |= (1 << 5); //program mode
		PRG_level = 1;
		DSP_prg();
		//all stop
		loc_speed[0] = B01100000; //stop
		loc_speed[1] = B01100000; //stop
		//loc_function[0] = 128;
		//loc_function[0] = 128;
	}
	else {
		GPIOR0 &= ~(1 << 5); //pendel mode
		PRG_level = 0;
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
		if (GPIOR0 & (1 << 5)) { //programmode, kan misschien met checken prg_level?
			SW_PRG(sw);
		}
		else {
			SW_pendel(sw);
		}
	}
}
void SW_PRG(byte sw) {
	boolean end = false;
	switch (PRG_level) {
		//++++++++LEVEL 1
	case 1:	//Soort instelling	
		switch (sw) {
		case 0:
			PRG_fase--;
			if (PRG_fase > prgmax)PRG_fase = prgmax;
			break;
		case 1:
			PRG_fase++;
			if (PRG_fase > prgmax)PRG_fase = 0;
			break;
		case 2:
			PRG_level++;
			break;
		case 3:
			GPIOR0 &= ~(1 << 5); //Pendel mode
			PRG_level = 0;
			DSP_start();
			return;
			break;
		}
		break;

		///++++++++LEVEL2
	case 2: //Level 2
		switch (sw) {
		case 0:
			PRG_dec();
			break;
		case 1:
			PRG_inc();
			break;
		case 2:
			switch (PRG_fase) {
			case 0: //DCC adressen instellen
				PRG_level++;
				break;
			case 1://Testen				
				PRG_level++;
				break;
			case 2: //schrijf dcc adres
				end = true;
				break;
			case 3:
				PRG_level++;
				break;
			}

			if (end == true) {
				MEM_update();
				PRG_level--;
			}
			break;
		case 3:
			PRG_level--;
			//GPIOR0 &= ~(1 << 6);
			MEM_cancel();
			break;
		}
		break;
		//+++++++++++++++LEVEL3
	case 3: //level 3
		//Serial.print("switch ");
		//Serial.println(sw);
		switch (sw) {
		case 0:
			PRG_dec();
			break;
		case 1:
			PRG_inc();
			break;
		case 2:
			switch (PRG_fase) {
			case 0: //instellen DCC adressen
				PRG_level--;
				MEM_update();
				break;
			case 1: //Testen
				switch (PRG_typeTest) {
				case 0: //loc 1
					loc_reg[0] ^= (1 << 0);
					loc_speed[0] = LOC_calc(loc_speedcount[0], 0);
					break;
				case 1: //loc 2
					break;
				case 2: //wissels
					break;
				case 3: //seinen
					break;
				case 4://melders
					break;
				}

				break;

			case 3: //CV programmering
				PRG_level++;
				break;
			}
			break;
		case 3:
			PRG_level--;
			MEM_cancel();
			break;
		}
		break;

		//********Level 4
	case 4:
		switch (PRG_fase) {
		case 3:
			switch (sw) {
			case 0:
				PRG_dec();
				break;
			case 1:
				PRG_inc();
				break;
			case 2:
				MEM_update();
				PRG_level--;
				break;
			case 3:
				PRG_level--;
				MEM_cancel;
				break;
			}
			break;
		}
		break;
	}
	if (~GPIOR1 & (1 << 1)) DSP_prg();
	//dit bit zorgt dat pas na de bewerking, bv. adres schrijven
//het display vernieuwd ibv. progressbar bv.
}
void SW_pendel(byte sw) { //nieuw
	//Serial.println(sw);
	switch (sw) {
	case 0:
		loc_speed[0] = B01101110; //drive	
		break;
	case 1:
		loc_speed[0] = B01100000; //stop
		break;
	case 2:
		loc_function[0] ^= (1 << 4); //headlights
		break;
	case 3:
		//PRG_cv(10,1,6);
		//
		//display.clearDisplay();
		//DSP_buttons(0);
		loc_function[0] ^= (1 << 1); //cabin
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
	int adrs;
	byte buttons;
	switch (PRG_level) {
		//**********************
	case 1: //soort instelling
		switch (PRG_fase) {
		case 0: //Instellen DCC adres
			cd;
			regel1; TXT(7); TXT(0); //Instellen
			regel2; TXT(1); //DCC adres	
			break;
		case 1: //Testen
			cd; regel1; TXT(11);
			break;
		case 2: //write loc adresses
			cd;
			regel1; TXT(10); TXT(0);
			regel2; TXT(1); //Write DCC adres
			break;
		case 3: //Cv programming
			cd;
			regel1; TXT(10); TXT(0);
			regel2; TXT(5);
			break;
		}
		buttons = 10;
		break;
		//**********************************level 2
	case 2: // program level 2
		switch (PRG_fase) {
		case 0: //keuze loc of accessoire
			cd;
			regel1s; TXT(7); TXT(1); TXT(0); regel2;
			switch (PRG_typeDCC) { //DCC van welke loc of accessoire keuze
			case 0:
				TXT(2); TXT(101);
				break;
			case 1:
				TXT(2); TXT(102);
				break;
			case 2:
				TXT(8);
				break;
			case 3:
				TXT(9);
				break;
			}
			buttons = 10;
			break;

		case 1: //Testen
			cd; regel1; TXT(11); regel2;
			switch (PRG_typeTest) {
			case 0:
				TXT(2); TXT(101);
				break;
			case 1:
				TXT(2); TXT(102);
				break;
			case 2:
				TXT(8);
				break;
			case 3:
				TXT(9);
				break;
			case 4:
				TXT(3);
				break;
			}
			buttons = 10;
			break;

		case 2: //Write adres in loc and accessoires
			cd; regel1s; TXT(10); TXT(1); regel2; TXT(2);
			switch (PRG_value) {
			case 0:
				TXT(101);
				break;
			case 1:
				TXT(102);
				break;
			}
			buttons = 10;
			break;
		case 3: //CV programming
			cd; regel1s; TXT(10); TXT(5); regel2;

			switch (PRG_value) {
			case 0: //loc1
				TXT(2); TXT(101);
				break;
			case 1: //loc2
				TXT(2); TXT(102);
				break;
			case 2://accessoire
				TXT(3);
				break;
			}
			buttons = 10;
			break;
		}
		break;

		//**********************************level 3
	case 3: // level 3
		switch (PRG_fase) {
		case 0: //Instellen DCC adressen
			cd; regel1s; TXT(1);
			switch (PRG_typeDCC) {
			case 0: //loc1
				TXT(2); TXT(101);
				regel2; display.print(loc_adres[0]);
				break;
			case 1: //loc2
				TXT(2); TXT(102);
				regel2; display.print(loc_adres[1]);
				break;
			case 2://wissels
				TXT(8); regel2; display.print(dcc_wissels);
				display.setTextSize(1);
				display.setCursor(display.getCursorX(), display.getCursorY() + 7);
				TXT(30);
				adrs = (((dcc_wissels - 1) * 4) + 1);
				display.print(adrs); TXT(32); display.print(adrs + 3); TXT(31);
				break;
			case 3://seinen
				TXT(9); regel2; display.print(dcc_seinen);
				display.setTextSize(1);
				display.setCursor(display.getCursorX(), display.getCursorY() + 7);
				TXT(30);
				adrs = (((dcc_seinen - 1) * 4) + 1);
				display.print(adrs); TXT(32); display.print(adrs + 7); TXT(31);//seinen heeft 8 adressen 16 leds
				break;
			}
			break;
		case 1: //Testen
			cd; regel1s; TXT(11);
			switch (PRG_typeTest) {
			case 0:  //snelheid en richting loco1
				TXT(2); TXT(101);
				regel2; 
				if (loc_reg[0] & (1 << 0)) {
					TXT(13);
				}
				else {
					TXT(14);
				}
				display.print(loc_speedcount[0]);
				buttons = 11;

				break;
			case 1:
				TXT(2); TXT(102);
				buttons = 11;
				break;
			case 2:
				TXT(8);
				break;
			case 3:
				TXT(9);
				break;
			case 4:
				TXT(3);
				break;
			}
			break;
		case 3: //CV programming level 3 CV keuze
			TXT_cv3();
			//display.print(PRG_cvs[0]);
			buttons = 10;
			break;
		}
		break;
		//**********************LEVEL 4
	case 4: //level 4
			//Serial.print("PRG_level:");
		//Serial.println(PRG_level);
		switch (PRG_fase) {
		case 3:
			TXT_cv3(); TXT(110); TXT(6);
			display.print(PRG_cvs[1]);
			break;
		}
		buttons = 10;
		break;
	}

	DSP_buttons(buttons);
	display.display();
}
void TXT_cv3() {
	cd; regel1s; TXT(10); TXT(5);
	switch (PRG_value) {
	case 0:
		TXT(2); TXT(101);
		break;
	case 1:
		TXT(2); TXT(102);
		break;
	case 2:
		TXT(3);
		break;
	}
	regel2; TXT(5); display.print(PRG_cvs[0]);
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

	case 10: //standaard programmeer balk
		TXT(21);
		break;
	case 11: //Loc testen balk 
		TXT(22);
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
byte DSP_speed(byte loco) {
	byte spd;
	spd = loc_speed[loco];
	spd = spd << 3;
	spd = spd >> 3;
	return spd;
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
	case 3:
		display.print("Melders ");
		break;
	case 4:
		display.print("**** "); //niet in gebruik
		break;
	case 5:
		display.print("CV ");
		break;
	case 6:
		display.print("W ");
		break;
	case 7:
		display.print("Instellen ");
		break;
	case 8:
		display.print("Wissels ");
		break;
	case 9:
		display.print("Seinen ");
		break;
	case 10:
		display.print("Schrijf ");
		break;
	case 11:
		display.print("Testen ");
		break;
	case 12:
		display.print("DCC ");
		break;
	case 13:
		display.print("<<< ");
		break;
	case 14:
		display.print(">>> ");
		break;
		//***********Onderbalken
	case 20:
		display.print("Start  loco   FL  F1");
		break;
	case 21:
		display.print(" -     +     V     X");
		break;
	case 22:
		display.print("S-    S+     <>    X");
		//******************
	case 30:
		display.print(" (");
		break;
	case 31:
		display.print(") ");
		break;
	case 32:
		display.print("-");
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
	case 110:
		display.print(" ");
		break;
	case 255:
		display.print("niet bepaald");
		break;
	}
}
void loop() {	//slow events timer
	count_slow++;
	if (count_slow > 10000) { //10000 makes frequency of command sending important 5000 is too fast
		count_slow = 0;
		//slow events
		SW_exe(); //switches
		//start DCC command transmit
		if (GPIOR1 & (1 << 0))DCC_write(); //writing dcc adres in loc
		DCC_command();
		dcc_fase = 1;
	}
}