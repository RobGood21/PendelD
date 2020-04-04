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

//declarations, 
//struct's
struct LOC
{
	byte reg; //register, flags  //3april niet in gebruik
	//bit0 niet in gebruik
	byte adres; //DCC adres
	byte speed; //Byte snelhed en richting
	byte velo; //waarde 1~28 decimale voorstelling snelheid
	byte route; //welke route is bezet door loc, 0xFF is geen
	byte function; //functies
	byte station; //welk station staat loc, 0 is nog niet bepaald.

}LOC[2];

struct route {
	byte reg; //registers
	/*bit0 richting.... ?????   nodig??
	*/
	byte Tgoal; //station is GOAL bij richting true, Fgoal=FROM
	byte Fgoal; //station is GOAL bij richting false, Tgoal=FROM

} ROUTE[8];


//count tellers, 
byte count_preample;
byte count_byte;
byte count_bit;
byte count_cv;
byte count_repeat;
int count_slow;
byte count_wa; //write adres
byte count_sw;
byte count_command;

byte dcc_fase;
byte dcc_data[6]; //bevat te verzenden DCC bytes, current DCC commando
byte dcc_aantalBytes; //aantal bytes current van het DCC commando
byte sw_statusC; //laatste stand van switches op C port
byte sw_statusD; //D port

byte loc_ta; //temp loc adres nodig in CV adres programming
//byte loc_function[2]; //moet van EEPROM komen

byte dcc_wissels; //dcc basis adres wissel decoder 
byte dcc_seinen; //dcc basis adres sein decoder
byte pos_wissels; //stand positie van de vier wissels
byte pos_seinen[2]; //stand positie van de seinen
byte pos_melders[2]; //stand van de melders
//byte pos_melderDir[2]; // richting van de melders hoog/laag actief
//program mode
byte PRG_fase;
byte PRG_level; //hoe diep in het programmeer proces
byte PRG_typeDCC; //actieve adres
byte PRG_typeTest; //actieve test
byte prg_wissels; //actieve wissel
byte prg_sein; //actief sein 16x
byte prg_typecv; //ingestelde waarde op PRG_level 2
byte PRG_cvs[2]; //0=CV 1=waarde
//Pendel mode
byte PDL_fase;



//temps
byte tempcount;


void setup() {
	Serial.begin(9600);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

	//poorten
	DDRB |= (1 << 3); //set PIN11 as output for DCC 
	PORTC |= (15 << 0); //set pin A0 A1 pull up *****

	DDRD = 0x00;
	DDRD |= (1 << 3); //pin3 as output
	PORTD |= (B11110000 << 0); //Pull up to pin 7,6,5,4

	DDRB |= (1 << 0); //PIN8 as output enable DCC

	PORTB |= (1 << 1); //puuup to pin 9
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
	//functies = B10000000;
	//DSP_start();
	MEM_read();

	//init
	LOC[0].speed = B01100000;
	LOC[1].speed = B01100000;

	LOC[0].function = 128;
	LOC[1].function = 128;
	pos_melders[0] = 0x0F; pos_melders[1] = 0x0F;
	DSP_pendel();
}
ISR(TIMER2_COMPA_vect) {
	GPIOR0 ^= (1 << 0);
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
	LOC[0].adres = EEPROM.read(100);
	LOC[1].adres = EEPROM.read(101);
	if (LOC[0].adres == 0xFF) {
		LOC[0].adres = 0x03;
		EEPROM.update(100, LOC[0].adres);
	}
	if (LOC[1].adres == 0xFF) {
		LOC[1].adres = 0x04;
		EEPROM.update(101, LOC[1].adres);
	}

	dcc_wissels = EEPROM.read(102);
	if (dcc_wissels == 0xFF) {
		dcc_wissels = 1; //default =1
		EEPROM.update(102, dcc_wissels);
	}
	dcc_seinen = EEPROM.read(103); //heeft 16 dcc kanalen)
	if (dcc_seinen == 0xFF) {
		dcc_seinen = 252;
		EEPROM.update(103, dcc_seinen);
	}
	//pos_melderDir[0] = EEPROM.read(104); //richting van de melders, hoog of laag actief default 0xFF;
	//pos_melderDir[1] = EEPROM.read(105);
	//default=0xFF 255

}
void MEM_update() { //sets new values/ sends CV 
	switch (PRG_fase) {
	case 0: //DCC adressen

		switch (PRG_typeDCC) {
		case 0: //loc1
			EEPROM.update(100, LOC[0].adres);
			break;
		case 1: //loc2
			EEPROM.update(101, LOC[1].adres);
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

		loc_ta = LOC[prg_typecv].adres;


		GPIOR1 |= (1 << 1); //blocks display updates 
		count_wa = 0;
		GPIOR1 |= (1 << 0); //enable dcc adress write to loc//DCC_write();			
		break;
		//*****************************Programfase 3
	case 3: //program CV
		//prg_typecv 0=loco1; 1=loco2; 2=accessoire
		//CVs[0]=CV; CVs[1]=Value
		switch (prg_typecv) {
		case 0:
			if (GPIOR0 & ~(1 << 2)) { //bit2 in GPIOR0 false
				//Serial.println("CV schrijven");
				DCC_cv(true, LOC[0].adres, PRG_cvs[0], PRG_cvs[1]);
				break;
		case 1://loco 2
			DCC_cv(true, LOC[1].adres, PRG_cvs[0], PRG_cvs[1]);
			break;
		case 2: //wissels
			DCC_cv(false, dcc_wissels, PRG_cvs[0], PRG_cvs[1]);
			break;
			}
		}
		break;
	}
}
void MEM_cancel() { //cancels, recalls value
	switch (PRG_fase) {
	case 0: //loc1 adres
		LOC[0].adres = EEPROM.read(100);
		break;
	case 1: //loc2 adres
		LOC[1].adres = EEPROM.read(101);
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
			DCC_cv(true, count_wa, 1, loc_ta); //loc_ta =loc tijdelijk adres
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
void DCC_acc(boolean ws, boolean onoff, byte channel, boolean poort) {
	byte da;	//ws=wissel of sein
	//num is welk volgnummer
	//maakt commandoos voor accessoires, wissels, seinen
	//poort true is afbuigend
	if (ws) { //true seinen
		da = dcc_seinen;
		//Serial.print("channel: "); Serial.print(channel); Serial.print("  ");
		//Serial.print("adres: "); Serial.print(da); Serial.print("  ");
		//Serial.print("Poort: "); Serial.println(poort);
		//Serial.println("*********");
		while (channel > 3) {
			da++;
			channel = channel - 4;
		}
		//Serial.print("channel: "); Serial.print(channel); Serial.print("  ");
		//Serial.print("adres: "); Serial.print(da); Serial.print("  ");
		//Serial.print("Poort: "); Serial.println(poort);
		//Serial.println("");
		//hier nog iets met adres ophoging bij de volgende decoder 
	}
	else { //wissels
		da = dcc_wissels;
	}
	DCC_accAdres(da);
	if (onoff)dcc_data[1] |= (1 << 3);
	dcc_data[1] |= (channel << 1);
	if (poort)dcc_data[1] &= ~(1 << 0);
	dcc_data[2] = dcc_data[0] ^ dcc_data[1];
	dcc_aantalBytes = 2;
	count_repeat = 4;
	GPIOR0 |= (1 << 2); //start zenden accessoire	
	//Serial.print("bytes: ");
	//Serial.print(dcc_data[0], BIN); Serial.print(" "); Serial.print(dcc_data[1], BIN); Serial.print(" "); Serial.println(dcc_data[2], BIN);
}
void DCC_accAdres(byte da) {
	dcc_data[0] = 0x00;
	dcc_data[1] = B11110001;
	//adres bepalen, max 255 decoderadressen
	if (da - 128 >= 0) {
		dcc_data[1] &= ~(1 << 5);
		da = da - 128;
	}
	if (da - 64 >= 0) {
		dcc_data[1] &= ~(1 << 4);
		da = da - 64;
	}
	//da=rest adres
	dcc_data[0] = da; dcc_data[0] |= (1 << 7);
}
void LOC_calc(byte loc) {
	//Serial.println(LOC[loc].speed, BIN);
	byte speed;
	if (LOC[loc].velo & (1 << 0)) { //oneven
		speed = LOC[loc].velo / 2 + 2;
	}
	else { //even/
		speed = LOC[loc].velo / 2 + 1;
		speed |= (1 << 4);
	}
	LOC[loc].speed &= ~(B00011111 << 0); //clear bit 0~4
	LOC[loc].speed = LOC[loc].speed + speed;

	//Serial.println(loc);
	//Serial.println(LOC[loc].speed, BIN);
}
void PRG_locadres(byte newadres, byte all) {
	//sets adres of loc// all= true sets all 127 loc adresses to new adres, if adres is unknown
	//and programs EEPROM 

}
void DCC_cv(boolean type, byte adres, byte cv, byte value) { //CV programming
	//type loco= true, acc is false
	//1110CCVV 0 VVVVVVVV 0 DDDDDDDD
	//old adres alleen bij first zetten van 
	cv = cv - 1;
	GPIOR0 |= (1 << 2);
	if (type) { //locomotive
		dcc_data[0] = adres;
		dcc_data[1] = B11101100; //instruction write CV
		dcc_data[2] = cv;
		dcc_data[3] = value; //adres 6
		dcc_data[4] = dcc_data[0] ^ dcc_data[1] ^ dcc_data[2] ^ dcc_data[3];
		dcc_aantalBytes = 4;
		count_repeat = 4;
	}
	else { //accessoire
		//Serial.println("CV sturen accessoire");

		DCC_accAdres(adres); //fills byte 0 and byte 1
		dcc_data[1] &= ~(15 << 0); //clear bits 0~3
		dcc_data[2] = B11101100;
		dcc_data[3] = cv;
		dcc_data[4] = value;
		dcc_data[5] = dcc_data[0] ^ dcc_data[1] ^ dcc_data[2] ^ dcc_data[3] ^ dcc_data[4];
		dcc_aantalBytes = 5;
		count_repeat = 4;

		Serial.print("bytes: ");
		Serial.print(dcc_data[0], BIN);
		Serial.print(" ");
		Serial.print(dcc_data[1], BIN);
		Serial.print(" ");
		Serial.print(dcc_data[2], BIN);
		Serial.print(" ");
		Serial.print(dcc_data[3], BIN);
		Serial.print(" ");
		Serial.print(dcc_data[4], BIN);
		Serial.print(" ");
		Serial.println(dcc_data[5], BIN);
	}
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
				LOC[0].adres--;
				if (LOC[0].adres == 0)LOC[0].adres = 127;
				break;
			case 1:
				LOC[1].adres--;
				if (LOC[1].adres == 0)LOC[1].adres = 127;
				break;
			case 2:
				dcc_wissels--;
				if (dcc_wissels == 0)dcc_wissels = 255;
				break;
			case 3:
				dcc_seinen--;
				if (dcc_seinen == 0)dcc_seinen = 252;
				break;
			}
			break;
		}
		break;
	case 1: //Testen
		switch (PRG_level) {
		case 2: //kiezen loc, wissels, seinen of melders
			PRG_typeTest--;
			if (PRG_typeTest > 4)PRG_typeTest = 4;
			break;
		case 3: //prglevel3
			if (PRG_typeTest < 2) {
				if (LOC[PRG_typeTest].velo > 0)LOC[PRG_typeTest].velo--;
				LOC_calc(PRG_typeTest);
			}
			switch (PRG_typeTest) {
			case 2: //Wissels testen
				prg_wissels--;
				if (prg_wissels > 3)prg_wissels = 3;
				break;
			case 3: //test seinen
				prg_sein--;
				if (prg_sein > 15)prg_sein = 15;
				break;
			case 4://test melders
				break;
			}
			break;
		}
		//break;
	//}
	case 2: //write DCC
		prg_typecv--;
		if (prg_typecv > 1) prg_typecv = 1; //loco 1 of loco 2
		break;

	case 3: //write CV
		switch (PRG_level) {
		case 2: //parameter
			prg_typecv--;
			if (prg_typecv > 2) prg_typecv = 2; //CV keuzes loc1 loc2 Decoder		
			break;
		case 3: //CV
			PRG_cvs[0]--;
			if (PRG_cvs[0] < 1) PRG_cvs[0] = 255;
			break;
		case 4://Value
			PRG_cvs[1]--;
			//if (PRG_cvs[1] < 1) PRG_cvs[1] = 255;
			break;
		}
		break;
	}
	//DSP_prg();
}
void PRG_inc() {
	switch (PRG_fase) {
	case 0: //inc DCC adres van loc, wissels of seinen
		switch (PRG_level) {
		case 2:
			PRG_typeDCC++;
			if (PRG_typeDCC > 3)PRG_typeDCC = 0;
			break;
		case 3:
			switch (PRG_typeDCC) {
			case 0:
				LOC[0].adres++;
				if (LOC[0].adres == 0)LOC[0].adres = 1;
				break;
			case 1:
				LOC[1].adres++;
				if (LOC[1].adres == 0)LOC[1].adres = 1;
				break;
			case 2:
				dcc_wissels++;
				if (dcc_wissels == 0)dcc_wissels = 1;
				break;
			case 3:
				dcc_seinen++;
				if (dcc_seinen > 252)dcc_seinen = 1;
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
			if (PRG_typeTest < 2) {
				if (LOC[PRG_typeTest].velo < 28) LOC[PRG_typeTest].velo++;
				LOC_calc(PRG_typeTest);
			}
			switch (PRG_typeTest) {
			case 2: //wissels test
				prg_wissels++;
				if (prg_wissels > 3)prg_wissels = 0;
				break;
			case 3: //seinen test
				prg_sein++;
				if (prg_sein > 15)prg_sein = 0;
				break;
			case 4: //melders test
				break;
			}
			break;
		}
		break;
	case 2: //write DCC
		prg_typecv++;
		if (prg_typecv > 1) prg_typecv = 0; //instellen dcc accessoires komen ook hier nu maar 2 loc1 en loc2
		//GPIOR0 ^= (1 << 7);
		break;
	case 3: //write CV
		switch (PRG_level) {
		case 2: //
			prg_typecv++;
			if (prg_typecv > 2) prg_typecv = 0;
			break;
		case 3:
			PRG_cvs[0]++;
			if (PRG_cvs[0] == 0) PRG_cvs[0] = 1;
			break;
		case 4://Value
			PRG_cvs[1]++;
			break;
		}
		break;
	}
}
void DCC_command() {
	byte loc = 0;
	if (GPIOR0 & (1 << 2)) { //Send CV or basic accessoire
		count_repeat--;
		if (count_repeat > 4) GPIOR0 &= ~(1 << 2); //end CV, accessoire transmit
	}
	else { //send loc data
		count_command++;
		if (count_command > 3)count_command = 0;
		switch (count_command) {
		case 0: //loc1 drive
			dcc_data[0] = LOC[0].adres;
			dcc_data[1] = LOC[0].speed;
			break;
		case 1:
			dcc_data[0] = LOC[1].adres;
			dcc_data[1] = LOC[1].speed;
			break;
		case 2:
			dcc_data[0] = LOC[0].adres;
			dcc_data[1] = LOC[0].function;
			break;
		case 3:
			dcc_data[0] = LOC[1].adres;
			dcc_data[1] = LOC[1].function;
			break;
		}
		dcc_aantalBytes = 2;
		dcc_data[2] = dcc_data[0] ^ dcc_data[1];
	}
	//Serial.print("Send: "); Serial.print(dcc_data[0], BIN); Serial.print("- "); Serial.println(dcc_data[1], BIN);
	//Serial.println(dcc_data[0],BIN);
//}//tempcount
}
void SW_exe() {
	byte poort; byte changed; byte temp;
	GPIOR0 ^= (1 << 4);
	if (GPIOR0 & (1 << 4)) {
		//DCC enabled switch lezen
		if ((PINB & (1 << 1)) != (GPIOR1 & (1 << 3))) {
			if (~PINB & (1 << 1)) PINB |= (1 << 0); //Serial.print("*");
			if (PINB & (1 << 1)) {
				GPIOR1 |= (1 << 3);
			}
			else {
				GPIOR1 &= ~(1 << 3);
			}
		}
		//Melders op poort B (pin 4~7) lezen bezig
		PIND |= (1 << 3); //toggle pin 3
		//if (PIND & (1 << 3))temp = 1;
		poort = PIND;
		poort = poort >> 4; //isolate 1 nibble
		changed = poort ^ pos_melders[bitRead(PIND, 3)];
		//[PIND & (1 << 3)]; kan niet array vraagt een cyfer niet een true or false
		if (changed > 0) {
			if (PIND & (1 << 3)) {
				pos_melders[1] = poort;
			}
			else {
				pos_melders[0] = poort;  //[PIND & (1 << 3)] = poort;
			}
			/*
			for (byte i = 0; i < 4; i++) {
			switched, beter md_exe niet gebruiken
			if (changed & (1 << i)) {
			if (PIND & (1 << 3)) {
			MD_exe(i, poort & (1 << i));
			}
			else {
			MD_exe(i + 4, poort & (1 << i));
			}
			}
			}
			*/
			if (PRG_fase == 1 & PRG_level == 3)DSP_prg();
		}


	}
	else { //switches on poort C lezen
		poort = PINC;
		changed = poort ^ sw_statusC;
		for (byte i = 0; i < 4; i++) {
			if (changed & (1 << i) & ~poort & (1 << i)) {
				SW_on(i);
			}
		}
		sw_statusC = poort;
	}
	//}//counter
}
void MD_exe(byte md, boolean onoff) { ////NIET IN GEBRUIK!!!! 
	/*
	waarschijnlijk geen goed idee om het bezet zetten van een melder te gebruiken als actie,
	melders hebben de neiging ernstig te denderen.
	Waarschijnlijk beter om de pos_melder nibbles te testen bij bv.het zoeken van een vrij station,
	en wanneer een loc rijd dus telkens even testen of het doelstation, blok bezet is.
	Dender zal dan veel minder invloed hebben
	*/
	Serial.print("POsmelder0: "); Serial.print(pos_melders[0], BIN);
	Serial.print(" "); Serial.print("Posmelder1: "); Serial.println(pos_melders[1], BIN);

	Serial.print("Melder: "); Serial.print(md); Serial.print(" "); Serial.println(onoff);
	if (PRG_fase == 1) DSP_prg();
}
void SW_double() { //called from SW_exe when sw2 and sw3 is pressed simultanus
	if (PRG_level == 0) {
		GPIOR0 |= (1 << 5); //program mode
		PRG_level = 1;
		DSP_prg();
		//all stop
		LOC[0].speed &= ~(B00011111 << 0); //stop niks doen met richting
		LOC[1].speed &= ~(B00011111 << 0); //stop
		//LOC[0].function = 128;
		//LOC[1].function = 128;
	}
	else { //leave program mode 
		GPIOR0 &= ~(1 << 5); //pendel mode
		PRG_level = 0;
		//DSP_start();
		PDL_fase = 0;
		DSP_pendel();
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
	}
	if (GPIOR0 & (1 << 5)) { //programmode, kan misschien met checken prg_level?
		SW_PRG(sw);
	}
	else {
		SW_pendel(sw);
	}
}
void SW_PRG(byte sw) {
	byte temp;
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
			PDL_fase = 0;
			DSP_pendel();
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
				MEM_update();
				PRG_level--;
				break;
			case 3:
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

			case 1: //Testen, knop 2(3e)
				switch (PRG_typeTest) {
				case 0: //loc 1
					LOC[0].speed ^= (1 << 5); //toggle direction	
					break;
				case 1: //loc 2
					LOC[1].speed ^= (1 << 5);
					break;
				case 2: //wissels
					if (GPIOR0 & ~(1 << 2)) {
						//Serial.println("vrij");
						pos_wissels ^= (1 << prg_wissels);
						//Wissel direct omleggen
						DCC_acc(0, 1, prg_wissels, (pos_wissels & (1 << prg_wissels)));
					}
					break;
				case 3: //seinen lvl3, omzetten sein decoders 	
					GPIOR1 &= ~(1 << 6);

					if (prg_sein < 8) {
						pos_seinen[0] ^= (1 << prg_sein);
						if (pos_seinen[0] & (1 << prg_sein))GPIOR1 |= (1 << 6);
					}
					else {
						pos_seinen[1] ^= (1 << (prg_sein - 8));
						if (pos_seinen[1] & (1 << prg_sein - 8))GPIOR1 |= (1 << 6);
					}
					//Serial.print(pos_seinen[0], BIN); Serial.print("  ");
					//Serial.println(pos_seinen[1], BIN);
					//Serial.print("poort: "); Serial.println(bitRead(GPIOR1,6));

					DCC_acc(true, true, prg_sein, GPIOR1 & (1 << 6));

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
			if (PRG_typeTest < 2) {
				LOC[PRG_typeTest].velo = 0;
				LOC_calc(PRG_typeTest);
			}
			PRG_level--;
			MEM_cancel();
			break;
		}
		break;

		//********Level 4
	case 4:
		switch (PRG_fase) {
		case 3: //CV waarde instellen en bevestigen
			switch (sw) {
			case 0:
				PRG_dec();
				break;
			case 1:
				PRG_inc();
				break;
			case 2:
				MEM_update(); //bevestigen en zenden
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
void SW_pendel(byte sw) {
	byte loc=0;
	if (GPIOR0 & (1 << 7))loc = 1;
	switch (PDL_fase) {
	case 0:
		switch (sw) {
		case 0:
			GPIOR0 ^= (1 << 7);
			//LOC[0].speed = B01101110; //drive	
			break;
		case 1:
			LOC[0].speed = B01100000; //stop
			break;
		case 2:
			//LOC[0].function ^= (1 << 4); //headlights
			break;
		case 3:
			PDL_fase++; //level hoger
			//if (PDL_fase > 3)PDL_fase = 0;
			//PRG_cv(10,1,6);
			//
			//display.clearDisplay();
			//DSP_buttons(0);
			//LOC[0].function ^= (1 << 1); //cabin
			break;
		}
		break;
	case 1: //PDL_fase 1
		Serial.print(loc);
		switch (sw) {
		case 0:
			LOC[loc].function ^= (1 << 0);
			break;
		case 1:
			LOC[loc].function ^= (1 << 1);
			break;
		case 2:
			LOC[loc].function ^= (1 << 2);
			break;
		case 3:
			break;
		}
		break;
	}

	DSP_pendel();
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
	//display.display();
}
void DSP_pendel() {
	byte loc = 0; byte button;
	if (GPIOR0 & (1 << 7))loc = 1; //loc keuze
	cd;
	switch (PDL_fase) {
	case 0:
		regel1; TXT(2);
		TXT(101 + loc);
		if (LOC[loc].speed & (1 << 5)) {
			TXT(13);
		}
		else {
			TXT(14);
		}
		display.print(LOC[loc].velo);
		display.fillRect(5, 22, 23, 23, WHITE);
		display.setTextColor(BLACK);
		display.setCursor(10, 27); display.print(LOC[loc].station);
		button = 1;
		break;
	case 1:
		regel1s, TXT(2); TXT(101 + loc); TXT(4);
		for (byte i = 0; i < 3; i++) {
			if (LOC[loc].function & (1 << i)) { //funcion on
				display.fillCircle(9 + (i * 34), 29, 9, WHITE);
			}
			else { //function off
				display.drawCircle(9 + (i * 34), 29, 9, WHITE);
			}
		}
		button = 2;
		break;
	}
	DSP_buttons(button);
}
void DSP_prg() {
	int adrs;
	byte buttons;
	byte position = 5;
	byte temp;
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

		case 2: //Write adres in loc and accessoires gebleven
			cd; regel1s; TXT(10); TXT(1); regel2; TXT(2);
			switch (prg_typecv) {
			case 0:
				TXT(101);
				break;
			case 1:
				TXT(102);
				break;
			}
			buttons = 10;
			break;
		case 3: //CV programming gebleven
			cd; regel1s; TXT(10); TXT(5); regel2;
			switch (prg_typecv) {
			case 0: //loc1
				TXT(2); TXT(101);
				break;
			case 1: //loc2
				TXT(2); TXT(102);
				break;
			case 2://Decoders
				TXT(6);
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
				regel2; display.print(LOC[0].adres);
				break;
			case 1: //loc2
				TXT(2); TXT(102);
				regel2; display.print(LOC[1].adres);
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
				display.print(adrs); TXT(32); display.print(adrs + 15); TXT(31);//seinen heeft 16 adressen max 32 leds


				break;
			}
			buttons = 10;
			break;
		case 1: //Testen
			cd; regel1s; TXT(11);
			switch (PRG_typeTest) {
			case 0:  //snelheid en richting loco1
				TXT(2); TXT(101);
				regel2;
				if (LOC[0].speed & (1 << 5)) {
					TXT(13);
				}
				else {
					TXT(14);
				}
				display.print(LOC[0].velo);
				buttons = 11;
				break;

			case 1: //Loco 2
				TXT(2); TXT(102);
				regel2;
				if (LOC[1].speed & (1 << 5)) {
					TXT(13);
				}
				else {
					TXT(14);
				}
				display.print(LOC[1].velo);
				buttons = 11;
				break;
			case 2: //Test wissels
				TXT(8);
				display.drawRect(1 + (32 * (prg_wissels)), 18, 30, 23, WHITE); //x,y, width, height

				for (byte i = 0; i < 4; i++) {
					if (pos_wissels & (1 << i)) {
						display.drawLine(position, 35, position + 21, 25, WHITE);
					}
					else {
						display.drawLine(position, 30, position + 21, 30, WHITE);
					}
					position = position + 32;
				}
				buttons = 12;
				break;
			case 3: //Test seinen (lvl3) 
				TXT(9); //Testen Seinen				

				regel2; TXT(15); display.print(1 + prg_sein);
				position = 32;

				if (prg_sein < 8) {
					if (pos_seinen[0] & (1 << prg_sein))position = 12;
				}
				else {
					if (pos_seinen[1] & (1 << prg_sein - 8))position = 12;
				}
				display.drawRect(88, 0, 27, 45, WHITE);
				display.fillCircle(101, position, 6, WHITE);
				buttons = 12;
				break;

			case 4: //Test melders (lvl3) bezig
				TXT(3);
				//show melders
				position = 0;
				temp = 15;
				for (byte a = 0; a < 2; a++) {
					for (byte i = 0; i < 4; i++) {

						if (pos_melders[a] & (1 << i)) {
							display.drawRect(position + (i * 20), 15 + (a*temp), 12, 12, WHITE);
						}
						else {
							display.fillRect(position + (i * 20), 15 + (a*temp), 12, 12, WHITE);
						}
					}
				}
				buttons = 13;
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
			TXT_cv3(); TXT(200);
			display.print(PRG_cvs[1]);
			break;
		}
		buttons = 10;
		break;
	}
	DSP_buttons(buttons);
}
void TXT_cv3() {
	cd; regel1s; TXT(10); TXT(5);
	switch (prg_typecv) {
	case 0:
		TXT(2); TXT(101);
		break;
	case 1:
		TXT(2); TXT(102);
		break;
	case 2:
		TXT(6);
		break;
	}
	regel2; TXT(5); display.print(PRG_cvs[0]);
}
void DSP_buttons(byte mode) {
	//Serial.println("buttons");
	//sets mode in display
	display.fillRect(0, 50, 128, 64, BLACK); //schoont het onderste stukje display, 
//seinen veroorzaakt een storing daar, misschien nog eens naar kijken waar het vandaan komt.
	//waarschijnlijk de conversie van nummer naar txt
	display.drawRect(0, 50, 128, 14, WHITE);
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(3, 54);
	switch (mode) {
	case 0:
		TXT(20);
		break;
	case 1: //Pendel, start LOC # S R PDL_fase=0
		TXT(25);
		break;
	case 2: //functies van de loc PDL_fase 1
		TXT(26);
		break;
	case 10: //standaard programmeer balk
		TXT(21);
		break;
	case 11: //Loc testen balk 
		TXT(22);
		break;
	case 12: //testen wissels/seinen
		TXT(23);
		break;
	case 13: //testen melders, alleen sluiten
		TXT(24);
		break;
	default:
		break;
	}

	display.display();
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
		display.print("Loc ");
		break;
	case 3:
		display.print("Melders ");
		break;
	case 4:
		display.print("functies "); //niet in gebruik
		break;
	case 5:
		display.print("CV ");
		break;
	case 6:
		display.print("Decoders");
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
		display.print("< ");
		break;
	case 14:
		display.print("> ");
		break;
	case 15:
		display.print("S ");
		break;
		//***********Onderbalken
	case 20:
		display.print("Start  loco   FL  F1"); //in gebruik??
		break;
	case 21:
		display.print(" -     +     V     X");
		break;
	case 22:
		display.print("S-    S+     <>    X");
		break;
	case 23:
		display.print(" <    >     []     X");
		break;
	case 24:
		display.print(" -    -      -     X");
		break;
	case 25:
		display.print("loc   start   ?   F");// PDL_fase = 0;
		break;
	case 26:
		display.print("F0    F1    F2    I"); //PDL_fase =1
		break;

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
	case 103:
		display.print("3 ");
		break;
	case 200:
		display.print(" ");
		break;
	case 255:
		display.print("niet bepaald");
		break;
	}
}
void loop() {	//slow events timer
	count_slow++;
	if (count_slow > 12000) {
		//12000 is hoe snel de commandoos elkaar opvolgen vooral lange commandoos als CV accessoires
		//bij 10000 werk dit niet misschien nog wat fine tunen...
		count_slow = 0;
		//slow events
		SW_exe(); //switches
		//start DCC command transmit
		if (GPIOR1 & (1 << 0))DCC_write(); //writing dcc adres in loc
		DCC_command();
		dcc_fase = 1;
	}
}