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
//teksten
#define cd display.clearDisplay()
#define regel1 DSP_settxt(0, 2, 2) //parameter eerste regel groot
#define regel2 DSP_settxt(0, 23, 2) //parameter tweede regel groot
#define regel3 DSP_settxt(10,23,2) //value tweede regel groot
#define regel1s DSP_settxt(0, 2, 1) //value eerste regel klein
#define regel2s DSP_settxt(0, 0, 1) //X Y size X=0 Y=0 geen cursor verplaatsing
Adafruit_SSD1306 display(128, 64, &Wire, -1);
#define TrueBit OCR2A = 115
#define FalseBit OCR2A = 230

//declarations, 
//struct's
struct LOC {
	byte reg;
	/*
	bit0 start, rijden true, stop false
	bit1 relatieve richting rijdend naar rechts of rijdend naar links
	bit2 tijdelijk bit geeft aan of er een route in de richting kan worden gekozen
	bit3 Versnellen false, vertragen true
	*/
	byte Vmax;
	byte Vmin;
	byte adres; //DCC adres
	byte speed; //Byte snelhed en richting
	byte velo; //waarde 1~28 decimale voorstelling snelheid
	byte vaart; //actuele snelheid stap
	byte acc;//accelaratie
	byte function; //functies
	byte station; //welk station staat loc in
	byte route; //welke route rijdt loc
	byte goal; //welk station is loc onderweg
	byte fase; //fase in pendel cyclus
	byte teller; //universele teller, oa voor wissels instellen
	byte slowcount; //hoe lang rijd loc in vmin
	int wait; //ingestelde wachttijd
	unsigned int count;
}LOC[2];

struct route {
	//byte reg; //registers
	/*bit0 richting.... ?????   nodig??
	*/
	byte stationl;
	byte stationr;
	byte wissels; //bit7~4 geblokkeert voor route, 3~0 richting wissels 7-3 wissel1 6-2 wissel2, 5-1 wissel3; 4-0 wissel4 
	byte blokkades;
	byte melders; //melders die niet bezet mogen zijn in een route (stations die worden overgeslagen, kan een loc op staan)
	byte Vloc[2];
} ROUTE[12];
//reserved items, niet in EEPROM, bij opstarten stations reserveren aan de hand van opgeslagen posities in EEPROM
//rest van de locks altijd vrij bij starten.
byte res_station;
byte res_wissels;
byte res_blok;
//count tellers, 
byte count_preample;
byte count_byte;
byte count_bit;
byte count_repeat;
int count_slow;
byte count_wa; //write adres
byte count_command;
byte count_locexe;
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
byte pos_melders[4]; //stand van de melders
//0=melders 1-4; 1=melders 5-8; 2 is samengevoegd laatst bepaald; 3=samengevoegd huidig

//program mode
byte PRG_fase;
byte PRG_level; //hoe diep in het programmeer proces
byte PRG_typeDCC; //actieve adres
byte PRG_typeTest; //actieve test
byte prg_wissels; //actieve wissel
byte prg_sein; //actief sein 16x
byte prg_blokkade;  //active blokkade, weke wordt er aangepast
byte prg_typecv; //ingestelde waarde op PRG_level 2
byte PRG_cvs[2]; //0=CV 1=waarde
//Pendel mode

//pendel mode
byte PDL_fase;
byte rt_sel;
//temps 
byte temp_rss;
//byte temp_blok;
//byte temp_wis;

void setup() {
	delay(2000); //wissels en accesoires moeten ook hardware matig opstarten, bij gelijk aanzetten van de voedingsspanning ontstaan problemen.
	Serial.begin(9600);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

	//poorten
	DDRB |= (1 << 3); //set PIN11 as output for DCC 
	PORTC |= (15 << 0); //set pin A0 A1 pull up *****
	DDRD = 0x00;
	DDRD |= (1 << 3); //pin3 as output
	PORTD |= (B11110000 << 0); //Pull up to pin 7,6,5,4
	DDRB |= (1 << 0); //PIN8 as output enable DCC
	PORTB |= (1 << 1); //pullup to pin 9, DCC on/off swtch
	PORTB |= (1 << 2); //Pullup to PIN 10, Short detect
	sw_statusC = 0xFF;
	sw_statusD = 0xFF;
	//interrupt register settings
	//TCCR2A � Timer/Counter Control Register A
	TCCR2A = 0x00; //clear register
	TCCR2A |= (1 << 6);//Toggle OC2A on Compare Match
	TCCR2A |= (1 << 1); //CTC mode clear timer at compare match
	TCCR2B = 2; //set register timer 2 prescaler 8
	TIMSK2 |= (1 << 1);
	PORTB |= (1 << 0); //set pin8 high
	 //functies = B10000000;
	//DSP_start();

	//Serial.println(PINC);
	if (PINC == 54)Factory();
	GPIOR1 &= ~(1 << 4);
	MEM_read();
	//init
	LOC[0].speed = B01100000;
	LOC[1].speed = B01100000;
	//LOC[0].function = 128;
	//LOC[1].function = 128;
	pos_melders[0] = 0x0F; pos_melders[1] = 0x0F;
	DSP_pendel();
}
void Factory() {
	//resets EEPROM to default
	for (byte i = 100; i < 200; i++) {
		EEPROM.update(i, 0xFF);
	}
}
ISR(TIMER2_COMPA_vect) {
	cli();
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

	//check for short
	//if (PINB & (1 << 2))PORTB &= ~(1 << 0); //disable H-bridge if short
	sei();
}
void MEM_read() {
	byte a;
	for (byte i = 0; i < 2; i++) {
		LOC[i].adres = EEPROM.read(100 + i);
		if (LOC[i].adres == 0xFF) {
			LOC[i].adres = 0x03 + i;
		}
		//merk op, update van eeprom is eigenlijk nergens voor nodig.
		LOC[i].function = EEPROM.read(180 + i);
		if (LOC[i].function == 0xFF)LOC[i].function = B10010000;
		LOC[i].Vmin = EEPROM.read(104 + i);
		if (LOC[i].Vmin > 10)LOC[i].Vmin = 1;
		LOC[i].Vmax = EEPROM.read(106 + i);
		if (LOC[i].Vmax > 28)LOC[i].Vmax = 28;
		//Vlocs, snelheid laden per route
		for (byte y = 0; y < 12; y++) {
			a = y; if (i == 1)a = a + 12;
			ROUTE[y].Vloc[i] = EEPROM.read(200 + a);
			if (ROUTE[y].Vloc[i] == 0xFF)ROUTE[y].Vloc[i] = 5;
		}
	}
	dcc_wissels = EEPROM.read(102);
	if (dcc_wissels == 0xFF) {
		dcc_wissels = 1; //default =1
		//EEPROM.update(102, dcc_wissels);
	}
	dcc_seinen = EEPROM.read(103); //heeft 16 dcc kanalen)
	if (dcc_seinen == 0xFF) {
		dcc_seinen = 252;
		//EEPROM.update(103, dcc_seinen);
	}
	MEM_readroute();
}
void MEM_readroute() { //zie beschrijving
	for (byte i = 0; i < 12; i++) {
		ROUTE[i].stationl = EEPROM.read(108 + i); //120
		ROUTE[i].stationr = EEPROM.read(120 + i); //132
		ROUTE[i].wissels = EEPROM.read(132 + i); // 144 let op default is 0xFF, dus false is bezet.....
		ROUTE[i].blokkades = EEPROM.read(144 + i); //156 blokkade instelling terugladen hoogste nu 140 (132+i(max 7)
		ROUTE[i].melders = EEPROM.read(156 + i);
		if (ROUTE[i].stationl > 8)ROUTE[i].stationl = 0;
		if (ROUTE[i].stationr > 8)ROUTE[i].stationr = 0;
	}
}
void MEM_loc_update(byte loc) {
	//updates loc data 
	byte number;
	number = 104 + loc;
	EEPROM.update(number, LOC[loc].Vmin);
	number = 106 + loc;
	EEPROM.update(number, LOC[loc].Vmax);
	number = 180;
	EEPROM.update(number, LOC[loc].function);
}
void MEM_update() { //sets new values/ sends CV 
	//	Serial.print("MEM_update PRG_fase=");
//	Serial.println(PRG_fase);
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

		if (GPIOR0 & ~(1 << 2)) { //bit2 in GPIOR0 false (verplaatst 17/4)
			//alleen uitvoeren als er  niet een CV programming bezig is
			switch (prg_typecv) {
			case 0:
				//if (GPIOR0 & ~(1 << 2)) { //bit2 in GPIOR0 false
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
	case 4: //Program fase 4 Routes
		//nog aanpassen zodat alleen bij verlaten van update routes dit wordt gedaan...niet na iedere druk op knop 3

		for (byte i; i < 12; i++) {
			EEPROM.update(108 + i, ROUTE[i].stationl); //120
			EEPROM.update(120 + i, ROUTE[i].stationr); //132
			EEPROM.update(132 + i, ROUTE[i].wissels); //144
			EEPROM.update(144 + i, ROUTE[i].blokkades); //156
			EEPROM.update(156 + i, ROUTE[i].melders);//168
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
	case 4: //restore route data
		MEM_readroute();
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
	//Serial.println("adres aangepast");
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

   // Serial.print("bytes: ");
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

	//vreemde plek?  iedere aanpassing in loc moet zichtbaar worden toch?
	if (~GPIOR0 & (1 << 5)) DSP_pendel(); //not in prg mode
}
void LOC_exe() {
	byte loc = 0; byte changed; byte route = 0; byte tmp; byte goal = 0x00; boolean tb; byte a=0;

	GPIOR1 ^= (1 << 2); //toggle active loc
	if (GPIOR1 & (1 << 2))loc = 1;
	//dit proces loopt alleen als loc.reg bit 0 = true
	if (LOC[loc].reg & (1 << 0)) {

		if (LOC[loc].wait < LOC[loc].count) {
			LOC[loc].count = 0; //reset clock

			switch (LOC[loc].fase) {
			case 0: //waiting in station
				if (LOC[loc].station == 0)LOC[loc].fase = 100;
				LOC[loc].fase++;
				LOC[loc].wait = 0;
				break;
			case 1: //begin route zoeken
				//een relatieve rijrichting is hier nodig, dit is de absolute richting waarin de loc 
				//dit station heeft bereikt in combinatie met het vinden van het beginstation door verlaten of door bereiken
				//het verlaten van een station moet de relatieve richting omzetten. reg. bit1 
				//false =links rechts; true = rechts links
				//handmatig omzetten van richting zet relatieve richting NIET om 
				//start is dus altijd de loc van links naar rechts. Starten in een rechts eindblok is niet toegestaan

				//eerst zoeken of er wel een route in deze richting is
				for (byte i = 0; i < 12; i++) {
					if (LOC[loc].reg & (1 << 1)) {
						if (ROUTE[i].stationr == LOC[loc].station)route++;
					}
					else {
						if (ROUTE[i].stationl == LOC[loc].station)route++;
					}
				}

				if (route == 0) {
					if (LOC[loc].reg & (1 << 2)) { //ook in andere richting geen route
						//geen route te vinden stoppen
						LOC[loc].fase = 200;
					}
					else { //richting aanpassen opnieuw testen
						LOC[loc].speed ^= (1 << 5); LOC[loc].reg ^= (1 << 1);
						LOC[loc].reg |= (1 << 2);
					}
					//Serial.println("geen route");
				}
				else { //er zijn mogelijke routes
					LOC[loc].fase = 5;
					LOC[loc].reg &= ~(1 << 2);
				}
				break;
			case 5:
				//voorkomen 'deadlock' als treinen tegen elkaar staaan na lange tijd kijken of in andere richting een route is
				LOC[loc].teller++;
				if (LOC[loc].teller > 150) { //loc keren
					LOC[loc].speed ^= (1 << 5); LOC[loc].reg ^= (1 << 1);
					LOC[loc].reg |= (1 << 2);
					LOC[loc].fase = 1; //opnieuw zoeken
					LOC[loc].teller = 0;
					LOC[loc].reg &= ~(1 << 2);
					break; //uit de function gaan
				}
				//nu een vrije route zoeken, wordt continue herhaald tot route is gevonden
				route = random(0, 12); //12 mogelijke routes
				if (LOC[loc].reg & (1 << 1)) { //l>R
					if (ROUTE[route].stationr == LOC[loc].station) goal = ROUTE[route].stationl;
				}
				else { //R>L
					if (ROUTE[route].stationl == LOC[loc].station) goal = ROUTE[route].stationr;
				}

				if (goal == 0) break; //uitspringen, geen route gevonden
				//Serial.println(goal);

				tmp = 0;
				//check doelstation, anders switch verlaten
				if (res_station & (1 << goal - 1)) tmp = 1;
				//check wissels

				for (byte i = 0; i < 4; i++) {
					if (~ROUTE[route].wissels & (1 << 7 - i)) { //wissel opgenomen in gewenste route
						//Serial.println("check wissels");
						if (res_wissels & (1 << i))tmp = 1; // wissel is bezet, verlaat proces
					}
				}
				//check blokkades en melders die niet bezet mogen zijn
				for (byte i = 0; i < 8; i++) {
					if (~ROUTE[route].blokkades & (1 << i)) {
						//Serial.println("check blokkade");
						if (res_blok & (1 << i)) tmp = 1; //als blokkade is bezet, verlaat proces
					}
					if (~ROUTE[route].melders & (1 << i)) {
						//check melders gebruik van changed is nu vrij (hoop ik)
						//changed = MELDERS();
						if (res_station & (1 << i))tmp = 1; //if false dan bezet
					}
				}
				if (tmp == 1)break;
				//doel station reserveren
				res_station |= (1 << goal - 1); //goal 1 = bit 0
				//route akkoord uitvoeren
				LOC[loc].route = route;
				LOC[loc].goal = goal;
				LOC[loc].teller = 0;
				LOC[loc].fase = 10;
				LOC[loc].wait = 0;
				//reserveer blokkades en melders(stations)
				for (byte i = 0; i < 8; i++) {
					if (~ROUTE[route].blokkades & (1 << i)) res_blok |= (1 << i);
					if (~ROUTE[route].melders & (1 << i)) res_station |= (1 << i);
				}
				break;
			case 10: //set wissels
				if (~ROUTE[LOC[loc].route].wissels & (1 << (7 - LOC[loc].teller))) {  //bit 7~4 
					//Serial.println("wissel");
					res_wissels |= (1 << LOC[loc].teller); //reserveer wissel
					tb = ROUTE[LOC[loc].route].wissels & (1 << (3 - LOC[loc].teller));

					//DCC_acc(false, true, LOC[loc].teller, !tb); //set wissel +1?		
					DCC_acc(false, true, LOC[loc].teller, tb); //set wissel +1?						
				}
				LOC[loc].teller++;
				if (LOC[loc].teller > 3) {
					LOC[loc].teller = 0;
					LOC[loc].fase = 20;
					LOC[loc].wait = 20;
				}
				break;

			case 20: //drive init
				LOC[loc].wait = 0;
				LOC[loc].velo = LOC[loc].Vmin; LOC_calc(loc);
				LOC[loc].vaart = LOC[loc].velo;
				LOC[loc].fase = 25;
				LOC[loc].reg &= ~(1 << 3); //versnellen
				LOC[loc].teller = 0;
				LOC[loc].acc = 4;
				LOC[loc].slowcount = 0;
				break;
			case 25:
				//test of doelstation is bereikt,  is weer vrij
				tmp = MELDERS();
				//Serial.println(tmp, BIN);
				if (~tmp & (1 << LOC[loc].goal - 1)) { //stations 1~8  melders 0~7// station bereikt
					LOC[loc].fase = 30;
				}
				else { //station niet bereikt, snelheid aanpassen
					LOC[loc].teller++;
					if (LOC[loc].teller > LOC[loc].acc) {
						LOC[loc].teller = 0;

						if (LOC[loc].reg & (1 << 3)) { //vertragen

							if (LOC[loc].vaart > 1) LOC[loc].vaart--;//limit to 1
							if (LOC[loc].velo > LOC[loc].Vmin) {
								LOC[loc].velo--; LOC_calc(loc);
							}
							else { //rijd op mininmale snelheid meten hoelang
								LOC[loc].slowcount++;
							}
						}
						else { //versnellen
							//Serial.println(ROUTE[LOC[loc].route].Vloc[loc]);
							if (LOC[loc].vaart < 255) LOC[loc].vaart++; //limit
							if (LOC[loc].vaart < ROUTE[LOC[loc].route].Vloc[loc]) {
								//versnellen
								if (LOC[loc].vaart < LOC[loc].Vmax) {
									LOC[loc].velo++; LOC_calc(loc);
								}
							}
							else { //omschakelen naar vertragen
								LOC[loc].reg |= (1 << 3);
								LOC[loc].acc = 8;
							}
						}
					}
				}

				break;
			case 30: //eerste test melders was positief
				tmp = MELDERS(); //2e x melders testen om spikes uit te vangen
				if (~tmp & (1 << LOC[loc].goal - 1)) { //stations 1~8  melders 0~7// station bereikt				

				//Vloc aanpassen...
					GPIOR1 &= ~(1 << 7); //use temp boolean
					if (LOC[loc].velo == LOC[loc].Vmin) { //stopt in minimale snelheid
						if (LOC[loc].slowcount > 3) {
							ROUTE[LOC[loc].route].Vloc[loc] ++;// = (LOC[loc].slowcount / 5) + ROUTE[LOC[loc].route].Vloc[loc];
							GPIOR1 |= (1 << 7);
						}
					}
					else { //stopt, rijd te snel
						ROUTE[LOC[loc].route].Vloc[loc]--;// = ROUTE[LOC[loc].route].Vloc[loc] - (LOC[loc].velo - LOC[loc].Vmin);
						GPIOR1 |= (1 << 7);
					}
					if (GPIOR1 & (1 << 7)) {
					//snelheidverschil groot, nieuwe routetijd opslaan in EEPROM
					Serial.println("opslaan");
					a = 200 + LOC[loc].route;
					if (loc == 1)a = a + 12;
					EEPROM.update(a, ROUTE[LOC[loc].route].Vloc[loc]);
					}
					GPIOR1 &= ~(1 << 7); //release temp boolean

					//restsnelheid bepalen, doorrijden					
					switch (LOC[loc].velo) {
					case 1:
						LOC[loc].wait = 5;
						break;
					case 2:
						LOC[loc].wait = 2;
						break;
					case 3:
						LOC[loc].wait = 1;
						break;

					default:
						tmp = 0;
						LOC[loc].wait = 0;
						LOC[loc].velo = 0; LOC_calc(loc);
						break;
					}

					//aanpassen in EEPROM
					//Serial.print("Vloc= "); Serial.println(ROUTE[LOC[loc].route].Vloc[loc]);
					res_station &= ~(1 << LOC[loc].station - 1);//oude beginstation vrij geven		

					//Serial.println(res_station,BIN);
					for (byte i = 0; i < 4; i++) {//wissels vrijgeven
						if (~ROUTE[LOC[loc].route].wissels & (1 << 7 - i))res_wissels &= ~(1 << i);
					}
					for (byte i = 0; i < 8; i++) {//blokkades en gemelde stations vrijgeven
						if (~ROUTE[LOC[loc].route].blokkades & (1 << i)) res_blok &= ~(1 << i);
						if (~ROUTE[LOC[loc].route].melders & (1 << i))res_station &= ~(1 << i);
					}

					LOC[loc].station = LOC[loc].goal;
					LOC[loc].goal = 0;
					LOC[loc].fase = 35;

				}
				else {
					LOC[loc].fase = 25; //opnieuw testen
				}
				break;

			case 35:
				//stop locomotief
				LOC[loc].velo = 0; LOC_calc(loc);
				LOC[loc].fase = 0;
				LOC[loc].wait = 10; //station wachttijd
				break;

			case 101: //begin find starting point (station)
				//LOC[loc].wait = 5; //timing testen melders
				//zorg dat de andere loc niet rijdt
				//zet alle wissels in recht posities zodanig dat de locs niet naar  elkaar kunnen rijden
				//Hier is een stukje handmatig en gezond verstand van de gebruiker nodig.

				//leg huidige melders status vast
				pos_melders[2] = MELDERS();
				pos_melders[3] = pos_melders[2];

				LOC[loc].velo = 3; LOC_calc(loc);
				LOC[loc].fase = 102;
				break;
			case 102:
				//Serial.println(pos_melders[2], BIN);
				//testen of er een verandering is in richting
				pos_melders[3] = MELDERS();
				changed = pos_melders[3] ^ pos_melders[2];
				if (changed > 0) {
					for (byte i = 0; i < 8; i++) {
						if (changed & (1 << i)) { //find changed melder
							if (pos_melders[3] & (1 << i)) { //melder vrij gekomen
								LOC[loc].speed ^= (1 << 5); //change actual direction
								LOC[loc].reg ^= (1 << 1);// change virtual direction (L<>R)
								pos_melders[2] = pos_melders[3];
							}
							else { //melder bezet, station bekend
								LOC[loc].station = i + 1;
								res_station |= (1 << i); //block station
								LOC[loc].velo = 0; LOC_calc(loc); //stop
								LOC[loc].fase = 0;

							}
						}
					}
				}
				break;
			case 200:
				//Serial.println("stop");
				LOC[loc].reg &= ~(1 << 0); //stop
				DSP_pendel();
				break;
			}
		}
		LOC[loc].count++;
	}
	//if (temp_rss != res_station) { Serial.print("res_stations: "); Serial.println(res_station, BIN); }
	//temp_rss = res_station;
	//if (temp_wis != res_wissels) { Serial.print("res_wissels: "); Serial.println(res_wissels, BIN); }
	//temp_wis = res_wissels;
	//if (temp_blok != res_blok) { Serial.print("res_blokkade: "); Serial.println(res_blok, BIN); }
	//temp_blok = res_blok;
}
void INIT_wissels() {
	//initialiseert de aangesloten wissels en seinen, false is wissels
	//wissels, gebruiken PRG_wissels ... kan problemen geven door dubbel gebruik
	if (~GPIOR0 & (1 << 2)) { //als verwerken vorige DCC commando klaar is
		//Serial.println(prg_wissels);
		DCC_acc(false, true, prg_wissels, GPIOR1 & (1 << 6));
		GPIOR1 ^= (1 << 6);
		if (~GPIOR1 & (1 << 6))prg_wissels++;
		if (prg_wissels > 4) {
			prg_wissels = 0;
			GPIOR1 |= (1 << 4); //exit wissel init
		}
	}
}
byte MELDERS() {
	byte melder;
	melder = pos_melders[1] << 4;
	melder = melder + pos_melders[0];
	//merk op 0=bezet
	return melder;
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
	case 4: //route keuze, let op nu een increment
		switch (PRG_level) {
		case 2: //route keuze
			rt_sel++;
			if (rt_sel > 11)rt_sel = 0;
			break;
		case 3: //keuze wissel in route
			prg_wissels++;
			if (prg_wissels > 3)prg_wissels = 0;
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
	case 4: //route station links instellen 
		switch (PRG_level) {
		case 2:
			ROUTE[rt_sel].stationl++;
			if (ROUTE[rt_sel].stationl > 8)ROUTE[rt_sel].stationl = 0;
			break;
		case 3: //route wissel bezet zetten
			ROUTE[rt_sel].wissels ^= (1 << (7 - prg_wissels)); //toggle bit 7~4 
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

			if (~PINB & (1 << 1)) PINB |= (1 << 0); //toggle DCC enabled

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

	Serial.print("POsmelder0: "); Serial.print(pos_melders[0], BIN);
	Serial.print(" "); Serial.print("Posmelder1: "); Serial.println(pos_melders[1], BIN);

	Serial.print("Melder: "); Serial.print(md); Serial.print(" "); Serial.println(onoff);
	if (PRG_fase == 1) DSP_prg();
*/
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
			if (PRG_fase > 4)PRG_fase = 4;
			break;
		case 1:
			PRG_fase++;
			if (PRG_fase > 4)PRG_fase = 0;
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
			case 3: //CV 
				PRG_level++;
				break;
			case 4: //Route instellen, stationr inc.
				ROUTE[rt_sel].stationr++;
				if (ROUTE[rt_sel].stationr > 8)ROUTE[rt_sel].stationr = 0;
				break;
			}
			break;
		case 3:
			switch (PRG_fase) {
			case 4:
				//MEM_update();
				PRG_level++;
				//Serial.println(PRG_level);
				break;

			default:
				PRG_level--;
				MEM_cancel();
				break;
			}
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
						pos_wissels ^= (1 << prg_wissels);
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
			case 4: //Route, direction wissel instellen
				ROUTE[rt_sel].wissels ^= (1 << 3 - prg_wissels);
				//UITVOEREN wissel omzetten hier....
				DCC_acc(false, true, prg_wissels, ROUTE[rt_sel].wissels & (1 << 3 - prg_wissels));
				break;
			}
			break;
		case 3: //switch 3, level 3
			switch (PRG_fase) {
			case 4:
				PRG_level++; //naar level 4
				break;
			default:
				PRG_level--;
				MEM_cancel();
				break;
			}
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

		case 4: //route instellen, seinen instellen
			switch (sw) {
			case 0:
				break;
			case 1:
				break;
			case 2:
				break;
			case 3:
				PRG_level++; //naar level 5
				break;
			}
			break;
		}
		break;
	case 5: //route blokkades instellen
		//alleen program fase 4, routes mogelijk. 14juni020
		switch (sw) {
		case 0:
			prg_blokkade++;
			if (prg_blokkade > 15)prg_blokkade = 0;
			break;
		case 1:
			if (prg_blokkade < 8) {
				ROUTE[rt_sel].blokkades ^= (1 << prg_blokkade);
			}
			else {
				ROUTE[rt_sel].melders ^= (1 << prg_blokkade - 8);
			}
			break;
		case 2:
			MEM_update();
			PRG_level = 1;
			break;
		case 3: //annuleren invoer
			PRG_level = 1;
			MEM_cancel();
			break;
		}
		break;
	}
	if (~GPIOR1 & (1 << 1)) DSP_prg();
	//DSP_prg();
	//dit bit zorgt dat pas na de bewerking, bv. adres schrijven
//het display vernieuwd ibv. progressbar bv.
}
void SW_pendel(byte sw) {
	byte loc = 0;
	if (GPIOR0 & (1 << 7))loc = 1;
	switch (PDL_fase) {
	case 0:
		switch (sw) {
		case 0:
			GPIOR0 ^= (1 << 7); //toggle loc keuze
			break;
		case 1:
			LOC[loc].reg ^= (1 << 0); //toggle start/stop

			if (LOC[loc].reg & (1 << 0)) {  //start rijden process
				LOC[loc].fase = 0;
				LOC[loc].wait = 0;
			}
			else { //stop locomotief
				LOC[loc].velo = 0; LOC_calc(loc);
			}
			break;
		case 2:
			if (LOC[loc].reg & (1 << 0)) { //loc rijdt
				//11juni nog geen functie
			}
			else {//loc staat stil
				//station kiezen
				//stations vrij geven
				if (LOC[loc].station > 0)res_station &= ~(1 << LOC[loc].station - 1);
				if (LOC[loc].goal > 0)res_station &= ~(1 << LOC[loc].goal - 1);
				LOC[loc].goal = 0; //wis doel en route
				//wis geblokkeerde wissels?
				//wis geblokkeerde blokkades?
				LOC[loc].station++;
				if (res_station & (1 << LOC[loc].station - 1)) LOC[loc].station++; //gereserveerde stations overslaan
				if (LOC[loc].station > 8) {
					LOC[loc].station = 1;
					if (res_station & (1 << 0))LOC[loc].station = 2;
				}
				res_station |= (1 << LOC[loc].station - 1); //reserveer station
			}
			break;
		case 3:
			PDL_fase++; //level hoger
			break;
		}
		break;
	case 1: //PDL_fase 1
		//Serial.print(loc);
		switch (sw) {
		case 0:
			LOC[loc].function ^= (1 << 4);
			break;
		case 1:
			LOC[loc].function ^= (1 << 0);
			break;
		case 2:
			LOC[loc].function ^= (1 << 1);
			break;
		case 3:
			PDL_fase++;
			break;
		}
		break;
	case 2: //PDL_fase 2, loc instellingen
		switch (sw) {
		case 0:
			LOC[loc].speed ^= (1 << 5);
			break;
		case 1:
			LOC[loc].Vmin++;
			if (LOC[loc].Vmin > 10)LOC[loc].Vmin = 1;
			break;
		case 2:
			LOC[loc].Vmax--;
			if (LOC[loc].Vmax < 10)LOC[loc].Vmax = 28;
			//merk op als eeprom fout wordt gelezen komt er 255 in het vmax
			break;
		case 3:
			PDL_fase = 0;
			MEM_loc_update(loc);
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
			TXT(14);
		}
		else {
			TXT(13);
		}
		display.print(LOC[loc].velo);
		regel2; display.print(LOC[loc].station); TXT(32); display.print(LOC[loc].goal);

		//display.fillRect(5, 22, 23, 23, WHITE);
		//display.setTextColor(BLACK);
		//display.setCursor(10, 27); display.print(LOC[loc].station);

		if (LOC[loc].reg & (1 << 0)) {
			button = 4;
		}
		else {
			button = 1;
		}
		break;
	case 1:
		regel1s, TXT(2); //TXT(101 + loc); 
		TXT(4);
		for (byte i = 0; i < 3; i++) {
			button = i - 1;
			if (button > 10)button = 4;
			if (LOC[loc].function & (1 << button)) { //funcion on
				display.fillCircle(9 + (i * 34), 29, 9, WHITE);
			}
			else { //function off
				display.drawCircle(9 + (i * 34), 29, 9, WHITE);
			}
		}
		button = 2;
		break;
	case 2:
		regel1s, TXT(2); //TXT(101 + loc); 
		TXT(7); regel2;
		if (LOC[loc].speed & (1 << 5)) {
			TXT(14);
		}
		else {
			TXT(13);
		}
		display.setCursor(32, 23); display.print(LOC[loc].Vmin);
		display.setCursor(75, 23); display.print(LOC[loc].Vmax);

		button = 3;
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
		cd; regel1;
		switch (PRG_fase) {

		case 0: //Instellen DCC adres
			TXT(7); TXT(0); //Instellen
			regel2; TXT(1); //DCC adres	
			break;
		case 1: //Testen
			TXT(11);
			break;
		case 2: //write loc adresses
			TXT(10); TXT(0);
			regel2; TXT(1); //Write DCC adres
			break;
		case 3: //Cv programming
			TXT(10); TXT(0);
			regel2; TXT(5);
			break;
		case 4: //instellen routes
			TXT(7); TXT(0); regel2; TXT(16);
			break;
		}

		buttons = 10;
		break;
		//**********************************level 2
	case 2: // program level 2
		buttons = 10;
		cd;
		regel1s;
		switch (PRG_fase) {
		case 0: //keuze loc of accessoire
			//regel1s; 
			TXT(7); TXT(1); TXT(0); regel2;
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
			//buttons = 10;
			break;

		case 1: //Testen
			//regel1s; 
			TXT(11); regel2;
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
			//buttons = 10;
			break;

		case 2: //Write adres in loc and accessoires gebleven
			//regel1s; 
			TXT(10); TXT(1); regel2; TXT(2);
			switch (prg_typecv) {
			case 0:
				TXT(101);
				break;
			case 1:
				TXT(102);
				break;
			}
			//buttons = 10;
			break;
		case 3: //CV programming gebleven
			//regel1s; 
			TXT(10); TXT(5); regel2;
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
			//buttons = 10;
			break;
		case 4:
			TXT(16); regel2;
			//rt_sel =0~7  toont 1~8
			display.print(rt_sel + 1); TXT(200); TXT(30); display.print(ROUTE[rt_sel].stationl);
			TXT(32); display.print(ROUTE[rt_sel].stationr); TXT(31);
			buttons = 14;
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
					TXT(14);
				}
				else {
					TXT(13);
				}
				display.print(LOC[0].velo);
				buttons = 11;
				break;

			case 1: //Loco 2
				TXT(2); TXT(102);
				regel2;
				if (LOC[1].speed & (1 << 5)) {
					TXT(14);
				}
				else {
					TXT(13);
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
		case 4: //routes wissels vastleggen
			cd; regel1s; TXT(16); display.print(rt_sel + 1); TXT(200); TXT(8);
			regel2; display.print(prg_wissels + 1);
			if (~ROUTE[rt_sel].wissels & (1 << (7 - prg_wissels))) { //wissel bezet als bit false is.
				display.fillRect(33, 25, 12, 12, WHITE);
			}
			else {
				display.drawRect(33, 25, 12, 12, WHITE);
			}
			if (ROUTE[rt_sel].wissels & (1 << 3 - prg_wissels)) { //toon wisselstand
				display.drawLine(72, 37, 88, 23, WHITE);
			}
			else {

				display.drawLine(70, 32, 90, 32, WHITE);
			}

			buttons = 15;
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
			buttons = 10;
			break;
		case 4: //Route program, seinen instellen
			cd; regel1s; TXT(16); display.print(rt_sel + 1); TXT(200); TXT(9);
			buttons = 16;
			break;
		}
		break;
		//************ Level 5
	case 5:
		cd; regel1s; TXT(16); display.print(rt_sel + 1); TXT(200);
		if (prg_blokkade < 8) { //blokkades
			TXT(17);
			regel2; display.print(prg_blokkade + 1);
			if (~ROUTE[rt_sel].blokkades & (1 << prg_blokkade)) { //if false bezet
				display.fillRect(33, 25, 12, 12, WHITE);
			}
			else {
				display.drawRect(33, 25, 12, 12, WHITE);
			}
		}
		else { //melders
			TXT(3);
			regel2; display.print(prg_blokkade - 7);
			if (~ROUTE[rt_sel].melders & (1 << prg_blokkade - 8)) { //if false bezet
				display.fillRect(33, 25, 12, 12, WHITE);
			}
			else {
				display.drawRect(33, 25, 12, 12, WHITE);
			}
		}
		buttons = 17;
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
			//start moet hier wisselen met stop....
		TXT(25);
		break;
	case 2: //functies van de loc PDL_fase 1
		TXT(26);
		break;
	case 3: //instellingen van de loc, dir, vmax vmin
		TXT(27);
		break;
	case 4:
		TXT(20); //pendel in stop
		break;
	case 5: //routes, instellen wissels
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
	case 14: //routes instellen
		TXT(28);
		break;
	case 15: //routes, wissels instellen
		TXT(29);
		break;
	case 16: //routes seinen instellen (false=groen)
		TXT(19);
		break;
	case 17: //routes, blokkades instellen
		TXT(18);
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
		display.println(F(""));
		break;
	case 1:
		display.print(F("DCC adres "));
		break;
	case 2:
		display.print(F("Loc "));
		break;
	case 3:
		display.print(F("Melders "));
		break;
	case 4:
		display.print(F("functies ")); //niet in gebruik
		break;
	case 5:
		display.print(F("CV "));
		break;
	case 6:
		display.print(F("Decoders"));
		break;
	case 7:
		display.print(F("Instellen "));
		break;
	case 8:
		display.print(F("Wissels "));
		break;
	case 9:
		display.print("Seinen ");
		break;
	case 10:
		display.print(F("Schrijf "));
		break;
	case 11:
		display.print(F("Testen "));
		break;
	case 12:
		display.print(F("DCC "));
		break;
	case 13:
		display.print(F("< "));
		break;
	case 14:
		display.print(F("> "));
		break;
	case 15:
		display.print(F("S "));
		break;
	case 16:
		display.print(F("Route "));
		break;
	case 17:
		display.print(F("Blokkades "));
		break;
		//***********Onderbalken
	case 18:
		display.print(F("B/M    *     V     X"));  //keuze routes, seinen
		break;

	case 19:
		display.print(F("<>    S     *    B/M"));  //keuze routes, seinen
		break;
	case 20:
		display.print(F("loc   stop    ?    F"));
		break;
	case 21:
		display.print(F(" -     +     V     X"));
		break;
	case 22:
		display.print(F("S-    S+     <>    X"));
		break;
	case 23:
		display.print(F(" <    >     []     X"));
		break;
	case 24:
		display.print(F(" -    -      -     X"));
		break;
	case 25:
		display.print(F("loc start station  F"));// PDL_fase = 0;
		break;
	case 26:
		display.print(F("F0    F1    F2    I")); //PDL_fase =1
		break;
	case 27:
		display.print(F("<>   Vmin   Vmax   D")); //PDL_fase =1
		break;
	case 28:
		display.print(F("R     SL     SR    W")); //PDL_fase =1
		break;
	case 29:
		display.print(F("W     *     <>     S")); //routes, instellen wissels
		break;

		//******************
	case 30:
		display.print(F(" ("));
		break;
	case 31:
		display.print(F(") "));
		break;
	case 32:
		display.print(F("-"));
		break;
		//case 33:
		//	display.print(F(" "));
		//	break;
	case 100:
		display.print(F("0 "));
		break;
	case 101:
		display.print(F("1 "));
		break;
	case 102:
		display.print(F("2 "));
		break;
	case 103:
		display.print(F("3 "));
		break;
	case 200:
		display.print(F(" "));
		break;
	case 255:
		//display.print("niet bepaald");
		break;
	}
}
void loop() {	//slow events timer
	count_slow++;
	if (count_slow > 12000) {

		if (PINB & (1 << 2))PORTB &= ~(1 << 0); //disable H-bridge if short


		//12000 is hoe snel de commandoos elkaar opvolgen vooral lange commandoos als CV accessoires
		//bij 10000 werk dit niet misschien nog wat fine tunen...
		count_slow = 0;
		//slow events
		SW_exe(); //switches
		//start DCC command transmit
		count_locexe++;
		if (count_locexe > 10) {
			if (GPIOR1 & (1 << 4)) {
				LOC_exe();
			}
			else {
				INIT_wissels();
			}
			count_locexe = 0;
		}
		if (GPIOR1 & (1 << 0))DCC_write(); //writing dcc adres in loc
		DCC_command();
		dcc_fase = 1;
	}
}