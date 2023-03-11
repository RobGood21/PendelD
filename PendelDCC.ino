/*
	Name:       PendelD.ino
	Created:	2020
	Author:     Rob Antonisse

Arduino sketch for automated train control.
Max 2 locomotive, 8 switches, 16 signals and 12 routes
Project name PenDelDCC.
S

Versions:
PendelD.ino V1.01 july 2020

Version V2.01 oktober 2021
toevoegingen tbv van draaischijf module
-Version zichtbaar in display welcome
-in diverse. Nieuwe instelling voor M8 als melder 8 of als acc rdy  (accessoire ready) geeft aan dat accessoire(s) klaar
zijn met instellen. Als klaar dan is M8 HOOG.
-In diverse:  Doorrijden na melder instellen als factor 'drf' per locomotief
-In diverse Slot stuurt 2 DCC accessoires aan als M8 wordt geactiveerd, voor grendelen of spoorweg bomen.
-In diverse Stoptijd, limiteerd de max wachttijd tussen twee ritten
Melder mode 8 wordt 2x gelezen.

debugging
26okt aantal commands in accessoire verhoogd naar 8 werkt niet fout wordt ernstiger verlaagt naar 3
28nov

V2.02 21sept2022
Snelheid en drf doorrijden, sterk vereenvoudigd, werkt zo veel smoother,
en kans op te vroeg stoppen verminderd.

V2.03

V3.01
Probleem is dat het melder traject niet te kort mag zijn.
Probleem is dan dat de pendeldcc de melding niet ziet en doorrijd.
Melding moet worden ingeklokt zodat deze minimaal 1 seconde duurt.
Spookmeldingen opgelost, werde veroorzaakt omdat PIN3 werd geschakeld en daarmee de TLP281 actief
en direct daarna de poort lezen. Dit aangepast.
V4.01
bugfixed:
S1 seinen offset werd niet goed weergegeven

new:
Serial verbinding en gegevens uitwisseling met WMapp erbij


*/

//libraries
#include <EEPROM.h>
//display 
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//teksten
#define version "V4.01" //tonen tijdens opstarten
#define cd display.clearDisplay() //verkorte txt
#define regel1 DSP_settxt(0, 2, 2) //parameter eerste regel groot
#define regel2 DSP_settxt(0, 23, 2) //parameter tweede regel groot
#define regel3 DSP_settxt(10,23,2) //value tweede regel groot
#define regel1s DSP_settxt(0, 2, 1) //value eerste regel klein
#define regel2s DSP_settxt(0, 0, 1) //X Y size X=0 Y=0 geen cursor verplaatsing

//Teksten V401
//#define wm "www.wisselmotor.nl"

Adafruit_SSD1306 display(128, 64, &Wire, -1); //constructor display
#define TrueBit OCR2A = 115 //pulsduur true bit
#define FalseBit OCR2A = 230 //pulsduur false  bit

//declarations, 
//struct's
struct LOC {
	byte reg;
	/*
	bit0 start, rijden true, stop false
	bit1 relatieve richting rijdend naar rechts of rijdend naar links
	bit2 tijdelijk bit geeft aan of er een route in de richting kan worden gekozen
	bit3 Versnellen false, vertragen true
	bit7 richting bij start, default = true
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
	byte drf; //=door rijden factor 
	unsigned int count;
}LOC[2];

struct route {
	byte stationl;
	byte stationr;
	byte wissels; //bit7~4 geblokkeert voor route, 3~0 richting wissels 7-3 wissel1 6-2 wissel2, 5-1 wissel3; 4-0 wissel4 
	byte seinen[4]; //b0=>0-7 b1=>8-15 b2=<0-7 b3=<8-15
	byte blokkades;
	byte melders; //melders die niet bezet mogen zijn in een route (stations die worden overgeslagen, kan een loc op staan)
	byte Vloc[2];
} ROUTE[12];


byte res_station; //gereserveerd station
byte res_wissels; //idem wissels
byte res_blok; //idem blok
//count tellers, 
byte count_preample;
byte count_byte;
byte count_bit;
byte count_repeat;
int count_slow;
byte count_wa; //write adres
byte count_command;
byte count_locexe;
byte count_slot;
byte dcc_fase;
byte dcc_data[6]; //bevat te verzenden DCC bytes, current DCC commando
byte dcc_aantalBytes; //aantal bytes current van het DCC commando

byte sw_statusC = 0xFF; //laatste stand van switches op C port
byte sw_statusD = 0xFF; //D port

byte loc_ta; //temp loc adres nodig in CV adres programming
byte dcc_wissels; //dcc basis adres wissel decoder 
byte dcc_seinen; //dcc basis adres sein decoder
byte pos_wissels; //stand positie van de vier wissels
byte pos_seinen[2]; //stand positie van de seinen
byte pos_melders[4]; //stand van de melders
//0=melders 1-4; 1=melders 5-8; 2 is samengevoegd laatst bepaald; 3=samengevoegd huidig
byte sein2;
byte possein2;
byte seinoff_route[2];
byte seinoff_dir[2];
byte seinoff_count[2];

//program mode
byte MEM_reg; //8 op te slaan booleans
byte PRG_fase;
byte PRG_level; //hoe diep in het programmeer proces
byte PRG_typeDCC; //actieve adres
byte PRG_typeTest; //actieve test
byte prg_wissels; //actieve wissel
byte prg_sein; //actief sein 16x
byte prg_seinoffset;
byte prg_blokkade;  //active blokkade, weke wordt er aangepast
byte prg_typecv; //ingestelde waarde op PRG_level 2
byte prg_diverse;
byte PRG_cvs[2]; //0=CV 1=waarde
//Pendel mode
//pendel mode
byte PDL_fase;
byte rt_sel;
byte slotstatus = 0; //bit0 uitvoeren bit1 aan of uit bit2 eerste of tweede servo, kant
byte stoptijd;

//V2.02 
unsigned long swtime = 0;

//V3.01 
byte Mcount = 0;
byte old_posmelders[2] = { 0xFF,0xFF };
byte SW_volgorde = 0;

//V4.01 //koppeling met WMapp erbij.
byte command[20]; //array 32 is rijkelijk te veel, als alles werkt kleiner maken.
byte commandcount;
byte AantalBytes;
byte old_melders = 0;

void setup() {
	Serial.begin(9600);
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	//Openings tekxt
	cd; //clear display
	regel1s; display.print(F("www.wisselmotor.nl"));//V401
	regel2; display.print(F("PenDelDCC"));
	DSP_settxt(80, 50, 1); display.print(version);
	display.display();

	delay(1000); //seconde wachten om de welkomst tekst te kunnen tonen

	//poorten
	DDRB |= (1 << 3); //set PIN11 as output for DCC 
	PORTC |= (15 << 0); //set pin A0 A1 pull up *****
	DDRD = 0x00;
	DDRD |= (1 << 3); //pin3 as output
	PORTD |= (B11110000 << 0); //Pull up to pin 7,6,5,4
	DDRB |= (1 << 0); //PIN8 as output enable DCC
	PORTB |= (1 << 1); //pullup to pin 9, DCC on/off swtch
	PORTB |= (1 << 2); //Pullup to PIN 10, Short detect
	//sw_statusC = 0xFF; //V401
	//sw_statusD = 0xFF; //V401

	//interrupt register settings
	//TCCR2A – Timer/Counter Control Register A, timer 2 used for generate DCC pulses
	TCCR2A = 0x00; //clear register
	TCCR2A |= (1 << 6);//Toggle OC2A on Compare Match
	TCCR2A |= (1 << 1); //CTC mode clear timer at compare match
	TCCR2B = 2; //set register timer 2 prescaler 8
	TIMSK2 |= (1 << 1);
	PORTB |= (1 << 0); //set pin8 high
	if (PINC == 54)Factory(); //holding button 1 and 4 starts EEPROM reset
	GPIOR1 &= ~(1 << 4); //bug28nov
	//GPIOR1 |= (1 << 4);
	MEM_read();
	//init
	//pos_melders[0] = 0x0F; pos_melders[1] = 0x0F;  //V401
	GPIOR0 |= (1 << 5);//disable DSP update //V401
}
void Factory() {
	//resets EEPROM to default
	for (int i = 0; i < EEPROM.length(); i++) {
		EEPROM.update(i, 0xFF);
	}
}
ISR(TIMER2_COMPA_vect) {
	//cli(); //V401
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
	//sei(); //V401
}
void MEM_read() {
	//dubbel gedaan zie onderop in de functie
	//LOC[0].speed = B01100000; //set start direction forward of locomotive //V401
	//LOC[1].speed = B01100000;    //V401

	byte a;

	for (byte i = 0; i < 2; i++) { //loc data ophalen
		LOC[i].adres = EEPROM.read(100 + i);
		if (LOC[i].adres == 0xFF) { //3feb23 default locadres bij 255 niet opslaan, in WMapp ook 3 en 4
			LOC[i].adres = 0x03 + i;
		}
		LOC[i].station = EEPROM.read(260 + i);
		if (LOC[i].station == 0xFF)LOC[i].station = 0x00;
		if (LOC[i].station > 0) res_station |= (1 << LOC[i].station - 1); //reserveer begin stations
		//merk op, update van eeprom is eigenlijk nergens voor nodig.
		LOC[i].function = EEPROM.read(195 + i);
		if (LOC[i].function == 0xFF)LOC[i].function = B10010000;
		LOC[i].Vmin = EEPROM.read(104 + i);
		if (LOC[i].Vmin > 10)LOC[i].Vmin = 1;
		LOC[i].Vmax = EEPROM.read(106 + i);
		if (LOC[i].Vmax > 28)LOC[i].Vmax = 28;

		LOC[i].drf = EEPROM.read(400 + i);
		if (LOC[i].drf > 24)LOC[i].drf = 1;

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
		dcc_seinen = 248;
		//EEPROM.update(103, dcc_seinen);
	}
	MEM_reg = EEPROM.read(250);
	prg_seinoffset = EEPROM.read(170);
	if (prg_seinoffset > 3)prg_seinoffset = 0;
	stoptijd = EEPROM.read(410); //stoptijd
	if (stoptijd > 250)stoptijd = 10;


	//set start direction forward of locomotive
	//251 reg loc0 252 reg loc1 
	//overige bits in reg niet lezen dan gaat het mis
	LOC[0].speed = B01100000;
	LOC[1].speed = B01100000;

	LOC[0].reg |= (1 << 7);
	LOC[1].reg |= (1 << 7);
	if (~EEPROM.read(251) & (1 << 7)) {
		LOC[0].speed &= ~(1 << 5);
		LOC[0].reg &= ~(1 << 7);
	}
	if (~EEPROM.read(252) & (1 << 7)) {
		LOC[1].speed &= ~(1 << 5);
		LOC[1].reg &= ~(1 << 7); //hier, na laden, zijn ze dus gelijk de speed en de reg 
	}
	MEM_readroute();
}
void MEM_readroute() { //zie beschrijving
	for (byte i = 0; i < 12; i++) {
		ROUTE[i].stationl = EEPROM.read(108 + i); //120
		ROUTE[i].stationr = EEPROM.read(120 + i); //132
		ROUTE[i].wissels = EEPROM.read(132 + i); // 144 let op default is 0xFF, dus false is bezet.....
		ROUTE[i].blokkades = EEPROM.read(144 + i); //156 blokkade instelling terugladen hoogste nu 140 (132+i(max 7)
		ROUTE[i].melders = EEPROM.read(156 + i); //hoogste nu 156+12=168

		for (byte b = 0; b < 4; b++) {
			ROUTE[i].seinen[b] = EEPROM.read(300 + i + (b * 12));
		}
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
	number = 195;
	EEPROM.update(number, LOC[loc].function);
	EEPROM.update(251 + loc, LOC[loc].reg);
}
void MEM_update() { //sets new values/ sends CV 
	switch (PRG_fase) {

	case 0: //DCC adressen

		// Voor aanpassing tbv. WMapp alle adressen updaten
		//switch (PRG_typeDCC) {
		//case 0: //loc1
		EEPROM.update(100, LOC[0].adres);
		//	break;
		//case 1: //loc2
		EEPROM.update(101, LOC[1].adres);
		//	break;
		//case 2://wissels
		EEPROM.update(102, dcc_wissels);
		//	break;
		//case 3: //seinen
		EEPROM.update(103, dcc_seinen);
		//	break;
		//}
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

			MEM_updateRoute(i); //V401
			//EEPROM.update(108 + i, ROUTE[i].stationl); //120
			//EEPROM.update(120 + i, ROUTE[i].stationr); //132
			//EEPROM.update(132 + i, ROUTE[i].wissels); //144
			//EEPROM.update(144 + i, ROUTE[i].blokkades); //156
			//EEPROM.update(156 + i, ROUTE[i].melders);//168
			//for (byte b = 0; b < 4; b++) {
			//	EEPROM.update(300 + i + (b * 12), ROUTE[i].seinen[b]);
			//}
		}
		break;

	case 5: // program fase 5 diverse V4.01
		//
		EEPROM.update(170, prg_seinoffset);
		EEPROM.update(400, LOC[0].drf);
		EEPROM.update(401, LOC[1].drf);
		EEPROM.update(410, stoptijd);
		break;
	}
	EEPROM.update(250, MEM_reg);
	//DSP_prg(); //V401
}
void MEM_updateRoute(byte rte) { //V401 update route per route
	EEPROM.update(108 + rte, ROUTE[rte].stationl); //120
	EEPROM.update(120 + rte, ROUTE[rte].stationr); //132
	EEPROM.update(132 + rte, ROUTE[rte].wissels); //144
	EEPROM.update(144 + rte, ROUTE[rte].blokkades); //156
	EEPROM.update(156 + rte, ROUTE[rte].melders);//168
	for (byte b = 0; b < 4; b++) {
		EEPROM.update(300 + rte + (b * 12), ROUTE[rte].seinen[b]);
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
void Serial_read() {
	//een connected boolean is niet nodig omdat alleen bij een connection met een DCCmonitor WMapp verder gaat.
	int inData;
	while (Serial.available() > 0) {
		inData = Serial.read();

		if (commandcount > 0) {
			if (commandcount == 1)AantalBytes = inData;
			command[commandcount - 1] = inData;
			commandcount++;
			if (commandcount > AantalBytes) { //volledig command van 4 bytes ontvangen 1xstart + 4databytes
				commandcount = 0;
				Command_exe(); //voer commando uit
				//Commandclear();
			}
		}
		else if (inData == 255) { //dus niet groter dan 0 dus 0 en 0xFF
			//start byte voor command, niks mee doen dus alleen teller omhoog
			commandcount++; //volgende doorloop counter op 1			
		}
	}
}
void ToonCommand() {
	cd;
	//display.clearDisplay();
	display.setCursor(5, 5);
	display.setTextColor(1);
	//for (byte i = 0; i < 6; i++) { //command 0~6 zijn der 7
	//	display.println(command[i]);
	//}
	display.print("okidki");

	display.display();
	delay(2000);

}
void Command_exe() { //voert hetvia usb ontvangen command uit
	////////toon ontvangst op display, alleen bij debug
	byte t = 0;
	bool temp;
	//if(command[0]>1)ToonCommand();

	switch (command[1]) {

	case 1: //rq data, vraag om data

		switch (command[2]) {
		case 1: //rq data product 
			SendProductID();
			break;

		case 2: //rq datadump 1
			SendInstelling();
			//Serial.flush();
			break;
		case 3: //rijden instelling 
			SendRijden();
			break;
		case 4: //rq data routes
			if (command[3] == 0) { //rq alle routes
				for (byte i = 0; i < 12; i++) {
					SendRoutes(i);
				}
			}
			else {
				SendRoutes(command[3]-1); //single route rq
			}
			break;

		}
		break; //end for request commands

	case 2: //Datapakket ontvangen instellingen
		MEM_reg = command[2];
		prg_seinoffset = command[3];

		stoptijd = command[4];
		LOC[0].drf = command[5];
		LOC[1].drf = command[6];

		LOC[0].adres = command[7];
		LOC[1].adres = command[8];
		dcc_wissels = command[9];
		dcc_seinen = command[10];

		//richting is bit 7 van LOC[0].reg
		if (command[11] & (1 << 7)) {
			LOC[0].reg |= (1 << 7);
			LOC[0].speed |= (1 << 5); //?? die toggle is een logisch verhaal, waarschijlijk bedoeld om
		}
		else {
			LOC[0].reg &= ~(1 << 7);
			LOC[0].speed &= ~(1 << 5);
		}

		//richting is bit 7 van LOC[0].reg
		if (command[12] & (1 << 7)) {
			LOC[1].reg |= (1 << 7);
			LOC[1].speed |= (1 << 5);
		}
		else {
			LOC[1].reg &= ~(1 << 7);
			LOC[1].speed &= ~(1 << 5);
		}

		LOC[0].Vmin = command[13];
		LOC[1].Vmin = command[14];
		LOC[0].Vmax = command[15];
		LOC[1].Vmax = command[16];


		//PRG_fase = 5; //set prg fase

		MEM_update();
		MEM_loc_update(0);
		MEM_loc_update(1);



		if (GPIOR0 & (1 << 5)) { //program mode
			DSP_prg();
		}
		else { //pendel in bedrijf
			DSP_pendel();
		}
		//PRG_fase = t; //herstel prg fase		
		//if (PRG_fase == 5)DSP_prg(); //display vernieuwen	//

		break;

	case 3: //instelling voor een route ontvangen

		ROUTE[command[2]].stationl = command[3];
		ROUTE[command[2]].stationr = command[4];
		ROUTE[command[2]].wissels = command[5];


		////kleiner maken? met for loop  maakt helemaal niks nadda uit. 
		//ROUTE[command[2]].seinen[0] = command[6];
		//ROUTE[command[2]].seinen[1] = command[7];
		//ROUTE[command[2]].seinen[2] = command[8];
		//ROUTE[command[2]].seinen[3] = command[9];
		
		for (byte i = 0; i < 4; i++) {
			ROUTE[command[2]].seinen[i] = command[6 + i];
		}

		ROUTE[command[2]].melders = command[10];
		ROUTE[command[2]].blokkades = command[11];
		ROUTE[command[2]].Vloc[0] = command[12];
		ROUTE[command[2]].Vloc[1] = command[13];


		MEM_updateRoute(command[2]);
		EEPROM.update(200, ROUTE[command[2]].Vloc[0]); 
		EEPROM.update(201, ROUTE[command[2]].Vloc[1]);

		//Vloc is een snelheids indicatie van de lengte van een route per loc, misschien iets leuks mee te doen?
		SendReady();
		break;

	case 50: //opdracht		

		switch (command[2]) { //type command

		case 1: //schrijf DCC adres in loc
			PRG_level = 2;
			PRG_fase = 2;
			switch (command[3]) {
			case 1: //loc 1
				//GPIOR0 |= (1 << 7);
				prg_typecv = 0;  ///de locs???
				break;
			case 2: //loc 2
				//GPIOR0 &= ~(1 << 7);
				prg_typecv = 1;
				break;
			}
			DSP_prg();
			MEM_update(); //hier wordt ook het adres schrijven geregeld
			break;
		case 2:
			//toggle pin 8 DCC uitsturing aan/uit
			PINB |= (1 << 0);
			SendDCCstatus();
			break;

		case 10: //Druk op knop 3, verzet dus station of start de S&O functie

			if (command[3] == 1) {
				GPIOR0 &= ~(1 << 7); //loc1 instellen
			}
			else {
				GPIOR0 |= (1 << 7); //loc 2
			}
			PDL_fase = 0; //forceer naar de hoofd/start display in pendelen
			SW_pendel(2); //druk op de knop
			break;

		case 11: //loc registers
			if (command[3] == 1) {
				LOC[0].reg = command[4];
				GPIOR0 &= ~(1 << 7); //flag voor loc 1 
				startlocs(0);
			}
			else {
				LOC[1].reg = command[4];
				GPIOR0 &= ~(1 << 7); //flag voor loc 2
				startlocs(1);
			}
			//LOC[command[3]-1].reg = command[4]; //command[3]=de loc command[4]= de loc.reg van de loc
			//startlocs(command[3] - 1);
			DSP_pendel();
			break;
		case 12: //loc functions

			if (command[3] == 1) {
				LOC[0].function = command[4];
				MEM_loc_update(0);
			}
			else {
				LOC[1].function = command[4];
				MEM_loc_update(1);
			}


			DSP_pendel();
			break;
		case 13: //loc speed
			if (command[3] == 1) {
				LOC[0].speed = command[4];
			}
			else {
				LOC[1].speed = command[4];
			}
			DSP_pendel();

			break;

		case 14: //toggle stand van een sein
			//byte sein = command[3] - 1;
			command[3]--;
			temp = true;
			//GPIOR1 |= (1 << 7); //gebruik bit 7 als temp boolean
			if (command[3] < 8) {
				pos_seinen[0] ^= (1 << command[3]);
				if (pos_seinen[0] & (1 << command[3])) temp = false;//GPIOR1 &= ~(1 << 7); //reset temp boolean
			}
			else {
				pos_seinen[1] ^= (1 << command[3] - 8);
				if (pos_seinen[1] & (1 << command[3] - 8)) temp = false; //GPIOR1 &= ~(1 << 7); //reset temp boolean
			}
			SET_sein(command[3], temp);
			break;

		case 15: //set wissel command[3]=wissel 0~3  command[4]=stand 0~1
			temp = false;
			if (command[4] == 1)temp = true;
			DCC_acc(false, true, command[3], temp);

			break;


		}
	}
}
void SendProductID() {
	//byte dataout[3];
	command[0] = 255; //start com
	command[1] = 3; //aantal bytes
	command[2] = 101; //data is product id
	command[3] = 20; //productId van PendelDCC
	SendCommand(4);

	SendDCCstatus();
}
void SendReady() {
	//byte dataout[3];
	command[0] = 255; //start com
	command[1] = 2; //aantal bytes
	command[2] = 10; //data is product id
	SendCommand(3);
}
void SendInstelling() {
	command[0] = 255; //start com
	command[1] = 17; //aantal bytes
	command[2] = 102;
	command[3] = MEM_reg;
	command[4] = prg_seinoffset;
	command[5] = stoptijd;
	command[6] = LOC[0].drf;
	command[7] = LOC[1].drf;
	command[8] = LOC[0].adres;
	command[9] = LOC[1].adres;
	command[10] = dcc_wissels;
	command[11] = dcc_seinen;
	command[12] = LOC[0].reg;
	command[13] = LOC[1].reg;
	command[14] = LOC[0].Vmin;
	command[15] = LOC[1].Vmin;
	command[16] = LOC[0].Vmax;
	command[17] = LOC[1].Vmax;
	SendCommand(18);
}
void SendMelders() {
	if (MELDERS() != old_melders) {
		command[0] = 255;
		command[1] = 3; //aantal bytes
		command[2] = 110;
		command[3] = MELDERS();//pos_melders[2];
		SendCommand(4);
	}
	old_melders = MELDERS();
}
void SendRijden() {
	command[0] = 255; //start com
	command[1] = 15; //aantal bytes AANPASSEN
	command[2] = 103;
	command[3] = LOC[0].station;
	command[4] = LOC[0].goal;
	command[5] = LOC[0].route;
	command[6] = LOC[0].reg;
	command[7] = LOC[0].speed;

	command[8] = LOC[1].station;
	command[9] = LOC[1].goal;
	command[10] = LOC[1].route;
	command[11] = LOC[1].reg;
	command[12] = LOC[1].speed;

	command[13] = LOC[0].function;
	command[14] = LOC[1].function;

	command[15] = MELDERS();
	SendCommand(16);
}
void SendSein(byte sein, byte stand) {
	command[0] = 255;
	command[1] = 4; //aantal bytes
	command[2] = 111; //data van een sein
	command[3] = sein;
	command[4] = stand;
	SendCommand(5);
}
void SendCommand(byte aantalbytes) {
	Serial.write(command, aantalbytes);
}
void SendWissel(byte wissel, byte stand) {
	command[0] = 255;
	command[1] = 4;
	command[2] = 112; //zend een wissel
	command[3] = wissel;
	command[4] = stand;
	SendCommand(5);
}
void SendRoutes(byte rte) {

	//ToonCommand();
	command[0] = 255;
	command[1] = 15; //aantal bytes
	command[2] = 120;
	command[3] = rte;
	command[4] = ROUTE[rte].stationl;
	command[5] = ROUTE[rte].stationr;
	command[6] = ROUTE[rte].wissels;
	command[7] = ROUTE[rte].seinen[0];
	command[8] = ROUTE[rte].seinen[1];
	command[9] = ROUTE[rte].seinen[2];
	command[10] = ROUTE[rte].seinen[3];
	command[11] = ROUTE[rte].blokkades;
	command[12] = ROUTE[rte].melders;
	command[13] = ROUTE[rte].Vloc[0];
	command[14] = ROUTE[rte].Vloc[1];
	SendCommand(16);
}
void SendDCCstatus() {

	command[0] = 255;
	command[1] = 3;
	command[2] = 113;

	if (PINB & (1 << 0)) { //status DCC on/off
		command[3] = 1;
	}
	else {
		command[3] = 0;
	}

	SendCommand(4);
}
void DCC_write() {
	//writes loc adress in locomotive
	if (~GPIOR0 & (1 << 2)) {
		count_wa++;
		if (count_wa & (1 << 7)) { //als bit7 true dan meer dan 127
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
	GPIOR1 &= ~(1 << 1);
	DSP_prg();
}
void DCC_acc(boolean ws, boolean onoff, byte channel, boolean poort) {
	//channel als 4 dan 1e van opvolgend decoderadres als 5 2e 
	byte da;	//ws=wissel of sein
	if (ws) { //true seinen
		da = dcc_seinen; //DCC adres ophalen 
		channel = channel + prg_seinoffset; //waarde geeft aan welke van de 32 dcc adressen voor de seinen
	}
	else { //wissels
		da = dcc_wissels;
		SendWissel(channel, poort);
	}
	while (channel > 3) { //bereken ds als het decoderadres
		da++;
		channel = channel - 4; //rest is channel in berekende decoder adres
	}

	DCC_accAdres(da); //plaats adres in te verzenden bytes
	if (onoff)dcc_data[1] |= (1 << 3); //bit3 van instructiebyte
	dcc_data[1] |= (channel << 1); //bit 1 en 2 van instructiebyte
	if (poort)dcc_data[1] &= ~(1 << 0); //bit 0 van instructiebyte
	dcc_data[2] = dcc_data[0] ^ dcc_data[1]; //bereken checksum byte 3
	dcc_aantalBytes = 2; //geef aab hoeveel bytes te verzenden (+1)
	count_repeat = 3; //Geeft aan hoe vaak herhalen verzenden command 26okt
	GPIOR0 |= (1 << 2); //start zenden accessoire, flag verzenden command bezig	
}
void DCC_accAdres(byte da) {
	//zet decoderadres in te verzenden databytes
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
	if (~GPIOR0 & (1 << 5)) DSP_pendel(); //only when enabled

	SendRijden(); //V401
}
void LOC_exe() {
	//Het automatisch proces, berekend vanuit de acties van een locomotief
	byte loc = 0; byte changed; byte route = 0; byte tmp; byte goal = 0x00; boolean tb; byte a = 0; byte bc = 0;
	GPIOR1 ^= (1 << 2); //toggle active loc
	if (GPIOR1 & (1 << 2))loc = 1;
	if (LOC[loc].reg & (1 << 0)) {//dit proces loopt alleen als loc.reg bit 0 = true
		if (LOC[loc].wait < LOC[loc].count) { //wacht cyclus, meerdere plaatsen aan te roepen door wait > 0 te maken
			LOC[loc].count = 0; //reset clock
			switch (LOC[loc].fase) {
			case 0: //start cyclus, staat loc staat in beginstation
				if (LOC[loc].station == 0)LOC[loc].fase = 100; //geen beginstation gedefinieerd, systeem probeert station te vinden
				LOC[loc].fase++;
				LOC[loc].wait = 0;
				break;
			case 1: //route zoeken
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
						LOC[loc].fase = 200;//geen route mogelijk
					}
					else { //richting wisselen opnieuw testen
						LOC[loc].speed ^= (1 << 5); LOC[loc].reg ^= (1 << 1);
						LOC[loc].reg |= (1 << 2);
					}
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
				//Vrije route zoeken, wordt continue herhaald tot route is gevonden
				route = random(0, 12); //12 mogelijke routes
				if (LOC[loc].reg & (1 << 1)) { //l>R
					if (ROUTE[route].stationr == LOC[loc].station) goal = ROUTE[route].stationl;
				}
				else { //R>L
					if (ROUTE[route].stationl == LOC[loc].station) goal = ROUTE[route].stationr;
				}

				if (goal == 0) break; //uitspringen, geen route gevonden

				tmp = 0;
				//check doelstation, anders switch verlaten
				if (res_station & (1 << goal - 1)) tmp = 1;
				//check wissels, of vrij in de route
				for (byte i = 0; i < 4; i++) {
					if (~ROUTE[route].wissels & (1 << 7 - i)) { //wissel opgenomen in gewenste route
						if (res_wissels & (1 << i))tmp = 1; // wissel is bezet, verlaat proces
					}
				}
				//check blokkades en melders die niet bezet mogen zijn
				for (byte i = 0; i < 8; i++) {
					if (~ROUTE[route].blokkades & (1 << i)) {
						if (res_blok & (1 << i)) tmp = 1; //als blokkade is bezet, verlaat proces
					}
					if (~ROUTE[route].melders & (1 << i)) {
						if (res_station & (1 << i))tmp = 1; //if false dan bezet
					}
				}
				if (tmp == 1)break;

				//V401 hier data sturen
				//Route is vrij, doel station reserveren
				res_station |= (1 << goal - 1); //goal 1 = bit 0
				LOC[loc].route = route;
				LOC[loc].goal = goal;
				LOC[loc].teller = 0;
				LOC[loc].fase = 10;
				LOC[loc].wait = 2; // wachten tussen wissels en sein bediening
				//reserveer blokkades en melders(stations)
				for (byte i = 0; i < 8; i++) {
					if (~ROUTE[route].blokkades & (1 << i)) res_blok |= (1 << i);
					if (~ROUTE[route].melders & (1 << i)) res_station |= (1 << i);
					if (i < 4) {
						if (~ROUTE[LOC[loc].route].wissels & (1 << (7 - i))) {
							res_wissels |= (1 << i); //reserveer wissel
						}
					}
				}

				//SendRijden();

				break;
			case 10: //set en reserveer wissels 
				if (~ROUTE[LOC[loc].route].wissels & (1 << (7 - LOC[loc].teller))) {  //bit 7~4 
					tb = ROUTE[LOC[loc].route].wissels & (1 << (3 - LOC[loc].teller)); //next wissel
					DCC_acc(false, true, LOC[loc].teller, tb); //set wissel					
				}
				LOC[loc].teller++;
				if (LOC[loc].teller > 3) {
					LOC[loc].teller = 0;
					LOC[loc].fase = 12;
					LOC[loc].wait = 1; //20?
					GPIOR1 &= ~(1 << 7); //let op dit kan een probleem geven als deze algemene boolean ergens anders wordt gebruikt
				}
				break;
			case 12: //set seinen
				a = 0; bc = LOC[loc].teller;
				if (LOC[loc].reg & (1 << 1))a = 2; //richting > byet 0-1 < byte 2-3
				if (LOC[loc].teller > 7) {
					a++;
					bc = bc - 8;
				}
				if (~ROUTE[LOC[loc].route].seinen[a] & (1 << bc)) {
					SET_sein(LOC[loc].teller, false);
					//Serial.print("sein: "); Serial.println(LOC[loc].teller);
				}
				LOC[loc].teller++;
				if (LOC[loc].teller > 15) { //alle 16 seinen bekeken voor deze route
					LOC[loc].teller = 0;
					LOC[loc].fase = 20;
					LOC[loc].wait = 5;
				}
				break;
			case 20: //drive init
				//Serial.println(pos_melders[0],BIN);

				//als in accessoire ready mode bit3 van pos_melder[0] hoog
				if (~MEM_reg & (1 << 3) && pos_melders[0] & (1 << 3)) { //8 als accessoire ready
					//Serial.println("wacht");
					LOC[loc].wait = 5;
				}
				else {

					LOC[loc].wait = 0;
					LOC[loc].velo = LOC[loc].Vmin; LOC_calc(loc);
					LOC[loc].vaart = LOC[loc].velo;
					LOC[loc].fase = 25;
					LOC[loc].reg &= ~(1 << 3); //versnellen
					LOC[loc].teller = 0;
					LOC[loc].acc = 4;
					LOC[loc].slowcount = 0;
				}
				break;
			case 25:
				//test of doelstation is bereikt,  is weer vrij
				tmp = MELDERS();
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
						a = 200 + LOC[loc].route;
						if (loc == 1)a = a + 12;
						EEPROM.update(a, ROUTE[LOC[loc].route].Vloc[loc]);
					}

					GPIOR1 &= ~(1 << 7); //release temp boolean
					//restsnelheid bepalen, doorrijden	

					//V2.02 22sept2022 sterk vereenvoudigd.
					//if (LOC[loc].velo > 1) {
					//	LOC[loc].velo = 1;
					//}


					LOC[loc].velo = LOC[loc].Vmin;
					LOC[loc].wait = 5 + LOC[loc].drf;
					LOC_calc(loc);

					//drf handmatig aanpassen aan Vmin van de loc

					/*
					switch (LOC[loc].velo) {
					case 1:
						LOC[loc].wait = 5 + LOC[loc].drf; //tijd van doorrijden??? was 5 //V2.02 + ipv *
						//in v2.02 21-9 optellen ipv vermenigvuldigen
						break;
					case 2:
						LOC[loc].velo = 1; LOC_calc(loc);
						LOC[loc].wait = 3 + LOC[loc].drf;
						break;
					case 3:
						LOC[loc].velo = 1; LOC_calc(loc);
						LOC[loc].wait = 2 + LOC[loc].drf;
						break;

					default:
						tmp = 0;
						//versie voor V2.02
						//LOC[loc].wait = 0;
						//LOC[loc].velo = 0; LOC_calc(loc);
						//
						// V2.02 22/9 niet direct stoppen
						LOC[loc].velo = 1; LOC_calc(loc);
						LOC[loc].wait = 3 + LOC[loc].drf;
						break;
					}
					*/

					res_station &= ~(1 << LOC[loc].station - 1);//oude beginstation vrij geven		
					for (byte i = 0; i < 4; i++) {//wissels vrijgeven
						if (~ROUTE[LOC[loc].route].wissels & (1 << 7 - i))res_wissels &= ~(1 << i);
					}
					for (byte i = 0; i < 8; i++) {//blokkades en gemelde stations vrijgeven
						if (~ROUTE[LOC[loc].route].blokkades & (1 << i)) res_blok &= ~(1 << i);
						if (~ROUTE[LOC[loc].route].melders & (1 << i))res_station &= ~(1 << i);
					}
					//seinen vrijgeven, rood maken
					seinoff_dir[loc] = (LOC[loc].reg & (1 << 1)); //richting loc					 
					seinoff_route[loc] = LOC[loc].route;
					seinoff_count[loc] = 0;
					//per loc
					GPIOR2 |= (1 << loc + 1); //start void SET_seinoff

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
				//hier route rijden, testen of auto stop actief is
				if (GPIOR2 & (1 << 5)) {
					LOC[loc].reg &= ~(1 << 0); //stop automatisch rijden
					EEPROM.update(260 + loc, LOC[loc].station);
					if (~LOC[0].reg & (1 << 0) & ~LOC[1].reg & (1 << 0)) {
						MEM_reg &= ~(1 << 2); //enable autostart
						//EEPROM.update(250, MEM_reg); V401
						PORTB &= ~(1 << 0); //stop DCC signal
						SendDCCstatus(); //V401
					}
					DSP_pendel(); //SendRijden(); //V401
				}
				LOC[loc].wait = random(1, 2 + (10 * stoptijd)); //station wachttijd 120 DEBUG LAGE WAARDE
				break;
			case 101: //begin find starting point (station)	
				pos_melders[2] = MELDERS();//leg huidige melders status vast
				pos_melders[3] = pos_melders[2];
				LOC[loc].velo = 3; LOC_calc(loc);
				LOC[loc].fase = 102;
				break;
			case 102:
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
				LOC[loc].reg &= ~(1 << 0); //stop
				DSP_pendel();// SendRijden();//V401
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
void SET_sein(byte sein, boolean stand) {
	SendSein(sein, stand); //stuur naar WMapp

	if (MEM_reg & (1 << 0)) { //dual stand
		GPIOR2 |= (1 << 3);
		sein2 = (sein * 2) + 1;
		possein2 = !stand;
		DCC_acc(true, true, sein * 2, stand);
		GPIOR1 |= (1 << 5);
	}
	else { //mono stand
		DCC_acc(true, true, sein, stand);
	}
}
void SET_sein2() { //sets 2e sein in dual mode
	DCC_acc(true, true, sein2, possein2);
	GPIOR1 &= ~(1 << 5);
	GPIOR2 &= ~(1 << 3);
}
void SET_seinoff(byte loc) {
	if (~GPIOR0 & (1 << 2)) {//accessoire programming not free
		byte b = 0; byte d = 0;
		if (seinoff_dir[loc])d = 2;
		if (seinoff_count[loc] > 7)b = 1;
		//schakelt alle seinen in een route voor een specifieke richting weer op rood
		if (~ROUTE[seinoff_route[loc]].seinen[b + d] & (1 << seinoff_count[loc] - (b * 8))) {
			SET_sein(seinoff_count[loc], true);
		}
		seinoff_count[loc]++;
		if (seinoff_count[loc] > 15) {
			GPIOR2 &= ~(1 << 1 + loc); //stop proces for this loc
		}
	}
}
void INIT_wissels() {
	//initialiseert de aangesloten wissels en seinen, false is wissels
	if (GPIOR0 & (1 << 2)) return;//als verwerken vorige DCC commando klaar is

	if (GPIOR1 & (1 << 7)) { //use algemene boolean voor wissels/seinen
		//seinen
		SET_sein(prg_sein, true);
		prg_sein++;
		if (prg_sein > 15) {
			prg_sein = 0;
			autostart(); //starten in bedrijf
			GPIOR1 &= ~(1 << 7); //Boolean vrijmaken voor gebruik ergens anders
		}
	}
	else {
		//wissels telkens 1 wissel
		DCC_acc(false, true, prg_wissels, GPIOR1 & (1 << 6));
		//DCC_acc(false, true, prg_wissels, false); //V2.01 wissels 1 kant uit 26okt
		GPIOR1 ^= (1 << 6);
		if (~GPIOR1 & (1 << 6))prg_wissels++;
		if (prg_wissels > 5) { //decoder adres 1 1~4  en adres 2 1~2
			prg_wissels = 0;
			GPIOR1 |= (1 << 7); //exit wissel init, naar seinen				
		}
	}
}
void autostart() {
	//Als goed is afgesloten kan automatisch proces worden gestart met alleen starten van de locs.
	if (MEM_reg & (1 << 2)) { //autostop niet gelukt, wis stations
		LOC[0].station = 0;
		LOC[1].station = 0;
		res_station = 0;
	}
	else { //autostop gelukt
		if (~MEM_reg & (1 << 1)) { //autostart staat aan
			if (LOC[0].station > 0)LOC[0].reg |= (1 << 0);
			if (LOC[1].station > 0)LOC[1].reg |= (1 << 0);
		}
	}
	MEM_reg |= (1 << 2);
	EEPROM.update(250, MEM_reg); //V401

	GPIOR1 |= (1 << 4); //exit init_wissels bug28nov
	DSP_pendel(); //1e dsp_pendel na opstarten
	GPIOR0 &= ~(1 << 5); //Enable DSP update
}
byte MELDERS() {
	byte melder;
	melder = pos_melders[0] << 4; //melder 1 en 0 omgekeerd
	melder = melder + pos_melders[1];
	return melder;
}
void DCC_cv(boolean type, byte adres, byte cv, byte value) {
	//CV programming om instellingen van locomotieven of accessoire decoders te kunnen wijzigen
	//type loco= true, acc is false
	//1110CCVV 0 VVVVVVVV 0 DDDDDDDD
	//old adres alleen bij first zetten van 
	cv--;
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
	else { //accessoire, wissels of seinen
		DCC_accAdres(adres); //fills byte 0 and byte 1
		dcc_data[1] &= ~(15 << 0); //clear bits 0~3
		dcc_data[2] = B11101100;
		dcc_data[3] = cv;
		dcc_data[4] = value;
		dcc_data[5] = dcc_data[0] ^ dcc_data[1] ^ dcc_data[2] ^ dcc_data[3] ^ dcc_data[4];
		dcc_aantalBytes = 5;
		count_repeat = 4;
	}
}
void PRG_dec() { //routine in het schakelproces.
	switch (PRG_fase) {
	case 0: //Instellen DCC adres
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
				if (dcc_seinen == 0)dcc_seinen = 248;
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
	case 5: //diverse instellingen tbv MEM_reg
		prg_diverse++;
		if (prg_diverse > 8)prg_diverse = 0;
		break;
	}
}
void PRG_inc() { //routine in schakelproces
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
				if (LOC[0].adres > 127)LOC[0].adres = 1;
				break;
			case 1:
				LOC[1].adres++;
				if (LOC[1].adres > 127)LOC[1].adres = 1;
				break;
			case 2:
				dcc_wissels++;
				if (dcc_wissels == 0)dcc_wissels = 1;
				break;
			case 3:
				dcc_seinen++;
				if (dcc_seinen > 248)dcc_seinen = 1;
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
	case 5: //diverse instellingen aanpassen o.a.in MEM_reg

		switch (prg_diverse) {

		case 2:
			prg_seinoffset++;
			if (prg_seinoffset > 3)prg_seinoffset = 0;
			break;
		case 4://drf loc1
			LOC[0].drf++;
			if (LOC[0].drf > 24)LOC[0].drf = 1;
			break;
		case 5: //drf loc2
			LOC[1].drf++;
			if (LOC[1].drf > 24)LOC[1].drf = 1;
			break;
		case 6: //Slot aan of uit, zend twee dcc commands
			MEM_reg ^= (1 << 4);
			break;
		case 7: //stoptijd gaat met 10 vermedigvuldig
			stoptijd++; if (stoptijd > 20)stoptijd = 0;
			break;
		case 8: //Initialisatie aan of uit 
			MEM_reg ^= (1 << 5);
			break;

		default: //MEM_reg booleans
			MEM_reg ^= (1 << prg_diverse);
			break;
		}
		break;
	}
}
void DCC_command() {
	byte loc = 0;
	if (GPIOR0 & (1 << 2)) { //Send CV or basic accessoire
		count_repeat--;
		if (count_repeat > 4) {
			GPIOR0 &= ~(1 << 2); //end CV, accessoire transmit
		}
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
}
void Meldertimer() { //zet melders na periode uit
	SendMelders(); //stuurt melder instelling data over de serial poort naar WMapp
	for (byte a = 0; a < 2; a++) {
		for (byte i = 0; i < 4; i++) {
			if (~pos_melders[a] & (1 << i)) { //Melder = actief 
				if (~old_posmelders[a] & (1 << i)) { //melder was bij vorige doorgang ook actief
					//Melder resetten

					old_posmelders[a] |= (1 << i); //reset 
					pos_melders[a] |= (1 << i); //reset melder


				}
				else { //melder was niet actief bij vorige doorgang, nieuwe melding
					old_posmelders[a] &= ~(1 << i); //set te onthouden melder
				}
			}
		}
	}
}
void SW_exe() { //1

	Mcount++;
	if (Mcount > 100) { //periode dat de melder ingeklokt blijft
		Mcount = 0; //reset de teller
		Meldertimer(); //bepaald of een melder weer moet worden gereset
	}

	byte poort; byte changed; //byte temp;
	switch (SW_volgorde) { //teller per doorloop wordt 1 van de drie switch typen gelezen
	case 0:
		//DCC enabled switch lezen
		if ((PINB & (1 << 1)) != (GPIOR1 & (1 << 3))) { //drukknop niet gelijk aan flag 
			if (~PINB & (1 << 1)) {
				PINB |= (1 << 0); //toggle DCC enabled als drukknop is gedrukt

				SendDCCstatus();

			}

			//flag aanpassen, geeft de oude stand van de drukknop
			if (PINB & (1 << 1)) {
				GPIOR1 |= (1 << 3);
			}
			else {
				GPIOR1 &= ~(1 << 3);
			}
		}
		break;

	case 1:
		//Lezen van de melders in twee sessies PORTD3 (PIN3) wisseld lezen optoos TLP281 afwisselend 			
		poort = PIND;
		poort = poort >> 4; //Bit 7~4 wordt bit 3~0 
		if (PIND & (1 << 3)) { //welke TLP281 is actief
			for (byte i = 0; i < 4; i++) {
				if (~poort & (1 << i)) pos_melders[0] &= ~(1 << i); //V3.01 alleen AANZETTEN van een melder
				//uitzetten gebeurt in Meldertimer()
			}
		}
		else {
			for (byte i = 0; i < 4; i++) {
				if (~poort & (1 << i)) pos_melders[1] &= ~(1 << i); //V3.01 alleen AANZETTEN van een melder
			}
		}
		if (PRG_fase == 1 && PRG_level == 3)DSP_prg(); //alleen i testmode
		PIND |= (1 << 3); //toggle pin 3, naar andere TLP281
		break;

	case 2:
		poort = PINC;
		changed = poort ^ sw_statusC;
		for (byte i = 0; i < 4; i++) {
			if (changed & (1 << i) & ~poort & (1 << i)) {
				SW_on(i);
			}
		}
		sw_statusC = poort;

		break;
	}
	SW_volgorde++;
	if (SW_volgorde > 2)SW_volgorde = 0;
} //1
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
	byte b = 0; //bit in sein selectie
	switch (PRG_level) {
		//++++++++LEVEL 1
	case 1:	//Soort instelling, programmering

		switch (sw) {
		case 0:
			PRG_fase--;
			if (PRG_fase > 5)PRG_fase = 5;
			break;
		case 1:
			PRG_fase++;
			if (PRG_fase > 5)PRG_fase = 0;
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
			case 5: //Diverse opslaan
				MEM_update();
				//EEPROM.update(250, MEM_reg);
				//EEPROM.update(170, prg_seinoffset);
				//EEPROM.update(400, LOC[0].drf);
				//EEPROM.update(401, LOC[1].drf);
				//EEPROM.update(410, stoptijd);
				PRG_level--;
				break;
			}
			break;
		case 3:
			switch (PRG_fase) {
			case 4:
				PRG_level++;
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
		switch (sw) {
		case 0: //knop1
			PRG_dec();
			break;
		case 1: //knop 2
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
					SET_sein(prg_sein, GPIOR1 & (1 << 6));
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
				prg_sein++;
				if (prg_sein > 15)prg_sein = 0;
				break;
			case 1:
				GPIOR2 ^= (1 << 0); //toggle sein/route richting
				break;
			case 2:
				GPIOR2 &= ~(1 << 4);
				if (~GPIOR2 & (1 << 0))b = 2;
				if (prg_sein < 8) {
					ROUTE[rt_sel].seinen[0 + b] ^= (1 << prg_sein);
					if (ROUTE[rt_sel].seinen[0 + b] & (1 << prg_sein))GPIOR2 |= (1 << 4);
				}
				else {
					ROUTE[rt_sel].seinen[1 + b] ^= (1 << prg_sein - 8);
					if (ROUTE[rt_sel].seinen[1 + b] & (1 << prg_sein - 8))GPIOR2 |= (1 << 4);
				}
				SET_sein(prg_sein, GPIOR2 & (1 << 4));
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
}
void startlocs(byte loc) { //vanaf V401 even gehaald uit SW_pendel 
	//ToonCommand();
	if (LOC[loc].reg & (1 << 0)) {  //start rijden process
		LOC[loc].fase = 0;
		LOC[loc].wait = 0;
		GPIOR2 &= ~(1 << 5); //disable auto stop S&O
		MEM_reg |= (1 << 2); //disable auto start
		EEPROM.update(250, MEM_reg);
	}
	else { //stop locomotief
		LOC[loc].velo = 0; LOC_calc(loc);
	}
}
void VerhoogStation(byte loc) {

	//verhoogd station en beindigd alle claims
	if (LOC[loc].reg & (1 << 0)) { //loc rijdt
		GPIOR2 |= (1 << 5); //auto stop actief S&O
	}
	else {//loc staat stil
		//station kiezen
		//stations vrij geven
		if (LOC[loc].station > 0)res_station &= ~(1 << LOC[loc].station - 1);
		if (LOC[loc].goal > 0)res_station &= ~(1 << LOC[loc].goal - 1);
		LOC[loc].goal = 0; //wis doel en route
		//wis geblokkeerde wissels?
		//wis geblokkeerde blokkades?


		LOC[loc].station++; //hier  wordt het station verhoogd

		if (res_station & (1 << LOC[loc].station - 1)) LOC[loc].station++; //gereserveerde stations overslaan

		if (LOC[loc].station > 8) {
			LOC[loc].station = 0;
			//if (res_station & (1 << 0))LOC[loc].station = 2;
		}
		else {
			res_station |= (1 << LOC[loc].station - 1); //reserveer station
		}
	}

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
			LOC[loc].reg ^= (1 << 0);
			startlocs(loc); //V401

			break;
		case 2:
			if (LOC[loc].reg & (1 << 0)) { //loc rijdt
				GPIOR2 |= (1 << 5); //auto stop actief S&O
			}
			else {//loc staat stil
				VerhoogStation(loc);
				// 
				// 
				////station kiezen
				////stations vrij geven
				//if (LOC[loc].station > 0)res_station &= ~(1 << LOC[loc].station - 1);
				//if (LOC[loc].goal > 0)res_station &= ~(1 << LOC[loc].goal - 1);
				//LOC[loc].goal = 0; //wis doel en route
				////wis geblokkeerde wissels?
				////wis geblokkeerde blokkades?


				//LOC[loc].station++;
				//if (res_station & (1 << LOC[loc].station - 1)) LOC[loc].station++; //gereserveerde stations overslaan
				//if (LOC[loc].station > 8) {
				//	LOC[loc].station = 0;
				//	//if (res_station & (1 << 0))LOC[loc].station = 2;
				//}
				//else {
				//	res_station |= (1 << LOC[loc].station - 1); //reserveer station
				//}
			}
			break;
		case 3:
			PDL_fase++; //level hoger
			break;
		}
		break;
	case 1: //PDL_fase 1
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
			LOC[loc].reg ^= (1 << 7);
			break;
		case 1:
			LOC[loc].Vmin++;
			if (LOC[loc].Vmin > 5)LOC[loc].Vmin = 1;
			break;
		case 2:
			LOC[loc].Vmax--;
			if (LOC[loc].Vmax < 5)LOC[loc].Vmax = 28;
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
//void DSP_start() {
//	display.clearDisplay();
//	display.setTextSize(1);
//	display.setTextColor(WHITE);
//	display.setCursor(10, 0);
//	display.print(wm); //display.print(F("www.wisselmotor.nl"));
//	display.drawLine(1, 10, 127, 10, WHITE);
//	display.setTextSize(2);
//	display.setCursor(3, 24);
//	display.print("PenDel DCC");
//	DSP_buttons(0);
//	//display.display();
//}
void DSP_pendel() {
	byte loc = 0; byte button; byte txt = 2;
	if (GPIOR0 & (1 << 7)) { loc = 1; txt = 48; } //V401

	cd;
	switch (PDL_fase) {
	case 0:
		regel1; TXT(txt); //"Loc "
		//TXT(101 + loc);
		if (LOC[loc].speed & (1 << 5)) {  //richting pijtje
			TXT(14);
		}
		else {
			TXT(13);
		}
		display.print(LOC[loc].velo);
		regel2; display.print(LOC[loc].station); TXT(32); display.print(LOC[loc].goal); //32 =streepje -

		if (GPIOR2 & (1 << 5)) { //s&O ingeschakeld
			DSP_settxt(100, 40, 1); display.print("Stop");
		}
		if (LOC[loc].reg & (1 << 0)) {
			button = 4;
		}
		else {
			button = 1;
		}
		break;

	case 1:
		regel1s, //TXT(2); //TXT(101 + loc);  
			TXT(4);
		for (byte i = 0; i < 3; i++) {
			button = i - 1;
			if (button > 10)button = 4;
			if (LOC[loc].function & (1 << button)) { //funcion on
				display.fillCircle(9 + (i * 34), 29, 9, WHITE);
				//display.fillRect(9 + (i * 34), 29, 9, 29,1);
			}
			else { //function off
				display.drawCircle(9 + (i * 34), 29, 9, WHITE);
			}
		}
		button = 2;
		break;
	case 2:
		regel1s,// TXT(2); //TXT(101 + loc); V401
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
	SendRijden(); //V401
}
void DSP_prg() {
	int adrs;
	byte buttons;
	byte position = 5;
	byte temp;
	byte b; byte s;
	switch (PRG_level) {
		//**********************
	case 1: //soort instelling
		cd; regel1;
		switch (PRG_fase) {

		case 0: //Instellen DCC adres
			TXT(7); //TXT(0); //Instellen V401
			regel2; TXT(1); //DCC adres	
			break;
		case 1: //Testen
			TXT(11);
			break;
		case 2: //write loc adresses
			TXT(10);// TXT(0);
			regel2; TXT(1); //Write DCC adres
			break;
		case 3: //Cv programming
			TXT(10);// TXT(0);
			regel2; TXT(5);
			break;
		case 4: //instellen routes
			TXT(7); //TXT(0); 
			regel2; TXT(16);
			break;
		case 5: //Diverse algemene instellingen
			TXT(7); //TXT(0); 
			regel2; TXT(33);
			break;
		}
		buttons = 10;
		break;
		//**********************************level 2
	case 2: // program level 2
		buttons = 10;
		cd;
		regel1s; //V401
		switch (PRG_fase) {
		case 0: //keuze loc of accessoire
			regel1s;
			TXT(7); TXT(1);// TXT(0); 
			regel2;
			switch (PRG_typeDCC) { //DCC van welke loc of accessoire keuze
			case 0:
				TXT(2);// TXT(101); //V401
				break;
			case 1:
				TXT(48);// TXT(102);
				break;
			case 2:
				TXT(8);
				break;
			case 3:
				TXT(9);
				break;
			}
			break;
		case 1: //Testen
			TXT(11); regel2;
			switch (PRG_typeTest) {
			case 0:
				TXT(2);// TXT(101);
				break;
			case 1:
				TXT(48);// TXT(102); //V401
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
		case 2: //Write adres in loc and accessoires gebleven
			TXT(10); TXT(1); regel2;// TXT(2);
			switch (prg_typecv) {
			case 0:
				TXT(2); //V401
				break;
			case 1:
				TXT(48);
				break;
			}
			break;
		case 3: //CV programming gebleven
			regel1s;
			TXT(10); TXT(5); regel2;
			switch (prg_typecv) {
			case 0: //loc1
				TXT(2); //TXT(101);
				break;
			case 1: //loc2
				TXT(48); //TXT(102); //V401
				break;
			case 2://Decoders
				TXT(6);
				break;
			}
			break;
		case 4://routes
			//TXT(16); //V401
			regel2;
			//rt_sel =0~7  toont 1~8
			display.print(rt_sel + 1); //TXT(200); 
			TXT(30); display.print(ROUTE[rt_sel].stationl);
			TXT(32); display.print(ROUTE[rt_sel].stationr); TXT(31);
			buttons = 14;
			break;
		case 5: //Instelling diverse
			//TXT(33);
			switch (prg_diverse) {
			case 0: //sein mono dual
				//TXT(33); 
				TXT(9); regel2;
				if (MEM_reg & (1 << 0)) {
					TXT(35);
				}
				else {
					TXT(34);
				}
				break;
			case 1: //autostart
				//TXT(33); 
				TXT(40); regel2;
				if (MEM_reg & (1 << 1)) {
					TXT(42);
				}
				else {
					TXT(41);
				}
				break;
			case 2: //Offset sein 1
				//TXT(33); 
				TXT(1); display.print("S1"); regel2;
				display.print(4 * (dcc_seinen - 1) + 1 + prg_seinoffset);
				break;
			case 3: //mode melder 8
				display.print(F("M8")); regel2;
				if (MEM_reg & (1 << 3)) { //default M8 as melder 8
					TXT(43);
				}
				else { //M8 as setting accessoires are ready (M8 true)
					TXT(44);
				}
				break;
			case 4: //drf loc1 drf=door rijden factor
				TXT(45);
				TXT(2); //TXT(101); 
				regel2;
				display.print(LOC[0].drf);
				break;
			case 5: //drf loc 2
				TXT(45);
				TXT(48); //TXT(102); 
				regel2;
				display.print(LOC[1].drf);
				break;
			case 6: //lock default off, aansturen DCC adres wissels=1 voor bv. servo
				TXT(46); regel2;
				if (MEM_reg & (1 << 4)) {
					TXT(42); //uit
				}
				else {
					TXT(41); //aan
				}
				break;
			case 7: //max wachttijd
				TXT(47);
				regel2;
				display.print(stoptijd);
				break;
			case 8:
				TXT(37); regel2;
				if (MEM_reg & (1 << 5)) {
					TXT(42); //uit
				}
				else {
					TXT(41); //aan
				}
				break;
			}
			buttons = 6;
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
				TXT(2);// TXT(101);
				regel2; display.print(LOC[0].adres);
				break;
			case 1: //loc2
				TXT(48);// TXT(102);
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
		case 1: //Test V401
			cd; regel1s; TXT(11);
			switch (PRG_typeTest) {
			case 0:  //snelheid en richting loco1
				TXT(2); //TXT(101);
				//display.print(F("Loc 1"));
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
				TXT(48);// TXT(102);
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
				//display.drawRect(1 + (32 * (prg_wissels)), 18, 30, 23, 1); //x,y, width, height
				//display.drawRect(32 * (prg_wissels), 18, 30, 23, 1); //x,y, width, height
				display.drawCircle((32 * (prg_wissels)) + 15, 30, 8, 1); //V401


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
			case 3: //Test seinen (lvl3) //V401 display versimpeld
				TXT(9); //Testen Seinen	
				regel2; TXT(15); display.print(1 + prg_sein);

				if (prg_sein < 8) { //bit voor stand sein
					if (pos_seinen[0] & (1 << prg_sein)) {
						display.fillCircle(101, 32, 6, WHITE);
					}
				}
				else {
					if (pos_seinen[1] & (1 << prg_sein - 8)) {
						display.fillCircle(101, 32, 6, WHITE);
					}
				}
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
							display.drawRect(position + (i * 20), 15 + (a * temp), 12, 12, WHITE);
							//display.drawCircle(position + (i * 20), 15 + (a * temp), 5, 1);
						}
						else {
							display.fillRect(position + (i * 20), 15 + (a * temp), 12, 12, WHITE);
							//display.fillCircle(position + (i * 20), 15 + (a * temp), 5, 1);
						}
					}
				}
				buttons = 13;
				break;
			}
			break;
		case 3: //CV programming level 3 CV keuze
			TXT_cv3();
			buttons = 10;
			break;
		case 4: //routes wissels vastleggen
			cd; regel1s; //TXT(16); V401
			//display.print(rt_sel + 1); //TXT(200); //V401
			TXT(8);
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
		switch (PRG_fase) {
		case 3:
			TXT_cv3();// TXT(200);//v401
			display.print(PRG_cvs[1]);
			buttons = 10;
			break;
		case 4: //Route program, seinen instellen
			cd; regel1s; //TXT(16);  //V401
			//display.print(rt_sel + 1); TXT(200); 
			TXT(9);
			regel2; display.print(prg_sein + 1);


			if (prg_sein < 8) {
				b = 0;
				s = prg_sein;
			}
			else {
				b = 1;
				s = prg_sein - 8;
			}

			display.setCursor(32, 23);
			if (GPIOR2 & (1 << 0)) { //richting route/sein
				TXT(14); //>				
			}
			else {
				TXT(13); //<
				b = b + 2;
			}
			if (~ROUTE[rt_sel].seinen[b] & (1 << s)) { //if false bezet
				display.fillRect(65, 25, 12, 12, WHITE);
			}
			else {
				display.drawRect(65, 25, 12, 12, WHITE);
			}
			buttons = 16;
			break;
		}
		break;
		//************ Level 5
	case 5:
		cd; regel1s; //TXT(16); V401
		//display.print(rt_sel + 1); TXT(200);
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
		TXT(2); //TXT(101);
		break;
	case 1:
		TXT(48);// TXT(102); V401
		break;
	case 2:
		TXT(6);
		break;
	}
	regel2; TXT(5); display.print(PRG_cvs[0]); display.print(F(" "));
}
void DSP_buttons(byte mode) {
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
		//case 5: //routes, instellen wissels
			//???????
		//	break;
	case 6: //diverse instellingen van MEM_reg
		TXT(36);
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
		//case 0:
		//	display.println(F(""));
		//	//txt = (F(""));
		//	break;
	case 1:
		display.print(F("Adres "));
		break;
	case 2:
		display.print(F("Loc 1 "));
		break;
	case 3:
		display.print(F("Melder"));
		break;
		//case 4:
		//	display.print(F("functies ")); //niet in gebruik
		//	break;
	case 5:
		display.print(F("CV "));
		break;
	case 6:
		display.print(F("Decoder"));
		break;
	case 7:
		display.print(F("Set "));
		break;
	case 8:
		display.print(F("Wissel"));
		break;
	case 9:
		display.print("Sein");
		break;
	case 10:
		display.print(F("Schrijf "));
		break;
	case 11:
		display.print(F("Test"));
		break;
		//case 12:
		//	display.print(F("DCC "));
		//	break;
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
		display.print(F("Route"));
		break;
	case 17:
		display.print(F("Blokkades"));
		break;
		//***********Onderbalken
	case 18:
		display.print(F("B/M    *     V     X"));  //keuze routes, seinen
		break;

	case 19:
		display.print(F("S    <>    *    B/M"));  //keuze routes, seinen
		break;
	case 20:
		display.print(F("loc   stop   S&O   F"));
		break;
	case 21:
		display.print(F(" -     +     V     X"));
		break;
	case 22:
		display.print(F("S-    S+     <>    X"));
		break;
	case 23:
		display.print(F(" <    >     *      X"));
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
	case 33:
		display.print(F("Diverse"));
		break;
	case 34:
		display.print(F("Mono"));
		break;
	case 35:
		display.print(F("Dual"));
		break;
	case 36:
		display.print(F(">     *     V      X"));  //prg diverse seinen mode mono/dual
		break;
	case 37:
		display.print(F("Init"));//initialisatie bij power-up
		break;
	case 40:
		display.print(F("auto"));
		break;
	case 41:
		display.print(F("Aan"));
		break;
	case 42:
		display.print(F("Uit"));
		break;
	case 43:
		display.print(F("Melder 8"));
		break;
	case 44:
		display.print(F("Vrij"));
		break;
	case 45:
		display.print(F("Drf "));
		break;
	case 46:
		display.print(F("Slot"));
		break;
	case 47:
		display.print(F("Wacht"));
		break;
	case 48:
		display.print(F("Loc 2 "));
		break;
		/*case 100:
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
	*/
	}

}
void M_status() {
	count_slot++;
	if (count_slot > 50) {
		count_slot = 0;
		if (MELDERS() & (1 << 7)) { //melder 8 hoog
			slotstatus |= (1 << 5); //temp opslaan
		}
		else {
			slotstatus &= ~(1 << 5); //temp
		}

		slotstatus ^= (1 << 6); //flip counter, 2x meting doen		
		if (slotstatus & (1 << 6)) {
			if (~slotstatus & (1 << 7) && slotstatus & (1 << 5))
				slotstatus = B10000001; //als 2 gelijke aan metingen
		}
		else if (slotstatus & (1 << 7) && ~slotstatus & (1 << 5)) {
			slotstatus = B00000011; //na 2 gelijke uit metingen
		}
	}
}
void loop() {

	if (count_slow == 5000)Serial_read(); //leest, kijkt of er info is ontvangen op de serial poort

	count_slow++;//slow events timer, only ISR runs on full speed (generating DCC pulses)
	if (count_slow > 10000) { //12000 goede waarde voor deze timer		
		count_slow = 0;



		if (PINB & (1 << 2) && PINB & (1 << 0)) { //V401
			//bug oude versie zet continue pin 8 naar nul
			PORTB &= ~(1 << 0); //disable H-bridge if short
			SendDCCstatus(); //zend status naar WMapp
		}


		//


		//seinen uit zetten, aan einde route

		if (~GPIOR2 & (1 << 3)) {
			if (GPIOR2 & (1 << 1))SET_seinoff(0);
			if (GPIOR2 & (1 << 2))SET_seinoff(1);
		}

		SW_exe(); //switches  verplaatst 21/7

		//Slotstatus=bit0 uitvoeren bit1 aan of uit, bit2 eerste of tweede servo, kant, bit 7 oude stand
		//hier de servo aansturing
		if (~MEM_reg & (1 << 4)) { //slot ingeschakeld bit 3 en bit 4 false V401
			M_status();
		}

		if (slotstatus & (1 << 0)) { //bit0 true servoos omzetten
			if (~GPIOR0 & (1 << 2)) { //DCC_command is klaar met verzenden vorig command
				byte s = 4; //wordt gebruikt als  channel dus channel 1 van volgend decoderadres
				if (slotstatus & (1 << 2))s++;
				DCC_acc(false, true, s, slotstatus & (1 << 1));
				//wissel, aan, channel 4(dus volgend decoderadres) + 0 of 1, rechtdoor of afslaand.
				slotstatus ^= (1 << 2);  //volgende servo 
				if (~slotstatus & (1 << 2))slotstatus &= ~(15 << 0); //klaar, slot, servos zetten verlaten 
			}
		}

		else {
			count_locexe++;
			if (count_locexe > 10) { //hoevaak wordt loc berekend voor V401 =10 

				if (GPIOR1 & (1 << 5)) { //2e sein decoder to be set
					//wait for accessoire sending is ready
					if (~GPIOR0 & (1 << 2)) {
						SET_sein2();
					}
				}
				else {
					if (GPIOR1 & (1 << 4)) {  //bug28nov
						LOC_exe();
					}
					else {
						if (MEM_reg & (1 << 5)) { //initialise at startup off
							autostart();
						}
						else { //init at startup on
							INIT_wissels();
						}
					}
				}
				count_locexe = 0;
				//SW_exe(); //nieuwe plek 21/7
			}
		} //slotstatus

		if (GPIOR1 & (1 << 0))DCC_write(); //writing dcc adres in loc
		DCC_command();
		dcc_fase = 1;
	}
}