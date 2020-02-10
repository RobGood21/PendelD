/*
 Name:		OledTest.ino
 Created:	2/10/2020 9:35:52 PM
 Author:	Rob Antonisse




*/


#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


// the setup function runs once when you press reset or power the board
void setup() {

	Serial.begin(9600); //(115200);

	display.begin(SSD1306_SWITCHCAPVCC, 0x3C);


	//if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
	//	Serial.println("SSD1306 allocation failed");
	//	for (;;); // Don't proceed, loop forever
	//}


	//delay(2000);
	display.clearDisplay();

	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(10, 10);
	display.println("www.wisselmotor.nl");
	
	//Use the drawLine(x1, y1, x2, y2, color) method to create a line.The(x1, y1) coordinates 
	//indicate the start of the line, and the(x2, y2) coordinates indicates where the line ends.
	display.display();
	delay(1000);	
	display.drawLine(10, 20, 115, 20, WHITE);
	display.display();
	delay(1000);
	//The drawRect(x, y, width, height, color) provides an easy way to draw a rectangle.
	//The(x, y) coordinates indicate the top left corner of the rectangle.Then, 
	//you need to specify the width, height and color:
	display.drawRect(5, 25, 100, 20, WHITE);
	display.display();
}

// the loop function runs over and over again until power down or reset
void loop() {
  
}
