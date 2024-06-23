/*buttonLED.ino

The following program demonstrates the LED toggled on/off by the onboard button.
*/

#include <EVN.h>


//by default, the board is set such that the button toggles on/off with each press, and the LED output is linked to the button
EVNAlpha board;

void setup() {
	board.begin();  //initialize board at start of void setup()
}

void loop() {
	//nothing here, but pressing the button should toggle the LED on and off
}