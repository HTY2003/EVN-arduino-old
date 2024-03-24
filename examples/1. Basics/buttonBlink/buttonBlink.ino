/*buttonBlink.ino

The following program demonstrates a basic blink toggled on/off by the onboard button.
*/

#include <EVN.h>


//by default, LED reflects the output of board.buttonRead() at all times
//this behaviour has been disabled (by setting link_led to false)
EVNAlpha board(BUTTON_TOGGLE, false, true);

void setup() {
	board.begin();  //initialize board at start of void setup()
}

void loop() {
	if (board.buttonRead())     //if button is on
	{
		board.ledWrite(false);    //turn off LED for 0.5s
		delay(500);
		board.ledWrite(true);   //turn on LED for 0.5s
		delay(500);
	}
	else {                      //else, turn LED off
		board.ledWrite(false);
	}
}