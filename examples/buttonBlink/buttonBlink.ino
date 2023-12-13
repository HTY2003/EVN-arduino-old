/*buttonBlink.ino

The following program demonstrates a basic blink toggled on/off by the onboard button.
*/

#include <EVN.h>


//by default, the LED reflects the output of board.buttonRead() at all times
//this behaviour has been disabled (by changing LED_LINK -> LED_UNLINK)
EVNAlpha board(BUTTON_TOGGLE, LED_UNLINK, MOTORS_LINK);

void setup() {
	board.begin();  //initialize board at start of void setup()
}

void loop() {
	if (board.buttonRead())     //if button is on
	{
		board.ledWrite(false);    //turn off LED for 1s
		delay(1000);
		board.ledWrite(true);   //turn on LED for 1s
		delay(1000);
	}
	else {                      //else, turn LED off
		board.ledWrite(false);
	}
}