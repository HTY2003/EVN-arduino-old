/*blink.ino

The following program demonstrates the LED blinking on and off.
*/

#include <EVN.h>


//by default, the board is set such that the button toggles on/off with each press, and the LED output is linked to the button
//here, this function is disabled so the LED can be controlled independently
EVNAlpha board(BUTTON_TOGGLE, false);

void setup() {
    board.begin();  //initialize board at start of void setup()
}

void loop() {
    //turn onboard blue LED on
    board.ledWrite(true);
    delay(1000);

    //turn onboard blue LED off
    board.ledWrite(false);
    delay(1000);
}