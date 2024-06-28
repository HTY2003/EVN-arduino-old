/*blink.ino

The following program demonstrates the LED blinking on and off.
*/

#include <EVN.h>


//by default, the board is set such that the button toggles on/off with each press, and the LED output is linked to the button
//in this program, this "link" is disabled so the LED can be controlled independently
EVNAlpha board(BUTTON_TOGGLE, false);

void setup() {
    board.begin();  //initialize board at start of void setup()
}

void loop() {
    //turn onboard blue LED on for 1000 milliseconds (1 second)
    board.ledWrite(true);
    delay(1000);

    //turn LED off for 1 second
    board.ledWrite(false);
    delay(1000);
}