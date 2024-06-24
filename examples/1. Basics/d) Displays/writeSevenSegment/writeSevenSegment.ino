/*writeSevenSegment.ino

The following program demonstrates some basic EVNSevenSegmentLED functionality.
*/

#include <EVN.h>

#define I2C_PORT 1  //set servo port here

EVNAlpha board(BUTTON_TOGGLE, true, true);
EVNSevenSegmentLED seg(I2C_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    seg.begin();
}

void loop()
{

}