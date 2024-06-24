/*writeRGBLED.ino

The following program demonstrates some basic EVNRGBLED functionality.
*/

#include <EVN.h>

#define SERVO_PORT 1  //set servo port here

EVNAlpha board(BUTTON_TOGGLE, true, true);
EVNRGBLED rgbled(SERVO_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    rgbled.begin();
}

void loop()
{
    for (int i = 0; i < 8; i++)
    {

    }

    delay(1000);


    delay(1000)
}