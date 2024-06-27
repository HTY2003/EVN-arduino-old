/*writeSevenSegment.ino

The following program demonstrates some basic EVNSevenSegmentLED functionality.
*/

#include <EVN.h>

#define I2C_PORT 1  //set I2C port here

EVNAlpha board;
EVNSevenSegmentLED seg(I2C_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    seg.begin();
}

void loop()
{
    //turn all LEDs on and off
    seg.writeAll();
    delay(1000);
    seg.clearAll();
    delay(1000);

    //write letters to each position
    //make sure to use single quotes (') instead of double quotes ("), since this is a single character in C
    seg.writeLetter(0, 'A');
    delay(1000);
    seg.writeLetter(1, 'B');
    delay(1000);
    seg.writeLetter(2, 'c'); //lowercase and uppercase inputs are both accepted, but they result in the same thing
    delay(1000);
    seg.writeLetter(3, 'd');
    delay(1000);

    seg.clearAll();
    delay(1000);

    for (int i = 0; i < 10; i++)
    {
        //writes digits 0-9 to position 0
        seg.writeDigit(0, i);
        delay(1000);

        //individual positions can be cleared too
        seg.clearPosition(0);
        delay(1000);
    }

    for (int i = 0; i < 4; i++)
    {
        //controls points in all positions
        //but with "show" set to false, so no changes are shown on display until seg.update() is called;
        seg.writePoint(0, i, false);
    }
    update();
    delay(1000);
    for (int i = 0; i < 4; i++)
    {
        //controls points in all positions
        //but with set to false
        seg.clearPoint(0, i, false);
    }
    update();
    delay(1000);

    seg.writeNumber(0.356);
    delay(1000);
    seg.writeNumber(1234);
    delay(1000);

    seg.writeColon(true);
    delay(1000);
    seg.clearColon();
    delay(1000);
}