/*writeRGBLED.ino

The following program demonstrates some basic EVNRGBLED functionality.
*/

#include <EVN.h>

#define SERVO_PORT 1  //set servo port here

EVNAlpha board;
EVNRGBLED rgb(SERVO_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    rgb.begin();
}

void loop()
{
    rgb.writeAll(255, 255, 255);
    delay(1000);
    rgb.clearAll();
    delay(1000);

    rgb.writeOne(255, 0, 0);
    delay(1000);
    rgb.writeOne(0, 255, 0);
    delay(1000);
    rgb.writeOne(0, 0, 255);
    delay(1000);
    rgb.clearOne();
    delay(1000);

    for (int i = 0; i < 255; i++)
    {
        rgb.writeLine(1, 6, i, 0, 0, true);
        delay(10);
    }

    for (int i = 255; i > 0; i--)
    {
        rgb.writeLine(1, 6, i, 0, 0, true);
        delay(10);
    }
    delay(1000);

    int r = 255;
    int g = 127;
    int b = 0;
    int led = 0;

    while (r != 0)
    {
        r += 1;
        g += 1;
        b += 1;

        r %= 255;
        g %= 255;
        b %= 255;

        for (int i = 0; i < 8; i++)
        {
            //show is set to false, so changes are not reflected on the display until rgb.update() is called
            //in this case, we only write the changes once all 8 pixels are edited
            rgb.writeOne(led, (r + i) % 255, (g + i) % 255, (b + i) % 255, false);
        }

        update();
        delay(10);
    }
}