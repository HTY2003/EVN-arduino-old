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

    rgb.writeOne(0, 255, 0, 0);
    delay(1000);
    rgb.writeOne(0, 0, 255, 0);
    delay(1000);
    rgb.writeOne(0, 0, 0, 255);
    delay(1000);
    rgb.clearOne(0);
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

    int start_time = millis();

    int r = 255;
    int g = 0;
    int b = 0;

    //run this loop for 2 seconds (3000 milliseconds)
    while (millis() - start_time < 3000)
    {
        //this part shifts the balance between the R, G and B values. They always add up to 255 (in theory, same brightness), but the ratios shift
        if (r > 0 && g != 255 && b == 0)
        {
            r -= 5;
            g += 5;
        }

        else if (g > 0 && b != 255 && r == 0)
        {
            g -= 5;
            b += 5;
        }

        else if (b > 0 && r != 255 && g == 0)
        {
            b -= 5;
            r += 5;
        }

        //show is set to false, so changes are not reflected on the display until rgb.update() is called
        rgb.writeOne(0, r, g, b, false);

        //now we are going to copy the RGB values and do the same shift on the copies 7 times (the original r,g and b variables are unaffected)
        //each time the copies are edited, we write the copy values to the next LED in the chain
        //this means every LED is a different shade (creating a "colour cycle")
        int copy_r = r, copy_g = g, copy_b = b;

        for (int i = 1; i < 8; i++)
        {
            if (copy_r > 0 && copy_g != 255 && copy_b == 0)
            {
                copy_r -= 5;
                copy_g += 5;
            }

            else if (copy_g > 0 && copy_b != 255 && copy_r == 0)
            {
                copy_g -= 5;
                copy_b += 5;
            }

            else if (copy_b > 0 && copy_r != 255 && copy_g == 0)
            {
                copy_b -= 5;
                copy_r += 5;
            }

            //show is set to false, so changes are not reflected on the display until rgb.update() is called
            rgb.writeOne(i, copy_r, copy_g, copy_b, false);
        }

        //in this case, we only write the changes once all 8 pixels have new values
        rgb.update();
        delay(10);
    }

    rgb.clearAll();
    delay(1000);
}