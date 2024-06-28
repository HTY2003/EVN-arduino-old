/*writeMatrix.ino

The following program demonstrates some basic EVNMatrixLED functionality.
*/

#include <EVN.h>

#define MATRIX_I2C_PORT 1  //set I2C port here

EVNAlpha board;
EVNMatrixLED matrix(MATRIX_I2C_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    matrix.begin();
}

void loop()
{
    //turn all LEDs on and off
    matrix.writeAll();
    delay(1000);
    matrix.clearAll();
    delay(1000);

    //turn one LED on and off
    matrix.writeOne(0, 0); //controls pixel at coordinate X=0, Y=0
    delay(1000);
    matrix.clearOne(0, 0);
    delay(1000);

    matrix.writeAll();
    delay(1000);
    for (int i = 0; i < 8; i = i + 2)
    {
        //writeX and writeY control entire rows/columns
        //write functions can be used to turn off LEDs too
        matrix.writeX(i, false);    //turns off any LEDs where X=i
        delay(1000);
    }
    matrix.clearAll();
    delay(1000);

    //the "show" input is set to false, so changes to the buffer are not reflected until matrix.update() is called
    matrix.clearAll(false);
    for (int i = 0; i < 8; i = i + 2)
    {
        matrix.writeY(i, true, false);  //turns on any LEDs where Y=i
    }
    matrix.update();
    delay(1000);

    matrix.clearAll();
    delay(1000);

    //partial lines can be displayed
    //controls pixels on line Y=1, from X=1 to X=6
    matrix.writeHLine(1, 1, 6);
    delay(1000);
    //controls pixels on line X=1, from Y=1 to Y=6
    matrix.writeVLine(1, 1, 6);
    delay(1000);

    matrix.clearAll();
    delay(1000);

    //and rectangular regions too
    matrix.writeRectangle(1, 6, 1, 6, true, false);
    matrix.writeRectangle(2, 5, 2, 5, false, false);
    matrix.writeRectangle(3, 4, 3, 4, true, false);
    matrix.update();
    delay(1000);
}