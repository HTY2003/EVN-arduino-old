/*writeMatrix.ino

The following program demonstrates some basic EVNMatrixLED functionality.
*/

#include <EVN.h>

#define I2C_PORT 1  //set servo port here

EVNAlpha board(BUTTON_TOGGLE, true, true);
EVNMatrixLED matrix(I2C_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    matrix.begin();
}

void loop()
{

}