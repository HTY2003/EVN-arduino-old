/*readColour.ino

The following program demonstrates some basic EVNColourSensor functionality.
*/

#include <EVN.h>

#define COLOUR_SENS_PORT 1  //set I2C port for colour sensor here

EVNAlpha board;
EVNColourSensor cs(COLOUR_SENS_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    Serial.begin(9600);
    cs.begin();
}

void loop()
{
    int clear = cs.readClear();
    int red = cs.readRed(false);
    int green = cs.readGreen(false);
    int blue = cs.readBlue(false);

    Serial.print("C ");
    Serial.print(clear);
    Serial.print("R ");
    Serial.print(red);
    Serial.print("G ");
    Serial.print(green);
    Serial.print("B ");
    Serial.println(blue);
}