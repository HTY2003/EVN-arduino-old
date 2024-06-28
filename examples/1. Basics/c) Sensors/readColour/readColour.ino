/*readColour.ino

The following program demonstrates some basic EVNColourSensor functionality.
*/

#include <EVN.h>

#define COL_I2C_PORT 1  //set I2C port for colour sensor here

EVNAlpha board;
EVNColourSensor cs(COL_I2C_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    cs.begin();     //sensor initialization comes after
    Serial.begin(9600);
}

void loop()
{
    int clear = cs.readClear();
    int red = cs.readRed(false);        //blocking is set to false
    int green = cs.readGreen(false);    //to ensure that the sensor does not wait for a new reading
    int blue = cs.readBlue(false);

    Serial.print("CLEAR ");
    Serial.print(clear);
    Serial.print(" RED ");
    Serial.print(red);
    Serial.print(" GREEN ");
    Serial.print(green);
    Serial.print(" BLUE ");
    Serial.println(blue);
}