/*readCompass.ino

The following program demonstrates some basic EVNCompassSensor functionality.
*/

#include <EVN.h>

#define COMPASS_I2C_PORT 1  //set I2C port for compass sensor here

EVNAlpha board;
EVNCompassSensor compass(COMPASS_I2C_PORT);

void setup()
{
    board.begin();      //initialize board at start of void setup()
    compass.begin();    //sensor initialization comes after
    Serial.begin(9600);
}

void loop()
{
    float yaw = compass.read();
    Serial.println(yaw);

    //Readings may be poor if the compass is not calibrated
    //Check out the guide on our docs website on how to calibrate compass sensors!
}