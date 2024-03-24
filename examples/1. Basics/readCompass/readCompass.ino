/*readCompass.ino

The following program demonstrates some basic EVNCompassSensor functionality.
*/

#include <EVN.h>

#define COMPASS_PORT 1  //set I2C port for compass sensor here

EVNAlpha board;
EVNCompassSensor compass(COMPASS_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    Serial.begin(9600);
    compass.begin();
}

void loop()
{
    double yaw = compass.read();
    Serial.println(yaw);
}