/*readDistance.ino

The following program demonstrates some basic EVNDistanceSensor functionality.
*/

#include <EVN.h>

#define DIST_I2C_PORT 1  //set I2C port for distance sensor here

EVNAlpha board;
EVNDistanceSensor ds(DIST_I2C_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    ds.begin();     //sensor initialization comes after
    Serial.begin(9600);
}

void loop()
{
    int dist = ds.read();
    Serial.println(dist);
}