/*readDistance.ino

The following program demonstrates some basic EVNDistanceSensor functionality.
*/

#include <EVN.h>

#define DISTANCE_SENS_PORT 1  //set I2C port for distance sensor here

EVNAlpha board;
EVNDistanceSensor ds(DISTANCE_SENS_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    Serial.begin(9600);
    ds.begin();
}

void loop()
{
    int dist = ds.read();
    Serial.println(dist);
}