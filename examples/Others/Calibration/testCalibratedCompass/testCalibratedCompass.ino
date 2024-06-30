/*testCalibratedCompass.ino

The following program prints the calibrated compass readings to be passed to MotionCal for testing.
*/

#include <EVN.h>

#define COMPASS_I2C_PORT 1  //set I2C port for compass sensor here

EVNAlpha board;
EVNCompassSensor compass(COMPASS_I2C_PORT,  //add in calibration values here!
    0, 0, 0
    1, 0, 0,
    0, 1, 0,
    0, 0, 1);

void setup()
{
    board.begin();      //initialize board at start of void setup()
    compass.begin();    //sensor initialization comes after
    Serial.begin(9600);
}

void loop()
{
    //collect calibrated readings
    float mx = compass.readCalX();
    float my = compass.readCalY(false);
    float mz = compass.readCalZ(false);

    //print calibrated readings in format expected by MotionCal
    //readings are scaled by 1000 and converted to integers

    Serial.print("Raw:0,0,0,0,0,0,");
    Serial.print((int)(mx * 1000));
    Serial.print(",");
    Serial.print((int)(my * 1000));
    Serial.print(",");
    Serial.println((int)(mz * 1000));
    delay(20);
}