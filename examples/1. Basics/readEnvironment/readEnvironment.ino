/*readEnvironment.ino

The following program demonstrates some basic EVNEnvSensor functionality.
*/

#include <EVN.h>

#define ENV_SENS_PORT 1  //set I2C port for environment sensor here

EVNAlpha board;
EVNEnvSensor env(ENV_SENS_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    Serial.begin(9600);
    env.begin();
}

void loop()
{
    float humidity = env.readClear();
    float temp = env.readTemp(false);
    float pressure = env.readPres(false);
    float altitude = env.readAltitude(false);

    Serial.print("Humidity ");
    Serial.print(humidity);
    Serial.print("% | Temperature ");
    Serial.print(temp);
    Serial.print("deg | Pressure");
    Serial.print(pressure);
    Serial.print("Pa | Altitude ");
    Serial.print(altitude);
    Serial.println("m");
}