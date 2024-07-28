/*readADC.ino

The following program demonstrates some basic EVNADC functionality.
*/

#include <EVN.h>

#define ADC_I2C_PORT 1  //set I2C port for ADC here

EVNAlpha board;
EVNADC adc();

void setup()
{
    board.begin();  //initialize board at start of void setup()
    adc.begin();    //ADC initialization comes after
    Serial.begin(9600);
}

void loop()
{
    int pin0_reading = adc.read(0);
    int pin1_reading = adc.read(1);
    int pin2_reading = adc.read(2);
    int pin3_reading = adc.read(3);

    Serial.print("Pin 0: ");
    Serial.print(pin0_reading);
    Serial.print(" | 1: ");
    Serial.print(pin1_reading);
    Serial.print(" | 2: ");
    Serial.print(pin2_reading);
    Serial.print(" | 3: ");
    Serial.println(pin3_reading);
}