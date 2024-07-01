/*I2CPortScanner.ino

The following program scans all 16 I2C Ports for connected devices.
*/

#include <EVN.h>

EVNAlpha board;

void setup()
{
    board.begin();
    Serial.begin(9600);
}

void loop()
{
    board.printPorts();
    delay(3000);
}