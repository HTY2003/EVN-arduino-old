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