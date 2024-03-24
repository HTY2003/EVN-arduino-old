#include <EVN.h>

#define BT_SERIAL Serial1 //can be Serial1 or Serial2

EVNAlpha board;

void setup()
{
    board.begin();
    Serial.begin(9600);
    BT_SERIAL.begin(38400);
}

void loop()
{
    if (BT_SERIAL.available())
    {
        while (BT_SERIAL.available())
            Serial.write(BT_SERIAL.read());
    }

    if (Serial.available())
    {
        while (Serial.available())
            BT_SERIAL.write(Serial.read());
        BT_SERIAL.println();
    }
}