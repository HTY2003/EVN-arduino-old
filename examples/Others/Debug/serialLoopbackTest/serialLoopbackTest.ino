#include <EVN.h>

EVNAlpha board;

void setup()
{
    board.begin();
    Serial1.begin(9600);
    Serial2.begin(9600);
}

void loop()
{
    bool serial1_test = true;
    for (int i = 0; i < 100; i++)
    {
        Serial1.write(i);
        delay(10);
        if (Serial1.available())
        {
            if (Serial1.read() != i)
                serial1_test = false;
        }
    }

    bool serial2_test = true;
    for (int i = 0; i < 100; i++)
    {
        Serial2.write(i);
        delay(10);
        if (Serial2.available())
        {
            if (Serial2.read() != i)
                serial2_test = false;
        }
    }

    if (serial1_test) Serial.println("Serial1 Working");
    else Serial.println("Serial1 Failed");

    if (serial2_test) Serial.println("Serial2 Working");
    else Serial.println("Serial2 Failed");

    delay(3000);
}