/*readWriteBluetooth.ino

The following program demonstrates some basic EVNBluetooth functionality.
*/

#include <EVN.h>

EVNAlpha board;

EVNBluetooth bt(1, 9600, "EVN Bluetooth", BT_REMOTE);

void setup()
{
    board.begin();
    bt.begin();
    Serial1.begin(9600);
    Serial.begin(9600);
}

void loop()
{
    //this piece of code reads any message received by the Bluetooth Module
    //and prints it to Serial Monitor
    while (Serial1.available())
        Serial.print(Serial1.read());

    //it also writes a message of 1 to the connected host
    //note: writing the number 1 is different from writing the character '1' using Serial1.print('1')
    Serial1.write(1);
    delay(1000);

    //however, you can pretty much do anything with Serial1 in void loop()
    //think of it as a wireless Serial connection
}