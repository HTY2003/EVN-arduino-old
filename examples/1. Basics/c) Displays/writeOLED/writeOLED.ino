/*writeOLED.ino

The following program demonstrates some basic EVNDisplay functionality.
*/

#include <EVN.h>

#define DISPLAY_PORT 1  //set I2C port for display here

EVNAlpha board;
EVNDisplay display(DISPLAY_PORT);

void setup()
{
    board.begin();          //initialize board at start of void setup()
    display.begin();        //initialise display (AFTER board)

    //please skip these lines in your project if you don't want them there
    display.splashEVN();    //display EVN logo animation
    delay(1000);            //show logo for 1 second (please skip these lines if you want to)
    display.clear();        //clear display

    //writeLabel() defines what appears at the start of the line
    //a common use case for labels is sensor debug printing (e.g. label is "Left Col: ")
    display.writeLabel(0, "Time: ");     //set label for row 0
    display.writeLabel(1, "Button: ");   //set label for row 1
}

void loop()
{
    //writeData() places the data after the row label
    //useful for frequently updated information
    display.writeData(0, millis());             //write time since program start to row 0
    display.writeData(1, board.buttonRead());   //write button read output to row 1
}