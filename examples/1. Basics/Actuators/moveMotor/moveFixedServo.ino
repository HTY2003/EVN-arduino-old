/*moveFixedServo.ino

The following program demonstrates some basic EVNServo functionality.
*/

#include <EVN.h>

#define SERVO_PORT 1  //set servo port here

EVNAlpha board(BUTTON_TOGGLE, true, true);
EVNServo servo(SERVO_PORT);

void setup1()
{
    //initialize servo
    //this can be run in void setup(), but running on the second core improves performance
    servo.begin();
}

void setup()
{
    board.begin();  //initialize board at start of void setup()
}

void loop()
{
    //if the button is "On"
    if (board.buttonRead())
    {
        servo.write(0, 1000);
        servo.write(270, 5000, 20);
    }

    //by default, if the user button is toggled "Off", all motor run functions will end instantly and all servos will stop
    //this can be disabled in EVNAlpha declaration (set link_movement to false)
    //Note: button being "off" does not end other blocking functions (e.g. delay())
}