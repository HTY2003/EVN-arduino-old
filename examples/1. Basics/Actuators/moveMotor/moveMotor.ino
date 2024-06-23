/*moveMotor.ino

The following program demonstrates some basic EVNMotor functionality.
*/

#include <EVN.h>

#define MOTOR_PORT 1  //set motor port here

EVNAlpha board(BUTTON_TOGGLE, true, true);
EVNMotor motor(MOTOR_PORT, EV3_MED);  //motor type can also be set to NXT_LARGE, EV3_LARGE or MOTOR_CUSTOM

void setup1()
{
	//initialize motor
	//this can be run in void setup(), but running on the second core improves performance
	motor.begin();
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
		motor.runTime(300, 3000);        //run motor at 50RPM for 3s
		motor.runAngle(400, -360);    //run motor at 100RPM for -360deg (runAngle(-100, 360) produces the same effect)
		motor.runSpeed(-600);            //run motor at -30RPM until new command is given
		delay(10000);                   //continue running for 10 seconds
	}

	//by default, if the button is toggled "Off", all motor run functions will end instantly and all motors will stop
	//this can be disabled in EVNAlpha declaration (set link_motors to false)
	//Note: button being "off" does not end other blocking functions (e.g. delay())
}