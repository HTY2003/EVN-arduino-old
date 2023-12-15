/*moveMotor.ino

The following program demonstrates some basic EVNMotor functionality.
*/

#include <EVN.h>

#define MOTOR_PORT 1  //set motor port here

EVNAlpha board(BUTTON_TOGGLE, LED_LINK, MOTORS_LINK);
EVNMotor motor(MOTOR_PORT, EV3_MED);  //motor type can be set to NXT_LARGE, EV3_MED or MOTOR_CUSTOM

void setup()
{
	board.begin();  //initialize board at start of void setup()
}

void loop()
{
	//if the button is "On"
	if (board.buttonRead())
	{
		motor.runTime(50, 3000);        //run motor at 50RPM for 3s
		motor.runDegrees(100, -360);    //run motor at 100RPM for -360deg (runDegrees(-100, 360) produces the same effect)
		motor.runSpeed(-30);            //run motor at -30RPM until new command is given
		delay(10000);                   //continue running for 10 seconds
	}

	//by default, if the button is toggled "Off", all motor movement functions will end instantly and all motors will stop
	//this can be disabled in EVNAlpha declaration (change MOTORS_LINK -> MOTORS_UNLINK)
	//Note: the button being toggled "Off" does not end other blocking functions (e.g. delay())
}

void setup1()
{
	//initialize motor
	//this can be run in void setup(), but running on the second core improves performance
	motor.begin();
}