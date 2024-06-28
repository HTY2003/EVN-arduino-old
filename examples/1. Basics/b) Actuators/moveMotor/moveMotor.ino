/*moveMotor.ino

The following program demonstrates some basic EVNMotor functionality.
*/

#include <EVN.h>

#define MOTOR_PORT 1  //set motor port here

EVNAlpha board;

//EVNAlpha board(BUTTON_TOGGLE, true, true);
//by default, any servos will not stop moving until they are instructed to stop using library functions 
//however, you can set link_movement to true to use the button as an enable/disable switch for motors and servos
//to try this out, uncomment line 11 and comment line 10

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
	//each button press toggles the button's output between "true" and "false" (by default, button outputs "false" on startup)
	//if button outputs "true", run the motor with different functions
	if (board.buttonRead())
	{
		motor.runTime(300, 3000); 	//run motor at 50RPM for 3s
		motor.runAngle(400, -360);	//run motor at 100RPM for -360deg (runAngle(-100, 360) produces the same effect)
		motor.runSpeed(-600);    	//run motor at -30RPM until new command is given
		delay(10000);           	//continue running for 10 seconds
	}
	else
	{
		motor.brake();	//otherwise, stop motor
	}
}