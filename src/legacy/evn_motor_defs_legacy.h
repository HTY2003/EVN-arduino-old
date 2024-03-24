//TUNING FOR CUSTOM GEARMOTOR
//edit this to change CUSTOM_MOTOR defaults
//example settings below are for the JGA25-370 motor, 6V 77RPM version
#define SPEED_PID_KP_CUSTOM		5       //speed PID gains (PI controller)
#define SPEED_PID_KI_CUSTOM		0.125
#define SPEED_PID_KD_CUSTOM		0
#define POS_PID_KP_CUSTOM		8       //position PID gains (PD controller)
#define POS_PID_KI_CUSTOM		0
#define POS_PID_KD_CUSTOM		16
#define CUSTOM_MOTOR_MAX_RPM	100      //max RPM of the motor shaft
#define CUSTOM_MOTOR_ACCEL      1000     //deceleration in rpm/s (60 means it takes 1 second to accelerate from 0RPM to 60)
#define CUSTOM_MOTOR_DECEL      1000     //deceleration in rpm/s (60 means it takes 1 second to decelerate from 60RPM to 0)
#define CUSTOM_PPR				823     //pulses on 1 encoder wheel for 1 revolution of the motor shaft

//LEGO MOTORS
#define LEGO_PPR				180

//TUNING FOR LEGO EV3 LARGE SERVO MOTOR
#define SPEED_PID_KP_EV3_LARGE	5
#define SPEED_PID_KI_EV3_LARGE	0.125
#define SPEED_PID_KD_EV3_LARGE	0
#define POS_PID_KP_EV3_LARGE    8
#define POS_PID_KI_EV3_LARGE	0
#define POS_PID_KD_EV3_LARGE	16
#define EV3_LARGE_MAX_RPM		155
#define EV3_LARGE_ACCEL         1550
#define EV3_LARGE_DECEL         1550

//TUNING FOR LEGO EV3 MEDIUM SERVO MOTOR
#define SPEED_PID_KP_EV3_MED	4.5
#define SPEED_PID_KI_EV3_MED	0.125
#define SPEED_PID_KD_EV3_MED	0
#define POS_PID_KP_EV3_MED	    8
#define POS_PID_KI_EV3_MED		0
#define POS_PID_KD_EV3_MED		16
#define EV3_MED_MAX_RPM			230
#define EV3_MED_ACCEL           2300
#define EV3_MED_DECEL           2300

//TUNING FOR LEGO NXT LARGE SERVO MOTOR
#define SPEED_PID_KP_NXT_LARGE	5
#define SPEED_PID_KI_NXT_LARGE	0.125
#define SPEED_PID_KD_NXT_LARGE	0
#define POS_PID_KP_NXT_LARGE	8
#define POS_PID_KI_NXT_LARGE	0
#define POS_PID_KD_NXT_LARGE	16
#define NXT_LARGE_MAX_RPM		155
#define NXT_LARGE_ACCEL         1550
#define NXT_LARGE_DECEL         1550