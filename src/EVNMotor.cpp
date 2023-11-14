#include <EVNMotor.h>
#include <EVNButton.h>
#include <Arduino.h>

encoder_state_t* EVNMotor::encoderArgs[];
speed_pid_t* EVNMotor::speedArgs[];
position_pid_t* EVNMotor::posArgs[];
time_pid_t* EVNMotor::timeArgs[];
RPI_PICO_Timer EVNMotor::timer(3);
RPI_PICO_ISR_Timer EVNMotor::ISRtimer;

EVNMotor::EVNMotor(uint8_t port, uint8_t motortype, EVNButton* button)
{
	if (button)
	{
		speed_pid.button = button;
	}

	switch (port)
	{
	case 1:
		_motora = OUTPUT1MOTORA;
		_motorb = OUTPUT1MOTORB;
		_enca = OUTPUT1ENCA;
		_encb = OUTPUT1ENCB;
		break;

	case 2:
		_motora = OUTPUT2MOTORA;
		_motorb = OUTPUT2MOTORB;
		_enca = OUTPUT2ENCA;
		_encb = OUTPUT2ENCB;
		break;
	case 3:
		_motora = OUTPUT3MOTORA;
		_motorb = OUTPUT3MOTORB;
		_enca = OUTPUT3ENCA;
		_encb = OUTPUT3ENCB;
		break;
	case 4:
		_motora = OUTPUT4MOTORA;
		_motorb = OUTPUT4MOTORB;
		_enca = OUTPUT4ENCA;
		_encb = OUTPUT4ENCB;
		break;
	}

	_motortype = constrain(motortype, 0, 2);

	switch (motortype)
	{
	case EV3_LARGE:
		_maxrpm = EV3_LARGE_MAX_RPM;
		speed_pid.pid = new PIDController(SPEED_PID_KP_EV3_LARGE, SPEED_PID_KI_EV3_LARGE, SPEED_PID_KD_EV3_LARGE, DIRECT);
		pos_pid.pid = new PIDController(POS_PID_KP_EV3_LARGE, POS_PID_KI_EV3_LARGE, POS_PID_KD_EV3_LARGE, DIRECT);
		break;
	case NXT_LARGE:
		_maxrpm = NXT_LARGE_MAX_RPM;
		speed_pid.pid = new PIDController(SPEED_PID_KP_NXT_LARGE, SPEED_PID_KI_NXT_LARGE, SPEED_PID_KD_NXT_LARGE, DIRECT);
		pos_pid.pid = new PIDController(POS_PID_KP_NXT_LARGE, POS_PID_KI_NXT_LARGE, POS_PID_KD_NXT_LARGE, DIRECT);
		break;
	case EV3_MED:
		_maxrpm = EV3_MED_MAX_RPM;
		speed_pid.pid = new PIDController(SPEED_PID_KP_EV3_MED, SPEED_PID_KI_EV3_MED, SPEED_PID_KD_EV3_MED, DIRECT);
		pos_pid.pid = new PIDController(POS_PID_KP_EV3_MED, POS_PID_KI_EV3_MED, POS_PID_KD_EV3_MED, DIRECT);
		break;
	case CUSTOM:
		_maxrpm = CUSTOM_MAX_RPM;
		speed_pid.pid = new PIDController(SPEED_PID_KP_CUSTOM, SPEED_PID_KI_CUSTOM, SPEED_PID_KD_CUSTOM, DIRECT);
		pos_pid.pid = new PIDController(POS_PID_KP_CUSTOM, POS_PID_KI_CUSTOM, POS_PID_KD_CUSTOM, DIRECT);
		break;
	}
	maxrpm = _maxrpm;
}

void EVNMotor::init()
{
	analogWriteFreq(OUTPUTPWMFREQ);
	pinMode(_motora, OUTPUT_8MA);
	pinMode(_motorb, OUTPUT_8MA);

	encoder.enca = _enca;
	encoder.encb = _encb;

	attach_enc_interrupt(_enca, &encoder);
	attach_enc_interrupt(_encb, &encoder);

	pinMode(_enca, INPUT);
	pinMode(_encb, INPUT);

	time_pid.running = false;

	speed_pid.motora = _motora;
	speed_pid.motorb = _motorb;
	speed_pid.maxrpm = (double)_maxrpm;
	speed_pid.running = false;
	speed_pid.error = 0;

	pos_pid.error = 0;
	pos_pid.hold = false;
	pos_pid.running = false;

	timer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, isrtimer);
	attach_pid_interrupt(_enca, &speed_pid, &pos_pid, &time_pid);
}

void EVNMotor::writePWM(double speed)
{
	speed_pid.running = false;
	time_pid.running = false;
	pos_pid.running = false;
	pos_pid.hold = false;
	writePWM_static(_motora, _motorb, speed);
}

double EVNMotor::getPos()
{
	return ((double)encoder.position / 2);
}

void EVNMotor::resetPos()
{
	encoder.position = 0;
}

double EVNMotor::getRPM()
{
	return getRPM_static(&encoder);
}

double EVNMotor::debugSpeedPID()
{
	return speed_pid.output;
}

double EVNMotor::debugPositionPID()
{
	return pos_pid.output;
}

void EVNMotor::runSpeed(double rpm)
{
	double rpmc = constrain(rpm, -_maxrpm, _maxrpm);

	if (!speed_pid.running || speed_pid.targetrpm != rpmc)
	{
		lastcommand_ms = millis();
	}
	speed_pid.targetrpm = rpmc;

	speed_pid.running = true;
	pos_pid.running = false;
	time_pid.running = false;
	pos_pid.hold = false;
}

void EVNMotor::runDegrees(double degrees, uint8_t stop_action, bool wait)
{
	uint8_t stop_actionc = min(3, stop_action);

	pos_pid.targetpos = this->getPos() + degrees;
	pos_pid.error = 0;
	pos_pid.stop_action = stop_actionc;

	pos_pid.running = true;
	time_pid.running = false;
	speed_pid.running = false;
	pos_pid.hold = false;

	if (wait)
	{
		while (!this->commandFinished())
			;
	}
}

void EVNMotor::runTime(double rpm, uint32_t time_ms, uint8_t stop_action, bool wait)
{
	double rpmc = constrain(rpm, -_maxrpm, _maxrpm);
	uint8_t stop_actionc = min(3, stop_action);

	time_pid.targetrpm = rpmc;
	time_pid.starttime = millis();
	time_pid.time_ms = time_ms;
	pos_pid.stop_action = stop_actionc;

	time_pid.running = true;
	pos_pid.running = false;
	speed_pid.running = false;
	pos_pid.hold = false;

	lastcommand_ms = millis();

	if (wait)
	{
		while (!this->commandFinished())
			;
	}
}

void EVNMotor::brake()
{
	speed_pid.running = false;
	pos_pid.running = false;
	time_pid.running = false;
	pos_pid.hold = false;
	pos_pid.stop_action = STOP_ACTION_BRAKE;
	stopAction_static(_motora, _motorb, &pos_pid, &encoder);
}

void EVNMotor::coast()
{
	speed_pid.running = false;
	pos_pid.running = false;
	time_pid.running = false;
	pos_pid.hold = false;
	pos_pid.stop_action = STOP_ACTION_COAST;
	stopAction_static(_motora, _motorb, &pos_pid, &encoder);
}

void EVNMotor::hold()
{
	speed_pid.running = false;
	pos_pid.running = false;
	time_pid.running = false;
	pos_pid.stop_action = STOP_ACTION_HOLD;
	stopAction_static(_motora, _motorb, &pos_pid, &encoder);
}

bool EVNMotor::commandFinished()
{
	return (!time_pid.running && !pos_pid.running);
}

uint64_t EVNMotor::timeSinceLastCommand()
{
	return millis() - lastcommand_ms;
}

EVNDrivebase::EVNDrivebase(uint32_t wheel_dia, uint32_t wheel_dist, EVNMotor* motor_left, EVNMotor* motor_right, uint8_t motor_left_inv, uint8_t motor_right_inv) : avg_pid(1, 0, 0, DIRECT), diff_pid(1, 0, 0, DIRECT), avg_follow_pid(1, 0, 0, DIRECT), diff_follow_pid(1, 0, 0, DIRECT)
{
	_motor_left = motor_left;
	_motor_right = motor_right;
	_maxrpm = min(motor_left->maxrpm, motor_left->maxrpm);
	_motor_left_inv = motor_left_inv;
	_motor_right_inv = motor_right_inv;
	_wheel_dist = wheel_dist;
	_wheel_dia = wheel_dia;
}

void EVNDrivebase::steer(double speed, double turn_rate)
{
	double leftspeed, rightspeed;
	double speedc = constrain(speed, -_maxrpm, _maxrpm);
	double turn_ratec = constrain(turn_rate, -1, 1);
	if (turn_ratec >= 0)
	{
		leftspeed = speedc;
		rightspeed = speedc - 2 * turn_ratec * speedc;
	}
	else
	{
		rightspeed = speedc;
		leftspeed = speedc + 2 * turn_ratec * speedc;
	}

	if (_motor_left_inv == REVERSE)
		leftspeed *= -1;
	if (_motor_right_inv == REVERSE)
		rightspeed *= -1;

	_motor_left->runSpeed(leftspeed);
	_motor_right->runSpeed(rightspeed);
}

// void EVNDrivebase::steer_better(double speed, double turn_rate)
// {
// 	double speedc = constrain(speed, -_maxrpm, _maxrpm);
// 	double turn_ratec = constrain(turn_ratec, -1, 1);
// 	double turn_radius, arclen_outer, arclen_inner, target_angvel;
// 	double outer_angvel, inner_angvel, avg_angvel, diff_angvel;
// 	double norm_avg_angvel_error, norm_diff_angvel_error;
// 	double avg_output, diff_output, avg_follow_output, diff_follow_output;

// 	if (turn_ratec != 0)
// 	{
// 		turn_radius = (double)_wheel_dist / (fabs(turn_ratec) * 2); //distance of centre of drivebase from centre of rotation
// 		arclen_outer = turn_radius * PI;	//arc length for outer wheel for 180deg
// 		arclen_inner = (turn_radius - (double)_wheel_dist) * PI;
// 		target_angvel = (speedc * _maxrpm / 60 * PI * _wheel_dia) / arclen_outer * PI; //target angular velocity in rad/s
// 	}
// 	else {
// 		arclen_inner = PI * _wheel_dia / 2;
// 		arclen_outer = PI * _wheel_dia / 2;
// 		target_angvel = 0;
// 	}

// 	//get real life angular velocity values
// 	if (turn_ratec > 0)
// 	{
// 		outer_angvel = (_motor_left->getRPM() / 60 * PI * _wheel_dia) / arclen_outer * PI;
// 		inner_angvel = (_motor_right->getRPM() / 60 * PI * _wheel_dia) / arclen_outer * PI;
// 	}
// 	else if (turn_ratec < 0)
// 	{
// 		outer_angvel = (_motor_right->getRPM() / 60 * PI * _wheel_dia) / arclen_outer * PI;
// 		inner_angvel = (_motor_left->getRPM() / 60 * PI * _wheel_dia) / arclen_outer * PI;
// 	}

// 	avg_angvel = (outer_angvel + inner_angvel) / 2; //normalise w targetangvel and maxangvel
// 	diff_angvel = (outer_angvel - inner_angvel) / 2; //normalise with targetdiffangvel (0) and maxangvel?
// 	norm_avg_angvel_error = (target_angvel - avg_angvel) / TWO_PI;
// 	norm_diff_angvel_error = (0 - diff_angvel) / TWO_PI;
// 	avg_output = avg_pid.compute((target_angvel - avg_angvel) / TWO_PI);
// 	diff_output = diff_pid.compute((0 - diff_angvel) / TWO_PI);
// 	avg_follow_output = avg_follow_pid.compute((target_angvel - avg_angvel) / TWO_PI);
// 	diff_follow_output = diff_follow_pid.compute((0 - diff_angvel) / TWO_PI);

// 	if (norm_avg_angvel_error > norm_diff_angvel_error)
// 	{
// 		if (turn_ratec > 0)
// 		{
// 			_motor_left->writePWM(avg_output * PWM_MAX_VAL);
// 			_motor_right->writePWM(avg_output * PWM_MAX_VAL + diff_follow_output * PWM_MAX_VAL);
// 		}
// 		else
// 		{
// 			_motor_right->writePWM(avg_output * PWM_MAX_VAL);
// 			_motor_left->writePWM(avg_output * PWM_MAX_VAL + diff_follow_output * PWM_MAX_VAL);
// 		}
// 	}
// 	else
// 	{
// 		if (turn_ratec > 0)
// 		{
// 			_motor_left->writePWM(diff_output * PWM_MAX_VAL);
// 			_motor_right->writePWM(diff_output * PWM_MAX_VAL + avg_follow_output);
// 		}
// 		else
// 		{
// 			_motor_right->writePWM(diff_output * PWM_MAX_VAL);
// 			_motor_left->writePWM(diff_output * PWM_MAX_VAL + avg_follow_output);
// 		}
// 	}
// }

void EVNDrivebase::brake()
{
	_motor_left->brake();
	_motor_right->brake();
}

void EVNDrivebase::coast()
{
	_motor_left->coast();
	_motor_right->coast();
}

void EVNDrivebase::hold()
{
	_motor_left->hold();
	_motor_right->hold();
}