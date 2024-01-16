#include "EVNMotor.h"

encoder_state_t* EVNMotor::encoderArgs[];
speed_pid_t* EVNMotor::speedArgs[];
position_pid_t* EVNMotor::posArgs[];
time_pid_t* EVNMotor::timeArgs[];

EVNMotor::EVNMotor(uint8_t port, uint8_t motortype, uint8_t motor_dir, uint8_t enc_dir)
{
	uint8_t motor_dirc = constrain(motor_dir, 0, 1);
	uint8_t enc_dirc = constrain(enc_dir, 0, 1);
	uint8_t portc = constrain(port, 1, 4);
	_motortype = constrain(motortype, 0, 2);

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

	switch (motortype)
	{
	case EV3_LARGE:
		_maxrpm = EV3_LARGE_MAX_RPM;
		speed_pid.pid = new PIDController(SPEED_PID_KP_EV3_LARGE, SPEED_PID_KI_EV3_LARGE, SPEED_PID_KD_EV3_LARGE, DIRECT);
		pos_pid.pid = new PIDController(POS_PID_KP_EV3_LARGE, POS_PID_KI_EV3_LARGE, POS_PID_KD_EV3_LARGE, DIRECT);
		encoder.ppr = LEGO_PPR;
		break;
	case NXT_LARGE:
		_maxrpm = NXT_LARGE_MAX_RPM;
		speed_pid.pid = new PIDController(SPEED_PID_KP_NXT_LARGE, SPEED_PID_KI_NXT_LARGE, SPEED_PID_KD_NXT_LARGE, DIRECT);
		pos_pid.pid = new PIDController(POS_PID_KP_NXT_LARGE, POS_PID_KI_NXT_LARGE, POS_PID_KD_NXT_LARGE, DIRECT);
		encoder.ppr = LEGO_PPR;
		break;
	case EV3_MED:
		_maxrpm = EV3_MED_MAX_RPM;
		speed_pid.pid = new PIDController(SPEED_PID_KP_EV3_MED, SPEED_PID_KI_EV3_MED, SPEED_PID_KD_EV3_MED, DIRECT);
		pos_pid.pid = new PIDController(POS_PID_KP_EV3_MED, POS_PID_KI_EV3_MED, POS_PID_KD_EV3_MED, DIRECT);
		encoder.ppr = LEGO_PPR;
		break;
	case CUSTOM_MOTOR:
		_maxrpm = CUSTOM_MOTOR_MAX_RPM;
		speed_pid.pid = new PIDController(SPEED_PID_KP_CUSTOM, SPEED_PID_KI_CUSTOM, SPEED_PID_KD_CUSTOM, DIRECT);
		pos_pid.pid = new PIDController(POS_PID_KP_CUSTOM, POS_PID_KI_CUSTOM, POS_PID_KD_CUSTOM, DIRECT);
		encoder.ppr = CUSTOM_PPR;
		break;
	}

	if (motor_dirc == REVERSE)
	{
		uint8_t pin = _motora;
		_motora = _motorb;
		_motorb = pin;
		pin = _enca;
		_enca = _encb;
		_encb = pin;
	}

	if (enc_dirc == REVERSE)
	{
		uint8_t pin = _enca;
		_enca = _encb;
		_encb = pin;
	}

	encoder.enca = _enca;
	encoder.encb = _encb;
	speed_pid.motora = _motora;
	speed_pid.motorb = _motorb;

	maxrpm = _maxrpm;
	speed_pid.maxrpm = (double)_maxrpm;
	speed_pid.running = false;
	pos_pid.hold = false;
	pos_pid.running = false;
	speed_pid.running = false;
	time_pid.running = false;
}

void EVNMotor::begin()
{
	if (!_started)
	{
		analogWriteFreq(OUTPUTPWMFREQ);
		analogWriteRange(PWM_MAX_VAL);
		pinMode(_motora, OUTPUT_8MA);
		pinMode(_motorb, OUTPUT_8MA);
		pinMode(_enca, INPUT);
		pinMode(_encb, INPUT);
		attach_enc_interrupt(_enca, &encoder);
		attach_enc_interrupt(_encb, &encoder);
		attach_pid_interrupt(_enca, &speed_pid, &pos_pid, &time_pid);

		_started = true;
	}
}

void EVNMotor::writePWM(double speed)
{
	speed_pid.running = false;
	time_pid.running = false;
	pos_pid.running = false;
	pos_pid.hold = false;
	writePWM_static(_motora, _motorb, speed);
}

//TODO: make position readout proper for custom motors
double EVNMotor::getPos()
{
	return getAbsPos_static(&encoder) - _position_offset;
}

void EVNMotor::resetPos()
{
	_position_offset = getAbsPos_static(&encoder);
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

void EVNMotor::runPosition(double rpm, double position, uint8_t stop_action, bool wait)
{
	double degrees = position - this->getPos();
	this->runDegrees(rpm, degrees, stop_action, wait);
}

void EVNMotor::runDegrees(double rpm, double degrees, uint8_t stop_action, bool wait)
{
	uint8_t stop_actionc = min(3, stop_action);
	double rpmc = constrain(rpm, -_maxrpm, _maxrpm);
	double degreesc = degrees;

	if (rpmc < 0)
	{
		degreesc *= -1;
		rpmc *= -1;
	}

	pos_pid.x0 = this->getPos();
	pos_pid.start = millis();
	pos_pid.xdminusx0 = degreesc;
	pos_pid.targetrpm = rpmc;
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
	pos_pid.stop_action = STOP_BRAKE;
	stopAction_static(_motora, _motorb, &pos_pid, &encoder);
}

void EVNMotor::coast()
{
	speed_pid.running = false;
	pos_pid.running = false;
	time_pid.running = false;
	pos_pid.hold = false;
	pos_pid.stop_action = STOP_COAST;
	stopAction_static(_motora, _motorb, &pos_pid, &encoder);
}

void EVNMotor::hold()
{
	if (pos_pid.hold = false)
		pos_pid.x0 = this->getPos();
	pos_pid.start = millis();
	pos_pid.xdminusx0 = 0;
	pos_pid.targetrpm = 0.5 * speed_pid.maxrpm;

	speed_pid.running = false;
	pos_pid.running = false;
	time_pid.running = false;
	pos_pid.stop_action = STOP_HOLD;
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

drivebase_state_t* EVNDrivebase::dbArg;

EVNDrivebase::EVNDrivebase(uint32_t wheel_dia, uint32_t wheel_dist, EVNMotor* motor_left, EVNMotor* motor_right)
{
	db.motor_left = motor_left;
	db.motor_right = motor_right;
	db.maxrpm = min(motor_left->maxrpm, motor_left->maxrpm);
	db.wheel_dist = wheel_dist;
	db.wheel_dia = wheel_dia;
}

void EVNDrivebase::begin()
{
	attach_db_interrupt(&db);
}

void EVNDrivebase::steer(double speed, double turn_rate)
{
	double speedc = constrain(speed, -db.maxrpm, db.maxrpm);
	double turn_ratec = constrain(turn_rate, -1, 1);

	db.running = true;
	db.speed = speedc;
	db.turn_rate = turn_ratec;
}

void EVNDrivebase::brake()
{
	db.running = false;
	db.motor_left->brake();
	db.motor_right->brake();
}

void EVNDrivebase::coast()
{
	db.running = false;
	db.motor_left->coast();
	db.motor_right->coast();
}

void EVNDrivebase::hold()
{
	db.running = false;
	db.motor_left->hold();
	db.motor_right->hold();
}