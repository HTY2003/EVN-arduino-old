#include "EVNMotor.h"

encoder_state_t* EVNMotor::encoderArgs[];
pid_control_t* EVNMotor::pidArgs[];
bool EVNMotor::ports_started[] = { false, false, false, false };

EVNMotor::EVNMotor(uint8_t port, uint8_t motortype, uint8_t motor_dir, uint8_t enc_dir)
{
	// clean inputs
	uint8_t motor_dirc = constrain(motor_dir, 0, 1);
	uint8_t enc_dirc = constrain(enc_dir, 0, 1);
	uint8_t portc = constrain(port, 1, 4);
	uint8_t motortypec = constrain(motortype, 0, 3);

	// set pins
	switch (port)
	{
	case 1:
		_pid_control.motora = OUTPUT1MOTORA;
		_pid_control.motorb = OUTPUT1MOTORB;
		_encoder.enca = OUTPUT1ENCA;
		_encoder.encb = OUTPUT1ENCB;
		break;
	case 2:
		_pid_control.motora = OUTPUT2MOTORA;
		_pid_control.motorb = OUTPUT2MOTORB;
		_encoder.enca = OUTPUT2ENCA;
		_encoder.encb = OUTPUT2ENCB;
		break;
	case 3:
		_pid_control.motora = OUTPUT3MOTORA;
		_pid_control.motorb = OUTPUT3MOTORB;
		_encoder.enca = OUTPUT3ENCA;
		_encoder.encb = OUTPUT3ENCB;
		break;
	case 4:
		_pid_control.motora = OUTPUT4MOTORA;
		_pid_control.motorb = OUTPUT4MOTORB;
		_encoder.enca = OUTPUT4ENCA;
		_encoder.encb = OUTPUT4ENCB;
		break;
	}

	// swap pins if needed
	uint8_t pin;
	if (motor_dirc == REVERSE)
	{
		pin = _pid_control.motora;
		_pid_control.motora = _pid_control.motorb;
		_pid_control.motorb = pin;
		pin = _encoder.enca;
		_encoder.enca = _encoder.encb;
		_encoder.encb = pin;
	}

	if (enc_dirc == REVERSE)
	{
		pin = _encoder.enca;
		_encoder.enca = _encoder.encb;
		_encoder.encb = pin;
	}

	// set parameters for speed & position control
	switch (motortypec)
	{
	case EV3_LARGE:
		_pid_control.max_rpm = EV3_LARGE_MAX_RPM;
		_pid_control.accel = EV3_LARGE_ACCEL;
		_pid_control.decel = EV3_LARGE_DECEL;
		_pid_control.speed_pid = new PIDController(SPEED_PID_KP_EV3_LARGE, SPEED_PID_KI_EV3_LARGE, SPEED_PID_KD_EV3_LARGE, DIRECT);
		_pid_control.pos_pid = new PIDController(POS_PID_KP_EV3_LARGE, POS_PID_KI_EV3_LARGE, POS_PID_KD_EV3_LARGE, DIRECT);
		_encoder.ppr = LEGO_PPR;
		break;
	case NXT_LARGE:
		_pid_control.max_rpm = NXT_LARGE_MAX_RPM;
		_pid_control.accel = NXT_LARGE_ACCEL;
		_pid_control.decel = NXT_LARGE_DECEL;
		_pid_control.speed_pid = new PIDController(SPEED_PID_KP_NXT_LARGE, SPEED_PID_KI_NXT_LARGE, SPEED_PID_KD_NXT_LARGE, DIRECT);
		_pid_control.pos_pid = new PIDController(POS_PID_KP_NXT_LARGE, POS_PID_KI_NXT_LARGE, POS_PID_KD_NXT_LARGE, DIRECT);
		_encoder.ppr = LEGO_PPR;
		break;
	case EV3_MED:
		_pid_control.max_rpm = EV3_MED_MAX_RPM;
		_pid_control.accel = EV3_MED_ACCEL;
		_pid_control.decel = EV3_MED_DECEL;
		_pid_control.speed_pid = new PIDController(SPEED_PID_KP_EV3_MED, SPEED_PID_KI_EV3_MED, SPEED_PID_KD_EV3_MED, DIRECT);
		_pid_control.pos_pid = new PIDController(POS_PID_KP_EV3_MED, POS_PID_KI_EV3_MED, POS_PID_KD_EV3_MED, DIRECT);
		_encoder.ppr = LEGO_PPR;
		break;
	case CUSTOM_MOTOR:
		_pid_control.max_rpm = CUSTOM_MOTOR_MAX_RPM;
		_pid_control.accel = CUSTOM_MOTOR_ACCEL;
		_pid_control.decel = CUSTOM_MOTOR_DECEL;
		_pid_control.speed_pid = new PIDController(SPEED_PID_KP_CUSTOM, SPEED_PID_KI_CUSTOM, SPEED_PID_KD_CUSTOM, DIRECT);
		_pid_control.pos_pid = new PIDController(POS_PID_KP_CUSTOM, POS_PID_KI_CUSTOM, POS_PID_KD_CUSTOM, DIRECT);
		_encoder.ppr = CUSTOM_PPR;
		break;
	}

	//set motor not to run (yet)
	_pid_control.hold = false;
	_pid_control.run_speed = false;
	_pid_control.run_time = false;
	_pid_control.run_deg = false;
}

void EVNMotor::begin()
{
	//configure pins
	analogWriteFreq(OUTPUTPWMFREQ);
	analogWriteRange(PWM_MAX_VAL);
	pinMode(_pid_control.motora, OUTPUT_8MA);
	pinMode(_pid_control.motorb, OUTPUT_8MA);
	pinMode(_encoder.enca, INPUT);
	pinMode(_encoder.encb, INPUT);

	//attach pin change interrupts and timer interrupt
	attach_interrupts(&_encoder, &_pid_control);
}

void EVNMotor::setSpeedPID(double p, double i, double d)
{
	_pid_control.speed_pid->setKp(p);
	_pid_control.speed_pid->setKi(i);
	_pid_control.speed_pid->setKd(d);
}
void EVNMotor::setPosPID(double p, double i, double d)
{
	_pid_control.pos_pid->setKp(p);
	_pid_control.pos_pid->setKi(i);
	_pid_control.pos_pid->setKd(d);
}

void EVNMotor::setAccel(double accel_rpm_per_s) { _pid_control.accel = fabs(accel_rpm_per_s); }
void EVNMotor::setDecel(double decel_rpm_per_s) { _pid_control.decel = fabs(decel_rpm_per_s); }
void EVNMotor::setMaxRPM(double max_rpm) { _pid_control.max_rpm = fabs(max_rpm); }
void EVNMotor::setPPR(uint32_t ppr) { _encoder.ppr = ppr; }
double EVNMotor::getPos() { return getAbsPos_static(&_encoder) - _position_offset; }
void EVNMotor::resetPos() { _position_offset = getAbsPos_static(&_encoder); }
double EVNMotor::getRPM() { return getRPM_static(&_encoder); }

void EVNMotor::writePWM(double speed)
{
	_pid_control.run_speed = false;
	_pid_control.run_time = false;
	_pid_control.run_deg = false;
	_pid_control.hold = false;
	writePWM_static(&_pid_control, speed);
}

void EVNMotor::runSpeed(double rpm)
{
	double rpmc = constrain(rpm, -_pid_control.max_rpm, _pid_control.max_rpm);

	_pid_control.target_rpm = rpmc;
	_pid_control.run_speed = true;
	_pid_control.run_deg = false;
	_pid_control.run_time = false;
	_pid_control.hold = false;
}

void EVNMotor::runPosition(double rpm, double position, uint8_t stop_action, bool wait)
{
	double degrees = position - this->getPos();
	this->runDegrees(rpm, degrees, stop_action, wait);
}

void EVNMotor::runDegrees(double rpm, double degrees, uint8_t stop_action, bool wait)
{
	double rpmc = constrain(rpm, -_pid_control.max_rpm, _pid_control.max_rpm);
	double degreesc = degrees;
	uint8_t stop_actionc = min(3, stop_action);

	if (rpmc < 0)
	{
		degreesc *= -1;
		rpmc *= -1;
	}

	_pid_control.stop_action = stop_actionc;
	_pid_control.run_deg_target_rpm = rpmc;
	_pid_control.x0 = this->getPos();
	_pid_control.xdminusx0 = degreesc;

	_pid_control.run_deg = true;
	_pid_control.run_time = false;
	_pid_control.run_speed = false;
	_pid_control.hold = false;

	if (wait) while (!this->commandFinished());
}

void EVNMotor::runTime(double rpm, uint32_t time_ms, uint8_t stop_action, bool wait)
{
	double rpmc = constrain(rpm, -_pid_control.max_rpm, _pid_control.max_rpm);
	uint8_t stop_actionc = min(3, stop_action);

	_pid_control.start_time_us = micros();
	_pid_control.stop_action = stop_actionc;
	_pid_control.target_rpm = rpmc;
	_pid_control.time_ms = time_ms;

	_pid_control.run_time = true;
	_pid_control.run_speed = false;
	_pid_control.run_deg = false;
	_pid_control.hold = false;

	if (wait) while (!this->commandFinished());
}

void EVNMotor::brake()
{
	_pid_control.stop_action = STOP_BRAKE;
	stopAction_static(&_pid_control, &_encoder);
}

void EVNMotor::coast()
{
	_pid_control.stop_action = STOP_COAST;
	stopAction_static(&_pid_control, &_encoder);
}

void EVNMotor::hold()
{
	_pid_control.stop_action = STOP_HOLD;
	stopAction_static(&_pid_control, &_encoder);
}

bool EVNMotor::commandFinished()
{
	return (!_pid_control.run_time && !_pid_control.run_deg);
}