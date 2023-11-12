#include <EVNMotor.h>
#include <EVNButton.h>
#include <Arduino.h>

encoder_state_t *EVNMotor::encoderArgs[];
speed_pid_t *EVNMotor::speedArgs[];
position_pid_t *EVNMotor::posArgs[];
time_pid_t *EVNMotor::timeArgs[];
RPI_PICO_Timer EVNMotor::timer(3);
RPI_PICO_ISR_Timer EVNMotor::ISRtimer;

// caa 051123

EVNMotor::EVNMotor(uint8_t port, uint8_t motortype, EVNButton *button)
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

	_motortype = max(0, min(motortype, 2));

	switch (motortype)
	{
	case EV3_LARGE:
		_maxrpm = EV3_LARGE_MAX_RPM;
		speed_pid.p = (double)SPEED_PID_KP_LARGE;
		speed_pid.i = (double)SPEED_PID_KI_LARGE;
		speed_pid.d = (double)SPEED_PID_KD_LARGE;
		pos_pid.p = (double)POS_PID_KP_LARGE;
		pos_pid.i = (double)POS_PID_KI_LARGE;
		pos_pid.d = (double)POS_PID_KD_LARGE;
		break;
	case EV3_MED:
		_maxrpm = EV3_MED_MAX_RPM;
		speed_pid.p = (double)SPEED_PID_KP_MED;
		speed_pid.i = (double)SPEED_PID_KI_MED;
		speed_pid.d = (double)SPEED_PID_KD_MED;
		pos_pid.p = (double)POS_PID_KP_MED;
		pos_pid.i = (double)POS_PID_KI_MED;
		pos_pid.d = (double)POS_PID_KD_MED;
		break;
	case CUSTOM:
		_maxrpm = CUSTOM_MAX_RPM;
		speed_pid.p = (double)SPEED_PID_KP_CUSTOM;
		speed_pid.i = (double)SPEED_PID_KI_CUSTOM;
		speed_pid.d = (double)SPEED_PID_KD_CUSTOM;
		pos_pid.p = (double)POS_PID_KP_CUSTOM;
		pos_pid.i = (double)POS_PID_KI_CUSTOM;
		pos_pid.d = (double)POS_PID_KD_CUSTOM;
		break;
	}
	maxrpm = _maxrpm;
}

void EVNMotor::init()
{
	analogWriteFreq(OUTPUTPWMFREQ);
	pinMode(_motora, OUTPUT_8MA);
	pinMode(_motorb, OUTPUT_8MA);

	pinMode(_enca, INPUT);
	pinMode(_encb, INPUT);
	encoder.enca = _enca;
	encoder.encb = _encb;

	attach_enc_interrupt(_enca, &encoder);
	attach_enc_interrupt(_encb, &encoder);

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

void EVNMotor::writePWM(uint8_t speed)
{
	uint8_t speedc = min(PWM_MAX_VAL, max(0, speed));

	speed_pid.running = false;
	time_pid.running = false;
	pos_pid.running = false;
	pos_pid.hold = false;

	if (speedc >= 0)
	{
		if (speedc == 0)
		{
			digitalWrite(_motora, LOW);
		}
		else
		{
			analogWrite(_motora, speedc);
		}
		digitalWrite(_motorb, LOW);
	}
	else
	{
		analogWrite(_motorb, -speedc);
		digitalWrite(_motora, LOW);
	}
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
	uint64_t currenttime = micros();
	double lastpulsetiming = (encoder.dataReady) ? (currenttime - encoder.pulsetimes[encoder.lastpulseindex]) : (currenttime - encoder.lastpulsetime);
	if (lastpulsetiming > encoder.avgpulsetiming)
	{
		encoder.avgpulsetiming = (double)lastpulsetiming;
		encoder.rpm = (1000000 / (encoder.avgpulsetiming * 3)) * encoder.dir;
	}
	return encoder.rpm;
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
	double rpmc = min(_maxrpm, max(-_maxrpm, rpm));

	if (!speed_pid.running || speed_pid.targetrpm != rpmc)
	{
		lastcommand_ms = millis();
	}
	speed_pid.running = true;
	pos_pid.running = false;
	time_pid.running = false;
	pos_pid.hold = false;

	speed_pid.targetrpm = rpmc;
}

void EVNMotor::runDegrees(double degrees, uint8_t stop_action, bool wait)
{
	uint8_t stop_actionc = min(3, stop_action);

	pos_pid.targetpos = this->getPos() + degrees;
	pos_pid.error = 0;
	pos_pid.preverror = 0;
	pos_pid.summederror = 0;
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
	double rpmc = min(_maxrpm, max(-_maxrpm, rpm));
	uint8_t stop_actionc = min(3, stop_action);
	uint32_t time_msc = time_ms;

	time_pid.targetrpm = rpmc;
	time_pid.starttime = millis();
	time_pid.time_ms = time_msc;
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

void EVNMotor::coast()
{
	speed_pid.running = false;
	pos_pid.running = false;
	time_pid.running = false;
	pos_pid.hold = false;
	digitalWrite(_motora, HIGH);
	digitalWrite(_motorb, HIGH);
}

void EVNMotor::brake()
{
	speed_pid.running = false;
	pos_pid.running = false;
	time_pid.running = false;
	pos_pid.hold = false;
	digitalWrite(_motora, LOW);
	digitalWrite(_motorb, LOW);
}

void EVNMotor::hold()
{
	speed_pid.running = false;
	pos_pid.running = false;
	time_pid.running = false;
	pos_pid.targetpos = (double)encoder.position / 2;
	pos_pid.hold = true;
}

bool EVNMotor::commandFinished()
{
	return (!time_pid.running && !pos_pid.running);
}

uint64_t EVNMotor::timeSinceLastCommand()
{
	return millis() - lastcommand_ms;
}

EVNDrivebase::EVNDrivebase(EVNMotor *motora, EVNMotor *motorb, uint8_t motora_inv, uint8_t motorb_inv)
{
	_motora = motora;
	_motorb = motorb;
	_maxrpm = min(motora->maxrpm, motorb->maxrpm);
	_motora_inv = motora_inv;
	_motorb_inv = motorb_inv;
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

	if (_motora_inv == REVERSE)
		leftspeed *= -1;
	if (_motorb_inv == REVERSE)
		rightspeed *= -1;

	_motora->runSpeed(leftspeed);
	_motorb->runSpeed(rightspeed);
}

void EVNDrivebase::brake()
{
	_motora->brake();
	_motorb->brake();
}

void EVNDrivebase::coast()
{
	_motora->coast();
	_motorb->coast();
}

void EVNDrivebase::hold()
{
	_motora->hold();
	_motorb->hold();
}