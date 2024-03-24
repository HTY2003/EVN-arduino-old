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
	switch (portc)
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

	// configure settings according to motor type
	switch (motortypec)
	{
	case EV3_LARGE:
		_pid_control.max_rpm = EV3_LARGE_MAX_RPM;
		_pid_control.accel = EV3_LARGE_ACCEL;
		_pid_control.decel = EV3_LARGE_DECEL;
		_pid_control.pos_pid = new PIDController(KP_EV3_LARGE, KI_EV3_LARGE, KD_MAX_EV3_LARGE, DIRECT);
		_encoder.ppr = LEGO_PPR;
		break;
	case NXT_LARGE:
		_pid_control.max_rpm = NXT_LARGE_MAX_RPM;
		_pid_control.accel = NXT_LARGE_ACCEL;
		_pid_control.decel = NXT_LARGE_DECEL;
		_pid_control.pos_pid = new PIDController(KP_NXT_LARGE, KI_NXT_LARGE, KD_MAX_NXT_LARGE, DIRECT);
		_encoder.ppr = LEGO_PPR;
		break;
	case EV3_MED:
		_pid_control.max_rpm = EV3_MED_MAX_RPM;
		_pid_control.accel = EV3_MED_ACCEL;
		_pid_control.decel = EV3_MED_DECEL;
		_pid_control.pos_pid = new PIDController(KP_EV3_MED, KI_EV3_MED, KD_MAX_EV3_MED, DIRECT);
		_encoder.ppr = LEGO_PPR;
		break;
	case CUSTOM_MOTOR:
		_pid_control.max_rpm = CUSTOM_MOTOR_MAX_RPM;
		_pid_control.accel = CUSTOM_MOTOR_ACCEL;
		_pid_control.decel = CUSTOM_MOTOR_DECEL;
		_pid_control.pos_pid = new PIDController(KP_CUSTOM, KI_CUSTOM, KD_MAX_CUSTOM, DIRECT);
		_encoder.ppr = CUSTOM_PPR;
		break;
	}

	//make sure motor does not run
	_pid_control.run_pwm = true;
	_pid_control.hold = false;
	_pid_control.run_speed = false;
	_pid_control.run_time = false;
	_pid_control.run_pos = false;
	_encoder.position = 0;
	_encoder.position_offset = 0;
}

void EVNMotor::begin()
{
	//configure pins
	analogWriteFreq(PWM_FREQ);
	analogWriteRange(PWM_MAX_VAL);
	pinMode(_pid_control.motora, OUTPUT_8MA);
	pinMode(_pid_control.motorb, OUTPUT_8MA);
	pinMode(_encoder.enca, INPUT);
	pinMode(_encoder.encb, INPUT);

	//attach pin change interrupts (encoder) and timer interrupt (PID control)
	attach_interrupts(&_encoder, &_pid_control);
}

void EVNMotor::setPID(double p, double i, double d)
{
	_pid_control.pos_pid->setKp(p);
	_pid_control.pos_pid->setKi(i);
	_pid_control.pos_pid->setKd(d);
}

void EVNMotor::setAccel(double accel_dps_sq)
{
	_pid_control.accel = fabs(accel_dps_sq);
}

void EVNMotor::setDecel(double decel_dps_sq)
{
	_pid_control.decel = fabs(decel_dps_sq);
}

void EVNMotor::setMaxRPM(double max_rpm)
{
	_pid_control.max_rpm = fabs(max_rpm);
}

void EVNMotor::setPPR(uint32_t ppr)
{
	_encoder.ppr = ppr;
}

double EVNMotor::getPosition()
{
	return getPosition_static(&_encoder);
}

double EVNMotor::getHeading()
{
	return fmod(fmod(getPosition_static(&_encoder), 360) + 360, 360);
}

void EVNMotor::resetPosition()
{
	_encoder.position_offset = getPosition_static(&_encoder);
}

double EVNMotor::getDPS()
{
	return getDPS_static(&_encoder);
}

void EVNMotor::runPWM(double duty_cycle)
{
	_pid_control.run_pwm = true;
	_pid_control.run_speed = false;
	_pid_control.run_time = false;
	_pid_control.run_pos = false;
	_pid_control.hold = false;
	runPWM_static(&_pid_control, duty_cycle);
}

double EVNMotor::clean_input_dps(double dps)
{
	return min(fabs(dps), _pid_control.max_rpm * 6);
}

uint8_t EVNMotor::clean_input_dir(double dps)
{
	return (dps >= 0) ? DIRECT : REVERSE;
}

uint8_t EVNMotor::clean_input_stop_action(uint8_t stop_action)
{
	return min(2, stop_action);
}

void EVNMotor::runDPS(double dps)
{
	_pid_control.target_dps = clean_input_dps(dps);
	_pid_control.run_dir = clean_input_dir(dps);

	_pid_control.run_pwm = false;
	_pid_control.run_pos = false;
	_pid_control.run_time = false;
	_pid_control.run_speed = true;
	_pid_control.hold = false;
}

void EVNMotor::runPosition(double dps, double position, uint8_t stop_action, bool wait)
{
	_pid_control.target_dps = clean_input_dps(dps);
	_pid_control.target_pos = position;
	_pid_control.stop_action = clean_input_stop_action(stop_action);

	_pid_control.run_pwm = false;
	_pid_control.run_pos = true;
	_pid_control.run_time = false;
	_pid_control.run_speed = false;
	_pid_control.hold = false;

	if (wait) while (!this->completed());
}

void EVNMotor::runAngle(double dps, double degrees, uint8_t stop_action, bool wait)
{
	double degreesc = degrees;
	if (dps < 0 && degreesc > 0)
		degreesc = -degreesc;

	this->runPosition(dps, this->getPosition() + degreesc, stop_action, wait);
}

void EVNMotor::runHeading(double dps, double heading, uint8_t stop_action, bool wait)
{
	double target_heading = constrain(heading, 0, 360);
	double current_heading = this->getHeading();
	double degreesc = target_heading - current_heading;
	if (degreesc > 180)
		degreesc -= 360;
	if (degreesc < -180)
		degreesc += 360;

	this->runPosition(dps, this->getPosition() + degreesc, stop_action, wait);
}

void EVNMotor::runTime(double dps, uint32_t time_ms, uint8_t stop_action, bool wait)
{
	_pid_control.target_dps = clean_input_dps(dps);
	_pid_control.run_dir = clean_input_dir(dps);
	_pid_control.run_time_ms = time_ms;
	_pid_control.stop_action = clean_input_stop_action(stop_action);

	_pid_control.run_pwm = false;
	_pid_control.run_pos = false;
	_pid_control.run_time = true;
	_pid_control.run_speed = false;
	_pid_control.hold = false;

	if (wait) while (!this->completed());
}

void EVNMotor::brake()
{
	_pid_control.stop_action = STOP_BRAKE;
	stopAction_static(&_pid_control, &_encoder, micros());
}

void EVNMotor::coast()
{
	_pid_control.stop_action = STOP_COAST;
	stopAction_static(&_pid_control, &_encoder, micros());
}

void EVNMotor::hold()
{
	_pid_control.stop_action = STOP_HOLD;
	stopAction_static(&_pid_control, &_encoder, micros());
}

bool EVNMotor::completed()
{
	return (!_pid_control.run_time && !_pid_control.run_pos);
}

bool EVNMotor::stalled()
{
	return _pid_control.stalled && loop_control_enabled(&_pid_control);
}

drivebase_state_t* EVNDrivebase::dbArgs[];
bool EVNDrivebase::dbs_enabled[] = { false, false };

EVNDrivebase::EVNDrivebase(double wheel_dia, double axle_track, EVNMotor* motor_left, EVNMotor* motor_right)
{
	db.motor_left = motor_left;
	db.motor_right = motor_right;
	db.max_rpm = min(motor_left->_pid_control.max_rpm, motor_right->_pid_control.max_rpm);

	db.turn_rate_pid = new PIDController(DRIVEBASE_KP_TURN_RATE, DRIVEBASE_KI_TURN_RATE, DRIVEBASE_KD_TURN_RATE, DIRECT);
	db.speed_pid = new PIDController(DRIVEBASE_KP_SPEED, DRIVEBASE_KI_SPEED, DRIVEBASE_KD_SPEED, DIRECT);

	db.speed_accel = fabs(USER_SPEED_ACCEL);
	db.speed_decel = fabs(USER_SPEED_DECEL);
	db.turn_rate_accel = fabs(USER_TURN_RATE_ACCEL);
	db.turn_rate_decel = fabs(USER_TURN_RATE_DECEL);

	db.axle_track = fabs(axle_track);
	db.wheel_dia = fabs(wheel_dia);
	db.drive = false;
	db.drive_position = false;

	db.max_speed = db.max_rpm / 60 * db.wheel_dia * M_PI;
	db.max_turn_rate = db.max_rpm * 6 * db.wheel_dia / axle_track;
	db.max_dps = db.max_rpm * 6;
}

void EVNDrivebase::setSpeedPID(double kp, double ki, double kd)
{
	db.speed_pid->setKp(kp);
	db.speed_pid->setKi(ki);
	db.speed_pid->setKd(kd);
}

void EVNDrivebase::setTurnRatePID(double kp, double ki, double kd)
{
	db.turn_rate_pid->setKp(kp);
	db.turn_rate_pid->setKi(ki);
	db.turn_rate_pid->setKd(kd);
}

void EVNDrivebase::setSpeedAccel(double speed_accel)
{
	db.speed_accel = fabs(speed_accel);
}

void EVNDrivebase::setSpeedDecel(double speed_decel)
{
	db.speed_decel = fabs(speed_decel);
}

void EVNDrivebase::setTurnRateAccel(double turn_rate_accel)
{
	db.turn_rate_accel = fabs(turn_rate_accel);
}

void EVNDrivebase::setTurnRateDecel(double turn_rate_decel)
{
	db.turn_rate_decel = fabs(turn_rate_decel);
}

void EVNDrivebase::begin()
{
	attach_db_interrupt(&db);
}

double EVNDrivebase::getDistance()
{
	return getDistance_static(&db);
}

double EVNDrivebase::getAngle()
{
	return getAngle_static(&db);
}

double EVNDrivebase::getHeading()
{
	return fmod(fmod(getAngle_static(&db), 360) + 360, 360);
}

double EVNDrivebase::getX()
{
	return db.position_x;
}

double EVNDrivebase::getY()
{
	return db.position_y;
}

void EVNDrivebase::resetXY()
{
	db.position_x = 0;
	db.position_y = 0;
}

double EVNDrivebase::getDistanceToPoint(double x, double y)
{
	if (db.position_x == 0 && db.position_y == 0)
		return 0;
	return sqrt(pow(db.position_x - x, 2) + pow(db.position_y - y, 2));
}

double EVNDrivebase::clean_input_turn_rate(double turn_rate)
{
	return constrain(turn_rate, -db.max_turn_rate, db.max_turn_rate);
}

double EVNDrivebase::clean_input_speed(double speed, double turn_rate)
{
	double scale = 1 - fabs(turn_rate) / db.max_turn_rate;
	return constrain(speed, -scale * db.max_speed, scale * db.max_speed);
}

uint8_t EVNDrivebase::clean_input_stop_action(uint8_t stop_action)
{
	return min(2, stop_action);
}

double EVNDrivebase::scaling_factor_for_maintaining_radius(double speed, double turn_rate)
{
	double w1 = fabs(turn_rate / db.max_turn_rate);
	double w2 = fabs(speed / db.max_speed);

	if (w1 + w2 > 1)
		return 1 / (w1 + w2);

	return 1;
}

double EVNDrivebase::radius_to_turn_rate(double speed, double radius)
{
	double radiusc = max(0, radius);
	double turn_rate;
	if (radiusc == 0)
	{
		turn_rate = db.max_turn_rate;
	}
	else
		turn_rate = fabs(speed / M_PI / radiusc * 180);
	return turn_rate;
}


void EVNDrivebase::enable_drive_position(uint8_t stop_action, bool wait)
{
	db.stop_action = clean_input_stop_action(stop_action);
	db.drive = true;
	db.drive_position = true;
	if (wait) while (!this->completed());
}

void EVNDrivebase::drive(double speed, double turn_rate)
{
	this->driveTurnRate(speed, turn_rate);
}

void EVNDrivebase::driveTurnRate(double speed, double turn_rate)
{
	double turn_ratec = clean_input_turn_rate(turn_rate);
	double speedc = clean_input_speed(speed, turn_ratec);

	db.target_speed = speedc;
	db.target_turn_rate = turn_ratec;
	db.drive = true;
}

void EVNDrivebase::driveRadius(double speed, double radius)
{
	double speedc = speed;
	double radiusc = max(0, radius);
	double turn_ratec = radius_to_turn_rate(speed, radius);
	if (radiusc == 0)
		speedc = 0;

	double scale = scaling_factor_for_maintaining_radius(speedc, turn_ratec);
	turn_ratec *= scale;
	speedc *= scale;

	this->driveTurnRate(speedc, turn_ratec);
}

void EVNDrivebase::straight(double speed, double distance, uint8_t stop_action, bool wait)
{
	double speedc = clean_input_speed(speed, 0);
	double distancec = distance;

	if (distancec < 0 && speedc > 0)
		speedc = -speedc;

	if (distancec > 0 && speedc < 0)
		distancec = -distancec;

	db.target_speed = speedc;
	db.target_turn_rate = 0;
	db.end_distance = db.current_distance + distancec;
	db.end_angle = db.current_angle;

	enable_drive_position(stop_action, wait);
}

void EVNDrivebase::curve(double speed, double radius, double angle, uint8_t stop_action, bool wait)
{
	this->curveRadius(speed, radius, angle, stop_action, wait);
}

void EVNDrivebase::curveRadius(double speed, double radius, double angle, uint8_t stop_action, bool wait)
{
	double speedc = speed;
	double radiusc = max(0, radius);
	double turn_ratec = radius_to_turn_rate(speedc, radiusc);
	if (radiusc == 0)
		speedc = 0;

	double scale = scaling_factor_for_maintaining_radius(speedc, turn_ratec);
	turn_ratec *= scale;
	speedc *= scale;

	this->curveTurnRate(speedc, turn_ratec, angle, stop_action, wait);
}

void EVNDrivebase::curveTurnRate(double speed, double turn_rate, double angle, uint8_t stop_action, bool wait)
{
	double turn_ratec = clean_input_turn_rate(turn_rate);
	double speedc = clean_input_speed(speed, turn_ratec);

	if ((angle < 0 && turn_ratec > 0) || (angle > 0 && turn_ratec < 0))
		turn_ratec = -turn_ratec;

	double distance = angle / turn_ratec * speedc;

	db.target_speed = speedc;
	db.target_turn_rate = turn_ratec;
	db.end_distance = db.current_distance + distance;
	db.end_angle = db.current_angle + angle;

	enable_drive_position(stop_action, wait);
}

void EVNDrivebase::turn(double turn_rate, double degrees, uint8_t stop_action, bool wait)
{
	this->turnDegrees(turn_rate, degrees, stop_action, wait);
}

void EVNDrivebase::turnDegrees(double turn_rate, double degrees, uint8_t stop_action, bool wait)
{
	this->curveTurnRate(0, turn_rate, degrees, stop_action, wait);
}

void EVNDrivebase::turnHeading(double turn_rate, double heading, uint8_t stop_action, bool wait)
{
	double target_heading = constrain(heading, 0, 360);
	double current_heading = fmod(fmod(db.current_angle, 360) + 360, 360);
	double turn_angle = heading - current_heading;
	if (turn_angle > 180)
		turn_angle -= 360;
	if (turn_angle < -180)
		turn_angle += 360;

	this->turnDegrees(turn_rate, turn_angle, stop_action, wait);
}

void EVNDrivebase::driveToXY(double speed, double turn_rate, double x, double y, uint8_t stop_action, bool restore_initial_heading)
{
	double angle_to_target = atan2(y - db.position_y, x - db.position_x);
	angle_to_target = angle_to_target / M_PI * 180;
	if (angle_to_target < 0)
		angle_to_target += 360;

	double initial_heading = fmod(fmod(db.current_angle, 360) + 360, 360);
	this->turnHeading(turn_rate, angle_to_target, stop_action, true);
	this->straight(speed, this->getDistanceToPoint(x, y), stop_action, true);
	if (restore_initial_heading)
		this->turnHeading(turn_rate, initial_heading, stop_action, true);
}

void EVNDrivebase::stop()
{
	this->brake();
}

void EVNDrivebase::brake()
{
	db.stop_action = STOP_BRAKE;
	stopAction_static(&db);
}

void EVNDrivebase::coast()
{
	db.stop_action = STOP_COAST;
	stopAction_static(&db);
}

void EVNDrivebase::hold()
{
	db.stop_action = STOP_HOLD;
	stopAction_static(&db);
}

bool EVNDrivebase::completed()
{
	return !db.drive_position;
}