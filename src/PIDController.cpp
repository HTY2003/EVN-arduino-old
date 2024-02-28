#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd, uint8_t dir)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;
	_dir = dir;
}

void PIDController::setKp(double kp) { _kp = kp; }
double PIDController::getKp() { return _kp; }
void PIDController::setKi(double ki) { _ki = ki; }
double PIDController::getKi() { return _ki; }
void PIDController::setKd(double kd) { _kd = kd; }
double PIDController::getKd() { return _kd; }

double PIDController::compute(double error, bool constrain_integral, bool constrain_input, bool constrain_output)
{
	double errorc = error;
	if (constrain_input)
		errorc = constrain(errorc, -1, 1);

	//calculate output value
	_preverror = _error;
	_error = errorc;
	_summederror += errorc;

	if (constrain_integral)
		_summederror = constrain(_summederror, -1 / _ki, 1 / _ki);

	_output = _kp * _error + _kd * (_error - _preverror) + _ki * _summederror;

	if (constrain_output)
		_output = constrain(_output, -1, 1);

	//flip value if needed
	if (_dir == REVERSE)
		_output *= -1;

	return _output;
}

void PIDController::reset()
{
	_error = 0;
	_preverror = 0;
	_summederror = 0;
}

void PIDController::constrainIntegral(double low, double high)
{
	_summederror = constrain(_summederror, low, high);
}

void PIDController::resetIntegral()
{
	_summederror = 0;
}