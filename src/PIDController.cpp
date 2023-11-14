#include <PIDController.h>
#include <Arduino.h>

PIDController::PIDController(double kp, double ki, double kd, uint8_t dir)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;
	_dir = dir;
}

double PIDController::compute(double error)
{
	double errorc = constrain(error, -1, 1);
	_preverror = _error;
	_error = errorc;
	_summederror += errorc;

	_output = _kp * _error + _kd * (_error - _preverror) + _ki * _summederror;
	if (_dir == REVERSE)
	{
		_output *= -1;
	}
	_output = constrain(_output, -1, 1);
	return _output;
}