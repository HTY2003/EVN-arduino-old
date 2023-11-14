#ifndef PIDController_h
#define PIDController_h

#include <Arduino.h>

#define DIRECT 0
#define REVERSE 1

class PIDController
{
public:
	PIDController(double kp, double ki, double kd, uint8_t dir);
	double compute(double error);

private:
	double _kp, _ki, _kd, _dir;
	double _error, _summederror, _preverror, _output;
};

#endif