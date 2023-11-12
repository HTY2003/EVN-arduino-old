#ifndef EVNPIDController_h
#define EVNPIDController_h

#include <Arduino.h>

#define DIRECT 0
#define REVERSE 1

class EVNPIDController
{
public:
	EVNPIDController(double kp, double ki, double kd, uint8_t dir);
	double compute(double error);

private:
	double _kp, _ki, _kd, _dir;
	double _error, _summederror, _preverror, _output;
};

#endif