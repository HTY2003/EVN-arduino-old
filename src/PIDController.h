#ifndef PIDController_h
#define PIDController_h

#include <Arduino.h>

#define DIRECT	0
#define REVERSE	1

class PIDController
{
public:
	PIDController(double kp, double ki, double kd, uint8_t dir);
	void setKp(double kp);
	double getKp();
	void setKi(double ki);
	double getKi();
	void setKd(double kd);
	double getKd();
	double compute(double error);
	void reset();

private:
	double _kp, _ki, _kd, _dir;
	double _error, _summederror, _preverror, _output;
};

#endif