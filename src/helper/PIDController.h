#ifndef PIDController_h
#define PIDController_h

#include <Arduino.h>

#define DIRECT	1
#define REVERSE	0

class PIDController
{
public:
	PIDController(float kp, float ki, float kd, uint8_t dir);
	void setKp(float kp);
	float getKp();
	void setKi(float ki);
	float getKi();
	void setKd(float kd);
	float getKd();
	float compute(float error, bool constrain_integral = false, bool constrain_input = false, bool constrain_output = false);
	void constrainIntegral(float low, float high);
	void resetIntegral();
	float getIntegral();
	void reset();

private:
	float _kp, _ki, _kd, _dir;
	float _error, _summederror, _preverror, _output;
};

#endif