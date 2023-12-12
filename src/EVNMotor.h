#ifndef EVNMotor_h
#define EVNMotor_h

#include <Arduino.h>
#include "EVNAlpha.h"
#include "EVNISRTimer.h"
#include "RPi_Pico_TimerInterrupt.h"
#include "RPi_Pico_ISR_Timer.hpp"
#include "PIDController.h"
#include "pins_evn_alpha.h"
#include "evn_motor_pid.h"
#include <cfloat>

//INPUT PARAMETER OPTIONS
#define DIRECT 0
#define REVERSE 1
#define EV3_LARGE 0
#define NXT_LARGE 1
#define EV3_MED 2
#define CUSTOM_MOTOR 3

#define STOP_BRAKE 0
#define STOP_COAST 1
#define STOP_HOLD 2

//HARDWARE STUFF
#define OUTPUTPWMFREQ 20000
#define PWM_MAX_VAL 255
#define PID_TIMER_INTERVAL_MS 2
#define NO_OF_PULSES_TIMED 2
#define ENCODER_PULSE_TIMEOUT 166667*2

typedef struct
{
	volatile uint8_t enca;
	volatile uint8_t encb;
	volatile uint8_t state;
	volatile double position; // if any ISR writes a value to it, use volatile
	volatile uint64_t pulsetimes[NO_OF_PULSES_TIMED];
	volatile uint8_t lastpulseindex;
	volatile double avgpulsetiming;
	volatile double ppr;
	volatile double rpm;
	volatile int8_t dir = 1;
} encoder_state_t;

typedef struct
{
	volatile bool running;
	volatile double maxrpm;
	uint8_t motora;
	uint8_t motorb;
	volatile double targetrpm;
	volatile double error;
	volatile double output;
	PIDController* pid;
} speed_pid_t;

typedef struct
{
	volatile uint32_t counter;
	volatile bool running;
	volatile bool hold;
	volatile uint8_t stop_action;
	volatile double targetrpm;

	volatile double x0;
	volatile double xdminusx0;
	volatile uint64_t start;
	volatile double T;

	volatile double targetpos;
	volatile double error;
	volatile double output;
	volatile bool flip;
	PIDController* pid;
} position_pid_t;

typedef struct
{
	volatile bool running;
	volatile uint64_t starttime;
	volatile uint64_t time_ms;
	volatile double targetrpm;
} time_pid_t;

class EVNMotor
{
public:
	EVNMotor(uint8_t port, uint8_t motortype = EV3_LARGE, uint8_t motor_dir = DIRECT, uint8_t enc_dir = DIRECT);
	void begin();
	void writePWM(double speed); // Directly write speed (-1 to 1) as PWM value to motor
	double getPos();			  // Return absolute position in degrees
	void resetPos();			  // Set current position as 0 deg
	double getRPM();			  // Return velocity in RPM rotations per minute

	bool commandFinished();
	uint64_t timeSinceLastCommand();
	double debugSpeedPID();
	double debugPositionPID();

	void runSpeed(double rpm);																	// Set motor to run at desired RPM
	void runDegrees(double rpm, double degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true); // Set motor to run desired angular displacement (in degrees)
	void runTime(double rpm, uint32_t time_ms, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void brake();
	void coast();
	void hold();

private:
	bool _started = false;
	uint8_t _motora, _motorb, _enca, _encb, _motortype, _maxrpm;
	uint32_t lastcommand_ms;
	speed_pid_t speed_pid;
	position_pid_t pos_pid;
	time_pid_t time_pid;

public:
	encoder_state_t encoder;
	uint8_t maxrpm;
	static encoder_state_t* encoderArgs[4]; // static list of pointers to each instances' structs
	static speed_pid_t* speedArgs[4];
	static position_pid_t* posArgs[4];
	static time_pid_t* timeArgs[4];

	static void rpm_update(encoder_state_t* arg)
	{
		uint8_t new_p0 = digitalRead(arg->enca);

		if (new_p0)
		{
			uint64_t currentpulsetime = micros();
			arg->lastpulseindex++;
			arg->lastpulseindex %= NO_OF_PULSES_TIMED;
			arg->pulsetimes[arg->lastpulseindex] = currentpulsetime;

			uint64_t pulsetimings = 0;
			for (uint8_t i = 0; i < (NO_OF_PULSES_TIMED - 1); i++)
			{
				uint8_t endindex = (arg->lastpulseindex - i + NO_OF_PULSES_TIMED) % NO_OF_PULSES_TIMED;
				uint8_t startindex = (endindex - 1 + NO_OF_PULSES_TIMED) % NO_OF_PULSES_TIMED;
				uint64_t start = arg->pulsetimes[startindex];
				uint64_t end = arg->pulsetimes[endindex];
				pulsetimings += end - start;
			}
			arg->avgpulsetiming = pulsetimings / (NO_OF_PULSES_TIMED - 1);
			arg->rpm = (1000000 / arg->avgpulsetiming) / (arg->ppr / 2) * 60 * arg->dir;
		}
	}

	static void update(encoder_state_t* arg)
	{
		uint8_t new_p0 = digitalRead(arg->enca);
		uint8_t new_p1 = digitalRead(arg->encb);
		uint8_t state = arg->state & 3;
		if (new_p0)
			state |= 4;
		if (new_p1)
			state |= 8;
		arg->state = (state >> 2);
		switch (state)
		{
		case 1:
		case 7:
		case 8:
		case 14:
			arg->position++;
			arg->dir = 1;
			return;
		case 2:
		case 4:
		case 11:
		case 13:
			arg->position--;
			arg->dir = -1;
			return;
		case 3:
		case 12:
			arg->position += 2;
			arg->dir = 1;
			return;
		case 6:
		case 9:
			arg->position -= 2;
			arg->dir = -1;
			return;
		}
	}

	static void writePWM_static(uint8_t motora, uint8_t motorb, double speed)
	{
		double speedc = constrain(speed, -1, 1);
		if (speedc >= 0)
		{
			if (speedc == 0)
				digitalWrite(motora, LOW);
			else
				analogWrite(motora, speedc * PWM_MAX_VAL);
			digitalWrite(motorb, LOW);
		}
		else
		{
			analogWrite(motorb, -speedc * PWM_MAX_VAL);
			digitalWrite(motora, LOW);
		}
	}

	static void stopAction_static(uint8_t motora, uint8_t motorb, position_pid_t* posArg, encoder_state_t* encoderArg)
	{
		switch (posArg->stop_action)
		{
		case STOP_BRAKE:
			digitalWrite(motora, LOW);
			digitalWrite(motorb, LOW);
			break;
		case STOP_COAST:
			digitalWrite(motora, HIGH);
			digitalWrite(motorb, HIGH);
			break;
		case STOP_HOLD:
			digitalWrite(motora, HIGH);
			digitalWrite(motorb, HIGH);
			posArg->hold = true;
			break;
		}
	}

	static double getRPM_static(encoder_state_t* encoderArg)
	{
		uint64_t currenttime = micros();
		uint64_t lastpulsetiming = currenttime - encoderArg->pulsetimes[encoderArg->lastpulseindex];
		if (lastpulsetiming > ENCODER_PULSE_TIMEOUT)
		{
			return 0;
		}
		else if (lastpulsetiming > encoderArg->avgpulsetiming)
		{
			// double avgpulsetiming = (encoderArg->avgpulsetiming * NO_OF_PULSES_TIMED + lastpulsetiming) / (NO_OF_PULSES_TIMED + 1);
			double avgpulsetiming = lastpulsetiming;
			double rpm = (1000000 / avgpulsetiming) / (encoderArg->ppr / 2) * 60 * encoderArg->dir;
			return rpm;
		}
		else
		{
			return encoderArg->rpm;
		}
	}

	static bool pid_update(speed_pid_t* speedArg, position_pid_t* posArg, time_pid_t* timeArg, encoder_state_t* encoderArg)
	{
		posArg->flip = !posArg->flip;

		bool buttonread, buttonlink;

		buttonread = EVNAlpha::sharedButton().read();
		buttonlink = EVNAlpha::sharedButton().button.linkMotors;

		if (!buttonlink) buttonread = true;

		if (buttonread)
		{
			double rpm = getRPM_static(encoderArg);

			if ((posArg->running || posArg->hold) && posArg->flip)
			{
				if (posArg->running && (fabs(posArg->x0 + posArg->xdminusx0 - ((double)encoderArg->position) / 2) <= 0.5))
				{
					stopAction_static(speedArg->motora, speedArg->motorb, posArg, encoderArg);
					posArg->running = false;
					posArg->pid->reset();
					return true;
				}

				uint64_t now = millis();
				double T = fabs(posArg->xdminusx0) / (posArg->targetrpm / 60 * 360);
				if (T == 0) T = FLT_MIN;
				double t = (((double)now - (double)posArg->start) / 1000.0);

				t = constrain(t, 0, T);
				posArg->targetpos = posArg->x0 + posArg->xdminusx0
					* (10 * pow(t / T, 3)
						- 15 * pow(t / T, 4)
						+ 6 * pow(t / T, 5));

				posArg->error = (posArg->targetpos - ((double)encoderArg->position) / 2) / 360;
				posArg->output = posArg->pid->compute(posArg->error);

				speedArg->targetrpm = posArg->output * speedArg->maxrpm;
				speedArg->targetrpm = constrain(speedArg->targetrpm, -posArg->targetrpm, posArg->targetrpm);
			}

			if ((speedArg->running || timeArg->running) || ((posArg->running || posArg->hold) && posArg->flip))
			{
				if (timeArg->running && ((millis() - timeArg->starttime) >= timeArg->time_ms))
				{
					stopAction_static(speedArg->motora, speedArg->motorb, posArg, encoderArg);
					timeArg->running = false;
					speedArg->pid->reset();
					return true;
				}

				if (timeArg->running) speedArg->targetrpm = timeArg->targetrpm;

				speedArg->error = (speedArg->targetrpm - rpm) / speedArg->maxrpm;
				speedArg->output = speedArg->pid->compute(speedArg->error);
				if (speedArg->targetrpm == 0) writePWM_static(speedArg->motora, speedArg->motorb, 0);
				else writePWM_static(speedArg->motora, speedArg->motorb, speedArg->output);
			}
		}
		else
		{
			posArg->stop_action = 0;
			stopAction_static(speedArg->motora, speedArg->motorb, posArg, encoderArg);
			speedArg->running = false;
			posArg->running = false;
			timeArg->running = false;
			posArg->hold = false;
			speedArg->pid->reset();
			posArg->pid->reset();
		}
		return true;
	}

private:
	static void
		attach_enc_interrupt(uint8_t pin, encoder_state_t* enc)
	{
		switch (pin)
		{
		case OUTPUT1ENCA:
			encoderArgs[0] = enc;
			attachInterrupt(OUTPUT1ENCA, isr0, CHANGE);
			break;
		case OUTPUT1ENCB:
			encoderArgs[0] = enc;
			attachInterrupt(OUTPUT1ENCB, isr1, CHANGE);
			break;
		case OUTPUT2ENCA:
			encoderArgs[1] = enc;
			attachInterrupt(OUTPUT2ENCA, isr2, CHANGE);
			break;
		case OUTPUT2ENCB:
			encoderArgs[1] = enc;
			attachInterrupt(OUTPUT2ENCB, isr3, CHANGE);
			break;
		case OUTPUT3ENCA:
			encoderArgs[2] = enc;
			attachInterrupt(OUTPUT3ENCA, isr4, CHANGE);
			break;
		case OUTPUT3ENCB:
			encoderArgs[2] = enc;
			attachInterrupt(OUTPUT3ENCB, isr5, CHANGE);
			break;
		case OUTPUT4ENCA:
			encoderArgs[3] = enc;
			attachInterrupt(OUTPUT4ENCA, isr6, CHANGE);
			break;
		case OUTPUT4ENCB:
			encoderArgs[3] = enc;
			attachInterrupt(OUTPUT4ENCB, isr7, CHANGE);
			break;
		}
	}

	static void attach_pid_interrupt(uint8_t pin, speed_pid_t* state1, position_pid_t* state2, time_pid_t* state3)
	{
		switch (pin)
		{
		case OUTPUT1ENCA:
		case OUTPUT1ENCB:
			speedArgs[0] = state1;
			posArgs[0] = state2;
			timeArgs[0] = state3;
			EVNISRTimer::sharedISRTimer().setInterval(PID_TIMER_INTERVAL_MS, pidtimer0);

			break;
		case OUTPUT2ENCA:
		case OUTPUT2ENCB:
			speedArgs[1] = state1;
			posArgs[1] = state2;
			timeArgs[1] = state3;
			EVNISRTimer::sharedISRTimer().setInterval(PID_TIMER_INTERVAL_MS, pidtimer1);
			break;
		case OUTPUT3ENCA:
		case OUTPUT3ENCB:
			speedArgs[2] = state1;
			posArgs[2] = state2;
			timeArgs[2] = state3;
			EVNISRTimer::sharedISRTimer().setInterval(PID_TIMER_INTERVAL_MS, pidtimer2);
			break;
		case OUTPUT4ENCA:
		case OUTPUT4ENCB:
			speedArgs[3] = state1;
			posArgs[3] = state2;
			timeArgs[3] = state3;
			EVNISRTimer::sharedISRTimer().setInterval(PID_TIMER_INTERVAL_MS, pidtimer3);
			break;
		}
	}

	static void isr0()
	{
		update(encoderArgs[0]);
		rpm_update(encoderArgs[0]);
	}
	static void isr1() { update(encoderArgs[0]); }
	static void isr2()
	{
		update(encoderArgs[1]);
		rpm_update(encoderArgs[1]);
	}
	static void isr3() { update(encoderArgs[1]); }
	static void isr4()
	{
		update(encoderArgs[2]);
		rpm_update(encoderArgs[2]);
	}
	static void isr5() { update(encoderArgs[2]); }
	static void isr6()
	{
		update(encoderArgs[3]);
		rpm_update(encoderArgs[3]);
	}
	static void isr7() { update(encoderArgs[3]); }

	static void pidtimer0() { pid_update(speedArgs[0], posArgs[0], timeArgs[0], encoderArgs[0]); }
	static void pidtimer1() { pid_update(speedArgs[1], posArgs[1], timeArgs[1], encoderArgs[1]); }
	static void pidtimer2() { pid_update(speedArgs[2], posArgs[2], timeArgs[2], encoderArgs[2]); }
	static void pidtimer3() { pid_update(speedArgs[3], posArgs[3], timeArgs[3], encoderArgs[3]); }
};

typedef struct
{
	volatile double speed;
	volatile double turn_rate;
	volatile double maxrpm;
	volatile uint32_t wheel_dia;
	volatile uint32_t wheel_dist;
	volatile bool running;
	EVNMotor* motor_left;
	EVNMotor* motor_right;
}	drivebase_state_t;

class EVNDrivebase
{
public:
	EVNDrivebase(uint32_t wheel_dia, uint32_t wheel_dist, EVNMotor* _motor_left, EVNMotor* _motor_right);
	void begin();
	void steer(double speed, double turn_rate);
	// TODO
	// void steerTime(double speed, double turn_rate, uint32_t time_ms);
	// void steerDistance(double speed, double turning_rate, uint32_t distance_mm);
	//  uint32_t timeSinceLastCommand();
	//  bool commandFinished();
	void brake();
	void coast();
	void hold();

private:
	drivebase_state_t db;
	static drivebase_state_t* dbArg;

	static void attach_db_interrupt(drivebase_state_t* arg)
	{
		dbArg = arg;
		EVNISRTimer::sharedISRTimer().setInterval(PID_TIMER_INTERVAL_MS * 2, pidtimer0);
	}

	static void pidtimer0() { pid_update(dbArg); }

	static void pid_update(drivebase_state_t* arg)
	{
		if (arg->running)
		{
			double turn_radius, arclen_outer, arclen_inner, target_angvel_outer, target_angvel_inner;
			double angvel_outer, angvel_inner;

			if (fabs(arg->turn_rate) == 0.5)
			{
				if (arg->turn_rate > 0)
				{
					arg->motor_left->runSpeed(arg->speed);
					arg->motor_right->hold();
				}
				else {
					arg->motor_right->runSpeed(arg->speed);
					arg->motor_left->hold();
				}
				return;
			}

			double targetleftspeed, targetrightspeed, actualleftspeed, actualrightspeed, ratio;
			double lefterror, righterror, speederror, angleerror;

			actualleftspeed = arg->motor_left->getRPM();
			actualrightspeed = arg->motor_right->getRPM();
			targetleftspeed = arg->speed;
			targetrightspeed = arg->speed - 2 * arg->turn_rate * arg->speed;
			ratio = targetrightspeed / targetleftspeed;

			lefterror = fabs((targetleftspeed - actualleftspeed) / (2 * targetleftspeed));
			righterror = fabs((targetrightspeed - actualrightspeed) / (2 * targetrightspeed));
			speederror = (lefterror + righterror) / 2;
			angleerror = (lefterror - righterror) / 2;



			if (arg->turn_rate >= 0)
			{
				// if (lefterror >= righterror)
				targetrightspeed = actualleftspeed * ratio;
				// else
				// 	targetleftspeed = actualrightspeed / ratio;

			}
			else
			{
				// if (fabs(targetleftspeed - actualleftspeed) >= fabs(targetrightspeed - actualrightspeed))
				// 	targetrightspeed = actualleftspeed / ratio;
				// else
				targetleftspeed = actualrightspeed / ratio;
			}

			arg->motor_left->runSpeed(targetleftspeed);
			arg->motor_right->runSpeed(targetrightspeed);
		}
	}
};

#endif