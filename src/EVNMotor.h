#ifndef EVNMotor_h
#define EVNMotor_h

#include <Arduino.h>
#include <EVNButton.h>
#include "RPi_Pico_TimerInterrupt.h"
#include "RPi_Pico_ISR_Timer.hpp"
#include "PIDController.h"

#define DIRECT 0
#define REVERSE 1

#define OUTPUTPWMFREQ 20000
#define PWM_MAX_VAL 255
#define HW_TIMER_INTERVAL_MS 1
#define PID_TIMER_INTERVAL_MS 2
#define NO_OF_PULSES_TIMED 2

#define SPEED_PID_KP_EV3_LARGE 5
#define SPEED_PID_KI_EV3_LARGE 0.125
#define SPEED_PID_KD_EV3_LARGE 0

#define SPEED_PID_KP_NXT_LARGE 5
#define SPEED_PID_KI_NXT_LARGE 0.125
#define SPEED_PID_KD_NXT_LARGE 0

#define SPEED_PID_KP_EV3_MED 4.5
#define SPEED_PID_KI_EV3_MED 0.125
#define SPEED_PID_KD_EV3_MED 0

#define SPEED_PID_KP_CUSTOM 5
#define SPEED_PID_KI_CUSTOM 0.125
#define SPEED_PID_KD_CUSTOM 0

#define POS_PID_KP_EV3_LARGE 400
#define POS_PID_KI_EV3_LARGE 0
#define POS_PID_KD_EV3_LARGE 600

#define POS_PID_KP_NXT_LARGE 400
#define POS_PID_KI_NXT_LARGE 0
#define POS_PID_KD_NXT_LARGE 600

#define POS_PID_KP_EV3_MED 300
#define POS_PID_KI_EV3_MED 0
#define POS_PID_KD_EV3_MED 600

#define POS_PID_KP_CUSTOM 400
#define POS_PID_KI_CUSTOM 0
#define POS_PID_KD_CUSTOM 600

#define EV3_LARGE 0
#define NXT_LARGE 1
#define EV3_MED 2
#define CUSTOM 3

#define EV3_LARGE_MAX_RPM 155
#define NXT_LARGE_MAX_RPM 155
#define EV3_MED_MAX_RPM 230
#define CUSTOM_MAX_RPM 155

#define STOP_ACTION_BRAKE 0
#define STOP_ACTION_COAST 1
#define STOP_ACTION_HOLD 2

#define OUTPUT1MOTORA 29
#define OUTPUT1MOTORB 28
#define OUTPUT2MOTORA 27
#define OUTPUT2MOTORB 26
#define OUTPUT3MOTORA 23
#define OUTPUT3MOTORB 22
#define OUTPUT4MOTORA 21
#define OUTPUT4MOTORB 20

#define OUTPUT1ENCA 18
#define OUTPUT1ENCB 19
#define OUTPUT2ENCA 17
#define OUTPUT2ENCB 16
#define OUTPUT3ENCA 14
#define OUTPUT3ENCB 15
#define OUTPUT4ENCA 13
#define OUTPUT4ENCB 12

typedef struct
{
	volatile uint8_t enca;
	volatile uint8_t encb;
	volatile uint8_t state;
	volatile double position; // if any ISR writes a value to it, use volatile
	volatile uint64_t pulsetimes[NO_OF_PULSES_TIMED];
	volatile uint8_t lastpulseindex;
	volatile uint64_t lastpulsetime;
	volatile double avgpulsetiming;
	volatile double rpm;
	volatile int8_t dir = 1;
	volatile bool dataReady;
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
	EVNButton* button;
	PIDController* pid;

} speed_pid_t;

typedef struct
{
	volatile uint32_t counter;
	volatile bool running;
	volatile bool hold;
	volatile uint8_t stop_action;
	volatile double targetpos;
	volatile double error;
	volatile double output;
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
	EVNMotor(uint8_t port, uint8_t motortype = EV3_LARGE, EVNButton* button = NULL);
	void init();
	void writePWM(double speed); // Directly write speed (-1 to 1) as PWM value to motor
	double getPos();			  // Return absolute position in degrees
	void resetPos();			  // Set current position as 0 deg
	double getRPM();			  // Return velocity in RPM rotations per minute

	bool commandFinished();
	void runSpeed(double rpm);																	// Set motor to run at desired RPM
	void runDegrees(double degrees, uint8_t stop_action = STOP_ACTION_BRAKE, bool wait = true); // Set motor to run desired angular displacement (in degrees)
	void runTime(double rpm, uint32_t time_ms, uint8_t stop_action = STOP_ACTION_BRAKE, bool wait = true);
	uint64_t timeSinceLastCommand();
	void brake();
	void coast();
	void hold();

	double debugSpeedPID();
	double debugPositionPID();

private:
	bool _encoded;
	uint8_t _motora, _motorb, _enca, _encb, _motortype, _maxrpm;
	uint32_t lastcommand_ms;

public:
	encoder_state_t encoder;
	uint8_t maxrpm;

	// private:
	speed_pid_t speed_pid;
	position_pid_t pos_pid;
	time_pid_t time_pid;

public:
	static encoder_state_t* encoderArgs[8]; // static list of pointers to each instances' structs
	static speed_pid_t* speedArgs[4];
	static position_pid_t* posArgs[4];
	static time_pid_t* timeArgs[4];
	static RPI_PICO_Timer timer;		// static timer shared by all instances
	static RPI_PICO_ISR_Timer ISRtimer; // means that no other class can use the ISRtimer (I think)

	static void rpm_update(encoder_state_t* arg)
	{
		uint8_t new_p0 = digitalRead(arg->enca);

		if (new_p0)
		{
			uint64_t currentpulsetime = micros();
			arg->dataReady = false;
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
			arg->rpm = (1000000 / (arg->avgpulsetiming * 3)) * arg->dir;
			arg->dataReady = true;
			arg->lastpulsetime = currentpulsetime;
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
		case STOP_ACTION_BRAKE:
			digitalWrite(motora, LOW);
			digitalWrite(motorb, LOW);
			break;
		case STOP_ACTION_COAST:
			digitalWrite(motora, HIGH);
			digitalWrite(motorb, HIGH);
			break;
		case STOP_ACTION_HOLD:
			digitalWrite(motora, HIGH);
			digitalWrite(motorb, HIGH);
			posArg->hold = true;
			posArg->targetpos = (double)encoderArg->position / 2;
			break;
		}
	}

	static double getRPM_static(encoder_state_t* encoderArg)
	{
		uint64_t currenttime = micros();
		uint64_t lastpulsetiming = (encoderArg->dataReady) ? (currenttime - encoderArg->pulsetimes[encoderArg->lastpulseindex]) : (currenttime - encoderArg->lastpulsetime);
		if (lastpulsetiming > encoderArg->avgpulsetiming)
		{
			double avgpulsetiming = (double)lastpulsetiming;
			double rpm = (1000000 / (avgpulsetiming * 3)) * encoderArg->dir;
			return rpm;
		}
		else
		{
			return encoderArg->rpm;
		}
	}

	static bool pid_update(speed_pid_t* speedArg, position_pid_t* posArg, time_pid_t* timeArg, encoder_state_t* encoderArg)
	{
		if (speedArg->button->read())
		{
			double rpm = getRPM_static(encoderArg);

			if (speedArg->running || timeArg->running)
			{
				if (timeArg->running && ((millis() - timeArg->starttime) >= timeArg->time_ms))
				{
					stopAction_static(speedArg->motora, speedArg->motorb, posArg, encoderArg);
					timeArg->running = false;
					return true;
				}

				if (timeArg->running) speedArg->targetrpm = timeArg->targetrpm;

				speedArg->error = (speedArg->targetrpm - rpm) / speedArg->maxrpm;
				speedArg->output = speedArg->pid->compute(speedArg->error);

				if (speedArg->targetrpm == 0)
					writePWM_static(speedArg->motora, speedArg->motorb, 0);
				else
					writePWM_static(speedArg->motora, speedArg->motorb, speedArg->output);
			}
			else if (posArg->running || posArg->hold)
			{
				if (posArg->running && (fabs(posArg->targetpos - ((double)encoderArg->position) / 2) < 1))
				{
					posArg->counter += 1;
					if (posArg->counter > 10)
					{
						stopAction_static(speedArg->motora, speedArg->motorb, posArg, encoderArg);
						posArg->running = false;
						return true;
					}
				}
				else
				{
					posArg->counter = 0;
				}

				posArg->error = (posArg->targetpos - ((double)encoderArg->position) / 2) / 360;
				posArg->output = posArg->pid->compute(posArg->error);
				writePWM_static(speedArg->motora, speedArg->motorb, posArg->output);
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
			encoderArgs[1] = enc;
			attachInterrupt(OUTPUT1ENCB, isr1, CHANGE);
			break;
		case OUTPUT2ENCA:
			encoderArgs[2] = enc;
			attachInterrupt(OUTPUT2ENCA, isr2, CHANGE);
			break;
		case OUTPUT2ENCB:
			encoderArgs[3] = enc;
			attachInterrupt(OUTPUT2ENCB, isr3, CHANGE);
			break;
		case OUTPUT3ENCA:
			encoderArgs[4] = enc;
			attachInterrupt(OUTPUT3ENCA, isr4, CHANGE);
			break;
		case OUTPUT3ENCB:
			encoderArgs[5] = enc;
			attachInterrupt(OUTPUT3ENCB, isr5, CHANGE);
			break;
		case OUTPUT4ENCA:
			encoderArgs[6] = enc;
			attachInterrupt(OUTPUT4ENCA, isr6, CHANGE);
			break;
		case OUTPUT4ENCB:
			encoderArgs[7] = enc;
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
			ISRtimer.setInterval(PID_TIMER_INTERVAL_MS, pidtimer0);
			break;
		case OUTPUT2ENCA:
		case OUTPUT2ENCB:
			speedArgs[1] = state1;
			posArgs[1] = state2;
			timeArgs[1] = state3;
			ISRtimer.setInterval(PID_TIMER_INTERVAL_MS, pidtimer1);
			break;
		case OUTPUT3ENCA:
		case OUTPUT3ENCB:
			speedArgs[2] = state1;
			posArgs[2] = state2;
			timeArgs[2] = state3;
			ISRtimer.setInterval(PID_TIMER_INTERVAL_MS, pidtimer2);
			break;
		case OUTPUT4ENCA:
		case OUTPUT4ENCB:
			speedArgs[3] = state1;
			posArgs[3] = state2;
			timeArgs[3] = state3;
			ISRtimer.setInterval(PID_TIMER_INTERVAL_MS, pidtimer3);
			break;
		}
	}

	static bool isrtimer(struct repeating_timer* t)
	{
		ISRtimer.run();
		return true;
	}

	static void isr0()
	{
		update(encoderArgs[0]);
		rpm_update(encoderArgs[0]);
	}
	static void isr1() { update(encoderArgs[1]); }
	static void isr2()
	{
		update(encoderArgs[2]);
		rpm_update(encoderArgs[2]);
	}
	static void isr3() { update(encoderArgs[3]); }
	static void isr4()
	{
		update(encoderArgs[4]);
		rpm_update(encoderArgs[4]);
	}
	static void isr5() { update(encoderArgs[5]); }
	static void isr6()
	{
		update(encoderArgs[6]);
		rpm_update(encoderArgs[6]);
	}
	static void isr7() { update(encoderArgs[7]); }

	static void pidtimer0() { pid_update(speedArgs[0], posArgs[0], timeArgs[0], encoderArgs[0]); }
	static void pidtimer1() { pid_update(speedArgs[1], posArgs[1], timeArgs[1], encoderArgs[2]); }
	static void pidtimer2() { pid_update(speedArgs[2], posArgs[2], timeArgs[2], encoderArgs[4]); }
	static void pidtimer3() { pid_update(speedArgs[3], posArgs[3], timeArgs[3], encoderArgs[6]); }
};

class EVNDrivebase
{
public:
	EVNDrivebase(uint32_t wheel_dia, uint32_t wheel_dist, EVNMotor* _motor_left, EVNMotor* _motor_right, uint8_t motor_left_inv = DIRECT, uint8_t motor_right_inv = DIRECT);
	void steer(double speed, double turn_rate);

	// TODO
	// void steer_better(double speed, double turn_rate);
	// void steerTime(double speed, double turn_rate, uint32_t time_ms);
	// void steerDistance(double speed, double turning_rate, uint32_t distance_mm);
	//  uint32_t timeSinceLastCommand();
	//  bool commandFinished();
	void brake();
	void coast();
	void hold();

private:
	EVNMotor* _motor_left, * _motor_right;
	PIDController avg_pid, diff_pid, avg_follow_pid, diff_follow_pid;
	bool _motor_left_inv, _motor_right_inv;
	uint32_t _wheel_dist, _wheel_dia;
	double _maxrpm;
};

#endif