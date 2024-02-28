#ifndef EVNMotor_h
#define EVNMotor_h

#include <Arduino.h>
#include "EVNAlpha.h"
#include "EVNISRTimer.h"
#include "PIDController.h"
#include "evn_alpha_pins.h"
#include "evn_motor_defs.h"
#include <cfloat>

//INPUT PARAMETER MACROS
#define DIRECT 1
#define REVERSE 0

#define EV3_LARGE 0
#define NXT_LARGE 1
#define EV3_MED 2
#define CUSTOM_MOTOR 3

#define STOP_BRAKE 0
#define STOP_COAST 1
#define STOP_HOLD 2

//CONTROL LOOP DEFS
#define NO_OF_PULSES_TIMED 2
#define RUNDEGREES_MIN_LOOP_COUNT 15
#define RUNDEGREES_MIN_ERROR_DEG 0.5

typedef struct
{
	//position measurement
	volatile uint8_t enca;
	volatile uint8_t encb;
	volatile int8_t dir = 1;
	volatile uint8_t state;
	volatile double ppr;
	volatile double position;

	//RPM measurement (pulse measuring)
	volatile uint8_t last_pulse_index;
	volatile uint64_t pulse_times[NO_OF_PULSES_TIMED];
	volatile double avg_pulse_timing;
	volatile double rpm;
} encoder_state_t;

typedef struct
{
	//runSpeed variables (or shared by all run funcs)
	volatile bool run_speed;
	uint8_t motora;
	uint8_t motorb;
	volatile uint64_t start_time_us;
	volatile double max_rpm;
	volatile double target_rpm;
	volatile double accel;
	volatile double decel;
	volatile double target_rpm_constrained;
	PIDController* speed_pid;

	//runTime variables
	volatile bool run_time;
	volatile uint64_t time_ms;

	//runDegrees variables
	volatile bool run_deg;
	volatile double run_deg_target_rpm;
	volatile double x0;
	volatile double xdminusx0;
	volatile uint8_t counter;
	volatile bool drivebase_run_pos;

	volatile bool hold;
	volatile uint8_t stop_action;
	PIDController* pos_pid;

} pid_control_t;

class EVNMotor
{
private:
	static const uint32_t OUTPUTPWMFREQ = 20000;
	static const uint32_t PWM_MAX_VAL = 255;
	static const uint8_t PID_TIMER_INTERVAL_MS = 2;
	static const uint32_t ENCODER_PULSE_TIMEOUT_US = 166667 * 2;

public:
	friend class EVNDrivebase;
	friend class EVNOmniDrivebase;

	EVNMotor(uint8_t port, uint8_t motortype = EV3_LARGE, uint8_t motor_dir = DIRECT, uint8_t enc_dir = DIRECT);
	void begin();
	double getPos();			  // Return absolute position in degrees
	void resetPos();			  // Set current position as 0 deg
	double getRPM();			  // Return velocity in RPM rotations per minute
	bool commandFinished();

	void setSpeedPID(double p, double i, double d);
	void setPosPID(double p, double i, double d);
	void setAccel(double accel_rpm_per_s);
	void setDecel(double decel_rpm_per_s);
	void setMaxRPM(double max_rpm);
	void setPPR(uint32_t ppr);

	void writePWM(double speed); // Directly write speed (-1 to 1) as PWM value to motor
	void runSpeed(double rpm); // Set motor to run at desired RPM
	void runPosition(double rpm, double position, uint8_t stop_action = STOP_BRAKE, bool wait = true); // Set motor to run to absolute position
	void runDegrees(double rpm, double degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true); // Set motor to run desired angular displacement (in degrees)
	void runTime(double rpm, uint32_t time_ms, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void stop();
	void brake();
	void coast();
	void hold();

protected:
	double _position_offset = 0;
	pid_control_t _pid_control;
	encoder_state_t _encoder;

	static bool ports_started[4];
	static encoder_state_t* encoderArgs[4]; // static list of pointers to each instances' structs
	static pid_control_t* pidArgs[4];

	static void rpm_update(encoder_state_t* arg)
	{
		uint8_t new_p0 = digitalRead(arg->enca);

		if (new_p0)
		{
			uint64_t now_us = micros();
			arg->last_pulse_index++;
			arg->last_pulse_index %= NO_OF_PULSES_TIMED;
			arg->pulse_times[arg->last_pulse_index] = now_us;

			uint64_t pulsetimings = 0;
			for (uint8_t i = 0; i < (NO_OF_PULSES_TIMED - 1); i++)
			{
				uint8_t endindex = (arg->last_pulse_index - i + NO_OF_PULSES_TIMED) % NO_OF_PULSES_TIMED;
				uint8_t startindex = (endindex - 1 + NO_OF_PULSES_TIMED) % NO_OF_PULSES_TIMED;
				uint64_t start = arg->pulse_times[startindex];
				uint64_t end = arg->pulse_times[endindex];
				pulsetimings += end - start;
			}

			arg->avg_pulse_timing = pulsetimings / (NO_OF_PULSES_TIMED - 1);
			arg->rpm = (1000000 / arg->avg_pulse_timing) / arg->ppr * 60 * arg->dir;
		}
	}

	static void enc_update(encoder_state_t* arg)
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
		case 1: case 7: case 8: case 14:
			arg->position++;
			arg->dir = 1;
			return;
		case 2: case 4: case 11: case 13:
			arg->position--;
			arg->dir = -1;
			return;
		case 3: case 12:
			arg->position += 2;
			arg->dir = 1;
			return;
		case 6: case 9:
			arg->position -= 2;
			arg->dir = -1;
			return;
		}
	}

	static void writePWM_static(pid_control_t* pidArg, double speed)
	{
		//clean input
		double speedc = constrain(speed, -1, 1);

		// if button is set to enable motors / button is unlinked from motors
		if (EVNAlpha::sharedButton().read() || !EVNAlpha::sharedButton().sharedState()->linkMotors)
		{
			if (speedc == 0)
			{
				digitalWrite(pidArg->motora, HIGH);
				digitalWrite(pidArg->motorb, HIGH);
			}
			else if (speedc > 0)
			{
				analogWrite(pidArg->motora, speedc * PWM_MAX_VAL);
				digitalWrite(pidArg->motorb, LOW);
			}
			else
			{
				analogWrite(pidArg->motorb, -speedc * PWM_MAX_VAL);
				digitalWrite(pidArg->motora, LOW);
			}
		}
	}

	static void stopAction_static(pid_control_t* pidArg, encoder_state_t* encoderArg)
	{
		//reset PID controllers
		pidArg->speed_pid->reset();
		pidArg->pos_pid->reset();

		//stop PID control
		pidArg->run_speed = false;
		pidArg->run_deg = false;
		pidArg->run_time = false;
		pidArg->hold = false;

		//coast and brake stop functions are based on DRV8833 datasheet
		//hold uses position PID control, setting the current position as an endpoints
		switch (pidArg->stop_action)
		{
		case STOP_BRAKE:
			digitalWrite(pidArg->motora, LOW);
			digitalWrite(pidArg->motorb, LOW);
			break;
		case STOP_COAST:
			digitalWrite(pidArg->motora, HIGH);
			digitalWrite(pidArg->motorb, HIGH);
			break;
		case STOP_HOLD:
			digitalWrite(pidArg->motora, HIGH);
			digitalWrite(pidArg->motorb, HIGH);
			pidArg->hold = true;
			pidArg->x0 = getAbsPos_static(encoderArg);
			pidArg->xdminusx0 = 0;
			pidArg->run_deg_target_rpm = pidArg->max_rpm / 2;
			break;
		}
	}

	static double getAbsPos_static(encoder_state_t* arg)
	{
		// resolution of encoder readout is in CPR (4 * PPR)
		return (double)arg->position * 360 / (4 * arg->ppr);
	}

	static double getRPM_static(encoder_state_t* arg)
	{
		uint64_t now_us = micros();
		uint64_t last_pulse_timing = now_us - arg->pulse_times[arg->last_pulse_index];

		// if timeout, RPM is 0
		if (last_pulse_timing > ENCODER_PULSE_TIMEOUT_US) return 0;

		// if latest pulse is longer than prior pulses, use latest pulse
		// this occurs when the motor is slowing down (pulses get longer and longer)
		else if (last_pulse_timing > arg->avg_pulse_timing)
		{
			double avg_pulse_timing = last_pulse_timing;
			double rpm = (1000000 / avg_pulse_timing) / arg->ppr * 60 * arg->dir;
			return rpm;
		}

		// otherwise, use the rpm generated by ISR by default
		else return arg->rpm;
	}

	static void pid_update(pid_control_t* pidArg, encoder_state_t* encoderArg)
	{
		double rpm = getRPM_static(encoderArg);

		// if button is set to enable motors / button is unlinked from motors
		if (EVNAlpha::sharedButton().read() || !EVNAlpha::sharedButton().sharedState()->linkMotors)
		{
			double accel_rpm = 0, decel_rpm = 1000000;
			if (pidArg->run_deg || pidArg->hold)
			{
				if (!pidArg->drivebase_run_pos)
				{
					if (pidArg->run_deg)
					{
						//if error is <= acceptable error for enough loops, stop position PID control
						if (pidArg->counter >= RUNDEGREES_MIN_LOOP_COUNT)
						{
							pidArg->counter = 0;
							stopAction_static(pidArg, encoderArg);
							return;
						}

						//if error <= acceptable error, increment a counter to count no. of consecutive loops with acceptable error
						if (fabs(pidArg->x0 + pidArg->xdminusx0 - getAbsPos_static(encoderArg)) <= (double)RUNDEGREES_MIN_ERROR_DEG)
						{
							pidArg->counter += 1;
							digitalWrite(pidArg->motora, HIGH);
							digitalWrite(pidArg->motorb, HIGH);
							return;
						}

						//if error > acceptable error, reset counter
						else pidArg->counter = 0;
					}

					if (pidArg->hold)
					{
						if (fabs(pidArg->x0 + pidArg->xdminusx0 - getAbsPos_static(encoderArg)) <= (double)RUNDEGREES_MIN_ERROR_DEG)
						{
							digitalWrite(pidArg->motora, HIGH);
							digitalWrite(pidArg->motorb, HIGH);
							return;
						}
					}
				}

				double target_pos, error, output;

				// by default, target position is end pos
				target_pos = pidArg->x0 + pidArg->xdminusx0;

				// calculate output RPM from pos error using PID controller
				error = (target_pos - getAbsPos_static(encoderArg)) / 360;
				output = pidArg->pos_pid->compute(error);

				if (output > 0) pidArg->target_rpm = min(pidArg->run_deg_target_rpm, output * pidArg->max_rpm);
				else pidArg->target_rpm = max(-pidArg->run_deg_target_rpm, output * pidArg->max_rpm);

				if (!pidArg->drivebase_run_pos)
				{
					// time taken to decel from target RPM to 0 (in ms)
					double timetodecel = (pidArg->run_deg_target_rpm / pidArg->decel * 1000);

					// deceltriangle = area under the v-t curve from start of decel to end of decel (RPM = 0)
					// area under v-t curve = displacement from start of decel to end of decel
					// however, this displacement is in RPM*ms so we /60000 to convert it to revolutions
					double deceltriangle = 0.5 * timetodecel * (pidArg->run_deg_target_rpm / 60000);

					// now we treat displacement error as area under curve from current time to end of decel (similar to decel triangle)
					// by sq rooting the ratio between the 2 triangles we get the scale factor between each triangle's v (v of decel triangle is target_rpm)
					// decel_rpm is target_rpm multiplied by scaling factor
					// as error decreases, decel_rpm will decrease along decel curve (to the end)
					decel_rpm = sqrt(fabs(error) / deceltriangle) * pidArg->run_deg_target_rpm;

					if (output > 0) pidArg->target_rpm = min(pidArg->target_rpm, decel_rpm);
					else pidArg->target_rpm = max(pidArg->target_rpm, -decel_rpm);
				}
			}

			if (pidArg->run_speed || pidArg->run_time || pidArg->run_deg || pidArg->hold)
			{
				double target_rpm, error, output;

				// if time has passed, stop motor
				if (pidArg->run_time)
				{
					if (micros() - pidArg->start_time_us >= pidArg->time_ms * 1000)
					{
						stopAction_static(pidArg, encoderArg);
						pidArg->speed_pid->reset();
						return;
					}
					// as runTime gets closer to the end, decelerate motor
					decel_rpm = (pidArg->start_time_us + pidArg->time_ms * 1000 - micros()) / 1000000 * pidArg->decel;
					if (pidArg->target_rpm >= 0) pidArg->target_rpm = min(pidArg->target_rpm, decel_rpm);
					else pidArg->target_rpm = max(pidArg->target_rpm, -decel_rpm);
				}

				if (!pidArg->drivebase_run_pos)
				{
					//TODO: convert to use time instead of assuming ISR timer is accurate
					if (pidArg->target_rpm_constrained <= pidArg->target_rpm)
					{
						if (pidArg->target_rpm_constrained > 0) pidArg->target_rpm_constrained += (double)PID_TIMER_INTERVAL_MS / 1000.0 * (double)pidArg->accel;
						else pidArg->target_rpm_constrained += (double)PID_TIMER_INTERVAL_MS / 1000.0 * pidArg->decel;
						if (pidArg->target_rpm_constrained > pidArg->target_rpm) pidArg->target_rpm_constrained = pidArg->target_rpm;
					}
					else
					{
						if (pidArg->target_rpm_constrained > 0) pidArg->target_rpm_constrained -= (double)PID_TIMER_INTERVAL_MS / 1000.0 * (double)pidArg->decel;
						else pidArg->target_rpm_constrained -= (double)PID_TIMER_INTERVAL_MS / 1000.0 * pidArg->accel;
						if (pidArg->target_rpm_constrained < pidArg->target_rpm) pidArg->target_rpm_constrained = pidArg->target_rpm;
					}
				}
				else
				{
					pidArg->target_rpm_constrained = pidArg->target_rpm;
				}
				pidArg->target_rpm_constrained = constrain(pidArg->target_rpm_constrained, -pidArg->max_rpm, pidArg->max_rpm);

				// calculate output PWM value from RPM error using PID controller
				error = (pidArg->target_rpm_constrained - rpm) / pidArg->max_rpm;
				output = pidArg->speed_pid->compute(error);

				//write PWM value to motors
				if (pidArg->target_rpm_constrained == 0) writePWM_static(pidArg, 0);
				else writePWM_static(pidArg, output);
			}
			else
			{
				pidArg->target_rpm_constrained = getRPM_static(encoderArg);
			}
		}
		else
		{
			//brake motors
			pidArg->stop_action = STOP_BRAKE;
			stopAction_static(pidArg, encoderArg);
			pidArg->target_rpm_constrained = getRPM_static(encoderArg);
		}
	}

	//used to attach pin change interrupts to both encoder pins
	//and a timer interrupt for each motor object
	//different ISR functions are attached depending on which port (determined by pins) is used
	static void attach_interrupts(encoder_state_t* encoderArg, pid_control_t* pidArg)
	{
		switch (encoderArg->enca)
		{
		case OUTPUT1ENCA:
		case OUTPUT1ENCB:
			if (!ports_started[0])
			{
				encoderArgs[0] = encoderArg;
				pidArgs[0] = pidArg;
				ports_started[0] = true;
				alarm_pool_add_repeating_timer_ms(EVNISRTimer::sharedAlarmPool(), -PID_TIMER_INTERVAL_MS, pidtimer0, NULL, &EVNISRTimer::sharedISRTimer(0));
				attachInterrupt(encoderArg->enca, isr0, CHANGE);
				attachInterrupt(encoderArg->encb, isr1, CHANGE);
			}
			break;
		case OUTPUT2ENCA:
		case OUTPUT2ENCB:
			if (!ports_started[1])
			{
				encoderArgs[1] = encoderArg;
				pidArgs[1] = pidArg;
				ports_started[1] = true;
				alarm_pool_add_repeating_timer_ms(EVNISRTimer::sharedAlarmPool(), -PID_TIMER_INTERVAL_MS, pidtimer1, NULL, &EVNISRTimer::sharedISRTimer(1));
				attachInterrupt(encoderArg->enca, isr2, CHANGE);
				attachInterrupt(encoderArg->encb, isr3, CHANGE);
			}
			break;
		case OUTPUT3ENCA:
		case OUTPUT3ENCB:
			if (!ports_started[2])
			{
				encoderArgs[2] = encoderArg;
				pidArgs[2] = pidArg;
				ports_started[2] = true;
				alarm_pool_add_repeating_timer_ms(EVNISRTimer::sharedAlarmPool(), -PID_TIMER_INTERVAL_MS, pidtimer2, NULL, &EVNISRTimer::sharedISRTimer(2));
				attachInterrupt(encoderArg->enca, isr4, CHANGE);
				attachInterrupt(encoderArg->encb, isr5, CHANGE);
			}
			break;
		case OUTPUT4ENCA:
		case OUTPUT4ENCB:
			if (!ports_started[3])
			{
				encoderArgs[3] = encoderArg;
				pidArgs[3] = pidArg;
				ports_started[3] = true;
				alarm_pool_add_repeating_timer_ms(EVNISRTimer::sharedAlarmPool(), -PID_TIMER_INTERVAL_MS, pidtimer3, NULL, &EVNISRTimer::sharedISRTimer(3));
				attachInterrupt(encoderArg->enca, isr6, CHANGE);
				attachInterrupt(encoderArg->encb, isr7, CHANGE);
			}
			break;
		}
	}

	// pin change interrupt ISRs (calling generic functions)
	// RPM measurement only uses pulses on one encoder wheel (more accurate, from our testing)
	static void isr0() { enc_update(encoderArgs[0]); rpm_update(encoderArgs[0]); }
	static void isr1() { enc_update(encoderArgs[0]); }
	static void isr2() { enc_update(encoderArgs[1]); rpm_update(encoderArgs[1]); }
	static void isr3() { enc_update(encoderArgs[1]); }
	static void isr4() { enc_update(encoderArgs[2]); rpm_update(encoderArgs[2]); }
	static void isr5() { enc_update(encoderArgs[2]); }
	static void isr6() { enc_update(encoderArgs[3]); rpm_update(encoderArgs[3]); }
	static void isr7() { enc_update(encoderArgs[3]); }

	// timer interrupt ISRs (calling the same generic function)
	static bool pidtimer0(struct repeating_timer* t) { pid_update(pidArgs[0], encoderArgs[0]); return true; }
	static bool pidtimer1(struct repeating_timer* t) { pid_update(pidArgs[1], encoderArgs[1]); return true; }
	static bool pidtimer2(struct repeating_timer* t) { pid_update(pidArgs[2], encoderArgs[2]); return true; }
	static bool pidtimer3(struct repeating_timer* t) { pid_update(pidArgs[3], encoderArgs[3]); return true; }
};

class EVNDrivebase
{
public:
	EVNDrivebase(uint32_t wheel_dia, uint32_t wheel_dist, EVNMotor* _motor_left, EVNMotor* _motor_right);
	void begin()
	{
		_max_rpm = min(_left->_pid_control.max_rpm, _right->_pid_control.max_rpm);
	};

	void steer(double speed, double turn_rate)
	{
		double speedc = constrain(speed, -_max_rpm, _max_rpm);
		double turn_ratec = constrain(turn_rate, -1, 1);
		if (turn_ratec > 0)
		{
			_left->runSpeed(speedc);
			_right->runSpeed(speedc * (1 - turn_ratec));
		}
		else {
			_right->runSpeed(speedc);
			_left->runSpeed(speedc * (1 - turn_ratec));
		}
	};

	void stop()
	{
		_left->brake();
		_right->brake();
	};

	void brake()
	{
		_left->brake();
		_right->brake();
	};

	void coast()
	{
		_left->coast();
		_right->coast();
	};

	void hold()
	{
		_left->hold();
		_right->hold();
	};

private:
	EVNMotor* _left;
	EVNMotor* _right;
	double _max_rpm;
};

class EVNOmniDrivebase
{
public:
	EVNOmniDrivebase(uint32_t wheel_dia, uint32_t wheel_dist, EVNMotor* fl, EVNMotor* fr, EVNMotor* bl, EVNMotor* br)
	{
		_fl = fl;
		_fr = fr;
		_bl = bl;
		_br = br;
	};

	void begin()
	{
		_max_rpm = min(min(_fl->_pid_control.max_rpm, _fr->_pid_control.max_rpm), min(_bl->_pid_control.max_rpm, _br->_pid_control.max_rpm));
	};

	void steer(double speed, double angle, double turn_rate)
	{
		double speedc, anglec, rotatec;
		double anglec_rad, speedc_x, speedc_y;
		double speedc_x_left, speedc_y_left, speedc_x_right, speedc_y_right;

		speedc = constrain(speed, -_max_rpm, _max_rpm);
		anglec = constrain(angle, 0, 360);
		anglec = fmod(anglec + 45, 360);
		rotatec = constrain(turn_rate, -1, 1);

		anglec_rad = anglec / 180 * M_PI;
		speedc_x = cos(anglec_rad) * speedc;
		speedc_y = sin(anglec_rad) * speedc;

		if (anglec >= 315 || anglec < 135)
		{
			if (rotatec >= 0)
			{
				speedc_y_right = speedc_y * (1 - 2 * rotatec);
				speedc_y_left = speedc_y;
			}
			else {
				speedc_y_left = speedc_y * (1 + 2 * rotatec);
				speedc_y_right = speedc_y;
			}
		}
		else {
			if (rotatec >= 0)
			{
				speedc_y_left = speedc_y * (1 - 2 * rotatec);
				speedc_y_right = speedc_y;
			}
			else {
				speedc_y_right = speedc_y * (1 + 2 * rotatec);
				speedc_y_left = speedc_y;
			}
		}

		if (anglec >= 225 || anglec < 45)
		{
			if (rotatec >= 0)
			{
				speedc_x_left = speedc_x * (1 - 2 * rotatec);
				speedc_x_right = speedc_x;
			}
			else {
				speedc_x_right = speedc_x * (1 + 2 * rotatec);
				speedc_x_left = speedc_x;
			}
		}
		else {
			if (rotatec >= 0)
			{
				speedc_x_right = speedc_x * (1 - 2 * rotatec);
				speedc_x_left = speedc_x;
			}
			else {
				speedc_x_left = speedc_x * (1 + 2 * rotatec);
				speedc_x_right = speedc_x;
			}
		}

		_fl->runSpeed(speedc_x_left);
		_br->runSpeed(speedc_x_right);
		_fr->runSpeed(speedc_y_right);
		_bl->runSpeed(speedc_y_left);
	};

	void stop()
	{
		this->brake();
	};

	void brake()
	{
		_fl->brake();
		_fr->brake();
		_bl->brake();
		_br->brake();
	};

	void coast()
	{
		_fl->coast();
		_fr->coast();
		_bl->coast();
		_br->coast();
	};

	void hold()
	{
		_fl->hold();
		_fr->hold();
		_bl->hold();
		_br->hold();
	};

private:
	EVNMotor* _fl;
	EVNMotor* _fr;
	EVNMotor* _bl;
	EVNMotor* _br;
	double _max_rpm;
};

#endif