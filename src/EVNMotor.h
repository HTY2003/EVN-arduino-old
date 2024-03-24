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
#define DIRECT			1
#define REVERSE			0

#define EV3_LARGE		0
#define NXT_LARGE		1
#define EV3_MED			2
#define CUSTOM_MOTOR	3

#define STOP_BRAKE		0
#define STOP_COAST		1
#define STOP_HOLD		2

//DPS MEASUREMENT (TIME BETWEEN PULSES)
#define NO_OF_PULSES_TIMED 3

typedef struct
{
	//POSITION MEASUREMENT
	volatile uint8_t enca;
	volatile uint8_t encb;
	volatile int8_t dir;
	volatile uint8_t state;
	volatile double ppr;
	volatile double position;
	volatile double position_offset;

	//DPS MEASUREMENT (TIME BETWEEN PULSES)
	volatile uint8_t last_pulse_index;
	volatile uint64_t pulse_times[NO_OF_PULSES_TIMED];
	volatile int8_t pulse_dirs[NO_OF_PULSES_TIMED];
	volatile double avg_pulse_timing;
} encoder_state_t;

typedef struct
{
	//MOTOR CHARACTERISTICS
	uint8_t motora;
	uint8_t motorb;
	volatile double max_rpm;
	volatile double accel;
	volatile double decel;

	//CONTROLLER
	PIDController* pos_pid;

	//USER-SET VARIABLES
	volatile bool run_pwm;
	volatile bool run_speed;
	volatile bool run_dir;
	volatile double target_dps;
	volatile bool run_pos;
	volatile double target_pos;
	volatile bool run_time;
	volatile uint64_t run_time_ms;
	volatile bool hold;
	volatile uint8_t stop_action;

	//LOOP VARIABLES
	volatile uint64_t last_update;
	volatile double target_dps_end_decel;
	volatile double target_dps_constrained;
	volatile double x;
	volatile double error;
	volatile double output;

	volatile uint8_t counter;
	volatile uint64_t start_time_us;

	volatile bool stalled;
} pid_control_t;

class EVNMotor
{
private:
	static const uint32_t PWM_FREQ = 20000;
	static const uint32_t PWM_MAX_VAL = 255;
	static const uint16_t PID_TIMER_INTERVAL_US = 1000;
	static const uint32_t ENCODER_PULSE_TIMEOUT_US = 166667 * 2;

public:
	friend class EVNDrivebase;

	EVNMotor(uint8_t port, uint8_t motortype = EV3_LARGE, uint8_t motor_dir = DIRECT, uint8_t enc_dir = DIRECT);
	void begin();
	double getPosition();
	double getHeading();
	void resetPosition();
	double getDPS();
	double getSpeed() { return this->getDPS(); };

	void setPID(double p, double i, double d);
	void setAccel(double accel_dps_sq);
	void setDecel(double decel_dps_sq);
	void setMaxRPM(double max_rpm);
	void setPPR(uint32_t ppr);

	void runPWM(double duty_cycle);
	void runSpeed(double dps) { this->runDPS(dps); };
	void runDPS(double dps);

	void runPosition(double dps, double position, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void runAngle(double dps, double degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void runHeading(double dps, double heading, uint8_t stop_action = STOP_BRAKE, bool wait = true);

	void runTime(double dps, uint32_t time_ms, uint8_t stop_action = STOP_BRAKE, bool wait = true);

	//heading -> 0 - 360
	//angle / degrees -> relative
	//position / target -> -inf - inf

	void stop();
	void brake();
	void coast();
	void hold();

	bool completed();
	bool stalled();
	//TODO: add end function

protected:
	double clean_input_dps(double dps);
	uint8_t clean_input_dir(double dps);
	uint8_t clean_input_stop_action(uint8_t stop_action);

	double _position_offset = 0;
	pid_control_t _pid_control;
	encoder_state_t _encoder;

	static bool ports_started[4];
	static encoder_state_t* encoderArgs[4]; // static list of pointers to each instances' structs
	static pid_control_t* pidArgs[4];

	static bool motors_enabled()
	{
		return ((EVNAlpha::sharedButton().read() && EVNAlpha::sharedButton().sharedState()->link_motors)

			|| !EVNAlpha::sharedButton().sharedState()->link_motors);
	}

	static bool timed_control_enabled(pid_control_t* arg)
	{
		return arg->run_time;
	}

	static bool position_control_enabled(pid_control_t* arg)
	{
		return arg->run_pos;
	}

	static bool loop_control_enabled(pid_control_t* arg)
	{
		return (arg->run_speed || arg->run_time || arg->run_pos || arg->hold);
	}

	static void velocity_update(encoder_state_t* arg, uint64_t now)
	{
		if (gpio_get(arg->enca))
		{
			arg->last_pulse_index++;
			arg->last_pulse_index %= NO_OF_PULSES_TIMED;
			arg->pulse_times[arg->last_pulse_index] = now;
			arg->pulse_dirs[arg->last_pulse_index] = arg->dir;
		}
	}

	static void pos_update(encoder_state_t* arg)
	{
		uint8_t state = arg->state & 3;
		if (gpio_get(arg->enca))
			state |= 4;
		if (gpio_get(arg->encb))
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

	static void runPWM_static(pid_control_t* pidArg, double speed)
	{

		double speedc = constrain(speed, -1, 1);

		// if button is set to enable motors / button is unlinked from motors
		if (motors_enabled()) {
			if (speedc == 0)
			{
				digitalWrite(pidArg->motora, HIGH);
				digitalWrite(pidArg->motorb, HIGH);
			}
			else if (speedc == 1)
			{
				digitalWrite(pidArg->motora, HIGH);
				digitalWrite(pidArg->motorb, LOW);
			}
			else if (speedc == -1)
			{
				digitalWrite(pidArg->motora, LOW);
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

	static void stopAction_static(pid_control_t* pidArg, encoder_state_t* encoderArg, uint64_t now)
	{
		//reset PID controller, stop loop control
		pidArg->pos_pid->reset();
		pidArg->run_pwm = false;
		pidArg->run_speed = false;
		pidArg->run_pos = false;
		pidArg->run_time = false;
		pidArg->hold = false;

		//keep start_time and x at most recent states
		pidArg->start_time_us = now;

		//coast and brake stop functions are based on DRV8833 datasheet
		//hold uses position PID control, setting the current position as an endpoint
		switch (pidArg->stop_action)
		{
		case STOP_COAST:
			digitalWrite(pidArg->motora, LOW);
			digitalWrite(pidArg->motorb, LOW);
			pidArg->x = getPosition_static(encoderArg);
			pidArg->target_dps_constrained = getDPS_static(encoderArg);
			break;
		case STOP_BRAKE:
			digitalWrite(pidArg->motora, HIGH);
			digitalWrite(pidArg->motorb, HIGH);
			pidArg->x = getPosition_static(encoderArg);
			pidArg->target_dps_constrained = getDPS_static(encoderArg);
			break;
		case STOP_HOLD:
			pidArg->target_dps = 0;
			pidArg->hold = true;
			break;
		}
	}

	static double getPosition_static(encoder_state_t* arg)
	{
		// resolution of encoder readout is in CPR (4 * PPR)
		return (double)arg->position * 360 / (4 * arg->ppr) - arg->position_offset;
	}

	static double getDPS_static(encoder_state_t* arg)
	{
		uint64_t now = micros();
		double last_pulse_timing = now - arg->pulse_times[arg->last_pulse_index];
		double sum_dps = 0;
		double sum_pulse_timing = 0;

		for (uint8_t i = 0; i < (NO_OF_PULSES_TIMED - 1); i++)
		{
			uint8_t end_index = (arg->last_pulse_index - i + NO_OF_PULSES_TIMED) % NO_OF_PULSES_TIMED;
			uint8_t start_index = (end_index - 1 + NO_OF_PULSES_TIMED) % NO_OF_PULSES_TIMED;
			uint64_t start = arg->pulse_times[start_index];
			uint64_t end = arg->pulse_times[end_index];

			double pulse_timing = end - start;
			double pulse_dps = (1000000 / pulse_timing) / arg->ppr * 360 * arg->pulse_dirs[end_index];
			sum_pulse_timing += pulse_timing;
			sum_dps += pulse_dps;
		}

		// if timeout, DPS is 0
		if (last_pulse_timing > ENCODER_PULSE_TIMEOUT_US) return 0;

		// if latest pulse is longer than prior pulses, use latest pulse
		// this occurs when the motor is slowing down (pulses get longer and longer)
		else if (last_pulse_timing > (sum_pulse_timing / (NO_OF_PULSES_TIMED - 1)))
		{
			double final_dps = (1000000 / last_pulse_timing) / arg->ppr * 360;
			return final_dps * ((sum_dps < 0) ? -1 : 1);
		}

		else
			return sum_dps / (NO_OF_PULSES_TIMED - 1);

	}

	static void pid_update(pid_control_t* pidArg, encoder_state_t* encoderArg)
	{
		uint64_t now = micros();
		double pos = getPosition_static(encoderArg);
		double dps = getDPS_static(encoderArg);
		double time_since_last_loop = ((double)now - (double)pidArg->last_update) / 1000000;
		pidArg->last_update = now;

		if (motors_enabled() && loop_control_enabled(pidArg))
		{
			double decel_dps = pidArg->target_dps;

			if (position_control_enabled(pidArg) || pidArg->hold)
			{
				if (!pidArg->hold)
				{
					if (fabs(pidArg->target_pos - pos) <= USER_RUN_DEGREES_MIN_ERROR_DEG)
						pidArg->counter++;
					else
						pidArg->counter = 0;

					if (pidArg->counter >= USER_RUN_DEGREES_MIN_LOOP_COUNT)
					{
						stopAction_static(pidArg, encoderArg, now);
						return;
					}
				}

				pidArg->run_dir = (pidArg->target_pos - pos > 0) ? DIRECT : REVERSE;

				double error = fabs(pidArg->target_pos - pos);
				double max_error_before_decel = pow(pidArg->target_dps, 2) / pidArg->decel / 2;

				if (error < max_error_before_decel)
					decel_dps = sqrt(error / max_error_before_decel) * pidArg->target_dps;
			}

			if (pidArg->run_time)
			{
				if ((now - pidArg->start_time_us) >= pidArg->run_time_ms * 1000)
				{
					stopAction_static(pidArg, encoderArg, now);
					return;
				}

				if ((pidArg->target_dps / pidArg->decel) * 1000000 > ((double)pidArg->run_time_ms * 1000 + pidArg->start_time_us - now))
					decel_dps = pidArg->decel * ((double)pidArg->run_time_ms * 1000 + (double)pidArg->start_time_us - (double)now) / 1000000;
			}

			pidArg->target_dps_end_decel = decel_dps;
			double signed_target_dps_end_decel = pidArg->target_dps_end_decel * ((pidArg->run_dir == REVERSE) ? -1 : 1);

			double old_x = pidArg->x;

			if (fabs(pidArg->x - pos) < 1 / pidArg->pos_pid->getKp()) //anti-windup
			{
				pidArg->stalled = false;

				if (pidArg->target_dps_constrained <= signed_target_dps_end_decel)
				{
					if (signed_target_dps_end_decel > 0)
						pidArg->target_dps_constrained += time_since_last_loop * pidArg->accel;
					else
						pidArg->target_dps_constrained += time_since_last_loop * pidArg->decel;

					if (pidArg->target_dps_constrained > signed_target_dps_end_decel)
						pidArg->target_dps_constrained = signed_target_dps_end_decel;
				}
				else
				{
					if (signed_target_dps_end_decel > 0)
						pidArg->target_dps_constrained -= time_since_last_loop * pidArg->decel;
					else
						pidArg->target_dps_constrained -= time_since_last_loop * pidArg->accel;

					if (pidArg->target_dps_constrained < signed_target_dps_end_decel)
						pidArg->target_dps_constrained = signed_target_dps_end_decel;
				}

				pidArg->x += time_since_last_loop * pidArg->target_dps_constrained;
			}
			else
			{
				pidArg->stalled = true;
			}

			if (position_control_enabled(pidArg) || pidArg->hold)
			{
				if (fabs(pidArg->target_pos - old_x) < fabs(pidArg->target_pos - pidArg->x))
				{
					pidArg->x = pidArg->target_pos;
				}
			}

			pidArg->error = pidArg->x - pos;

			double kp = pidArg->pos_pid->getKp();
			double ki = pidArg->pos_pid->getKi();
			double kd = pidArg->pos_pid->getKd();

			if (!position_control_enabled(pidArg))
				pidArg->pos_pid->setKi(0);

			pidArg->pos_pid->setKd((1 - fabs(pidArg->target_dps) / pidArg->max_rpm / 6) * kd);

			if (pidArg->hold)
			{
				pidArg->pos_pid->setKp(kp / 2);
			}

			pidArg->output = pidArg->pos_pid->compute(pidArg->error, false, false, false);
			pidArg->pos_pid->setKp(kp);
			pidArg->pos_pid->setKi(ki);
			pidArg->pos_pid->setKd(kd);

			runPWM_static(pidArg, pidArg->output);
		}
		else if (motors_enabled() && pidArg->run_pwm)
		{
		}
		else
		{
			pidArg->stop_action = STOP_BRAKE;
			stopAction_static(pidArg, encoderArg, now);
		}
	}

	//used to attach pin change interrupts to both encoder pins
	//different ISR functions are attached depending on port (determined by pins)
	//also adds one timer interrupt shared by all ports (if not already added)
	static void attach_interrupts(encoder_state_t* encoderArg, pid_control_t* pidArg)
	{
		if (!ports_started[0] && !ports_started[1] && !ports_started[2] && !ports_started[3])
			alarm_pool_add_repeating_timer_us(EVNISRTimer::sharedAlarmPool(), -PID_TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer::sharedISRTimer(0));

		switch (encoderArg->enca)
		{
		case OUTPUT1ENCA:
		case OUTPUT1ENCB:
			encoderArgs[0] = encoderArg;
			pidArgs[0] = pidArg;
			ports_started[0] = true;
			attachInterrupt(encoderArg->enca, isr0, CHANGE);
			attachInterrupt(encoderArg->encb, isr1, CHANGE);
			break;

		case OUTPUT2ENCA:
		case OUTPUT2ENCB:
			encoderArgs[1] = encoderArg;
			pidArgs[1] = pidArg;
			ports_started[1] = true;
			attachInterrupt(encoderArg->enca, isr2, CHANGE);
			attachInterrupt(encoderArg->encb, isr3, CHANGE);
			break;

		case OUTPUT3ENCA:
		case OUTPUT3ENCB:
			encoderArgs[2] = encoderArg;
			pidArgs[2] = pidArg;
			ports_started[2] = true;
			attachInterrupt(encoderArg->enca, isr4, CHANGE);
			attachInterrupt(encoderArg->encb, isr5, CHANGE);
			break;

		case OUTPUT4ENCA:
		case OUTPUT4ENCB:
			encoderArgs[3] = encoderArg;
			pidArgs[3] = pidArg;
			ports_started[3] = true;
			attachInterrupt(encoderArg->enca, isr6, CHANGE);
			attachInterrupt(encoderArg->encb, isr7, CHANGE);
			break;
		}
	}

	// pin change interrupt ISRs (calling generic functions)
	// RPM measurement only uses pulses on one encoder wheel (more accurate, from our testing)
	static void isr0() { uint64_t now = micros(); pos_update(encoderArgs[0]); velocity_update(encoderArgs[0], now); }
	static void isr1() { pos_update(encoderArgs[0]); }
	static void isr2() { uint64_t now = micros(); pos_update(encoderArgs[1]); velocity_update(encoderArgs[1], now); }
	static void isr3() { pos_update(encoderArgs[1]); }
	static void isr4() { uint64_t now = micros(); pos_update(encoderArgs[2]); velocity_update(encoderArgs[2], now); }
	static void isr5() { pos_update(encoderArgs[2]); }
	static void isr6() { uint64_t now = micros(); pos_update(encoderArgs[3]); velocity_update(encoderArgs[3], now); }
	static void isr7() { pos_update(encoderArgs[3]); }

	// timer interrupt ISR (calling generic functions for each port)
	static bool timerisr(struct repeating_timer* t)
	{
		for (int i = 0; i < 4; i++)
		{
			if (ports_started[i])
				pid_update(pidArgs[i], encoderArgs[i]);
		}
		return true;
	}
};

typedef struct
{
	//DRIVEBASE CHARACTERISTICS
	volatile double max_rpm;
	volatile double max_dps;
	volatile double max_turn_rate;
	volatile double max_speed;
	volatile double wheel_dia;
	volatile double axle_track;
	EVNMotor* motor_left;
	EVNMotor* motor_right;

	volatile double speed_accel;
	volatile double speed_decel;
	volatile double turn_rate_accel;
	volatile double turn_rate_decel;

	//CONTROLLERS
	PIDController* turn_rate_pid;
	PIDController* speed_pid;

	//LOOP
	volatile uint64_t last_update;
	volatile uint8_t stop_action;

	//USER-SET
	volatile double target_speed;
	volatile double target_turn_rate;

	volatile double target_speed_constrained;
	volatile double target_turn_rate_constrained;

	volatile bool drive;
	volatile bool drive_position;

	//DRIVE
	volatile double target_angle;
	volatile double target_distance;
	// volatile double target_position_x;
	// volatile double target_position_y;

	volatile double angle_to_target;
	volatile double angle_error;
	volatile double angle_output;

	volatile double speed_error;
	volatile double speed_output;

	volatile double target_motor_left_dps;
	volatile double target_motor_right_dps;

	//DRIVE POSITION
	// volatile double start_position_x;
	// volatile double start_position_y;
	// volatile double start_angle;
	// volatile double start_distance;
	// volatile double end_position_x;
	// volatile double end_position_y;
	volatile double end_angle;
	volatile double end_distance;

	//ODOMETRY
	volatile double current_distance;
	volatile double prev_distance;
	volatile double current_angle;
	volatile double position_x;
	volatile double position_y;
	// volatile double motor_left_x;
	// volatile double motor_left_y;
	// volatile double motor_right_x;
	// volatile double motor_right_y;
	volatile uint8_t counter;
}	drivebase_state_t;

class EVNDrivebase
{
private:
	static const uint8_t MAX_DB_OBJECTS = 2;
	static const uint16_t PID_TIMER_INTERVAL_US = 1000;

public:
	EVNDrivebase(double wheel_dia, double axle_track, EVNMotor* motor_left, EVNMotor* motor_right);
	void begin();

	void setSpeedPID(double kp, double ki, double kd);
	void setTurnRatePID(double kp, double ki, double kd);
	void setSpeedAccel(double speed_accel);
	void setSpeedDecel(double speed_decel);
	void setTurnRateAccel(double turn_rate_accel);
	void setTurnRateDecel(double turn_rate_decel);

	double getDistance();
	double getAngle();
	double getHeading();
	double getX();
	double getY();
	void resetXY();
	double getDistanceToPoint(double x, double y);

	void drive(double speed, double turn_rate);
	void driveTurnRate(double speed, double turn_rate);
	void driveRadius(double speed, double radius);
	void straight(double speed, double distance, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void curve(double speed, double radius, double angle, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void curveRadius(double speed, double radius, double angle, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void curveTurnRate(double speed, double turn_rate, double angle, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void turn(double turn_rate, double degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void turnDegrees(double turn_rate, double degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void turnHeading(double turn_rate, double heading, uint8_t stop_action = STOP_BRAKE, bool wait = true);
	void driveToXY(double speed, double turn_rate, double x, double y, uint8_t stop_action = STOP_BRAKE, bool restore_initial_heading = true);

	void stop();
	void brake();
	void coast();
	void hold();

	bool completed();

	//TODO: add end function

private:
	double clean_input_turn_rate(double turn_rate);
	double clean_input_speed(double speed, double turn_rate);
	uint8_t clean_input_stop_action(uint8_t stop_action);
	double scaling_factor_for_maintaining_radius(double speed, double turn_rate);
	double radius_to_turn_rate(double speed, double radius);
	void enable_drive_position(uint8_t stop_action, bool wait);

	drivebase_state_t db;
	static drivebase_state_t* dbArgs[MAX_DB_OBJECTS];
	static bool dbs_enabled[MAX_DB_OBJECTS];

	static void attach_db_interrupt(drivebase_state_t* arg)
	{
		bool timerisr_started = false;
		for (int i = 0; i < MAX_DB_OBJECTS; i++)
		{
			if (!dbs_enabled[i])
			{
				dbArgs[i] = arg;
				dbs_enabled[i] = true;
			}
			else
				timerisr_started = true;
		}

		if (!timerisr_started)
			add_repeating_timer_us(-PID_TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer::sharedISRTimer(1));
	}

	static bool timerisr(struct repeating_timer* t) {

		for (int i = 0; i < MAX_DB_OBJECTS; i++)
		{
			if (dbs_enabled[i])
				pid_update(dbArgs[i]);
		}
		return true;
	}

	static void stopAction_static(drivebase_state_t* arg)
	{
		switch (arg->stop_action)
		{
		case STOP_BRAKE:
			arg->motor_left->brake();
			arg->motor_right->brake();
			break;
		case STOP_COAST:
			arg->motor_left->coast();
			arg->motor_right->coast();
			break;
		case STOP_HOLD:
			arg->motor_left->hold();
			arg->motor_right->hold();
			break;
		}

		arg->target_speed_constrained = 0;
		arg->target_turn_rate_constrained = 0;

		arg->target_angle = getAngle_static(arg);
		arg->target_distance = getDistance_static(arg);
		// arg->target_position_x = arg->position_x;
		// arg->target_position_y = arg->position_y;

		arg->turn_rate_pid->reset();
		arg->speed_pid->reset();

		arg->drive = false;
		arg->drive_position = false;
	}

	static double getDistance_static(drivebase_state_t* arg)
	{
		return (arg->motor_right->getPosition() + arg->motor_left->getPosition()) * arg->wheel_dia / 2 / 360 * M_PI;
	}

	static double getAngle_static(drivebase_state_t* arg)
	{
		return (arg->motor_right->getPosition() - arg->motor_left->getPosition()) * arg->wheel_dia / (2 * arg->axle_track);
	}

	static bool motors_enabled()
	{
		return ((EVNAlpha::sharedButton().read() && EVNAlpha::sharedButton().sharedState()->link_motors) || !EVNAlpha::sharedButton().sharedState()->link_motors);
	}

	static void pid_update(drivebase_state_t* arg)
	{
		uint64_t now = micros();
		double time_since_last_loop = ((double)now - (double)arg->last_update) / 1000000;
		arg->last_update = now;

		arg->current_angle = getAngle_static(arg);
		arg->current_distance = getDistance_static(arg);
		double distance_travelled_in_last_loop = arg->current_distance - arg->prev_distance;
		arg->prev_distance = arg->current_distance;

		arg->position_x += distance_travelled_in_last_loop * cos(arg->current_angle / 180 * M_PI);
		arg->position_y += distance_travelled_in_last_loop * sin(arg->current_angle / 180 * M_PI);
		// arg->motor_left_x = arg->position_x + 0.5 * arg->axle_track * cos((arg->current_angle + 90) / 180 * M_PI);
		// arg->motor_left_y = arg->position_y + 0.5 * arg->axle_track * sin((arg->current_angle + 90) / 180 * M_PI);
		// arg->motor_right_x = arg->position_x + 0.5 * arg->axle_track * cos((arg->current_angle - 90) / 180 * M_PI);
		// arg->motor_right_y = arg->position_y + 0.5 * arg->axle_track * sin((arg->current_angle - 90) / 180 * M_PI);

		if (motors_enabled() && arg->drive)
		{
			double target_speed_after_decel = arg->target_speed;
			double target_turn_rate_after_decel = arg->target_turn_rate;

			//calculation of speed & turn rate when motor is coming to a pause
			if (arg->drive_position)
			{
				double stop_speed_decel = arg->speed_decel;
				double stop_turn_rate_decel = arg->turn_rate_decel;
				double stop_time_to_decel_speed = fabs(arg->target_speed) / arg->speed_decel;
				double stop_time_to_decel_turn_rate = fabs(arg->target_turn_rate) / arg->turn_rate_decel;

				//stretch either decel to match the slower one
				if (stop_time_to_decel_speed != 0 && stop_time_to_decel_turn_rate != 0)
				{
					if (stop_time_to_decel_speed > stop_time_to_decel_turn_rate)
						stop_turn_rate_decel *= stop_time_to_decel_turn_rate / stop_time_to_decel_speed;
					else if (stop_time_to_decel_speed > stop_time_to_decel_turn_rate)
						stop_speed_decel *= stop_time_to_decel_speed / stop_time_to_decel_turn_rate;
				}

				//calculate what the current speed & turn rate should be, based on respective error
				double error_to_start_decel_speed = fabs(pow(arg->target_speed, 2) / 2 / stop_speed_decel);
				double error_to_start_decel_turn_rate = fabs(pow(arg->target_turn_rate, 2) / 2 / stop_turn_rate_decel);
				double current_distance_error = fabs(arg->end_distance - arg->current_distance);
				double current_angle_error = fabs(arg->end_angle - arg->current_angle);

				if (current_distance_error < error_to_start_decel_speed)
					target_speed_after_decel = arg->target_speed * fabs(sqrt(current_distance_error / error_to_start_decel_speed));

				if (current_angle_error < error_to_start_decel_turn_rate)
					target_turn_rate_after_decel = arg->target_turn_rate * fabs(sqrt(current_angle_error / error_to_start_decel_turn_rate));
			}

			// arg->angle_to_target = atan2(arg->target_position_y - arg->position_y, arg->target_position_x - arg->position_x);
			// arg->angle_to_target = arg->angle_to_target / M_PI * 180;
			// if (arg->angle_to_target < 0)
			// 	arg->angle_to_target += 360;
			// double error_att_to_cur = arg->angle_to_target - fmod(fmod(arg->current_angle, 360) + 360, 360);
			// if (error_att_to_cur > 180)
			//  error_att_to_cur -= 360;
			// if (error_att_to_cur < -180)
			//  error_att_to_cur += 360;
			// error_att_to_cur /= 180;
			// angle error->difference between angle to target and current angle(-1 to 1)
			// double cx = arg->target_position_x + cos(arg->target_angle / 180 * M_PI);
			// double cy = arg->target_position_y + sin(arg->target_angle / 180 * M_PI);
			// double perpendicular_distance = fabs((cx - arg->target_position_x) * (arg->position_y - arg->target_position_y) - (cy - arg->target_position_y) * (arg->position_x - arg->target_position_x));

			//angle error = the difference between the robot's current angle and the angle it should travel at
			arg->angle_error = arg->target_angle - arg->current_angle;
			if (arg->angle_error > 180)
				arg->angle_error -= 360;
			if (arg->angle_error < -180)
				arg->angle_error += 360;
			arg->angle_error /= 180;

			arg->angle_error = constrain(arg->angle_error, -1, 1);
			arg->angle_output = arg->turn_rate_pid->compute(arg->angle_error, true, false, true);

			//speed error is the euclidean distance between the db position and its target (converted to motor degrees)
			arg->speed_error = arg->target_distance - arg->current_distance;
			arg->speed_error = arg->speed_error / M_PI / arg->wheel_dia * 360;
			arg->speed_output = arg->speed_pid->compute(arg->speed_error, false, false, true);

			//calculate motor speeds
			//speed output   -> average speed
			//heading output -> difference between speeds
			arg->target_motor_left_dps = (arg->speed_output - arg->angle_output) * arg->max_dps;
			arg->target_motor_right_dps = (arg->speed_output + arg->angle_output) * arg->max_dps;

			//maintain ratio between speeds when either speed exceeds motor limits
			double ratio = arg->target_motor_left_dps / arg->target_motor_right_dps;
			if (fabs(arg->target_motor_left_dps) > arg->max_dps)
			{
				if (arg->target_motor_left_dps > 0)
					arg->target_motor_left_dps = arg->max_dps;
				else
					arg->target_motor_left_dps = -arg->max_dps;
				arg->target_motor_right_dps = arg->target_motor_left_dps / ratio;
			}

			if (fabs(arg->target_motor_right_dps) > arg->max_dps)
			{
				if (arg->target_motor_right_dps > 0)
					arg->target_motor_right_dps = arg->max_dps;
				else
					arg->target_motor_right_dps = -arg->max_dps;
				arg->target_motor_left_dps = arg->target_motor_right_dps * ratio;
			}

			//write speeds to motors
			arg->motor_left->runSpeed(arg->target_motor_left_dps);
			arg->motor_right->runSpeed(arg->target_motor_right_dps);

			//stored to later check if target angle & dist is going beyond the end angle & dist
			double target_dist_from_end, target_angle_from_end;
			if (arg->drive_position)
			{
				target_dist_from_end = fabs(arg->target_distance - arg->end_distance);
				target_angle_from_end = fabs(arg->target_angle - arg->end_angle);
			}

			//increment target angle and XY position
			//if output of speed or turn rate output is saturated or motors are stalled, stop incrementing (avoid excessive overshoot that PID cannot correct)
			if (fabs(arg->speed_error) * arg->speed_pid->getKp() < 1 && fabs(arg->angle_error) * arg->turn_rate_pid->getKp() < 1
				&& !arg->motor_left->stalled() && !arg->motor_right->stalled())
			{
				//calculating time taken to decel/accel to target speed & turn rate
				double new_speed_accel = arg->speed_accel;
				double new_speed_decel = arg->speed_decel;
				double new_turn_rate_accel = arg->turn_rate_accel;
				double new_turn_rate_decel = arg->turn_rate_decel;
				double time_to_decel_speed = 0;
				double time_to_accel_speed = 0;
				bool stretch_speed_accel = true;
				double time_to_decel_turn_rate = 0;
				double time_to_accel_turn_rate = 0;
				bool stretch_turn_rate_accel = true;

				if (target_speed_after_decel >= 0 && arg->target_speed_constrained >= 0)
				{
					double difference = target_speed_after_decel - arg->target_speed_constrained;
					if (difference > 0)
						time_to_accel_speed = fabs(difference) / arg->speed_accel;
					else
					{
						stretch_speed_accel = false;
						time_to_decel_speed = fabs(difference) / arg->speed_decel;
					}
				}
				else if (target_speed_after_decel <= 0 && arg->target_speed_constrained <= 0)
				{
					double difference = target_speed_after_decel - arg->target_speed_constrained;
					if (difference < 0)
						time_to_accel_speed = fabs(difference) / arg->speed_accel;
					else
					{
						stretch_speed_accel = false;
						time_to_decel_speed = fabs(difference) / arg->speed_decel;
					}
				}
				else
				{
					time_to_decel_speed = fabs(0 - arg->target_speed_constrained) / arg->speed_decel;
					time_to_accel_speed = fabs(target_speed_after_decel - 0) / arg->speed_accel;
				}

				if (target_turn_rate_after_decel >= 0 && arg->target_turn_rate_constrained >= 0)
				{
					double difference = target_turn_rate_after_decel - arg->target_turn_rate_constrained;
					if (difference > 0)
						time_to_accel_turn_rate = fabs(difference) / arg->turn_rate_accel;
					else
					{
						stretch_turn_rate_accel = false;
						time_to_decel_turn_rate = fabs(difference) / arg->turn_rate_decel;
					}
				}
				else if (target_turn_rate_after_decel < 0 && arg->target_turn_rate_constrained < 0)
				{
					double difference = target_turn_rate_after_decel - arg->target_turn_rate_constrained;
					if (difference < 0)
						time_to_accel_turn_rate = fabs(difference) / arg->turn_rate_accel;
					else
					{
						stretch_turn_rate_accel = false;
						time_to_decel_turn_rate = fabs(difference) / arg->turn_rate_decel;
					}
				}
				else
				{
					time_to_decel_turn_rate = fabs(0 - arg->target_turn_rate_constrained) / arg->turn_rate_decel;
					time_to_accel_turn_rate = fabs(target_turn_rate_after_decel - 0) / arg->turn_rate_accel;
				}

				//stretch the accel/decel trajectories for turn rate and speed to match each other
				double new_time;

				if (time_to_accel_speed + time_to_decel_speed != 0 && time_to_decel_turn_rate + time_to_accel_turn_rate != 0)
				{
					if (time_to_accel_speed + time_to_decel_speed > time_to_decel_turn_rate + time_to_accel_turn_rate)
					{
						//slow down turn rate to match time taken to hit target speed
						new_time = time_to_accel_speed + time_to_decel_speed;
						if (stretch_turn_rate_accel)
						{
							new_time -= time_to_decel_turn_rate;
							if (new_time != 0)
								new_turn_rate_accel *= constrain(time_to_accel_turn_rate / new_time, 0, 1);
						}
						else
						{
							new_time -= time_to_accel_turn_rate;
							if (new_time != 0)
								new_turn_rate_decel *= constrain(time_to_decel_turn_rate / new_time, 0, 1);
						}
					}
					else
					{
						//slow down speed to match time taken to hit target turn_rate
						new_time = time_to_decel_turn_rate + time_to_accel_turn_rate;
						if (stretch_speed_accel)
						{
							new_time -= time_to_decel_speed;
							if (new_time != 0)
								new_speed_accel *= constrain(time_to_accel_speed / new_time, 0, 1);
						}
						else
						{
							new_time -= time_to_accel_speed;
							if (new_time != 0)
								new_speed_decel *= constrain(time_to_decel_speed / new_time, 0, 1);
						}
					}
				}

				//apply accel/decel to hit target speed
				if (arg->target_speed_constrained <= target_speed_after_decel)
				{
					if (target_speed_after_decel > 0)
						arg->target_speed_constrained += time_since_last_loop * new_speed_accel;
					else
						arg->target_speed_constrained += time_since_last_loop * new_speed_decel;

					if (arg->target_speed_constrained > target_speed_after_decel)
						arg->target_speed_constrained = target_speed_after_decel;
				}
				else
				{
					if (target_speed_after_decel > 0)
						arg->target_speed_constrained -= time_since_last_loop * new_speed_decel;
					else
						arg->target_speed_constrained -= time_since_last_loop * new_speed_accel;

					if (arg->target_speed_constrained < target_speed_after_decel)
						arg->target_speed_constrained = target_speed_after_decel;
				}

				//apply accel/decel to hit target turn rate
				if (arg->target_turn_rate_constrained <= target_turn_rate_after_decel)
				{
					if (target_turn_rate_after_decel > 0)
						arg->target_turn_rate_constrained += time_since_last_loop * new_turn_rate_accel;
					else
						arg->target_turn_rate_constrained += time_since_last_loop * new_turn_rate_decel;

					if (arg->target_turn_rate_constrained > target_turn_rate_after_decel)
						arg->target_turn_rate_constrained = target_turn_rate_after_decel;
				}
				else
				{
					if (target_turn_rate_after_decel > 0)
						arg->target_turn_rate_constrained -= time_since_last_loop * new_turn_rate_decel;
					else
						arg->target_turn_rate_constrained -= time_since_last_loop * new_turn_rate_accel;

					if (arg->target_turn_rate_constrained < target_turn_rate_after_decel)
						arg->target_turn_rate_constrained = target_turn_rate_after_decel;
				}

				//increment target angle and distance
				arg->target_angle += time_since_last_loop * arg->target_turn_rate_constrained;
				arg->target_distance += time_since_last_loop * arg->target_speed_constrained;
				// arg->target_position_x += time_since_last_loop * arg->target_speed * cos(arg->target_angle / 180 * M_PI) * (1 - fabs(arg->angle_output));
				// arg->target_position_y += time_since_last_loop * arg->target_speed * sin(arg->target_angle / 180 * M_PI) * (1 - fabs(arg->angle_output));
			}

			if (arg->drive_position)
			{
				double new_target_dist_from_end = fabs(arg->target_distance - arg->end_distance);
				double new_target_angle_from_end = fabs(arg->target_angle - arg->end_angle);

				if (new_target_dist_from_end > target_dist_from_end ||
					new_target_angle_from_end > target_angle_from_end)
				{
					arg->target_distance = arg->end_distance;
					arg->target_angle = arg->end_angle;
				}

				if (fabs(arg->end_angle - arg->current_angle) <= USER_DRIVE_POS_MIN_ERROR_DEG
					&& fabs(arg->end_distance - arg->current_distance) <= USER_DRIVE_POS_MIN_ERROR_MM)
					arg->counter++;
				else
					arg->counter = 0;

				if (arg->counter >= USER_DRIVE_POS_MIN_LOOP_COUNT)
					stopAction_static(arg);
			}
		}
		else
		{
			arg->stop_action = STOP_BRAKE;
			stopAction_static(arg);
		}
	}
};

#endif