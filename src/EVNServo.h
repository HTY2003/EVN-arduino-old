#ifndef EVNServo_h
#define EVNServo_h

#include <Arduino.h>
#include <EVNAlpha.h>
#include <EVNISRTimer.h>
#include <Servo.h>
#include "evn_alpha_pins.h"

#define DIRECT	1
#define REVERSE	0

typedef struct
{
	Servo* servo;
	volatile bool sweep;
	volatile double pulse;
	volatile bool servo_dir;
	volatile uint8_t pin;
	volatile uint8_t port;
	volatile uint16_t range;
	volatile uint16_t min_pulse_us;
	volatile uint16_t max_pulse_us;
	volatile double angle;
	volatile double end_angle;
	volatile double dps;
	volatile double max_dps;
	volatile uint64_t last_loop;
}	servo_state_t;

class EVNServo
{
public:

	static const uint16_t TIMER_INTERVAL_US = 10000;

	EVNServo(uint8_t port, bool servo_dir = DIRECT, uint16_t range = 270, double start_angle = 135, uint16_t min_pulse_us = 600, uint16_t max_pulse_us = 2400, double max_dps = 500);
	void begin();
	void write(double angle, uint16_t wait_time_ms = 0, double dps = 0);
	void writeAngle(double angle, uint16_t wait_time_ms = 0, double dps = 0);
	void writeMicroseconds(uint16_t pulse_us, uint16_t wait_time_ms = 0);
	void writeDutyCycle(double duty_cycle);
	uint16_t getRange() { return _servo.range; };
	double getMaxDPS() { return _servo.max_dps; };

private:
	servo_state_t _servo;
	static servo_state_t* servoArgs[4];
	static bool servos_enabled[4];

	static void attach_servo_interrupt(servo_state_t* arg)
	{
		if (!servos_enabled[0] && !servos_enabled[1] && !servos_enabled[2] && !servos_enabled[3])
			alarm_pool_add_repeating_timer_us(EVNISRTimer::sharedAlarmPool(), -TIMER_INTERVAL_US, timerisr, NULL, &EVNISRTimer::sharedISRTimer(1));

		servoArgs[arg->port - 1] = arg;
		servos_enabled[arg->port - 1] = true;
	}

	static bool timerisr(struct repeating_timer* t)
	{
		for (int i = 0; i < 4; i++)
		{
			if (servos_enabled[i])
				update(servoArgs[i]);
		}
		return true;
	}

	static void update(servo_state_t* arg)
	{
		uint64_t now = micros();
		double time_since_last_loop = ((double)now - (double)arg->last_loop) / 1000000;
		arg->last_loop = now;

		if (motors_enabled())
		{
			if (arg->sweep)
			{
				if (arg->angle == arg->end_angle)
					arg->sweep = false;

				else
				{
					double deg_per_loop = arg->dps * time_since_last_loop;
					if (arg->angle > arg->end_angle)
					{
						arg->angle -= deg_per_loop;
						if (arg->angle < arg->end_angle) arg->angle = arg->end_angle;
					}

					if (arg->angle < arg->end_angle)
					{
						arg->angle += deg_per_loop;
						if (arg->angle > arg->end_angle) arg->angle = arg->end_angle;
					}

					double pulse = (double)(arg->angle / arg->range) * (double)(arg->max_pulse_us - arg->min_pulse_us);
					if (arg->servo_dir == DIRECT)
						pulse = (double)arg->min_pulse_us + pulse;
					else
						pulse = (double)arg->max_pulse_us - pulse;
					arg->servo->writeMicroseconds(pulse);
				}
			}
			else
			{
				arg->servo->writeMicroseconds(arg->pulse);
			}
		}
		else
		{
			arg->sweep = false;
		}
	}

	static bool motors_enabled()
	{
		return ((EVNAlpha::sharedButton().read() && EVNAlpha::sharedButton().sharedState()->link_motors)

			|| !EVNAlpha::sharedButton().sharedState()->link_motors);
	}
};

#endif