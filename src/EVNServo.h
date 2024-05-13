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
	volatile float pulse;
	volatile bool servo_dir;
	volatile uint8_t pin;
	volatile uint8_t port;
	volatile uint16_t range;
	volatile uint16_t min_pulse_us;
	volatile uint16_t max_pulse_us;
	volatile float angle;
	volatile float end_angle;
	volatile float dps;
	volatile float max_dps;
	volatile uint64_t last_loop;
}	servo_state_t;

class EVNServo
{
public:

	static const uint16_t TIMER_INTERVAL_US = 10000;

	EVNServo(uint8_t port, bool servo_dir = DIRECT, uint16_t range = 270, float start_angle = 135, uint16_t min_pulse_us = 600, uint16_t max_pulse_us = 2400, float max_dps = 500);
	void begin();
	void write(float angle, uint16_t wait_time_ms = 0, float dps = 0);
	void writeAngle(float angle, uint16_t wait_time_ms = 0, float dps = 0);
	void writeMicroseconds(uint16_t pulse_us, uint16_t wait_time_ms = 0);
	void writeDutyCycle(float duty_cycle);
	uint16_t getRange() { return _servo.range; };
	float getMaxDPS() { return _servo.max_dps; };

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
		float time_since_last_loop = ((float)now - (float)arg->last_loop) / 1000000;
		arg->last_loop = now;

		if (motors_enabled())
		{
			if (arg->sweep)
			{
				if (arg->angle == arg->end_angle)
					arg->sweep = false;

				else
				{
					float deg_per_loop = arg->dps * time_since_last_loop;
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

					float pulse = (float)(arg->angle / arg->range) * (float)(arg->max_pulse_us - arg->min_pulse_us);
					if (arg->servo_dir == DIRECT)
						pulse = (float)arg->min_pulse_us + pulse;
					else
						pulse = (float)arg->max_pulse_us - pulse;
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