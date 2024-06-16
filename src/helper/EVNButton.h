#ifndef EVNButton_h
#define EVNButton_h

#include <Arduino.h>
#include "EVNISRTimer.h"
#include "../evn_alpha_pins.h"

#define BUTTON_DISABLE		0
#define BUTTON_TOGGLE		1
#define BUTTON_PUSHBUTTON	2

typedef struct
{
	volatile bool started;
	volatile bool state;
	volatile bool substate;
	volatile uint32_t last_change = 0;
	volatile bool link_led;
	volatile bool link_motors;
	volatile uint8_t mode;
	volatile uint8_t flash;
	volatile uint8_t flash_counter;
} button_state_t;

class EVNButton
{
private:
	static const uint16_t DEBOUNCE_TIMING_MS = 5;
	static const uint16_t LED_MIN_INTERVAL_MS = 100;

public:
	EVNButton(uint8_t mode = BUTTON_TOGGLE, bool link_led = true, bool link_motors = true);
	void begin();
	bool read();
	void setMode(uint8_t mode);
	void setLinkLED(bool link_led);
	void setLinkMotors(bool link_motors);

	//singleton for state struct
	button_state_t* sharedState() { return &button; }

private:
	static button_state_t button;
	static void isr()
	{
		if (millis() - button.last_change > DEBOUNCE_TIMING_MS)
		{
			uint8_t reading = gpio_get(BUTTONPIN);

			if (button.mode == BUTTON_PUSHBUTTON)
			{
				button.state = !reading;
			}
			else if (button.mode == BUTTON_TOGGLE)
			{
				if (!button.substate && !reading) button.state = !button.state;
				button.substate = !reading;
			}
			button.last_change = millis();
		}
	}
	static bool pidtimer(struct repeating_timer* t)
	{
		if (button.flash)
		{
			button.flash_counter = !button.flash_counter;
			digitalWrite(LEDPIN, button.flash_counter);
		}

		if (millis() - button.last_change > DEBOUNCE_TIMING_MS
			&& gpio_get(BUTTONPIN))
		{
			if (button.mode == BUTTON_PUSHBUTTON && button.state)
				button.state = false;

			else if (button.mode == BUTTON_TOGGLE && button.substate)
				button.substate = false;

			button.last_change = millis();
		}

		//ensures that LED reflects output of read()
		if (button.link_led) digitalWrite(LEDPIN, button.state); return true;
	}
};

#endif