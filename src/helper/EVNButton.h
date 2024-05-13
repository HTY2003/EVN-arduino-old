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
	volatile uint64_t last_pressed = -10000;
	volatile bool link_led;
	volatile bool link_motors;
	volatile uint8_t mode;
	volatile uint8_t flash;
	volatile uint8_t flash_counter;
} button_state_t;

class EVNButton
{
private:
	static const uint16_t DEBOUNCE_TIMING_MS = 300;
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
		uint8_t reading = digitalRead(BUTTONPIN);

		//falling edge (button pressed, debounced)
		if (!reading)
		{
			if ((millis() - button.last_pressed) > DEBOUNCE_TIMING_MS)
			{
				if (button.mode == BUTTON_PUSHBUTTON)
					button.state = true;
				else if (button.mode == BUTTON_TOGGLE)
					button.state = !button.state;

				button.last_pressed = millis();
			}
		}
		//rising edge (button released)
		else {
			if (button.mode == BUTTON_PUSHBUTTON)
				button.state = false;
		}
	}
	static bool pidtimer(struct repeating_timer* t)
	{
		if (button.flash)
		{
			button.flash_counter = !button.flash_counter;
			digitalWrite(LEDPIN, button.flash_counter);
		}

		//ensures that LED reflects output of read()
		if (button.link_led) digitalWrite(LEDPIN, button.state); return true;
	}
};

#endif