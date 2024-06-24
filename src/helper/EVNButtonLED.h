#ifndef EVNButtonLED_h
#define EVNButtonLED_h

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
	volatile uint32_t last_change;

	volatile uint8_t mode;
	volatile bool link_led;
	volatile bool link_movement;
	volatile bool button_invert;

	volatile bool led_output;

	volatile bool flash;
	volatile uint8_t flash_counter;
	volatile uint32_t last_flash;
} button_led_state_t;

class EVNButtonLED
{
private:
	static const uint16_t DEBOUNCE_TIMING_MS = 10;
	static const uint16_t UPDATE_INTERVAL_MS = 100;
	static const uint16_t UPDATES_PER_FLASH = 10;
	static const uint16_t TIME_BETWEEN_FLASHES_MS = 3000;

public:
	EVNButtonLED(uint8_t mode = BUTTON_TOGGLE, bool link_led = true, bool link_movement = false, bool button_invert = false);
	void begin();
	bool read();
	void write(bool state);

	void setMode(uint8_t mode);
	void setLinkLED(bool enable);
	void setLinkMovement(bool enable);
	void setButtonInvert(bool enable);
	void setFlash(bool enable);

	uint8_t getMode();
	bool getLinkLED();
	bool getLinkMovement();
	bool getButtonInvert();
	bool getFlash();

	//singleton for state struct
	button_led_state_t* sharedState() { return &button; }

private:
	static button_led_state_t button;
	static void isr()
	{
		if (millis() - button.last_change > DEBOUNCE_TIMING_MS)
		{
			uint8_t reading = digitalRead(BUTTONPIN);

			if (button.mode == BUTTON_PUSHBUTTON)
			{
				//flip reading for button state, as pin is LOW when button is pressed
				if (button.button_invert)
					button.state = reading;
				else
					button.state = !reading;
			}
			else if (button.mode == BUTTON_TOGGLE)
			{
				//for toggle mode, the raw button reading is stored in substate instead of state
				//when button was not pressed (substate false) and is pressed now (reading false), state is flipped
				if (!button.substate && !reading)
					button.state = !button.state;

				button.substate = !reading;
			}
			button.last_change = millis();
		}

		//ensures that LED reflects output of read()
		if (button.link_led) button.led_output = button.state;
	}

	static bool update(struct repeating_timer* t)
	{
		if (button.flash && button.flash_counter > 0)
		{
			digitalWrite(LEDPIN, !digitalRead(LEDPIN));
			if (button.flash_counter == 1) button.last_flash = millis();
			button.flash_counter--;
		}

		else if (button.flash && millis() - button.last_flash > TIME_BETWEEN_FLASHES_MS)
			button.flash_counter = UPDATES_PER_FLASH;

		else
		{
			if (!button.flash) button.flash_counter = 0;
			isr();
			digitalWrite(LEDPIN, button.led_output);
		}

		return true;
	}
};

#endif