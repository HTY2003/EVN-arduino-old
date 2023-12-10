#ifndef EVNButton_h
#define EVNButton_h

#include <Arduino.h>
#include "pins_evn_alpha.h"

#define DEBOUNCE_TIMING_MS	300

#define BUTTON_DISABLE		0
#define BUTTON_TOGGLE		1
#define BUTTON_PUSHBUTTON	2
#define LED_UNLINK			0
#define LED_LINK			1
#define MOTORS_UNLINK		0
#define MOTORS_LINK			1

typedef struct
{
	volatile bool state;
	volatile uint8_t linkLED;
	volatile uint8_t linkMotors;
	volatile uint8_t mode;
	volatile unsigned long last_pressed = -DEBOUNCE_TIMING_MS;
} button_state_t;

class EVNButton
{
public:
	EVNButton(uint8_t mode = BUTTON_TOGGLE, uint8_t linkLED = LED_LINK, uint8_t linkMotors = MOTORS_LINK);
	void begin();
	bool read();
	void setMode(uint8_t mode);
	void setLinkLED(uint8_t linkLED);
	void setLinkMotors(uint8_t linkMotors);

	static button_state_t button; // static list of pointers to each instances' structs

private:
	static void isr()
	{
		if ((millis() - button.last_pressed) > DEBOUNCE_TIMING_MS)
		{
			button.state = !button.state;
			if (button.mode == BUTTON_TOGGLE && button.linkLED == LED_LINK) digitalWrite(LEDPIN, button.state);
			button.last_pressed = millis();
		}
	}
};

#endif