#ifndef EVNButton_h
#define EVNButton_h

#include <Arduino.h>
#define BUTTONPIN 24
#define DEBOUNCE_TIMING_MS 300

typedef struct
{
	volatile bool state;
	volatile bool link_led;
	volatile unsigned long last_pressed;
} button_state_t;

class EVNButton
{
public:
	EVNButton();
	void init();
	bool read();

	button_state_t button;
	static button_state_t* buttonArg; // static list of pointers to each instances' structs

private:
	static void attach_button_interrupt(button_state_t* arg)
	{
		buttonArg = arg;
		attachInterrupt(BUTTONPIN, isr, FALLING);
	}
	static void isr()
	{
		if ((millis() - buttonArg->last_pressed) > DEBOUNCE_TIMING_MS)
		{
			buttonArg->state = !buttonArg->state;
			buttonArg->last_pressed = millis();
		}
	}
};

#endif