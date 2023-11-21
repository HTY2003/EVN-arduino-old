#include "EVNButton.h"

button_state_t EVNButton::button;

EVNButton::EVNButton(uint8_t mode, uint8_t linkLED)
{
	button.state = false;
	button.mode = constrain(mode, 0, 2);
	button.linkLED = constrain(linkLED, 0, 1);
}

void EVNButton::begin()
{
	if (button.mode != BUTTON_DISABLE)
	{
		pinMode(BUTTONPIN, INPUT_PULLUP);
		attachInterrupt(BUTTONPIN, isr, FALLING);
		if (button.linkLED == LED_LINK)
			pinMode(LEDPIN, OUTPUT_8MA);
	}
}

bool EVNButton::read()
{
	if (button.mode == BUTTON_DISABLE)
	{
		return true;
	}
	if (button.mode == BUTTON_TOGGLE)
	{
		return button.state;
	}
	else {
		bool reading = false;
		if ((millis() - button.last_pressed) > DEBOUNCE_TIMING_MS) reading = digitalRead(BUTTONPIN);
		if (button.linkLED == LED_LINK)	digitalWrite(LEDPIN, !reading);
		return !reading;
	}
}

void EVNButton::setMode(uint8_t mode)
{
	button.mode = constrain(mode, 0, 2);
}

void EVNButton::setLinkLED(uint8_t linkLED)
{
	button.linkLED = constrain(linkLED, 0, 1);
}