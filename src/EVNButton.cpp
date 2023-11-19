#include "EVNButton.h"
#include <Arduino.h>

button_state_t* EVNButton::buttonArg;

EVNButton::EVNButton(bool toggle, bool linkLED)
{
	button.state = false;
	button.toggle = toggle;
	button.linkLED = linkLED;
}

void EVNButton::init()
{
	if (button.toggle)
	{
		attach_button_interrupt(&button);
	}
	pinMode(BUTTONPIN, INPUT_PULLUP);
	if (button.linkLED)
	{
		pinMode(LEDPIN, OUTPUT_8MA);
	}
}

bool EVNButton::read()
{
	if (button.toggle)
	{
		return button.state;
	}
	else {
		bool reading = digitalRead(BUTTONPIN);
		if (button.linkLED)
		{
			if (reading)
			{
				digitalWrite(LEDPIN, LOW);
			}
			else
			{
				digitalWrite(LEDPIN, HIGH);
			}
		}
		return reading;
	}
}