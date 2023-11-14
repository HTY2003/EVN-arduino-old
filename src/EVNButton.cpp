#include <EVNButton.h>
#include <Arduino.h>

button_state_t* EVNButton::buttonArg;

EVNButton::EVNButton()
{
	button.state = false;
}

void EVNButton::init()
{
	attach_button_interrupt(&button);
	pinMode(BUTTONPIN, INPUT_PULLUP);
}

bool EVNButton::read()
{
	return button.state;
}