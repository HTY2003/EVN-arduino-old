#include <EVNButton.h>
#include <Arduino.h>

// caa 051123

button_state_t *EVNButton::buttonArg;

EVNButton::EVNButton()
{
	button.state = false;
}

void EVNButton::init()
{
	pinMode(BUTTONPIN, INPUT_PULLUP);
	attach_button_interrupt(&button);
}

bool EVNButton::read()
{
	return button.state;
}