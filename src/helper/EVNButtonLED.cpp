#include "EVNButtonLED.h"

button_led_state_t EVNButtonLED::button;

EVNButtonLED::EVNButtonLED(uint8_t mode, bool link_led, bool link_movement, bool button_invert)
{
	button.state = false;

	button.mode = constrain(mode, 0, 2);
	button.link_led = link_led;
	button.link_movement = link_movement;
	button.button_invert = button_invert;

	if (button.mode == BUTTON_TOGGLE && button.button_invert)
		button.state = !button.state;

	button.flash = false;
}

void EVNButtonLED::begin()
{
	//use button.started to ensure that setup only occurs once
	if (!button.started)
	{
		//button pin change interrupt
		pinMode(BUTTONPIN, INPUT_PULLUP);
		attachInterrupt(BUTTONPIN, isr, CHANGE);

		//led timer interrupt
		pinMode(LEDPIN, OUTPUT_8MA);
		if (rp2040.cpuid == 0)
			alarm_pool_add_repeating_timer_ms(EVNISRTimer0::sharedAlarmPool(), UPDATE_INTERVAL_MS, update, NULL, &EVNISRTimer0::sharedISRTimer(2));
		else
			alarm_pool_add_repeating_timer_ms(EVNISRTimer1::sharedAlarmPool(), UPDATE_INTERVAL_MS, update, NULL, &EVNISRTimer1::sharedISRTimer(2));
		button.started = true;
	}
}

bool EVNButtonLED::read() { return (button.mode == BUTTON_DISABLE) ? true : button.state; }

void EVNButtonLED::setMode(uint8_t mode) { button.mode = constrain(mode, 0, 2); }
uint8_t EVNButtonLED::getMode() { return button.mode; }

void EVNButtonLED::setLinkLED(bool enable) { button.link_led = enable; }
bool EVNButtonLED::getLinkLED() { return button.link_led; }

void EVNButtonLED::setLinkMovement(bool enable) { button.link_movement = enable; }
bool EVNButtonLED::getLinkMovement() { return button.link_movement; }

void EVNButtonLED::setButtonInvert(bool enable)
{
	if (button.mode == BUTTON_TOGGLE && enable != button.button_invert)
		button.state = !button.state;
	button.button_invert = enable;
}

bool EVNButtonLED::getButtonInvert() { return button.button_invert; }

void EVNButtonLED::setFlash(bool enable) { button.flash = enable; }
bool EVNButtonLED::getFlash() { return button.flash; }

