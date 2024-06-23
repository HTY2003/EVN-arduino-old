#include "EVNButton.h"

button_state_t EVNButton::button;

EVNButton::EVNButton(uint8_t mode, bool link_led, bool link_movement)
{
	button.state = false;
	button.mode = constrain(mode, 0, 2);
	button.link_led = link_led;
	button.link_movement = link_movement;
	button.flash = false;
}

void EVNButton::begin()
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

bool EVNButton::read() { return (button.mode == BUTTON_DISABLE) ? true : button.state; }
void EVNButton::setMode(uint8_t mode) { button.mode = constrain(mode, 0, 2); }
void EVNButton::setLinkLED(bool enable) { button.link_led = enable; }
void EVNButton::setLinkMovement(bool enable) { button.link_movement = enable; }

void EVNButton::setFlash(bool enable)
{
	if (!button.flash) button.flash_counter = 0;
	button.flash = enable;
}

bool EVNButton::getFlash() { return button.flash; }

