#include "EVNButton.h"

button_state_t EVNButton::button;

EVNButton::EVNButton(uint8_t mode, uint8_t linkLED, uint8_t linkMotors)
{
	button.state = false;
	button.mode = constrain(mode, 0, 2);
	button.linkLED = constrain(linkLED, 0, 1);
	button.linkMotors = constrain(linkMotors, 0, 1);
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
		alarm_pool_add_repeating_timer_ms(EVNISRTimer::sharedAlarmPool(), LED_MIN_INTERVAL_MS, pidtimer, NULL, &EVNISRTimer::sharedISRTimer(5));

		button.started = true;
	}
}

bool EVNButton::read() { return (button.mode == BUTTON_DISABLE) ? true : button.state; }
void EVNButton::setMode(uint8_t mode) { button.mode = constrain(mode, 0, 2); }
void EVNButton::setLinkLED(uint8_t linkLED) { button.linkLED = constrain(linkLED, 0, 1); }
void EVNButton::setLinkMotors(uint8_t linkMotors) { button.linkMotors = constrain(linkMotors, 0, 1); }