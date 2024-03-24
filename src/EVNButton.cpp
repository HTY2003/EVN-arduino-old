#include "EVNButton.h"

button_state_t EVNButton::button;

EVNButton::EVNButton(uint8_t mode, bool link_led, bool link_motors)
{
	button.state = false;
	button.mode = constrain(mode, 0, 2);
	button.link_led = link_led;
	button.link_motors = link_motors;
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
		alarm_pool_add_repeating_timer_ms(EVNISRTimer::sharedAlarmPool(), LED_MIN_INTERVAL_MS, pidtimer, NULL, &EVNISRTimer::sharedISRTimer(5));

		button.started = true;
	}
}

bool EVNButton::read() { return (button.mode == BUTTON_DISABLE) ? true : button.state; }
void EVNButton::setMode(uint8_t mode) { button.mode = constrain(mode, 0, 2); }
void EVNButton::setLinkLED(bool link_led) { button.link_led = link_led; }
void EVNButton::setLinkMotors(bool link_motors) { button.link_motors = link_motors; }