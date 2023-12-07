#include "EVNAlpha.h"

EVNButton EVNAlpha::button(_mode, _linkLED, _linkMotors);
EVNPortSelector EVNAlpha::ports(_i2c_freq);
uint8_t EVNAlpha::_mode;
uint8_t EVNAlpha::_linkLED;
uint8_t EVNAlpha::_linkMotors;
uint16_t EVNAlpha::_i2c_freq;

EVNAlpha::EVNAlpha(uint8_t mode, uint8_t linkLED, uint8_t linkMotors, uint16_t i2c_freq)
{
    _mode = constrain(mode, 0, 2);
    _linkLED = constrain(linkLED, 0, 1);
    _linkMotors = constrain(linkMotors, 0, 1);
    _i2c_freq = i2c_freq;
}

void EVNAlpha::begin()
{
    pinMode(LEDPIN, OUTPUT_8MA);
    ports.begin();
    button.begin();
}