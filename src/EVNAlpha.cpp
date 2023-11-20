#include "EVNAlpha.h"

EVNButton EVNAlpha::button(_mode, _linkLED);
EVNPortSelector EVNAlpha::ports;

uint8_t EVNAlpha::_mode;
uint8_t EVNAlpha::_linkLED;

EVNAlpha::EVNAlpha(uint8_t mode, uint8_t linkLED)
{
    _mode = constrain(mode, 0, 2);
    _linkLED = constrain(linkLED, 0, 1);
}

void EVNAlpha::begin()
{
    pinMode(LEDPIN, OUTPUT_8MA);
    ports.begin();
    button.begin();
}