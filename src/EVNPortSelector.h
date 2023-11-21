#ifndef EVNPortSelector_h
#define EVNPortSelector_h

#include <Arduino.h>
#include <Wire.h>
#include "pins_evn_alpha.h"

#define TCAADDR	0x70
#define DEFAULT_I2C_FREQ (uint16_t)400000

class EVNPortSelector
{
public:
	EVNPortSelector(uint16_t i2c_freq = DEFAULT_I2C_FREQ);
	void begin();
	void setPort(uint8_t port);
	uint8_t getPort();
	void printPorts();

private:
	uint8_t _wire0SensorPort = 1, _wire1SensorPort = 9, _port = 1;
	uint16_t _i2c_freq;
	bool _started = false;
};

#endif