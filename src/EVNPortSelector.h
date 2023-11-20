#ifndef EVNPortSelector_h
#define EVNPortSelector_h

#include <Arduino.h>
#include <Wire.h>
#include "pins_evn_alpha.h"

#define TCAADDR 0x70

class EVNPortSelector
{
public:
	EVNPortSelector();
	void begin();
	void setPort(uint8_t port);
	uint8_t getPort();
	void printPorts();

private:
	uint8_t _wire0SensorPort = 1, _wire1SensorPort = 9, _port = 1;
};

#endif