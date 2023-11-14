#ifndef EVNSensorHelper_h
#define EVNSensorHelper_h

#include <Arduino.h>
#include <Wire.h>

#define TCAADDR 0x70
#define WIRE0_SDA 4
#define WIRE0_SCL 5
#define WIRE1_SDA 6
#define WIRE1_SCL 7
#define SERIAL1_RX 1
#define SERIAL1_TX 0
#define SERIAL2_RX 9
#define SERIAL2_TX 8

class EVNSensorHelper
{
public:
	EVNSensorHelper();
	void init();
	void selectPort(uint8_t bus, uint8_t port);
	uint8_t getPort(uint8_t bus);
	double calibrate(double reading, double low, double high);
	void printPortScanner();

private:
	uint8_t _wire0SensorPort, _wire1SensorPort;
};

#endif