#include <EVNSensorHelper.h>
#include <Arduino.h>
#include <Wire.h>

// caa 051123

EVNSensorHelper::EVNSensorHelper()
{
}

void EVNSensorHelper::init()
{
	Wire.setSDA(WIRE0_SDA);
	Wire.setSCL(WIRE0_SCL);
	Wire1.setSDA(WIRE1_SDA);
	Wire1.setSCL(WIRE1_SCL);

	Serial1.setRX(SERIAL1_RX);
	Serial1.setTX(SERIAL1_TX);
	Serial2.setRX(SERIAL2_RX);
	Serial2.setTX(SERIAL2_TX);

	Wire.begin();
	Wire1.begin();
	Serial.begin();
}

void EVNSensorHelper::printPortScanner()
{
	for (uint8_t t = 0; t < 8; t++)
	{
		this->selectPort(0, t);
		Serial.print("---Bus 0 Port #");
		Serial.print(t);
		Serial.println("---");

		for (uint8_t addr = 0; addr <= 127; addr++)
		{
			if (addr == TCAADDR)
				continue;
			Wire.beginTransmission(addr);
			if (!Wire.endTransmission())
			{
				Serial.print("Address 0x");
				Serial.print(addr, HEX);
				Serial.println(" found");
			}
		}
	}

	for (uint8_t t = 0; t < 8; t++)
	{
		this->selectPort(1, t);
		Serial.print("Bus 1 Port #");
		Serial.println(t);

		for (uint8_t addr = 0; addr <= 127; addr++)
		{
			if (addr == TCAADDR)
				continue;
			Wire1.beginTransmission(addr);
			if (!Wire1.endTransmission())
			{
				Serial.print("Address 0x");
				Serial.print(addr, HEX);
				Serial.println(" found");
			}
		}
	}
}

void EVNSensorHelper::selectPort(uint8_t bus, uint8_t port)
{
	if (bus > 1 || port > 7)
		return;

	if (bus == 0)
	{
		Wire.beginTransmission(TCAADDR);
		Wire.write(1 << port);
		Wire.endTransmission();
		_wire0SensorPort = port;
	}
	else
	{
		Wire1.beginTransmission(TCAADDR);
		Wire1.write(1 << port);
		Wire1.endTransmission();
		_wire1SensorPort = port;
	}
}

uint8_t EVNSensorHelper::getPort(uint8_t bus)
{
	if (bus > 1)
		return -1;

	return (bus == 0) ? _wire0SensorPort : _wire1SensorPort;
}