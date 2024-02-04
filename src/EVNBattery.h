#ifndef EVNBattery_h
#define EVNBattery_h

#include <Arduino.h>
#include <Wire.h>
#include <EVNSensor.h>

#define BQ25887_PORT 16
#define BATT_NOMINAL 0
#define BATT_CELL1 1
#define BATT_CELL2 2

class EVNBattery : private EVNSensor
{
private:
	static const uint8_t I2C_ADDR = 0x6A;
	static const uint8_t ID_REG_PART_NUMBER = 0x05;
	static const uint8_t PART_INFO_MASK = 0b01111000;

	enum class reg : uint8_t
	{
		ADC_CONTROL = 0x15,
		VBAT_ADC1 = 0x1D,
		VBAT_ADC0 = 0x1E,
		VCELLTOP_ADC1 = 0x1F,
		VCELLTOP_ADC0 = 0x20,
		PART_INFO = 0x25,
	};

	int16_t _vnom, _vcell1, _vcell2;

public:
	EVNBattery() : EVNSensor(BQ25887_PORT) {};

	bool begin()
	{
		EVNAlpha::sharedPorts().begin();

		//check for BQ25887 part number
		//NOTE: BQ25887 not in use for our V1.2 users
		uint8_t id = (read8(I2C_ADDR, (uint8_t)reg::PART_INFO) & PART_INFO_MASK) >> 3;
		if (id != ID_REG_PART_NUMBER) return false;

		//enable ADCs on BQ25887
		write8(I2C_ADDR, (uint8_t)reg::ADC_CONTROL, 0b10110000);

		return true;
	};

	int16_t getVoltage(uint8_t cell = BATT_NOMINAL)
	{
		uint8_t cellc = constrain(cell, 0, 2);

		_vnom = read16(I2C_ADDR, (uint8_t)reg::VBAT_ADC1, false);
		_vcell1 = read16(I2C_ADDR, (uint8_t)reg::VCELLTOP_ADC1, false);
		_vcell2 = _vnom - _vcell1;

		switch (cellc) {
		case BATT_NOMINAL:
			return _vnom;
			break;
		case BATT_CELL1:
			return _vcell1;
			break;
		case BATT_CELL2:
			return _vcell2;
			break;
		}
		return 0;
	};

	int16_t getBattVoltage() { return getVoltage(BATT_NOMINAL); };
	int16_t getCell1Voltage() { return getVoltage(BATT_CELL1); };
	int16_t getCell2Voltage() { return getVoltage(BATT_CELL2); };
};

#endif