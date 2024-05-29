#include "EVNAlpha.h"

EVNButton EVNAlpha::button(_mode, _link_led, _link_motors);
EVNPortSelector EVNAlpha::ports(_i2c_freq);
uint8_t EVNAlpha::_mode;
bool EVNAlpha::_link_led;
bool EVNAlpha::_link_motors;
uint32_t EVNAlpha::_i2c_freq;

EVNAlpha::EVNAlpha(uint8_t mode, bool link_led, bool link_motors, uint32_t i2c_freq)
{
    _battery_adc_started = false;
    _mode = constrain(mode, 0, 2);
    _link_led = link_led;
    _link_motors = link_motors;
    _i2c_freq = i2c_freq;
}

void EVNAlpha::begin()
{
    //set correct I2C and Serial pins
    //TODO: to be removed after getting our board in arduino-pico core
    Wire.setSDA(WIRE0_SDA);
    Wire.setSCL(WIRE0_SCL);
    Wire1.setSDA(WIRE1_SDA);
    Wire1.setSCL(WIRE1_SCL);

    Serial1.setRX(SERIAL1_RX);
    Serial1.setTX(SERIAL1_TX);
    Serial2.setRX(SERIAL2_RX);
    Serial2.setTX(SERIAL2_TX);

    //initialize helper objects
    ports.begin();
    button.begin();

    //initialize battery ADC if available
    if (this->beginBatteryADC()) _battery_adc_started = true;
}

bool EVNAlpha::beginBatteryADC()
{
    this->sharedPorts().setPort(BQ25887_IC_I2C_PORT);

    //check for BQ25887 ID
    //NOTE: BQ25887 not in use for our pre-V1.3 users, so this is important
    uint8_t id = 0;
    if (BQ25887_IC_I2C_PORT > 8)
    {
        Wire1.beginTransmission(BQ25887_IC_I2C_ADDRESS);
        Wire1.write(BQ25887_REG_PART_INFO);
        Wire1.endTransmission();
        Wire1.requestFrom(BQ25887_IC_I2C_ADDRESS, (uint8_t)1);
        id = Wire1.read();
    }
    else
    {
        Wire.beginTransmission(BQ25887_IC_I2C_ADDRESS);
        Wire.write(BQ25887_REG_PART_INFO);
        Wire.endTransmission();
        Wire.requestFrom(BQ25887_IC_I2C_ADDRESS, (uint8_t)1);
        id = Wire.read();
    }

    id = (id & BQ25887_MASK_PART_INFO) >> 3;

    if (id != BQ25887_ID)
    {
        return false;
    }

    //enable ADCs on BQ25887
    if (BQ25887_IC_I2C_PORT > 8)
    {
        //enable cell monitoring ADCs
        Wire1.beginTransmission(BQ25887_IC_I2C_ADDRESS);
        Wire1.write(BQ25887_REG_ADC_CONTROL);
        Wire1.write(BQ25887_CMD_ADC_CONTROL_ENABLE);
        Wire1.endTransmission();

        //disable watchdog, or ADC resets after 40s
        Wire1.beginTransmission(BQ25887_IC_I2C_ADDRESS);
        Wire1.write(BQ25887_REG_CHG_CONTROL1);
        Wire1.write(BQ25887_CMD_WATCHDOG_DISABLE);
        Wire1.endTransmission();
    }
    else
    {
        Wire.beginTransmission(BQ25887_IC_I2C_ADDRESS);
        Wire.write(BQ25887_REG_ADC_CONTROL);
        Wire.write(BQ25887_CMD_ADC_CONTROL_ENABLE);
        Wire.endTransmission();

        Wire.beginTransmission(BQ25887_IC_I2C_ADDRESS);
        Wire.write(BQ25887_REG_CHG_CONTROL1);
        Wire.write(BQ25887_CMD_WATCHDOG_DISABLE);
        Wire.endTransmission();
    }

    return true;
}

int16_t EVNAlpha::getBatteryVoltage()
{
    if (_battery_adc_started)
    {
        this->sharedPorts().setPort(BQ25887_IC_I2C_PORT);

        uint16_t vnom = 0;
        if (BQ25887_IC_I2C_PORT > 8)
        {
            Wire1.beginTransmission(BQ25887_IC_I2C_ADDRESS);
            Wire1.write(BQ25887_REG_VBAT_ADC1);
            Wire1.endTransmission();
            Wire1.requestFrom(BQ25887_IC_I2C_ADDRESS, (uint8_t)2);
            vnom = Wire1.read() << 8 | Wire1.read();
        }
        else
        {
            Wire.beginTransmission(BQ25887_IC_I2C_ADDRESS);
            Wire.write(BQ25887_REG_VBAT_ADC1);
            Wire.endTransmission();
            Wire.requestFrom(BQ25887_IC_I2C_ADDRESS, (uint8_t)2);
            vnom = Wire.read() << 8 | Wire.read();
        }

        button.sharedState()->flash = (vnom < LOW_BATTERY_THRESHOLD_MV);

        return vnom;
    }
    return 0;
}

int16_t EVNAlpha::getCell1Voltage()
{
    if (_battery_adc_started)
    {
        this->sharedPorts().setPort(BQ25887_IC_I2C_PORT);

        uint16_t vcell1 = 0;
        if (BQ25887_IC_I2C_PORT > 8)
        {
            Wire1.beginTransmission(BQ25887_IC_I2C_ADDRESS);
            Wire1.write(BQ25887_REG_VCELLTOP_ADC1);
            Wire1.endTransmission();
            Wire1.requestFrom(BQ25887_IC_I2C_ADDRESS, (uint8_t)2);
            vcell1 = Wire1.read() << 8 | Wire1.read();
        }
        else
        {
            Wire.beginTransmission(BQ25887_IC_I2C_ADDRESS);
            Wire.write(BQ25887_REG_VCELLTOP_ADC1);
            Wire.endTransmission();
            Wire.requestFrom(BQ25887_IC_I2C_ADDRESS, (uint8_t)2);
            vcell1 = Wire.read() << 8 | Wire.read();
        }

        return vcell1;
    }
    return 0;
}

int16_t EVNAlpha::getCell2Voltage()
{
    if (_battery_adc_started)
    {
        //use total voltage reading / cell 1 voltage reading
        return this->getBatteryVoltage() - this->getCell1Voltage();
    }
    return 0;
}