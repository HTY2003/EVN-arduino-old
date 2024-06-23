#ifndef EVNAlpha_h
#define EVNAlpha_h

#include "helper/EVNButton.h"
#include "helper/EVNPortSelector.h"
#include "evn_alpha_pins.h"
#include <Arduino.h>

class EVNAlpha {
public:
    static const uint16_t LOW_CELL_THRESHOLD_MV = 3350;

    enum bq25887 : uint8_t
    {
        I2C_ADDR = 0x6A,
        I2C_PORT = 16,
        ID = 0x05,
        REG_CHG_CONTROL1 = 0x05,
        REG_ADC_CONTROL = 0x15,
        REG_VBAT_ADC1 = 0x1D,
        REG_VCELLTOP_ADC1 = 0x1F,
        REG_PART_INFO = 0x25,
        CMD_ADC_CONTROL_ENABLE = 0b10110000,
        CMD_WATCHDOG_DISABLE = 0b10001101,
        MASK_PART_INFO = 0b01111000,
    };

    EVNAlpha(uint8_t mode = BUTTON_TOGGLE, bool link_led = true, bool link_movement = true, uint32_t i2c_freq = DEFAULT_I2C_FREQ);
    void begin();

    //Button & LED Functions
    bool read() { return this->buttonRead(); };
    bool buttonRead() { return button.read(); };
    void write(bool state) { this->ledWrite(state); };

    void ledWrite(bool state)
    {
        bool flash = button.getFlash();
        button.setFlash(false);
        digitalWrite(LEDPIN, state);
        button.setFlash(flash);
    };

    void setMode(uint8_t mode) { button.setMode(mode); };
    void setLinkLED(bool enable) { button.setLinkLED(enable); };
    void setLinkMovement(bool enable) { button.setLinkMovement(enable); };
    void setFlash(bool enable) { button.setFlash(enable); };

    //I2C Multiplexer Functions
    void setPort(uint8_t port) { ports.setPort(port); }
    uint8_t getPort() { return ports.getPort(); }
    uint8_t getWirePort() { return ports.getWire0Port(); }
    uint8_t getWire1Port() { return ports.getWire1Port(); }
    int16_t getBatteryVoltage(bool flash_when_low = true);
    int16_t getCell1Voltage(bool flash_when_low = true);
    int16_t getCell2Voltage(bool flash_when_low = true);
    void printPorts() { ports.printPorts(); }

    //Singletons for Port Selector and Button
    static EVNPortSelector& sharedPorts() { static EVNAlpha shared; return shared.ports; }
    static EVNButton& sharedButton() { static EVNAlpha shared; return shared.button; }

private:
    bool beginBatteryADC();
    bool _battery_adc_started;
    bool _low_battery;
    uint16_t _low_battery_threshold;

    static EVNButton button;
    static EVNPortSelector ports;
    static uint8_t _mode;
    static bool _link_led, _link_movement;
    static uint32_t _i2c_freq;
};

#endif