#ifndef EVNAlpha_h
#define EVNAlpha_h

#include "helper/EVNButtonLED.h"
#include "helper/EVNPortSelector.h"
#include "evn_alpha_pins.h"
#include <Arduino.h>

class EVNAlpha {
private:
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

public:
    EVNAlpha(uint8_t mode = BUTTON_TOGGLE, bool link_led = true, bool link_movement = false, bool button_invert = false, uint32_t i2c_freq = DEFAULT_I2C_FREQ);
    void begin();

    //Button & LED Functions
    bool read() { return this->buttonRead(); };
    bool buttonRead() { return button_led.read(); };
    void write(bool state) { this->write(state); };
    void ledWrite(bool state) { button_led.write(state); };

    void setMode(uint8_t mode) { button_led.setMode(mode); };
    void setLinkLED(bool enable) { button_led.setLinkLED(enable); };
    void setLinkMovement(bool enable) { button_led.setLinkMovement(enable); };
    void setButtonInvert(bool enable) { button_led.setButtonInvert(enable); };
    void setFlash(bool enable) { button_led.setFlash(enable); };

    uint8_t getMode() { return button_led.getMode(); };
    bool getLinkLED() { return button_led.getLinkLED(); };
    bool getLinkMovement() { return button_led.getLinkMovement(); };
    bool getButtonInvert() { return button_led.getButtonInvert(); };
    bool getFlash() { return button_led.getFlash(); };

    //I2C Multiplexer Functions
    void setPort(uint8_t port) { ports.setPort(port); }
    uint8_t getPort() { return ports.getPort(); }
    uint8_t getWirePort() { return ports.getWire0Port(); }
    uint8_t getWire1Port() { return ports.getWire1Port(); }

    int16_t getBatteryVoltage(bool flash_when_low = true, uint16_t low_threshold_mv = 6700);
    int16_t getCell1Voltage(bool flash_when_low = true, uint16_t low_threshold_mv = 3350);
    int16_t getCell2Voltage(bool flash_when_low = true, uint16_t low_threshold_mv = 3350);

    void printPorts() { ports.printPorts(); }

    //Singletons for Port Selector and Button/LED
    static EVNPortSelector& sharedPorts() { static EVNAlpha shared; return shared.ports; }
    static EVNButtonLED& sharedButtonLED() { static EVNAlpha shared; return shared.button_led; }

private:
    bool beginBatteryADC();
    bool _battery_adc_started;
    bool _low_battery;
    uint16_t _low_battery_threshold;

    static EVNButtonLED button_led;
    static EVNPortSelector ports;
    static uint8_t _mode;
    static bool _link_led, _link_movement, _button_invert;
    static uint32_t _i2c_freq;
};

#endif