#ifndef EVNAlpha_h
#define EVNAlpha_h

#include "helper/EVNButton.h"
#include "helper/EVNPortSelector.h"
#include "evn_alpha_pins.h"
#include "bq25887_defs.h"
#include <Arduino.h>

class EVNAlpha {
public:
    EVNAlpha(uint8_t mode = BUTTON_TOGGLE, bool link_led = true, bool link_motors = true, uint32_t i2c_freq = DEFAULT_I2C_FREQ);
    void begin();

    //Button & LED Functions
    bool read() { return this->buttonRead(); };
    bool buttonRead() { return button.read(); };
    void write(bool state) { this->ledWrite(state); };
    void ledWrite(bool state) { digitalWrite(LEDPIN, state); };

    void setMode(uint8_t mode) { button.setMode(mode); };
    void setLinkLED(bool link_led) { button.setLinkLED(link_led); };
    void setLinkMotors(bool link_motors) { button.setLinkMotors(link_motors); };

    //I2C Multiplexer Functions
    void setPort(uint8_t port) { ports.setPort(port); }
    uint8_t getPort() { return ports.getPort(); }
    uint8_t getWirePort() { return ports.getWire0Port(); }
    uint8_t getWire1Port() { return ports.getWire1Port(); }
    int16_t getBatteryVoltage();
    int16_t getCell1Voltage();
    int16_t getCell2Voltage();
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
    static bool _link_led, _link_motors;
    static uint32_t _i2c_freq;
};

#endif