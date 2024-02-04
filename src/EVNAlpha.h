#ifndef EVNAlpha_h
#define EVNAlpha_h

// #include "EVNBattery.h"
#include "EVNButton.h"
#include "EVNPortSelector.h"
#include "evn_alpha_pins.h"
#include <Arduino.h>

//TODO: Add battery low LED alert function (some kind of irregular blinking)
//Will be active regardless of whether LED_LINK is set

class EVNAlpha {
public:
    EVNAlpha(uint8_t mode = BUTTON_TOGGLE, uint8_t linkLED = LED_LINK, uint8_t linkMotors = MOTORS_LINK, uint32_t i2c_freq = DEFAULT_I2C_FREQ);
    void begin();

    //Button & LED Functions
    bool buttonRead() { return button.read(); };
    void ledWrite(bool state) { digitalWrite(LEDPIN, state); };
    void setMode(uint8_t mode) { button.setMode(mode); };
    void setLinkLED(uint8_t linkLED) { button.setLinkLED(linkLED); };
    void setLinkMotors(uint8_t linkMotors) { button.setLinkMotors(linkMotors); };

    //I2C Multiplexer Functions
    void setPort(uint8_t port) { ports.setPort(port); }
    uint8_t getPort() { return ports.getPort(); }
    uint8_t getWire0Port() { return ports.getWire0Port(); }
    uint8_t getWire1Port() { return ports.getWire1Port(); }
    // int16_t getBattVoltage() { return (_has_battery_adc ? batt.getBattVoltage() : 0); }
    // int16_t getCell1Voltage() { return (_has_battery_adc ? batt.getCell1Voltage() : 0); }
    // int16_t getCell2Voltage() { return (_has_battery_adc ? batt.getCell2Voltage() : 0); }
    void printPorts() { ports.printPorts(); }

    //Singletons for Port Selector and Button
    static EVNPortSelector& sharedPorts() { static EVNAlpha shared; return shared.ports; }
    static EVNButton& sharedButton() { static EVNAlpha shared; return shared.button; }

private:
    // static bool _has_battery_adc;
    // static EVNBattery batt;
    static EVNButton button;
    static EVNPortSelector ports;
    static uint8_t _linkLED, _mode, _linkMotors;
    static uint32_t _i2c_freq;
};

#endif