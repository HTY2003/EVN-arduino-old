#ifndef EVNAlpha_h
#define EVNAlpha_h

#include "EVNButton.h"
#include "EVNPortSelector.h"
#include "pins_evn_alpha.h"
#include <Arduino.h>

class EVNAlpha {
public:
    EVNAlpha(uint8_t mode = BUTTON_TOGGLE, uint8_t linkLED = LED_UNLINK);
    void begin();

    static EVNPortSelector& sharedPorts() { return ports; }
    static EVNButton& sharedButton() { return button; }

    bool buttonRead() { return button.read(); };
    void ledWrite(bool state) { digitalWrite(LEDPIN, state); };
    void setPort(uint8_t port) { ports.setPort(port); }
    uint8_t getPort() { return ports.getPort(); }
    void printPorts() { ports.printPorts(); }

private:
    static EVNButton button;
    static EVNPortSelector ports;
    static uint8_t _linkLED, _mode;
};

#endif