#ifndef EVNBluetooth_h
#define EVNBluetooth_h

#include <Arduino.h>
#include <string.h>

#define BT_REMOTE_MODE  0
#define BT_HOST_MODE    1

class EVNBluetooth
{
private:
    static const uint32_t AT_MODE_BAUD_RATE = 38400;
    static const uint16_t AT_MODE_DELAY_MS = 20;

public:
    EVNBluetooth(uint8_t serial_port, uint32_t baud_rate = 9600, char* name = (char*)"EVN Bluetooth", uint8_t mode = BT_REMOTE_MODE, char* addr = NULL);
    bool begin(bool exit_program_mode = false);

    bool inProgramMode();
    bool exitProgramMode();
    bool factoryReset();

    bool setName(char* name);
    bool setHostMode(char* addr);
    bool setRemoteMode();
    bool setBaudRate(uint32_t baud_rate);

    uint32_t getBaudRate();

    void getMode(char* array);
    void getAddress(char* array);
    void getVersion(char* array);
    void getName(char* array);

    void printBaudRate();
    void printMode();
    void printAddress();
    void printVersion();
    void printName();
    void printAll();

private:
    void _writeCommand(char* cmd);
    uint32_t _filterBaudRate(uint32_t baud_rate);

    SerialUART* _serial;
    // SerialPIO* _sserial; //TODO: Add SoftwareSerial support
    uint32_t _baud_rate;
    uint32_t _supported_baud_rates[10] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1382400 };
    bool _at_mode;
    uint8_t _mode;
    char* _name;
    char* _addr;
};


#endif