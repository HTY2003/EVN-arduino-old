#include "EVNBluetooth.h"

EVNBluetooth::EVNBluetooth(uint8_t serial_port, uint32_t baud_rate, char* name, uint8_t mode, char* addr)
{
    uint8_t serial_portc = constrain(serial_port, 1, 2);
    uint8_t modec = constrain(mode, 0, 1);

    _serial = (serial_portc == 1) ? &Serial1 : &Serial2;
    _baud_rate = _filterBaudRate(baud_rate);
    _mode = modec;
    _name = name;
    _addr = addr;
    _at_mode = false;
}

uint32_t EVNBluetooth::_filterBaudRate(uint32_t baud_rate)
{
    uint32_t final_baud_rate = _supported_baud_rates[0];
    for (int i = 1; i < 10; i++)
    {
        if (_supported_baud_rates[i] > baud_rate) break;
        final_baud_rate = _supported_baud_rates[i];
    }
    return final_baud_rate;
}

void EVNBluetooth::_writeCommand(char* cmd)
{
    _serial->write(cmd);
    _serial->println();
    delay(AT_MODE_DELAY_MS);
}

bool EVNBluetooth::begin(bool exit_program_mode)
{
    _serial->begin(AT_MODE_BAUD_RATE);
    _writeCommand((char*)"AT");

    if (_serial->available())
    {
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);
        if (strcmp(response, "OK\r") == 0)
        {
            _at_mode = true;
            this->setBaudRate(_baud_rate);
            this->setName(_name);
            if (_mode == BT_HOST && _addr != NULL)
                this->setHostMode(_addr);
            else
                this->setRemoteMode();

            if (exit_program_mode)
            {
                _serial->end();
                _serial->begin(_baud_rate);
            }
            return true;
        }
    }
    _serial->end();
    _serial->begin(_baud_rate);
    return false;
}

bool EVNBluetooth::inProgramMode() { return _at_mode; };

bool EVNBluetooth::exitProgramMode()
{
    if (_at_mode)
    {
        _writeCommand((char*)"AT+RESET");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        if (strcmp(response, "OK\r") == 0)
        {
            _at_mode = false;
            _serial->end();
            _serial->begin(_baud_rate);
            return true;
        }
        else
            return false;
    }
    return true;
}

bool EVNBluetooth::factoryReset()
{
    if (_at_mode)
    {
        _writeCommand((char*)"AT+ORGL");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        if (strcmp(response, "OK\r") == 0)
        {
            _at_mode = false;
            _serial->end();
            _serial->begin(_baud_rate);
            return true;
        }
    }
    return false;
}

bool EVNBluetooth::setName(char* name)
{
    if (_at_mode)
    {
        _name = name;
        _serial->write("AT+NAME=");
        _writeCommand(_name);
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        if (strcmp(response, "OK\r") == 0)
            return true;
    }
    return false;
}

bool EVNBluetooth::setHostMode(char* addr)
{
    if (_at_mode)
    {
        _writeCommand((char*)"AT+ROLE=1");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);
        if (strcmp(response, "OK\r") != 0)
            return false;

        _writeCommand((char*)"AT+CMODE=0");
        char response2[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);
        if (strcmp(response2, "OK\r") != 0)
            return false;

        _serial->write("AT+BIND=");
        _writeCommand((char*)addr);
        char response3[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);
        if (strcmp(response3, "OK\r") == 0)
            return true;
    }
    return false;
}

bool EVNBluetooth::setRemoteMode()
{
    if (_at_mode)
    {
        _writeCommand((char*)"AT+ROLE=0");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);
        if (strcmp(response, "OK\r") == 0)
            return true;
    }
    return false;
}

bool EVNBluetooth::setBaudRate(uint32_t baud_rate)
{
    if (_at_mode)
    {
        _baud_rate = _filterBaudRate(baud_rate);

        _serial->write("AT+UART=");
        _serial->print(_baud_rate);
        _writeCommand((char*)",0,0");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        if (strcmp(response, "OK\r") == 0)
            return true;
    }
    return false;
}

uint32_t EVNBluetooth::getBaudRate()
{
    if (_at_mode)
    {
        uint32_t baud_rate = 0;
        _writeCommand((char*)"AT+UART?");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        char subresponse[7];
        strncpy(subresponse, response, 6);
        subresponse[6] = '\0';

        if (strcmp(subresponse, "+UART:") == 0)
        {
            char baud_string[32] = { 0 };
            for (int i = 6; i < 32; i++)
            {
                if (response[i] == ',') break;
                baud_string[i - 6] = response[i];
            }
            baud_rate = String(baud_string).toInt();
        }

        char response2[32] = { 0 };
        _serial->readBytesUntil('\n', response2, 32);
        if (strcmp(response2, "OK\r") == 0)
            return baud_rate;
    }
    return 0;
}

void EVNBluetooth::getMode(char* array)
{
    if (_at_mode)
    {
        uint8_t mode = 0;
        _writeCommand((char*)"AT+ROLE?");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        char subresponse[7];
        strncpy(subresponse, response, 6);
        subresponse[6] = '\0';

        if (strcmp(subresponse, "+ROLE:") == 0)
        {
            char mode_string[32] = { 0 };
            for (int i = 6; i < 32; i++)
            {
                if (response[i] == '\r') break;
                mode_string[i - 6] = response[i];
            }
            mode = String(mode_string).toInt();
        }

        char response2[32] = { 0 };
        _serial->readBytesUntil('\n', response2, 32);

        if (strcmp(response2, "OK\r") == 0)
        {
            if (mode == BT_REMOTE)
                strcpy(array, (char*)"Remote");
            else if (mode == BT_HOST)
                strcpy(array, (char*)"Host");
            return;
        }
    }
}

void EVNBluetooth::getAddress(char* array)
{
    if (_at_mode)
    {
        char addr_string[32] = { 0 };
        _writeCommand((char*)"AT+ADDR?");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        char subresponse[7];
        strncpy(subresponse, response, 6);
        subresponse[6] = '\0';

        if (strcmp(subresponse, "+ADDR:") == 0)
        {
            for (int i = 6; i < 32; i++)
            {
                if (response[i] == '\r') break;
                addr_string[i - 6] = response[i];
            }
        }

        char response2[32] = { 0 };
        _serial->readBytesUntil('\n', response2, 32);
        if (strcmp(response2, "OK\r") == 0)
            strcpy(array, addr_string);
    }
}

void EVNBluetooth::getVersion(char* array)
{
    if (_at_mode)
    {
        char ver_string[32] = { 0 };
        _writeCommand((char*)"AT+VERSION?");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        char subresponse[9];
        strncpy(subresponse, response, 8);
        subresponse[8] = '\0';

        if (strcmp(subresponse, "VERSION:") == 0)
        {
            for (int i = 8; i < 32; i++)
            {
                if (response[i] == '\r') break;
                ver_string[i - 8] = response[i];
            }
        }

        char response2[32] = { 0 };
        _serial->readBytesUntil('\n', response2, 32);
        if (strcmp(response2, "OK\r") == 0)
            strcpy(array, ver_string);
    }
}

void EVNBluetooth::getName(char* array)
{
    if (_at_mode)
    {
        char name_string[32] = { 0 };
        _writeCommand((char*)"AT+NAME?");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        char subresponse[7];
        strncpy(subresponse, response, 6);
        subresponse[6] = '\0';

        if (strcmp(subresponse, "+NAME:") == 0)
        {
            for (int i = 6; i < 32; i++)
            {
                if (response[i] == '\r') break;
                name_string[i - 6] = response[i];
            }
        }

        char response2[32] = { 0 };
        _serial->readBytesUntil('\n', response2, 32);
        if (strcmp(response2, "OK\r") == 0)
            strcpy(array, name_string);
    }
}

void EVNBluetooth::printMode()
{
    if (_at_mode)
    {
        uint8_t mode = 0;
        _writeCommand((char*)"AT+ROLE?");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        char subresponse[7];
        strncpy(subresponse, response, 6);
        subresponse[6] = '\0';

        if (strcmp(subresponse, "+ROLE:") == 0)
        {
            char mode_string[32] = { 0 };
            for (int i = 6; i < 32; i++)
            {
                if (response[i] == '\r') break;
                mode_string[i - 6] = response[i];
            }
            mode = String(mode_string).toInt();
        }

        char response2[32] = { 0 };
        _serial->readBytesUntil('\n', response2, 32);

        if (strcmp(response2, "OK\r") == 0)
        {
            if (mode == BT_REMOTE)
                Serial.println("Mode: Remote");
            else if (mode == BT_HOST)
                Serial.println("Mode: Host");
            return;
        }
        Serial.println("Mode: Unknown");
        return;
    }
    Serial.println("Mode: Unknown (Not in Program Mode)");
}

void EVNBluetooth::printBaudRate()
{
    if (_at_mode)
    {
        uint32_t baud_rate = 0;
        _writeCommand((char*)"AT+UART?");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        char subresponse[7];
        strncpy(subresponse, response, 6);
        subresponse[6] = '\0';

        if (strcmp(subresponse, "+UART:") == 0)
        {
            char baud_string[32] = { 0 };
            for (int i = 6; i < 32; i++)
            {
                if (response[i] == ',') break;
                baud_string[i - 6] = response[i];
            }
            baud_rate = String(baud_string).toInt();
        }

        char response2[32] = { 0 };
        _serial->readBytesUntil('\n', response2, 32);
        if (strcmp(response2, "OK\r") == 0) {
            Serial.print("Baud Rate: ");
            Serial.println(baud_rate);
            return;
        }
        Serial.println("Baud Rate: Unknown");
        return;
    }
    Serial.println("Baud Rate: Unknown (Not in Program Mode)");
}

void EVNBluetooth::printAddress()
{
    if (_at_mode)
    {
        char addr_string[32] = { 0 };
        _writeCommand((char*)"AT+ADDR?");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        char subresponse[7];
        strncpy(subresponse, response, 6);
        subresponse[6] = '\0';

        if (strcmp(subresponse, "+ADDR:") == 0)
        {
            for (int i = 6; i < 32; i++)
            {
                if (response[i] == '\r') break;
                addr_string[i - 6] = response[i];
            }
        }

        char response2[32] = { 0 };
        _serial->readBytesUntil('\n', response2, 32);
        if (strcmp(response2, "OK\r") == 0)
        {
            Serial.print("Address: ");
            Serial.println(addr_string);
            return;
        }

        Serial.println("Address: Unknown");
        return;
    }
    Serial.println("Address: Unknown (Not in Program Mode)");
}

void EVNBluetooth::printVersion()
{
    if (_at_mode)
    {
        char ver_string[32] = { 0 };
        _writeCommand((char*)"AT+VERSION?");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        char subresponse[9];
        strncpy(subresponse, response, 8);
        subresponse[8] = '\0';

        if (strcmp(subresponse, "VERSION:") == 0)
        {
            for (int i = 8; i < 32; i++)
            {
                if (response[i] == '\r') break;
                ver_string[i - 8] = response[i];
            }
        }

        char response2[32] = { 0 };
        _serial->readBytesUntil('\n', response2, 32);
        if (strcmp(response2, "OK\r") == 0)
        {
            Serial.print("Version: ");
            Serial.println(ver_string);
            return;
        }
        Serial.println("Version: Unknown");
        return;
    }
    Serial.println("Version: Unknown (Not in Program Mode)");
}

void EVNBluetooth::printName()
{
    if (_at_mode)
    {
        char name_string[32] = { 0 };
        _writeCommand((char*)"AT+NAME?");
        char response[32] = { 0 };
        _serial->readBytesUntil('\n', response, 32);

        char subresponse[7];
        strncpy(subresponse, response, 6);
        subresponse[6] = '\0';

        if (strcmp(subresponse, "+NAME:") == 0)
        {
            for (int i = 6; i < 32; i++)
            {
                if (response[i] == '\r') break;
                name_string[i - 6] = response[i];
            }
        }

        char response2[32] = { 0 };
        _serial->readBytesUntil('\n', response2, 32);
        if (strcmp(response2, "OK\r") == 0)
        {
            Serial.print("Name: ");
            Serial.println(name_string);
            return;
        }
        Serial.println("Name: Unknown");
        return;
    }
    Serial.println("Name: Unknown (Not in Program Mode)");
}

void EVNBluetooth::printAllInfo()
{
    this->printName();
    this->printVersion();
    this->printBaudRate();
    this->printMode();
    this->printAddress();
}