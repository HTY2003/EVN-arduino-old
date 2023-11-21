#include "EVNDisplay.h"

EVNDisplay::EVNDisplay(uint8_t port, bool rotate)
{
    _port = constrain(port, 1, 16);
    if (_port > 8)
    {
        _display = new U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
        _display8x8 = new U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C(U8X8_PIN_NONE);
    }
    else
    {
        _display = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
        _display8x8 = new U8X8_SSD1306_128X64_NONAME_HW_I2C(U8X8_PIN_NONE);
    }
    _rotate = rotate;
}

void EVNDisplay::begin()
{
    uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
    EVNAlpha::sharedPorts().setPort(_port);
    _display->begin();
    _display8x8->begin();
    _display8x8->setFont(u8x8_font_pxplusibmcgathin_f);

    this->clear();

    if (_rotate)
    {
        _display8x8->setFlipMode(_rotate);
    }
    EVNAlpha::sharedPorts().setPort(prev_port);
}

void EVNDisplay::clear()
{
    uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
    EVNAlpha::sharedPorts().setPort(_port);
    _display8x8->clearDisplay();
    for (int i = 0; i < NO_OF_ROWS; i++)
    {
        _rownamelen[i] = 0;
    }
    EVNAlpha::sharedPorts().setPort(prev_port);
}

void EVNDisplay::clearLine(uint8_t row)
{
    uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
    EVNAlpha::sharedPorts().setPort(_port);
    uint8_t rowc = constrain(row, 0, NO_OF_ROWS - 1);
    _display8x8->clearLine(rowc);
    _rownamelen[rowc] = 0;
    EVNAlpha::sharedPorts().setPort(prev_port);
}

void EVNDisplay::rotate()
{
    uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
    EVNAlpha::sharedPorts().setPort(_port);
    _rotate = !_rotate;
    _display8x8->setFlipMode(_rotate);
    EVNAlpha::sharedPorts().setPort(prev_port);
}

void EVNDisplay::splashEVN()
{
    uint8_t prev_port = EVNAlpha::sharedPorts().getPort();
    EVNAlpha::sharedPorts().setPort(_port);
    _display->clearBuffer();
    _display->drawXBM(round((SCREEN_WIDTH - LOGO_WIDTH) / 2), round((SCREEN_HEIGHT - LOGO_HEIGHT) / 2), LOGO_WIDTH, LOGO_HEIGHT, logo);
    _display->sendBuffer();
    EVNAlpha::sharedPorts().setPort(prev_port);
}