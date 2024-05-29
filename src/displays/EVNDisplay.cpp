#include "EVNDisplay.h"

EVNDisplay::EVNDisplay(uint8_t port, bool flip_180deg)
{
    _sensor_started = false;
    _port = constrain(port, 1, 16);

    //use diff i2c bus depending on port
    if (_port > 8)
        _display8x8 = new U8X8_SSD1306_128X64_NONAME_2ND_HW_I2C(U8X8_PIN_NONE);
    else
        _display8x8 = new U8X8_SSD1306_128X64_NONAME_HW_I2C(U8X8_PIN_NONE);
    _flip_180deg = flip_180deg;
}

bool EVNDisplay::begin()
{
    EVNAlpha::sharedPorts().begin();
    EVNAlpha::sharedPorts().setPort(_port);

    //init, set font, wipe prev display
    if (!_display8x8->begin())
    {
        return _sensor_started;
    }

    _sensor_started = true;

    _display8x8->setFont(u8x8_font_pxplusibmcgathin_f);
    this->clear();

    //flip if needed
    if (_flip_180deg)
    {
        _display8x8->setFlipMode(_flip_180deg);
    }

    return _sensor_started;
}

void EVNDisplay::clear()
{
    if (_sensor_started)
    {
        EVNAlpha::sharedPorts().setPort(_port);

        //wipe display
        _display8x8->clearDisplay();
        //erase all stored row names
        for (int i = 0; i < 8; i++)
            _rownamelen[i] = 0;

        _display8x8->setFlipMode(_flip_180deg);
    }
}

void EVNDisplay::clearLine(uint8_t row)
{
    if (_sensor_started)
    {
        EVNAlpha::sharedPorts().setPort(_port);
        uint8_t rowc = constrain(row, 0, NO_OF_ROWS - 1);

        //empty line on display
        _display8x8->clearLine(rowc);

        //erase stored row name
        _rownamelen[rowc] = 0;
    }
}

void EVNDisplay::rotate()
{
    if (_sensor_started)
    {
        EVNAlpha::sharedPorts().setPort(_port);

        //flip the rotation the display is set to
        _flip_180deg = !_flip_180deg;

        //and write it to the display
        _display8x8->setFlipMode(_flip_180deg);
    }
}

void EVNDisplay::splashEVN()
{
    if (_sensor_started)
    {
        EVNAlpha::sharedPorts().setPort(_port);

        //clear display
        this->clear();

        //draw logo line by line
        for (int y = 0; y < (NO_OF_ROWS / 2); y++)
        {
            for (int x = 0; x < MAX_CHAR; x++)
            {
                uint8_t start_tile[8] = { 0 }, end_tile[8] = { 0 };
                for (int i = 0; i < 8; i++)
                {
                    start_tile[i] = logo[y * SCREEN_WIDTH + x * 8 + i];
                    end_tile[i] = logo[(NO_OF_ROWS - 1 - y) * SCREEN_WIDTH + (MAX_CHAR - 1 - x) * 8 + i];
                }
                _display8x8->drawTile(x, y, 1, start_tile);
                _display8x8->drawTile(MAX_CHAR - 1 - x, NO_OF_ROWS - 1 - y, 1, end_tile);
                delay(30);
            }
            // uint8_t tile[SCREEN_WIDTH] = { 0,0,0,0,0,0,0,0 };
            // for (int i = 0; i < SCREEN_WIDTH; i++)
            //     tile[i] = logo[y * SCREEN_WIDTH + i];
            // _display8x8->drawTile(0, y, MAX_CHAR, tile);
        }
    }
}