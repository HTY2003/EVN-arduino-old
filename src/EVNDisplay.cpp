#include <EVNDisplay.h>
#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

EVNDisplay::EVNDisplay(uint8_t bus)
{
    _bus = constrain(bus, 0, 1);
    if (_bus)
        _display = new U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
    else
        _display = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
}

void EVNDisplay::init(bool rotate)
{
    _display->begin();
    _display->setFont(u8g2_font_profont12_tr);
    this->clear(); //clear graphics from previous program
    if (rotate)
        this->rotate();
}

void EVNDisplay::clear()
/*
Clears display and buffer (including all row names and data).
*/
{
    _display->clearBuffer();
    _display->sendBuffer();
    for (int i = 0; i < NO_OF_ROWS; i++)
    {
        _rownamelen[i] = 0;
    }
}

void EVNDisplay::rotate()
/*
Rotates display by 180deg. Previous graphics on screen are not removed.
*/
{
    _rotate = !_rotate;
    _display->setFlipMode(_rotate);

}

void EVNDisplay::splashEVN()
/*
Draws EVN Logo on display. Previous graphics on screen are removed.
*/
{
    _display->clearBuffer();
    _display->drawXBM(round((SCREEN_WIDTH - LOGO_WIDTH) / 2), round((SCREEN_HEIGHT - LOGO_HEIGHT) / 2), LOGO_WIDTH, LOGO_HEIGHT, logo);
    _display->sendBuffer();
}

void EVNDisplay::setRowName(uint8_t row, char const* name)
/*
Writes row name to display buffer. To display the row name, call EVNDisplay.updateRowName(uint8_t row).
When EVNDisplay.clear() is called, all row names are removed from buffer.
*/
{
    uint8_t rowc = constrain(row, 0, NO_OF_ROWS - 1);
    uint8_t namelen = strlen(name);

    //clear previous row name in buffer and mark for erasure (if needed)
    if (namelen < _rownamelen[rowc])
    {
        _display->setDrawColor(0);
        _display->drawBox(0, rowc * 8, _rownamelen[rowc] * 8, 8);
        _rownamelen_prev[rowc] = _rownamelen[rowc];
    }

    //store the area used for row names (update separately from data)
    _rownamelen[rowc] = namelen;

    //write row name (placed at start of row) to buffer
    _display->setDrawColor(1);
    _display->setCursor(0, (rowc + 1) * 8);
    _display->print(name);
}


//EVNDisplay.setRowInfo() is defined in EVNDisplay.h


void EVNDisplay::updateRowName(uint8_t row)
/*
Writes row name in buffer to display.
*/
{
    uint8_t rowc = constrain(row, 0, NO_OF_ROWS - 1);

    //transmit buffer area used to display the row name to display
    //if needed, clear previous row name too
    if (_rownamelen_prev[rowc] > 0)
    {
        _display->updateDisplayArea(0, rowc, _rownamelen_prev[rowc], 1);
        _rownamelen_prev[rowc] = 0;
    }
    else
    {
        _display->updateDisplayArea(0, rowc, _rownamelen[rowc], 1);
    }
}

void EVNDisplay::updateRowInfo(uint8_t row)
/*
Writes row data in buffer to display.
*/
{
    //transmit buffer area used to display the row data to display
    uint8_t rowc = constrain(row, 0, NO_OF_ROWS - 1);
    _display->updateDisplayArea(_rownamelen[rowc], rowc, (SCREEN_WIDTH / 8) - 1 - _rownamelen[rowc], 1);
}