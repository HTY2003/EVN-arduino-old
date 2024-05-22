#ifndef EVNMatrixLED_h
#define EVNMatrixLED_h

#include <Arduino.h>
#include <Wire.h>
#include "../EVNAlpha.h"
#include "ht16k33/EVN_HT16K33.h"

class EVNMatrixLED : public EVN_HT16K33
{
public:
    EVNMatrixLED(uint8_t port) : EVN_HT16K33(port)
    {
    };

    void invertCol(bool enable)
    {
        _invert_col = enable;
    };

    void invertRow(bool enable)
    {
        _invert_row = enable;
    };

    void swapRowCol(bool enable)
    {
        _swap_row_col = enable;
    };

    void write(uint8_t row, uint8_t col, bool on, bool show = true)
    {
        writeRaw(convertRowColtoRaw(row, col), on, show);
    };

    void writeRow(uint8_t row, uint8_t start_col, uint8_t end_col, bool on, bool show = true)
    {
        if (_sensor_started)
        {
            for (int i = start_col;i <= end_col; i++)
            {
                writeRaw(convertRowColtoRaw(row, i), on, false);
            }

            if (show) this->update();
        }
    };

    void writeCol(uint8_t col, uint8_t start_row, uint8_t end_row, bool on, bool show = true)
    {
        if (_sensor_started)
        {
            for (int i = start_row;i <= end_row; i++)
            {
                writeRaw(convertRowColtoRaw(i, col), on, false);
            }

            if (show) this->update();
        }
    };

    void writeFullRow(uint8_t row, bool on, bool show = true)
    {
        writeRow(row, 0, 7, on, show);
    };

    void writeFullCol(uint8_t col, bool on, bool show = true)
    {
        writeCol(col, 0, 7, on, show);
    };

    void clearFullRow(uint8_t row, bool show = true)
    {
        writeFullRow(row, false, show);
    };

    void clearFullCol(uint8_t col, bool show = true)
    {
        writeFullCol(col, false, show);
    };

    void writeRectangle(uint8_t top_left_row, uint8_t top_left_col, uint8_t bottom_right_row, uint8_t bottom_right_col, bool on, bool show = true)
    {
        if (_sensor_started)
        {
            for (int i = top_left_row;i <= bottom_right_row; i++)
            {
                for (int j = top_left_col;j <= bottom_right_col; j++)
                {
                    writeRaw(convertRowColtoRaw(i, j), on, false);
                }
            }

            if (show) this->update();
        }
    };

private:
    uint8_t convertRowColtoRaw(uint8_t row, uint8_t col)
    {
        if (col > 7 || row > 7) return 255;

        uint16_t rowc;
        uint16_t colc;

        if (!_swap_row_col)
        {
            rowc = row;
            colc = col;
        }
        else
        {
            rowc = col;
            colc = row;
        }

        if (_invert_col) colc = 7 - colc;
        if (_invert_row) rowc = 7 - rowc;

        return rowc * 16 + (colc + 7) % 8;
    };

    bool _invert_col, _invert_row, _swap_row_col;
};

#endif