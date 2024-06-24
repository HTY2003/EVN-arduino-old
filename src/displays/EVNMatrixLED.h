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

    void invertX(bool enable)
    {
        _invert_x = enable;
    };

    void invertY(bool enable)
    {
        _invert_y = enable;
    };

    void swapXY(bool enable)
    {
        _swap_xy = enable;
    };

    void writeOne(uint8_t x, uint8_t y, bool on = true, bool show = true)
    {
        writeRaw(convertXYtoRaw(x, y), on, show);
    };

    void clearOne(uint8_t x, uint8_t y, bool show = true)
    {
        writeOne(y, x, false, show);
    };

    void writeHLine(uint8_t y, uint8_t start_x, uint8_t end_x, bool on = true, bool show = true)
    {
        if (_sensor_started)
        {
            for (int i = start_x;i <= end_x; i++)
            {
                writeRaw(convertXYtoRaw(i, y), on, false);
            }

            if (show) this->update();
        }
    };

    void clearHLine(uint8_t y, uint8_t start_x, uint8_t end_x, bool show = true)
    {
        writeHLine(y, start_x, end_x, false, show);
    };

    void writeVLine(uint8_t x, uint8_t start_y, uint8_t end_y, bool on = true, bool show = true)
    {
        if (_sensor_started)
        {
            for (int i = start_y;i <= end_y; i++)
            {
                writeRaw(convertXYtoRaw(x, i), on, false);
            }

            if (show) this->update();
        }
    };

    void clearVLine(uint8_t y, uint8_t start_x, uint8_t end_x, bool show = true)
    {
        writeVLine(y, start_x, end_x, false, show);
    };

    void writeY(uint8_t y, bool on = true, bool show = true)
    {
        writeHLine(y, 0, 7, on, show);
    };

    void writeX(uint8_t x, bool on = true, bool show = true)
    {
        writeVLine(x, 0, 7, on, show);
    };

    void clearY(uint8_t y, bool show = true)
    {
        clearHLine(y, 0, 7, show);
    };

    void clearX(uint8_t x, bool show = true)
    {
        clearVLine(x, 0, 7, show);
    };

    void writeRectangle(uint8_t start_x, uint8_t end_x, uint8_t start_y, uint8_t end_y, bool on = true, bool show = true)
    {
        if (_sensor_started)
        {
            for (int i = start_y;i <= end_y; i++)
            {
                for (int j = start_x;j <= end_x; j++)
                {
                    writeRaw(convertXYtoRaw(j, i), on, false);
                }
            }

            if (show) this->update();
        }
    };

    void clearRectangle(uint8_t start_x, uint8_t end_x, uint8_t start_y, uint8_t end_y, bool show = true)
    {
        writeRectangle(start_y, start_x, end_y, end_x, false, show);
    };

private:
    uint8_t convertXYtoRaw(uint8_t x, uint8_t y)
    {
        if (x > 7 || y > 7) return 255;

        uint16_t yc;
        uint16_t xc;

        if (!_swap_xy)
        {
            yc = y;
            xc = x;
        }
        else
        {
            yc = x;
            xc = y;
        }

        if (_invert_x) xc = 7 - xc;
        if (_invert_y) yc = 7 - yc;

        return yc * 16 + (xc + 7) % 8;
    };

    bool _invert_x, _invert_y, _swap_xy;
};

#endif