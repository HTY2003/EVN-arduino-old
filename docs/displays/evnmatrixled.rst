``EVNMatrixLED``
================

This class provides the following features and functionalities for our LED Matrix Standard Peripheral (HT16K33/VK16K33 IC):

    * Controllable 8-by-8 matrix of red LEDs
    * 16 adjustable levels of display brightness

.. note:: This class does I2C port selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

Wiring (I2C)
------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
SCL   C           I2C Serial Clock
SDA   D           I2C Serial Data
GND   -           Ground (0V)
3V3   +           3.3V Power
====  ==========  ===========

Constructor
-----------

.. class:: EVNMatrixLED(uint8_t port)

    :param port: I2C port the sensor is connected to (1-16)

Functions
---------

.. function:: bool begin()

    Initializes display. Call this function before using the other functions.

    :returns: Boolean indicating whether the sensor was successfully initialized. If ``false`` is returned, all other functions may fail.

.. function:: void setDisplayMode(mode mode)

    Set mode of display (off, on, or blinking at given frequency)

    :param mode: Mode to run display in

    * ``EVNSevenSegmentLED::mode::OFF`` (Off)
    * ``EVNSevenSegmentLED::mode::ON`` (On)
    * ``EVNSevenSegmentLED::mode::BLINK_HZ_0_5`` (Blinking at 0.5Hz)
    * ``EVNSevenSegmentLED::mode::BLINK_HZ_1`` (Blinking at 1Hz)
    * ``EVNSevenSegmentLED::mode::BLINK_HZ_2`` (Blinking at 2Hz)
    
.. function:: void setBrightness(uint8_t brightness)

    Set brightness level of display (1-16)

    :param brightness: Brightness level from 1 to 16

.. function:: void swapXY(bool enable)

    Swaps X and Y axes (e.g. the pixel at X = 0, Y = 7 becomes X = 7, Y = 0)

    :param enable: Boolean indicating whether X and Y axes should be swapped

.. function:: void invertY(bool enable)

    Inverts order of Y axis (e.g. when set to ``true``, the row Y = 0 becomes X = 7)

    :param enable: Boolean indicating whether Y axis should be inverted

.. function:: void invertX(bool enable)

    Inverts order of X axis (e.g. when set to ``true``, the column X = 0 becomes X = 7)

    :param enable: Boolean indicating whether X axis should be inverted

.. function:: void update()

    Write buffer to display

.. function:: void fill(bool show = true)

    Set all LEDs to be turned on in buffer. If ``show`` is ``true``, write buffer to display.

    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``

.. function:: void clear(bool show = true)

    Set all LEDs to be turned off in buffer. If ``show`` is ``true``, write buffer to display.

    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``

.. function:: void writePixel(uint8_t x, uint8_t y, bool on = true, bool show = true)

    Set one LED at given XY coordinate to given state ``on``. If ``show`` is ``true``, write buffer to display.
    
    :param x: X-coordinate (or column) of pixel
    :param y: Y-coordinate (or row) of pixel
    :param on: Boolean indicating whether to set LED to be on or off in buffer. Defaults to ``true``
    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``

.. function:: void clearPixel(uint8_t x, uint8_t y, bool show = true)

    Set one LED at given XY coordinate to be turned off. If ``show`` is ``true``, write buffer to display.

    Same as ``clearPixel()`` but specified pixels are turned off and the ``on`` input field is removed.

    :param x: X-coordinate (or column) of pixel
    :param y: Y-coordinate (or row) of pixel
    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``

.. function:: void writeVLine(uint8_t x, uint8_t start_y, uint8_t end_y, bool on = true, bool show = true)

    Set vertical line of LEDs with given X-coordinate and within given range of Y-coordinates to given state ``on``. If ``show`` is ``true``, write buffer to display.

    :param x: X-coordinate (or column) of line
    :param start_y: start Y-coordinate (or row) of line
    :param end_y: ending Y-coordinate (or row) of line
    :param on: Boolean indicating whether to set LEDs to be on or off in buffer. Defaults to ``true``
    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``

.. function:: void clearVLine(uint8_t x, uint8_t start_row, uint8_t end_row, bool show = true)

    Set vertical line of LEDs with given X-coordinate and within given range of Y-coordinates to be turned off. If ``show`` is ``true``, write buffer to display.
    
    Same as ``writeVLine()`` but specified pixels are turned off and the ``on`` input field is removed.

    :param x: X-coordinate (or column) of line
    :param start_y: start Y-coordinate (or row) of line
    :param end_y: ending Y-coordinate (or row) of line
    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``

.. function:: void writeHLine(uint8_t y, uint8_t start_x, uint8_t end_x, bool on = true, bool show = true)

    Set horizontal line of LEDs with given Y-coordinate and within given range of X-coordinates to given state ``on``. If ``show`` is ``true``, write buffer to display.

    :param y: Y-coordinate (or row) of line
    :param start_x: start X-coordinate (or column) of line
    :param end_x: ending X-coordinate (or column) of line
    :param on: Boolean indicating whether to set LEDs to be on or off in buffer. Defaults to ``true``
    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``

.. function:: void clearHLine(uint8_t y, uint8_t start_x, uint8_t end_x, bool on = true, bool show = true)

    Set horizontal line of LEDs with given Y-coordinate and within given range of X-coordinates to be turned off. If ``show`` is ``true``, write buffer to display.

    Same as ``writeHLine()`` but specified pixels are turned off and the ``on`` input field is removed.

    :param y: Y-coordinate (or row) of line
    :param start_x: start X-coordinate (or column) of line
    :param end_x: ending X-coordinate (or column) of line
    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``

.. function:: void writeY(uint8_t y, bool on = true, bool show = true)

    Set entire row of LEDs with given Y-coordinate to given state ``on``. If ``show`` is ``true``, write buffer to display.

    :param y: Y-coordinate (or row) of line
    :param on: Boolean indicating whether to set LEDs to be on or off in buffer. Defaults to ``true``
    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``

.. function:: void clearY(uint8_t y, bool show = true)
    
    Set entire row of LEDs with given Y-coordinate to be turned off. If ``show`` is ``true``, write buffer to display.

    Same as ``writeRow()`` but specified pixels are turned off and the ``on`` input field is removed.

    :param y: Y-coordinate (or row) of line
    :param show: Boolean indicating whether to write buffer to display

.. function:: void writeX(uint8_t x, bool on = true, bool show = true)

    Set entire column of LEDs with given X-coordinate to given state ``on``. If ``show`` is ``true``, write buffer to display.

    :param x: X-coordinate (or column) of line
    :param on: Boolean indicating whether to set LEDs to be on or off in buffer. Defaults to ``true``
    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``

.. function:: void clearX(uint8_t x, bool show = true)

    Set entire column of LEDs with given X-coordinate to be turned off. If ``show`` is ``true``, write buffer to display.

    Same as ``writeX()`` but specified pixels are turned off and the ``on`` input field is removed.

    :param x: X-coordinate (or column) of line
    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``

.. function:: void clearRectangle(uint8_t start_x, uint8_t end_x, uint8_t start_y, uint8_t end_y, bool on = true, bool show = true)

    Set rectanglular region of LEDs within given range of X and Y coordinates to given state ``on``. If ``show`` is ``true``, write buffer to display.
    
    :param start_x: start X-coordinate (leftmost column) of region
    :param end_x: ending X-coordinate (rightmost column) of region
    :param start_y: start Y-coordinate (top column) of region
    :param end_y: ending Y-coordinate (bottom column) of region
    :param on: Boolean indicating whether to set LEDs to be on or off in buffer. Defaults to ``true``
    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``

.. function:: void clearRectangle(uint8_t start_x, uint8_t end_x, uint8_t start_y, uint8_t end_y, bool show = true)

    Set rectanglular region of LEDs within given range of X and Y coordinates to be turned off. If ``show`` is ``true``, write buffer to display.

    Same as ``writeRectangle()`` but specified pixels are turned off and the ``on`` input field is removed.

    :param start_x: start X-coordinate (leftmost column) of region
    :param end_x: ending X-coordinate (rightmost column) of region
    :param start_y: start Y-coordinate (top column) of region
    :param end_y: ending Y-coordinate (bottom column) of region
    :param show: Boolean indicating whether to write buffer to display. Defaults to ``true``
    




