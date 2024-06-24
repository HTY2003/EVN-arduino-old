``EVNSevenSegmentLED``
======================

This class provides the following features and functionalities for our Seven Segment LED Standard Peripheral (HT16K33/VK16K33 IC):

    * 4 Controllable Seven-Segment LED elements with 6 more LEDs representing 4 Decimal Points and 1 Colon
    * 16 Adjustable Levels of Display Brightness

.. note:: This class does I2C port selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

Wiring (I2C)
------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
SCL   SCL         I2C Serial Clock
SDA   SDA         I2C Serial Data
GND   GND         Ground (0V)
3V3   VCC         3.3V Power
====  ==========  ===========

Constructor
-----------

.. class:: EVNSevenSegmentLED(uint8_t port)

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

.. function:: void show()

    Writes buffer to display

.. function:: void writePoint(uint8_t position, bool on = true, bool show = true)

    Sets decimal point in given position to given state in buffer. If ``show`` is ``true``, writes buffer to display.

    :param position: Position of decimal point to be written to (0-3)
    :param on: Whether LEDs are set to "on" in buffer. Defaults to ``true``
    :param show: Whether buffer will be written to display. Defaults to ``true``

.. function:: void clearPoint(uint8_t position, bool show = true)
    
    Sets colon to given state in buffer and if ``show`` is ``true``, writes buffer to display.

.. function:: void writeColon(bool on = true, bool show = true)

    Sets colon to given state in buffer and if ``show`` is ``true``, writes buffer to display.

    :param on: Whether LEDs are set to "on" in buffer. Defaults to ``true``
    :param show: Whether buffer will be written to display. Defaults to ``true``

.. function:: void clearColon(bool show = true)

    Sets colon to be turned off in buffer. If ``show`` is true, writes buffer to display.

    :param show: Whether buffer will be written to display. Defaults to ``true``

.. function:: void writeDigit(uint8_t position, uint8_t digit, bool show = true)

    Writes given digit (0-9) to given element in buffer. If ``show`` is true, writes buffer to display.

    :param position: Position of element to be written to (0-3)
    :param digit: Digit to be written to element (0-9)
    :param show: Whether buffer will be written to display. Defaults to ``true``

.. function:: void writeLetter(uint8_t position, char letter, bool show = true)

    Writes given letter to given element in buffer. If ``show`` is true, writes buffer to display.

    Not all letters are supported, as seven segment displays do not officially support letter output.
    
    Supported letters: ``'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'L', 'N', 'O', 'P', 'R', 'T', 'U', 'Y'``

    Passing uppercase and lowercase versions of the same letter to the function will lead to the same result.

    :param position: Position of element to be written to (0-3)
    :param digit: Digit to be written to element (0-9)
    :param show: Whether buffer will be written to display. Defaults to ``true``

.. function::   void writeNumber(float number, bool show = true)
                void writeNumber(double number, bool show = true)
                void writeNumber(long double number, bool show = true)
                void writeNumber(long number, bool show = true)
                void writeNumber(unsigned int number, bool show = true)
                void writeNumber(unsigned long number, bool show = true)
                void writeNumber(int number, bool show = true)

    Writes given number to buffer. If ``show`` is true, writes buffer to display.

    Integers larger than 9999 will only have the first 4 digits shown, and floating-point numbers will be displayed with a maximum precision of 3 decimal places.

    :param number: Number to be written to display (accepts any of the data types listed above)
    :param show: Whether buffer will be written to display. Defaults to ``true``

.. function:: void clearPosition(uint8_t position, bool clear_point = true, bool show = true)

    Sets all LEDs in specified element to be turned off in buffer. If ``show`` is true, writes buffer to display.

    :param position: Position of element to be cleared (0-3)
    :param clear_point: Whether decimal point for element will be cleared. Defaults to ``true``
    :param show: Whether buffer will be written to display. Defaults to ``true``

.. function:: void writeAll(bool show = true)

    Set all LEDs to be turned on in buffer. If ``show`` is ``true``, write buffer to display.

.. function:: void clearAll(bool show = true)

    Set all LEDs to be turned off in buffer. If ``show`` is ``true``, write buffer to display.
