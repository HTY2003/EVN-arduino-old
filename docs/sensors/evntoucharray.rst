``EVNTouchArray``
===========================

This class provides the following features and functionalities for our Capacitive Touch Array Standard Peripheral (MPR121 IC):

    * 12 Capacitive Touch Buttons (3*4 array)
    * Configurable Touch and Release Thresholds

.. note:: This class does I2C port selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

Constructor
-----------

.. class:: EVNTouchArray(uint8_t port, uint8_t touch_threshold = 12, uint8_t release_threshold = 6)

    :param port: I2C port the sensor is connected to (1-16)
    :param touch_threshold: Threshold for a button press to be registered (0-255)
    :param release_threshold: Threshold for a button released to be registered (0-255)

Wiring (I2C)
------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
3V3   VIN         3.3V Power
 --   IRQ         Not Connected
SCL   SCL         I2C Serial Clock
SDA   SDA         I2C Serial Data
GND   GND         Ground (0V)
====  ==========  ===========

Functions
---------

.. function:: bool begin()

    Initializes capacitive touch array. Call this function before using the other functions.

    :returns: Boolean indicating whether the array was successfully initialized. If ``false`` is returned, all other functions will return 0.

.. function:: void setThresholds(uint8_t channel, uint8_t touch_threshold, uint8_t release_threshold)

    Set touch and release thresholds for the given channel.
    
    Lowering the touch threshold increases the sensitivity of the touch array to button presses. 
    
    Conversely, lowering the release threshold lowers the sensitivity to button releases.

    :param channel: Channel to calibrate (0-11)
    :param touch_threshold: Threshold for a button press to be registered (0-255)
    :param release_threshold: Threshold for a button released to be registered (0-255)

.. function::   bool read(uint8_t channel)

    :returns: ``true`` if the given channel (0-11) is being pressed, ``false`` otherwise

.. function:: bool pressed()

    :returns: ``true`` if any of the 12 buttons are pressed, ``false`` otherwise

.. function:: uint8_t lastPressed()

    :returns: last button channel to be pressed (0-11)

.. function:: uint8_t lastReleased()

    :returns: last button channel to be released (0-11)