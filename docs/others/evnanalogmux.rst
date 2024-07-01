``EVNAnalogMux``
================

This class provides the following features and functionalities for our Analog Multiplexer Standard Peripheral (ADS1115 IC):

    * 16-bit precision Analog-to-Digital Converter (ADC)
    * ADC can be multiplexed to 4 Single-Ended Inputs or 2 Differential Inputs
    * Programmable Data Rates from 8 samples/second to 860 samples/second.

.. note:: This class does I2C port selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

Wiring (I2C)
------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
3V3   V           3.3V Power
GND   G           Ground (0V)
SCL   SCL         I2C Serial Clock
SDA   SDA         I2C Serial Data
 --   ADDR        Not Connected
 --   ALERT       Not Connected
 --   A0          Not Connected
 --   A1          Not Connected
 --   A2          Not Connected
 --   A3          Not Connected
====  ==========  ===========

blah blah

Constructor
-----------

.. class:: EVNAnalogMux(uint8_t port)

    :param port: I2C port the multiplexer is connected to (1-16)

Functions
---------
.. function:: bool begin()

    Initializes analog multiplexer. Call this function before using the other functions.

    :returns: Boolean indicating whether the multiplexer was successfully initialized. If ``false`` is returned, all other functions will return 0.

Set Functions
"""""""""""""""

.. function:: void setDataRate(data_rate data_rate)

.. function:: void setRange(range range)

Set Functions
"""""""""""""""

.. function:: data_rate getDataRate()
    
.. function:: range getRange()

Single-Shot Measurement
""""""""""""""""""""""""

.. function:: bool ready()
.. function:: bool request(uint8_t pin)
.. function:: float receive(bool blocking = true)
.. function:: float read(uint8_t pin, bool blocking = true)


Continuous Measurement
"""""""""""""""""""""""

.. function:: void startContinuous(uint8_t pin)
.. function:: bool readyContinuous()
.. function:: float readContinuous(bool blocking = true)
