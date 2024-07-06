``EVNADC``
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
 --   A0          Analog Input 0
 --   A1          Analog Input 1
 --   A2          Analog Input 2
 --   A3          Analog Input 3
====  ==========  ===========

The analog inputs do not connect to EVN Alpha, but instead they are meant to connect to your devices with analog output e.g. analog light sensors or microphones.

Minimally, you will also need to connect GND (Ground) to these analog devices as well. You may also need to connect 3V3 Power to power the device.

The analog inputs on the multiplexer are not rated for voltages higher than the peripheral's supply voltage (usually 3.3V), so do not connect voltages higher than 3.3V to the inputs.

If you need voltage ranges up to 5V, consider powering the peripheral from 5V pins instead, since the I2C ports are 5V-tolerant.

Constructor
-----------

.. class:: EVNADC(uint8_t port)

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

Get Functions
"""""""""""""""

.. function:: data_rate getDataRate()
    
.. function:: range getRange()

Single-Shot Measurement (Blocking)
""""""""""""""""""""""""""""""""""

.. function:: float read(uint8_t pin)

    Reads analog voltage on an input channel using 

    This function is blocking because it requests the ADC to start a measurement, waits for the ADC to measure before collecting the reading.

    However, this may be fast enough depending on your use case. 

.. function:: bool ready()


.. function:: bool request(uint8_t pin)


.. function:: float receive(bool blocking = true)


Continuous Measurement
"""""""""""""""""""""""

.. function:: void startContinuous(uint8_t pin)
.. function:: bool readyContinuous()
.. function:: float readContinuous()
