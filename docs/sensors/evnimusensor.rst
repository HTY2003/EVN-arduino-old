``EVNIMUSensor``
================

This class provides the following features and functionalities for our 6DoF IMU Sensor Standard Peripheral (MPU6500 IC):

    * 3-Axis Accelerometer Measurements
    * 3-Axis Gyroscope Measurements

.. note:: This class does I2C port selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

.. note:: We're working on fusing the accelerometer and gyroscope readings together to obtain orientation data, so that should be coming soon!

Wiring (I2C)
------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
 --   FSYNC       Not Connected
 --   NCS         Not Connected
 --   INT         Not Connected
 --   AD0/SDO     Not Connected
 --   EDA         Not Connected
 --   ECL         Not Connected
SDA   SDA/SDI     I2C Serial Data
SCL   SCL/SCLK    I2C Serial Clock
GND   GND         Ground (0V)
3V3   VCC         3.3V Power
====  ==========  ===========

Constructor
-----------

.. class:: EVNIMUSensor(uint8_t port) : EVNI2CDevice(port)

    :param port: I2C port the sensor is connected to (1-16)

Functions
---------

.. function:: bool begin()

    Initializes IMU sensor. Call this function before using the other functions.

    :returns: Boolean indicating whether the sensor was successfully initialized. If ``false`` is returned, all other functions will return 0.

Accelerometer Measurements
""""""""""""""""""""""""""

.. function:: float readAccelX(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw X-axis accelerometer reading (in g)

.. function:: float readAccelY(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw Y-axis accelerometer reading (in g)

.. function:: float readAccelZ(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw Z-axis accelerometer reading (in g)

Gyroscope Measurements
""""""""""""""""""""""

.. function:: float readGyroX(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw X-axis gyroscope reading (in degrees per second)

.. function:: float readGyroY(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw Y-axis gyroscope reading (in degrees per second)

.. function:: float readGyroZ(bool blocking = true)

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: raw Z-axis gyroscope reading (in degrees per second)

Sensor Settings
"""""""""""""""
.. function:: void setAccelRange(accel_range range)

    :param range: Range of accelerometer measurements

    * ``EVNIMUSensor::accel_range::G_2`` (+-2g)
    * ``EVNIMUSensor::accel_range::G_4`` (+-4g)
    * ``EVNIMUSensor::accel_range::G_8`` (+-8g)
    * ``EVNIMUSensor::accel_range::G_16`` (+-16g)

.. function:: void setGyroRange(gyro_range range)

    :param range: Range of gyroscope measurements

    * ``EVNIMUSensor::gyro_range::DPS_250`` (+-250DPS)
    * ``EVNIMUSensor::gyro_range::DPS_500`` (+-500DPS)
    * ``EVNIMUSensor::gyro_range::DPS_1000`` (+-1000DPS)
    * ``EVNIMUSensor::gyro_range::DPS_2000`` (+-2000DPS)

.. function:: void setDataRate(data_rate data_rate)

    :param data_rate: Data rate of gyroscope and accelerometer measurements

    * ``EVNIMUSensor::data_rate::HZ_5`` (5 Hz)
    * ``EVNIMUSensor::data_rate::HZ_10`` (10 Hz)
    * ``EVNIMUSensor::data_rate::HZ_20`` (20 Hz)
    * ``EVNIMUSensor::data_rate::HZ_41`` (41 Hz)
    * ``EVNIMUSensor::data_rate::HZ_92`` (92 Hz)
    * ``EVNIMUSensor::data_rate::HZ_184`` (184 Hz)