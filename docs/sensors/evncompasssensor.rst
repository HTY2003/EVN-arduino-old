``EVNCompassSensor``
====================

This class provides the following features and functionalities for our Compass Sensor Standard Peripheral (HMC5883L / QMC5883L IC):

* Drift-free Yaw Measurements
    
    * Since this measures heading using the earth's geomagnetic field, magnets (like those in motors) can result in poor readings when placed too close to the sensor
    
    * Compasses are also susceptible to local hard-iron and soft-iron interference. We will be adding functions and a tutorial on calibrating to local environments soon!

* Non-Blocking & Blocking Read Functions
* Configurable Timing Budget for Readings
* Built-in I2C Port Selection and De-selection

.. note:: This class does I2C port selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

Wiring (I2C)
------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
3V3   VCC         3.3V Power
GND   GND         Ground (0V)
SCL   SCL         I2C Serial Clock
SDA   SDA         I2C Serial Data
--    DRDY        Not Connected
====  ==========  ===========

Constructor
-----------

.. class:: EVNCompassSensor(uint8_t port, float hard_x = 0, float hard_y = 0, float hard_z = 0, float soft_x_0 = 1, float soft_x_1 = 0, float soft_x_2 = 0, float soft_y_0 = 0, float soft_y_1 = 1, float soft_y_2 = 0, float soft_z_0 = 0, float soft_z_1 = 0, float soft_z_2 = 1)
  
    :param port: I2C port the sensor is connected to (1-16)
    :param hard_x: X-axis hard iron calibration value. Defaults to 0
    :param hard_y: Y-axis hard iron calibration value. Defaults to 0
    :param hard_z: Z-axis hard iron calibration value. Defaults to 0
    :param soft_x_0: X-axis soft iron calibration value 0. Defaults to 1
    :param soft_x_1: X-axis soft iron calibration value 1. Defaults to 0
    :param soft_x_2: X-axis soft iron calibration value 2. Defaults to 0
    :param soft_y_0: Y-axis soft iron calibration value 0. Defaults to 0
    :param soft_y_1: Y-axis soft iron calibration value 1. Defaults to 1
    :param soft_y_2: Y-axis soft iron calibration value 2. Defaults to 0
    :param soft_z_0: Z-axis soft iron calibration value 0. Defaults to 0
    :param soft_z_1: Z-axis soft iron calibration value 1. Defaults to 0
    :param soft_z_2: Z-axis soft iron calibration value 2. Defaults to 1

Functions
---------

.. function:: bool begin()
    
    Initializes compass sensor. Call this function before using the other functions.

    :returns: Boolean indicating whether the sensor was successfully initialized. If ``false`` is returned, all other functions will return 0.

.. function:: bool isQMC()

    :returns: ``true`` if the sensor has a QMC chip, ``false`` otherwise

.. function:: bool isHMC()

    :returns: ``true`` if the sensor has a HMC chip, ``false`` otherwise

.. function:: bool isCalibrated()

    :returns: ``true`` if the sensor has been supplied with calibration values, ``false`` otherwise

.. function:: void setCalibration(float hard_x = 0, float hard_y = 0, float hard_z = 0, float soft_x_0 = 1, float soft_x_1 = 0, float soft_x_2 = 0, float soft_y_0 = 0, float soft_y_1 = 1, float soft_y_2 = 0, float soft_z_0 = 0, float soft_z_1 = 0, float soft_z_2 = 1)

    :param hard_x: X-axis hard iron calibration value. Defaults to 0
    :param hard_y: Y-axis hard iron calibration value. Defaults to 0
    :param hard_z: Z-axis hard iron calibration value. Defaults to 0
    :param soft_x_0: X-axis soft iron calibration value 0. Defaults to 1
    :param soft_x_1: X-axis soft iron calibration value 1. Defaults to 0
    :param soft_x_2: X-axis soft iron calibration value 2. Defaults to 0
    :param soft_y_0: Y-axis soft iron calibration value 0. Defaults to 0
    :param soft_y_1: Y-axis soft iron calibration value 1. Defaults to 1
    :param soft_y_2: Y-axis soft iron calibration value 2. Defaults to 0
    :param soft_z_0: Z-axis soft iron calibration value 0. Defaults to 0
    :param soft_z_1: Z-axis soft iron calibration value 1. Defaults to 0
    :param soft_z_2: Z-axis soft iron calibration value 2. Defaults to 1
    
Reading Yaw / Heading
""""""""""""""""""""""

.. function:: float read(bool blocking = true)

    Get yaw measurement (i.e. heading) from sensor

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: Yaw measurement (from 0-360deg)

..function:: void setNorth(void)

    Set current yaw as North (heading = 0).

Reading Magnetometer Values
"""""""""""""""""""""""""""

.. function:: float readRawX()

    :returns: raw X-axis magnetometer measurement (in uT)

.. function:: float readRawY()

    :returns: raw Y-axis magnetometer measurement (in uT)

.. function:: float readRawZ()

    :returns: raw Z-axis magnetometer measurement (in uT)

.. function:: float readCalX()

    Ensure that the sensor has received calibration values for this function to work properly.

    :returns: calibrated X-axis magnetometer measurement (in uT)

.. function:: float readCalY()

    Ensure that the sensor has received calibration values for this function to work properly.

    :returns: calibrated Y-axis magnetometer measurement (in uT)

.. function:: float readCalZ()

    Ensure that the sensor has received calibration values for this function to work properly.

    :returns: calibrated Z-axis magnetometer measurement (in uT)

Advanced Sensor Settings (HMC)
"""""""""""""""""""""""""""""""

.. function:: void setModeHMC(hmc_mode mode)

    :param mode: Mode to run sensor in

    * ``EVNCompassSensor::hmc_mode::CONTINUOUS`` (measurement enabled)
    * ``EVNCompassSensor::hmc_mode::STANDBY`` (measurement disabled)

.. function:: void setDataRateHMC(hmc_data_rate data_rate)

    :param data_rate: Rate at which the sensor takes measurements

    * ``EVNCompassSensor::hmc_data_rate::HZ_0_75`` (0.75Hz)
    * ``EVNCompassSensor::hmc_data_rate::HZ_1_5`` (1.5Hz)
    * ``EVNCompassSensor::hmc_data_rate::HZ_3`` (3Hz)
    * ``EVNCompassSensor::hmc_data_rate::HZ_7_5`` (7.5Hz)
    * ``EVNCompassSensor::hmc_data_rate::HZ_15`` (15Hz)
    * ``EVNCompassSensor::hmc_data_rate::HZ_30`` (30Hz)
    * ``EVNCompassSensor::hmc_data_rate::HZ_75`` (75Hz)

.. function:: void setRangeHMC(hmc_range range)

    :param range: Measurable magnetic range of readings (in Gauss)

    * ``EVNCompassSensor::hmc_range::GA_0_88`` (+-0.88Ga)
    * ``EVNCompassSensor::hmc_range::GA_1_3`` (+-1.3Ga)
    * ``EVNCompassSensor::hmc_range::GA_1_9`` (+-1.9Ga)
    * ``EVNCompassSensor::hmc_range::GA_2_5`` (+-2.5Ga)
    * ``EVNCompassSensor::hmc_range::GA_4`` (+-4Ga)
    * ``EVNCompassSensor::hmc_range::GA_4_7`` (+-4.7Ga)
    * ``EVNCompassSensor::hmc_range::GA_5_6`` (+-5.6Ga)
    * ``EVNCompassSensor::hmc_range::GA_8_1`` (+-8.1Ga)

.. function:: void setSamplesHMC(hmc_samples samples)

    :param samples: Number of samples taken per reading

    * ``EVNCompassSensor::hmc_samples::X1`` (1)
    * ``EVNCompassSensor::hmc_samples::X2`` (2)
    * ``EVNCompassSensor::hmc_samples::X3`` (3)
    * ``EVNCompassSensor::hmc_samples::X4`` (4)

Advanced Sensor Settings (QMC)
""""""""""""""""""""""""""""""

.. function:: void setModeQMC(qmc_mode mode)

    :param mode: Mode to run sensor in

    * ``EVNCompassSensor::qmc_mode::CONTINUOUS`` (measurement enabled)
    * ``EVNCompassSensor::qmc_mode::STANDBY`` (measurement disabled)

.. function:: void setDataRateQMC(qmc_data_rate data_rate)

    :param data_rate: Rate at which the sensor takes measurements

    * ``EVNCompassSensor::qmc_data_rate::HZ_10`` (10Hz)
    * ``EVNCompassSensor::qmc_data_rate::HZ_50`` (50Hz)
    * ``EVNCompassSensor::qmc_data_rate::HZ_100`` (100Hz)
    * ``EVNCompassSensor::qmc_data_rate::HZ_200`` (200Hz)

.. function:: void setRangeQMC(qmc_range range)

    :param range: Measurable magnetic range of readings (in Gauss)

    * ``EVNCompassSensor::qmc_range::GA_2`` (+-2Ga)
    * ``EVNCompassSensor::qmc_range::GA_8`` (+-8Ga)

.. function:: void setSamplesQMC(qmc_samples samples)

    :param samples: Number of samples taken per reading

    * ``EVNCompassSensor::qmc_samples::X64`` (64)
    * ``EVNCompassSensor::qmc_samples::X128`` (128)
    * ``EVNCompassSensor::qmc_samples::X256`` (256)
    * ``EVNCompassSensor::qmc_samples::X512`` (512)