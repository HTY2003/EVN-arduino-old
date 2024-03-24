``EVNCompassSensor`` - Drift-free Heading Measurements
===============================================================

This class provides the following features and functionalities for our Compass Sensor Standard Peripheral (HMC5883L / QMC5883L IC):

    * Drift-free Yaw Measurements
        
        * Since this measures heading using the earth's geomagnetic field, magnets (like those in motors) can result in poor readings when placed too close to the sensor
        
        * Compasses are also susceptible to local hard-iron and soft-iron interference. We will be adding functions and a tutorial on calibrating to local environments soon!
    
    * Non-Blocking & Blocking Read Functions
    * Configurable Timing Budget for Readings
    * Built-in I2C Port Selection and De-selection

.. note:: This class does I2C port selection and de-selection automatically, so users do not need to call ``setPort()`` using their EVNAlpha objects.

.. note:: This library is still a heavy work-in-progress! Expect many new functions and features until official release.

.. class:: EVNCompassSensor(uint8_t port)
  
    :param port: I2C port the sensor is connected to (1-16)

Functions
---------

.. function:: bool begin()
    
    Initializes compass sensor. Call this function before using the other functions.

    :returns: Boolean indicating whether the sensor was successfully initialized. If ``false`` is returned, all other functions will return 0.

Reading Distance
""""""""""""""""

.. function:: double read(bool blocking = true)

    Get yaw measurement (i.e. heading) from sensor

    :param blocking: Block function from returning a value until a new reading is obtained. Defaults to ``true``

    :returns: Yaw measurement (from 0-360deg)

