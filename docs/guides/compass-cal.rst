Compass Calibration
==========================

If you have tried using the compass sensor, you may find that the readings are not great all the time. In fact, you may find that they are **bad** most of the time.
This is a guide to calibrate the compass sensor for much better readings.

Some Theory First!
------------------
While "compass" isn't a bad descriptor, a more technical term for this sensor is a 3-axis magnetometer.
It can measure magnetic field strength in the X, Y & Z axis (all perpendicular to each other).
The most common use for this is to measure the Earth's magnetic field (hence "compass"), but it can also be used for some other purposes (e.g. detecting the local presence of a magnet).

More importantly, the sensor captures **any** kind of nearby magnetic field, which results in interference. That is why calibration is needed.

Hard Iron Interference
""""""""""""""""""""""
Hard iron interference occurs when the sensor is placed close to any kind of permanent magnet (e.g. the magnets in motors), as they produce their own magnetic field separate from the Earth.

If placed too close, hard iron distortions can overpower the magnitude of the Earth's magnetic field at the sensor's position, making it impossible to detect heading.
The easy solution to this is to simply move the sensor further away from the magnets by elevating it.

Soft Iron Interference
""""""""""""""""""""""
Soft iron interference occurs when the sensor is placed near ferromagnetic (attracted by magnets) metal. 

These metals stretch and distort the existing magnetic field (from the Earth or any other permanent magnets). Likewise, our calibration aims to correct this distortion.

In most cases, the effects of soft iron interference are less pronounced than that of hard iron.

Limitations of Calibration
"""""""""""""""""""""""""""
Our calibration aims to remove the distortions caused by both hard-iron and soft-iron sources. 

However, the distortions change when the distance and differences in orientation between the sensor and sources of interference change. For example, 
the distortion created by a strong magnet 10cm away can be very different in magnitude from that of a strong magnet 20cm away.

Hence, our calibration can only really compensate for **internal** sources of interference, that always remain at the same position and orientation for the sensor. 
For instance, a robot's motors usually remain in the same distance and orientation relative to the motor, so its interference can be accounted for.

But this isn't to say calibration is useless. Usually it makes the difference between "decent" readings and "unusable" ones. 
Just keep in mind that the more your robot's environment/location change, the less valid your calibration values may be.

For a better explanation by people who know a lot more, check out this `webpage`_ by VectorNav. 

Calibration
------------

As long as you have enough readings of the robot in various orientations, one could plot and calculate the calibration values with a math program like MATLAB, or code.

However, what we will be doing is using a ready-made program called MotionCal made by Paul Stoffregen of PJRC, creator of the Teensy microcontrollers. 

Motioncal will read the raw XYZ magnetometer readings and spit out the calibration values once enough readings have been collected.

Steps
""""""

1. Download MotionCal from `this link`_.

    You may have to right-click, and then select ``Save link as...`` to download the file.

2. Connect your Compass Sensor Standard Peripheral to an I2C port on EVN Alpha

2. Upload the calibrateCompass.ino sketch from the EVN library examples to EVN Alpha (File > Examples > EVN > Others > Calibration > calibrateCompass). 

    This sketch prints the raw (uncalibrated) readings from the Compass Sensor.

    Remember to set the I2C port to the port your Compass Sensor is connected to!

    Once uploaded, Serial Monitor should continuously print the raw readings in the format below.

    .. code-block::

        Raw:0,0,0,0,0,0,312,120,313
        Raw:0,0,0,0,0,0,302,112,331
        Raw:0,0,0,0,0,0,294,123,342
        Raw:0,0,0,0,0,0,283,111,353

    Once you've checked that it's all working, close Serial Monitor to leave the serial port available for MotionCal to read from.

3. Open MotionCal. Select the COM Port for the EVN Alpha board. Once selected, MotionCal will begin plotting each reading as a red dot in a 3D space (X, Y and Z).

Picture

4. Start rotating the robot. Ideally, there should be a reading to evenly cover the entire "sphere" of possible orientations.

Picture

5. Read the 12 values on the right of the screen from left-to-right, top-to-bottom, and place them in the EVNCompassSensor declaration.

6. Test your new calibration

    As a quick test, you can run the readCompass example sketch (File > Examples > EVN > 1. Basics > c) Sensors > readCompass).

    When the sensor's heading (or yaw) is printed, a rotation of the sensor by 90 degrees should result in the number being printed also shifting by 90 degrees.

    But if you want a more thorough check, we can use the testCalibratedCompass sketch (File > Examples > EVN > Others > Calibration > testCalibratedCompass).

    It's exactly the same as 

.. _this link: https://www.pjrc.com/store/prop_shield.html
.. _webpage: https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-hsicalibration