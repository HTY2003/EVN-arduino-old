Magnetometer Calibration Guide
==============================

If you have tried using the compass sensor, you may find that the readings are not great all the time. In fact, you may find that they are bad most of the time.
This is a guide to calibrate the compass sensor for much better readings.

Some Theory First!
------------------

While "compass" isn't a bad descriptor, a more technical term for this sensor is a 3-axis magnetometer.
It can measure magnetic field strength in the X, Y & Z axis (all perpendicular to each other).
The most common use for this is to measure the Earth's magnetic field (hence "compass"), but it can also be used for some other purposes (e.g. detect local presence of a magnet).

More importantly, it's important to note that the sensor captures **any** kind of nearby magnetic field, which results in interference. This is why calibration is needed.

Hard Iron Interference
""""""""""""""""""""""

If the sensor is placed close to any kind of magnet (e.g. the magnets in motors),
they can end up. The easy solution to this is to simply move the sensor further away from the magnets by elevating it.

Soft Iron Interference
""""""""""""""""""""""

Soft Iron interference is caused


For a better explanation by people who know a lot more, check out this great webpage by ... 

Calibration
------------

As long as you have enough readings of the robot in various orientations, one could plot and calculate the calibration values with a math program like MATLAB, or code.

However, what we will be doing is using a ready-made program called MotionCal made by Paul Stoffregen, creator of the Teensy microcontrollers. 

Motioncal will read the raw XYZ magnetometer readings and spit out the calibration values once enough readings have been collected.

Steps
""""""

1. Download MotionCal from this link.

2. Run this code

.. code-block:: 

3. Open MotionCal. Select the COM Port for the EVN Alpha board. Once selected, ...

4. Start rotating the robot. Ideally, there should be a reading to evenly cover the entire sphere of possible orientations.

5. Read the 12 values on the right of the screen.

6. Place them in the compass sensor declaration.

.. note:: For those technical enough to attempt writing their own calibration code, scale your calibration values by 10 before writing them into the declaration. The MotionCal readings are multiplied for 10, and the library is written to account for this.