Uploading
=========

Test Upload
-----------
Start by opening EVN's Button Blink example sketch in Arduino (``File`` - ``Examples`` - ``EVN`` - ``01.Basics`` - ``buttonBlink``).

Board & COM Port Selection
""""""""""""""""""""""""""

To upload to EVN Alpha, select Generic RP2040 as your board (``Tools`` - ``Board`` - ``Raspberry Pi RP2040 Boards`` - ``Generic RP2040``), then the Generic SPI /2 Boot Stage 2 option (`Tools` - `Boot Stage 2` - `Generic SPI /2`). These settings ensure that the USB Serial connection and upload process are stable.

Under ``Tools`` - ``Port``, you should be able to see a COM Port available with a label (e.g. ``Raspberry Pi Pico W`` or ``Raspberry Pi Pico``). If you see it, select it. But if you do not, proceed with the next section on setting your board to USB BOOTSEL mode.

USB BOOTSEL mode
""""""""""""""""

If you're having trouble uploading code to your board, USB BOOTSEL mode is your best bet at solving it.

When incorrectly written or incorrectly compiled code is uploaded to EVN Alpha, the RP2040's processing core (responsible for resetting during upload) can crash.

To make the board available for upload again, we need to set it into USB BOOTSEL mode, which makes the board appear as a USB flash drive and stops any uploaded code from running.

There are 2 ways to set the board into USB BOOTSEL mode:
1. Reset - BOOTSEL

    * Press and Hold the Reset Button
    * Press and Hold the BOOTSEL Button
    * Release the Reset Button
    * Release the BOOTSEL Button

2. Power - BOOTSEL

    * Press the On/Off Button to set the board into Off mode
    * Press and Hold the BOOTSEL Button
    * Press the On/Off Button again to set the board into On mode
    * Release the BOOTSEL Button

Now, the board should appear as a USB flash drive when connected to your computer, and it will appear as a ``UF2 Board`` option in ``Tools`` - ``Port``. Select it, and upload your code.

After your new code has been uploaded, look through the available ports in ``Tools`` - ``Port`` again. If the code runs properly, a COM Port with a label (e.g. ``Raspberry Pi Pico W``) should be available for you to upload with and read from using Serial Monitor.

After selecting the correct board, press the Upload button (right arrow) in the top left corner. For fresh installs of the Arduino-Pico core, the first compile may take a while, but subsequent uploads should take less time.

When you press the user button (leftmost one), your LED should start blinking!

.. note:: We plan to add the EVN Alpha to the Arduino-Pico core as its own Board option by official release.






Writing Your First Sketch
-------------------------
To start, include the EVN library using ``#include <EVN.h>`` in your program.

All EVN sketches will use the board class ``EVNAlpha`` for built-in functions. Aside for this class, there are classes for each of our supported devices.

* Actuators
    * ``EVNMotor``
    * ``EVNDrivebase``
* Sensors
    * ``EVNColourSensor``
    * ``EVNIRDistanceSensor``
    * ``EVNCompassSensor``
* Displays
    * ``EVNDisplay``

This is what an example sketch with 2 motors and 2 colour sensors would look like:

.. code-block:: cpp

    #include <EVN.h>

    EVNAlpha board;
    EVNMotor motor_left(3, EV3_LARGE),
             motor_right(2, EV3_LARGE);

    EVNColourSensor cs1(1), cs2(2);

    void setup()
    {
        //board.begin() is always at the start of void setup()
        board.begin();

        cs1.begin();
        cs2.begin();
    }

    void setup1()
    {
        //motor begin can be called on 1st core, but calling on 2nd core improves performance
        motor_left.begin();
        motor_right.begin();
    }

    void loop()
    {
        int cs1_reading = cs1.read();
        int cs2_reading = cs2.read();

        //do something with colour sensor readings here

        motor_left.runSpeed(600);
        motor_right.runSpeed(600);
    }


More examples can be found in the EVN library examples (```File``` - ```Examples``` - ``EVN``...).


.. note:: Example sketches are a little sparse right now, we hope to add more as soon as we can