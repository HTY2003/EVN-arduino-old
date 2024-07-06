``EVNAlpha``
========================================

EVNAlpha is a class used to interface with the onboard hardware on EVN Alpha.

* Button
* LED
* I2C Multiplexers
* Battery ADC

Some Technical Details
----------------------

EVNAlpha uses **hardware interrupts** for the onboard button to act as an toggle switch (default) or pushbutton. This approach allows other libraries to use the values read by EVNButton, without the user having to poll and work around blocking functions.

By default, the button output is also linked to 2 other functions. Both can disabled by the user if needed:

* Motor Operation: the button acts as a motor disable switch, so that users can pause their robot before uploading
* LED: the LED acts as an indicator for the button output

EVN Alpha has 2 TCA9548A I2C multiplexers, 1 on each I2C bus. This allows users to connect multiple I2C devices with the same I2C address without worrying about address clashing. However, users must set the port (1-16) for a given peripheral before communicating with it. EVNAlpha includes functions for port selection and de-selection to ease this process.

.. note::
    Our EVN-specific peripheral libaries (e.g. EVNColourSensor) do the port selection automatically, so port selection functions are not needed for them.

.. note::
    For Ports 9-16, 3rd party libraries and end-user code have to use Wire1 to interface with their sensors, as these ports are connected to the I2C1 bus.

.. class:: EVNAlpha(uint8_t mode = BUTTON_TOGGLE, bool link_led = true, bool link_movement = false, bool button_invert = false, uint32_t i2c_freq = 100000)
    
    :param mode: Determines behaviour of ``buttonRead()``. Defaults to ``BUTTON_TOGGLE``

        * ``BUTTON_TOGGLE`` -- Returns false on startup and toggles between true and false with each button press
        * ``BUTTON_PUSHBUTTON`` -- Returns true only when button is pressed
        * ``BUTTON_DISABLE`` -- Always returns true

    :param link_led: Links LED to display ``buttonRead()`` output. Defaults to ``true``.

    :param link_movement: Links all motor and servo operation to ``buttonRead()`` output. Defaults to ``false``.

    :param button_invert: Inverts output of ``buttonRead()``. Defaults to ``false``.

    :param i2c_freq: I2C frequency in Hz. Defaults to 400000 (400kHz).

Example Usage:

.. code-block:: cpp

    //constructor with default options
    EVNAlpha board;

    //constructor with non-default options
    EVNAlpha board(BUTTON_PUSHBUTTON, false, false, 400000);

Functions
---------
.. function:: void begin()

    Initializes on-board hardware. This function should be called at the start of ``void setup()``, before anything else.

Example Program:

.. code-block:: cpp

    EVNAlpha board;

    void setup()
    {
        board.begin();
    }

LED / Button
""""""""""""

.. function::   bool read()
                bool buttonRead()

    Get button output (varies depending on mode parameter in class constructor)

    :returns: boolean signifying button output

.. function::   void write(bool state)
                void ledWrite(bool state)

    Set LED to turn on (``true``) or off (``false``). However, the LED state can be overridden by the battery reading functions (see below).

    :param state: state to write to LED

Example Usage:

.. code-block:: cpp

    board.read();
    board.write(true);  //LED on
    board.write(false); //LED off

I2C Port Control
""""""""""""""""

.. function:: void setPort(uint8_t port)

    :param port: I2C port to be enabled (1--16)

.. function:: uint8_t getPort()

    :returns: last I2C port called using ``setPort()`` (1--16)

.. function:: uint8_t getWirePort()

    :returns: last Wire I2C port called using ``setPort()`` (1--8)

.. function:: uint8_t getWire1Port()

    :returns: last Wire1 I2C port called using ``setPort()`` (9--16)

.. function:: void printPorts()

    Prints all I2C devices on every port using ``Serial``

Example Usage:

.. code-block:: cpp

    board.getPort();        //returns 1 on startup
    board.getWirePort();    //returns 1 on startup
    board.getWire1Port();   //returns 9 on startup

    board.setPort(3);       //set Wire to connect to Port 3
    board.setPort(10);      //set Wire1 to connect to Port 10

    board.getPort();        //returns 10
    board.getWirePort();    //returns 3
    board.getWire1Port();   //returns 10

Battery Voltage Reading
""""""""""""""""""""""""
All battery voltage reading functions have a ``flash_when_low`` input. 
This is a low battery alert function, which flashes the LED at a rate of 5Hz (5 blinks per second) when the battery voltage is too low.

When the alert is on, the LED's previous output (whether linked to button or controlled by the user) will be overridden.
To add the alert to your code, add ``getBatteryVoltage()`` (or ``getCell1Voltage()`` **and** ``getCell2Voltage()``) to ``void loop()`` and they will check the voltage each loop.

.. code-block:: c++

    void loop()
    {
      //main code here
      
      board.getBatteryVoltage(); //battery alert!
    }


.. function:: int16_t getBatteryVoltage(bool flash_when_low = true, uint16_t low_threshold_mv = 6900)

    :param flash_when_low: Sets LED to flash when battery voltage falls below ``low_threshold_mv``. Defaults to ``true``
    :param low_threshold_mv: Battery voltage threshold (in millivolts). When battery voltage falls below this voltage and ``flash_when_low`` is ``true``, low voltage alert is triggered. Defaults to 6900.

    :returns: combined voltage of both battery cells in millivolts

.. function:: int16_t getCell1Voltage(bool flash_when_low = true, uint16_t low_threshold_mv = 3450)

    Cell 1 refers to the cell nearer to the edge of the board.
    
    :param flash_when_low: Sets LED to flash when battery voltage falls below ``low_threshold_mv``. Defaults to ``true``
    :param low_threshold_mv: Cell voltage threshold (in millivolts). When this cell's voltage falls below this threshold and ``flash_when_low`` is ``true``, low battery alert is triggered. Defaults to 3450.

    :returns: voltage of first cell in millivolts

.. function:: int16_t getCell2Voltage(bool flash_when_low = true, uint16_t low_threshold_mv = 3450)

    Cell 2 refers to the cell nearer to the centre of the board.

    :param flash_when_low: Sets LED to flash when battery voltage falls below ``low_threshold_mv``. Defaults to ``true``
    :param low_threshold_mv: Cell voltage threshold (in millivolts). When this cell's voltage falls below this threshold and ``flash_when_low`` is ``true``, the low battery alert is triggered. Defaults to 3450.

    :returns: voltage of second cell in millivolts

Example Program:

.. code-block:: cpp

    EVNAlpha board;

    void setup()
    {
        board.begin();

        int batt = board.getBatteryVoltage();
        int cell1 = board.getCell1Voltage();
        int cell2 = board.getCell2Voltage();
    }

Example Output (on Serial Monitor):

.. code-block:: cpp

    8392
    4198
    4194

Set Functions
"""""""""""""
.. function:: void setMode(uint8_t mode)

    :param mode: Determines behaviour of ``buttonRead()``
    
    * ``BUTTON_TOGGLE``
    * ``BUTTON_PUSHBUTTON``
    * ``BUTTON_DISABLE``

.. function:: void setLinkLED(bool enable)

    :param enable: Links LED to display ``buttonRead()`` output

.. function:: void setLinkMovement(bool enable)

    :param enable: Links all motor and servo operation to ``buttonRead()`` output

.. function:: void setButtonInvert(bool enable)

    :param enable: Inverts output of ``buttonRead()``

Get Functions
""""""""""""""

.. function:: uint8_t getMode()

    This function returns the button mode in numbers, as shown below.

    The written button modes (e.g. ``BUTTON_TOGGLE``, ``BUTTON_PUSHBUTTON``) are converted to these numbers when compiled, 
    so statements like ``if (board.getMode() == BUTTON_TOGGLE) {}`` are valid.

    :returns: Mode of button in numerical form
    
    * 0 (``BUTTON_DISABLE``)
    * 1 (``BUTTON_TOGGLE``)
    * 2 (``BUTTON_PUSHBUTTON``)

.. function:: bool setLinkLED()

    :returns: Whether LED is linked to ``buttonRead()`` output

.. function:: bool setLinkMovement()

    :returns: Whether motor and servo operation is linked to ``buttonRead()`` output

.. function:: bool setButtonInvert()

    :returns: Whether output of ``buttonRead()`` is inverted

