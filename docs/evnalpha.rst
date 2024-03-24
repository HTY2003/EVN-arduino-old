``EVNAlpha`` -- Built-in board functions
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
    Our EVN-specific peripheral libaries (e.g. EVNColourSensor) do the port selection and deselection automatically, so these port selection functions are not needed.

.. note::
    For Ports 9-16, 3rd party libraries and end-user code have to use Wire1 to interface with their sensors, as these ports are connected to the I2C1 bus.

.. class:: EVNAlpha(uint8_t mode = BUTTON_TOGGLE, bool link_led = true, bool link_motors = true, uint32_t i2c_freq = 100000)
    
    :param mode: Determines behaviour of `buttonRead()`. Defaults to ``BUTTON_TOGGLE``

        * ``BUTTON_TOGGLE`` -- Returns false on startup and toggles between true and false with each button press
        * ``BUTTON_PUSHBUTTON`` -- Returns true only when button is pressed
        * ``BUTTON_DISABLE`` -- Always returns true

    :param link_led: Links LED to display `buttonRead()` output. Defaults to true.

    :param link_motors: Links motor operation to `buttonRead()` output. Defaults to true.

    :param i2c_freq: I2C frequency in Hz. Defaults to 100000 (100kHz).

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

    Set LED to turn on (`true`) or off (`false`)

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

    :returns: last I2C port called using `setPort()` (1--16)

.. function:: uint8_t getWirePort()

    :returns: last Wire I2C port called using `setPort()` (1--8)

.. function:: uint8_t getWire1Port()

    :returns: last Wire1 I2C port called using `setPort()` (9--16)

.. function:: void printPorts()

    Prints all I2C devices on every port over `Serial`

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

.. function:: int16_t getBatteryVoltage()

    :returns: combined voltage of both battery cells in millivolts

.. function:: int16_t getCell1Voltage()

    :returns: voltage of first cell in millivolts

.. function:: int16_t getCell2Voltage()

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

Settings
""""""""
Docs coming soon!

.. function:: void setMode(uint8_t mode)

.. function:: void setLinkLED(bool link_led)

.. function:: void setLinkMotors(bool link_motors)