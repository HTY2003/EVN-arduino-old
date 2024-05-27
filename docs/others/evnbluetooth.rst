``EVNBluetooth``
================

The Bluetooth Module Standard Peripheral uses the HC-05 Bluetooth Module. This module stores

The module can be set into 2 modes: Host & Remote.

Running as a Remote device means that a Bluetooth host (such as your laptop or phone) can connect to it and transmit commands or receive data.

Running as a Host device allows you to connect to other Remote devices, which can be used for communication between 2 Bluetooth Modules (1 Host & 1 Remote).

The module's settings are stored onboard, and to edit them, users have to hold a button to start it in Program Mode (AT mode) and send text commands.

This library makes it simple to configure simple settings on the board like baud rate and host/remote mode, without needing to set text commands.

For, Serial1 or Serial2 will be used instead. It is functionally equivalent to a bidirectional Serial connection, but wireless!

Wiring (Serial)
---------------

====  ==========  ===========
Host  Peripheral  Description
====  ==========  ===========
3V3   VCC         3.3V Power
GND   GND         Ground (0V)
RX    TX          Host Receive
TX    RX          Host Transmit
====  ==========  ===========

Starting Module in Program Mode
--------------------------------

1. Set EVN into Off mode using the On/Off button.
2. Press and hold the Bluetooth Module's onboard button.
3. Set EVN into On mode using the On/Off button.
4. Hold the button for 2s before releasing it. If the Bluetooth Module's red LED starts blinking once every 2 seconds, it has successfully entered program mode.

Constructor
-----------

.. class:: EVNBluetooth(uint8_t serial_port, uint32_t baud_rate = 9600, char* name = (char*)"EVN Bluetooth", uint8_t mode = BT_REMOTE, char* addr = NULL);

    :param serial_port: Serial port the sensor is connected to (1 or 2)

    :param baud_rate: Baud rate used when not in Program Mode. Defaults to 9600. Supported baud rates: 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1382400
    
    :param name: Name of Bluetooth device. Defaults to "EVN Bluetooth" 
    
    :param mode: Mode to run module in. Defaults to ``BT_REMOTE``

        * ``BT_HOST``: Host Mode
        * ``BT_REMOTE``: Remote Mode

    :param addr: Bluetooth Address of the device to connect to. This parameter is only used in Host Mode.

Functions
---------

.. function:: bool begin(bool exit_program_mode = true)

    If the module is set to Program Mode, this function writes all settings to the module, before exiting program mode depending on user input.

    :param exit_program_mode: Boolean indicating whether to exit program mode after updating settings. Defaults to true

.. function:: bool inProgramMode()
    
    :returns: true the module is in program mode, false otherwise

.. function:: bool exitProgramMode()

    Exits Program Mode for normal operation

    :returns: true if module is not in program mode, false otherwise

.. function:: bool factoryReset()

    Resets all settings stored in the module to their factory defaults. This function is useful for resetting more advanced settings which are not exposed by this library.

    :returns: true if module was in program mode & successfully reset, false otherwise

Retrieving Module Info
----------------------

.. function:: void getAddress(char* array)

    :param array: memory address of character array to write address to

    

.. function:: bool printAddress()

    Prints address of Bluetooth Module. Write this address to the constructor of another EVNBluetooth object in Host Mode to connect to it.

    :returns: true if address was successfully retrieved

Example
-------