``EVNRGBLED``
=============

Some Technical Info
-------------------
These peripherals use WS2812B RGB LEDs, which feature 3 brightness-controlled channels for Red, Green and Blue.

Normally, a microcontroller would need to transmit a signal of carefully timed HIGHs and LOWs to control the LEDs, which would block the rest of the program due to the use of ``delay()``. 
However, this library uses Programmable IO (PIO) to handle the transmissions, so the main program loop is not blocked when transmitting to the LEDs.

Another thing to note is that these can be daisy-chained, so you can control up to 48 LEDs (6 Peripherals) with one Servo port!

.. note:: Each instance of ``EVNRGBLED`` consumes one of the RP2040's 8 PIO state machines, so keep this in mind if you are using PIO for your own purposes.

Wiring (Servo)
--------------

====  ==========   ===========
Host  Peripheral   Description
====  ==========   ===========
SIG   IN           Signal
5V    VCC          5V Power
GND   GND          Ground (0V)
====  ==========   ===========

Chaining Multiple RGB LEDs
--------------------------

It's possible to chain up to 6 RGB LED Peripherals in a row without needing to use more Servo ports.

Simply connect the OUT of one Peripheral to the IN pin of the next one in the chain.

==========  ===============   ===========
Peripheral  Next Peripheral   Description
==========  ===============   ===========
OUT         IN                Signal
VCC         VCC               5V Power
GND         GND               Ground (0V)
==========  ===============   ===========

.. warning:: Each RGB LED consumes 60mA of current at max brightness. Using 48 RGB LEDs consumes 2.88A, which is close to the 3A limit of the Servo ports. Keep this in mind if you have other peripherals powered by Servo ports!

Constructor
-----------

.. class:: EVNRGBLED(uint8_t port, uint8_t num_leds = 8)

    :param port: Servo port (1-4)
    :param num_leds: Number of LEDs connected to the servo port. Defaults to 8

Functions
---------

.. function:: void write(uint16_t led, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, bool show = true)

    Updates given LED's RGB values in buffer and if ``show`` is true, writes buffer to peripheral.

    :param led: Index of LED to update (0 to (num_leds-1)). LED 0 is the LED closest to the signal & power pins
    :param r: Red channel intensity. Defaults to 0
    :param g: Green channel intensity. Defaults to 0
    :param b: Blue channel intensity. Defaults to 0
    :param show: Whether to write buffer to LEDs. Defaults to true

.. function:: void writeAll(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, bool show = true)

    Updates all LEDs' RGB values in buffer and if ``show`` is true, writes buffer to peripheral.

    :param r: Red channel intensity. Defaults to 0
    :param g: Green channel intensity. Defaults to 0
    :param b: Blue channel intensity. Defaults to 0
    :param show: Whether to write buffer to LEDs. Defaults to true

.. function:: void clear(bool show = true)

    Resets buffer and if ``show`` is true, writes buffer to peripheral, essentially turning all LEDs off.

    :param show: Whether to write buffer to LEDs. Defaults to true

.. function:: void update()

    Writes buffer to LEDs. Use this function if buffer has been updated but not written to the LEDs yet.


