# EVNAlpha
EVNAlpha is a class used to interface with the button, LED and I2C multiplexer hardware built into EVN Alpha.

## Some Technical Details
EVNAlpha uses hardware interrupts for the onboard user button to act as an toggle switch (default) or pushbutton. This approach also allows other libraries to use the values read by EVNButton, without the user having to constantly poll and work around blocking functions. By default, the button state determines whether the onboard blue LED is on/off, and whether the motors will run.


EVN Alpha has 2 TCA9548A I2C multiplexers, 1 on each I2C bus for a total of 16 multiplexed I2C ports. Hence, users can connect multiple of the same sensor without worrying about I2C address clashing. However, users must select the port for a given peripheral before communicating with it. To make this process easier, EVN Alpha includes functions for port selection, which are also demonstrated in an example (useI2CDevice.ino).


Note: EVN-specific peripheral libaries (e.g. EVNColourSensor) do the port selection and deselection automatically, so these port selection functions are not needed for using EVN Standard Peripherals.


Note 2: for Ports 9-16, 3rd party libraries and end-user code have to use Wire1 to interface with their sensors, as these ports are on the I2C1 bus.

## List of Functions
- [Constructor](#evnalpha)
- [begin()](#void-begin)
- [buttonRead()](#bool-buttonread)
- [ledWrite()](#void-ledwrite)
- [setPort()](#void-setport)
- [getPort()](#uint8t-getport)

## Constructor
##### `EVNAlpha(uint8_t mode = BUTTON_TOGGLE, uint8_t linkLED = LED_LINK, uint8_t linkMotors = MOTORS_LINK)`

Arguments:
* mode:
    * `BUTTON_TOGGLE` (default, each toggles `buttonRead()` output between `true` and `false`)
    * `BUTTON_PUSHBUTTON`(`buttonRead()` only returns `true` when pressed, `false` otherwise)
    * `BUTTON_DISABLE`(`buttonRead()` always returns `true`)

* linkLED:
    * `LED_LINK` (default, LED matches output of buttonRead())
    * `LED_UNLINK` (LED can be controlled independently)

* linkMotors:
    * `MOTORS_LINK` (default, when buttonRead() returns `false` all EVNMotor commands end instantly and all motors will pause)
    * `MOTORS_UNLINK` (motors controlled separately)

Example:
```
EVNAlpha board();
```
Another example:
```
EVNAlpha board(BUTTON_PUSHBUTTON, LED_UNLINK, MOTOR_UNLINK);
```

## Functions
##### `void begin()`
Initializes button, LED and hardware for interfacing with sensors and peripherals. This function also sets the correct GPIO pins for all I2C buses and UARTs, so it is very important! It should always be placed at the very start of void setup().

Example:
```
void setup()
{
    board.begin();
    //...
}
```

### Button
##### `bool buttonRead()`
Returns reading of button state, depending on mode.

Returns:
* In `BUTTON_TOGGLE` mode
    * `false` when button is first intialized
    * upon button press, the boolean returned flips from `true` to `false` or vice versa, and remains that way until the next press

* In `BUTTON_PUSHBUTTON` mode
    * `false` when button is not pressed
    * `true` when button is pressed

* In `BUTTON_DISABLE` mode
    * Always returns `true`

Example:
```
void loop()
{
    bool reading = board.buttonRead();
    //...
}
```

### LED
##### `void ledWrite(bool state)`
Turns LED on or off.

Arguments:
* state:
    * `true` to light up LED
    * `false` to turn off LED

Example:
```
void loop()
{
    board.ledWrite(true);
    //...
}
```

### Port Selection
##### `void setPort(uint8_t port)`
Selects desired I2C port.

Note: for Ports 9-16, 3rd party libraries and end-user code have to use Wire1 to interface with their sensors, as these ports are on the I2C1 bus.

Arguments:
* port: port number to be selected (from 1-16)

Example:
```
void loop()
{
    board.setPort(1);
    //Wire.startTransmission(...)
    //...

    board.setPort(9);
    //Wire1.startTransmission(...)
    //...
}
```

##### `uint8_t getPort()`
Returns currently selected I2C port.

Returns:
* port number (1-16)

Example:
```
void loop()
{
    int current_port = board.getPort();
    //...
}
```