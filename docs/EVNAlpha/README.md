# EVNAlpha
EVNAlpha is a class used to interface with the button, LED and port selector hardware built into EVN Alpha.

## Constructor
##### `EVNAlpha(uint8_t mode = BUTTON_TOGGLE, uint8_t linkLED = LED_UNLINK)`

Arguments:
* mode: 

* linkLED: 

Example:
```
EVNAlpha board();
```
Or
```
EVNAlpha board(BUTTON_PUSHBUTTON, LINK_LED);
```

## Functions
##### `void begin()`
Initializes button, LED and hardware for interfacing with sensors and peripherals.

Example:
```
void setup1()
{
    board.begin();
}
```

### LED
##### `void ledWrite(bool state)`
Turns LED on or off.

Arguments:
* state: `true` to light up LED, or `false` to turn off LED

Example:
```
void loop()
{
    board.ledWrite(true);
    //...
}
```

### Button
##### `bool buttonRead()`
Returns reading of button state, depending on mode.

Returns:
* `BUTTON_TOGGLE` mode
  * `false` when button is first intialized
  * when button is pressed, the boolean returned flips from `true` to `false` or vice versa, and remains that way until button is pressed again

* `BUTTON_PUSHBUTTON` mode
  * `false` when button is not being pressed
  * `true` when button is being pressed

* `BUTTON_DISABLE` mode
  * Always returns `true`

Example:
```
void loop()
{
    bool reading = board.buttonRead();
    //...
}
```

### Port Selection
For the 16 I2C Ports on EVN Alpha, only one can be connected to the I2C bus at a time (to avoid address clashing).

Hence, before using an I2C peripheral on any port, the port must be selected with the below function first.

##### `void setPort(uint8_t port)`
Selects desired I2C port.

Arguments:
* port: port number to be selected (1-16)

Example:
```
void loop()
{
    board.setPort(1);
    //commands for peripheral on I2C Port 1
    //...
}
```

##### `uint8_t getPort()`
Returns currently selected I2C port.

Returns:
* port number(1-16)

Example:
```
void loop()
{
    int current_port = board.getPort();
    //...
}
```