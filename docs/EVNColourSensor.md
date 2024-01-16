# EVNColourSensor
EVNColourSensor is a class used to interface EVN Alpha with our Colour Sensor Standard Peripheral, which uses a TCS34725 IC for colour detection.

## Some Technical Details
Compared to other libraries, the main benefit of EVNColourSensor is that instead of polling an I2C register to determine if a new reading is ready (which can take a while if you load 16 sensors), the time between sensor reads is measured and new data is only read when the required integration time for a new reading has lapsed. An additional perk is that the sensor integration time for each reading can be configured at a more granular level (in 2.4ms steps).


Currently, only reading raw and normalised RGBC values are supported. 


Note: EVNColourSensor does port selection and deselection automatically, so users do not need to call `setPort()` on their `EVNAlpha` objects.


## List of Functions
- [Constructor](#constructor)
- [begin()](#bool-begin)
- [setIntegrationCycles()](#void-setintegrationcyclesuint8t-integrationcycles)
- [setGain()](#void-setgainuint8t-gain)
- [setRedRange()](#void-setredrangeuint16t-low-uint16t-high)
- [setGreenRange()](#void-setgreenrangeuint16t-low-uint16t-high)
- [setBlueRange()](#void-setbluerangeuint16t-low-uint16t-high)
- [setClearRange()](#void-setclearrangeuint16t-low-uint16t-high)
- [readRed()](#uint16t-readred)
- [readGreen()](#uint16t-readgreen)
- [readBlue()](#uint16t-readblue)
- [readClear()](#uint16t-readclear)
- [readRedCal()](#double-readredcal)
- [readGreenCal()](#double-readgreencal)
- [readBlueCal()](#double-readbluecal)
- [readClearCal()](#double-readclearcal)

## Constructor
##### `EVNColourSensor(port, uint8_t integration_cycles = 1, uint8_t gain = GAIN_4X)`

Arguments:
* port: the I2C port the display is connected to (1-16, left to right)
* integration_cycles: number of 2.4ms integration_cycles per reading from 1-255 (integration time = 2.4ms * integration cycles), default is 1
* gain: multiplier applied to raw readings before being transmitted to sensor (can widen or reduce range of readings)
    * GAIN_1X: 1x gain
    * GAIN_4X: 4x gain
    * GAIN_16X: 16x gain
    * GAIN_60X: 60x gain

Example:
```
EVNColourSensor cs(5);
```
Another example:
```
EVNColourSensor cs(5, 10, GAIN_1X); //24ms integration time, 1x gain
```

## Functions
##### `bool begin()`
Initializes colour sensor to start running in continuous mode.

Returns:
* `true` if successfully detected and initialized, `false` otherwise

Example:
```
void setup()
{
    cs.begin();
    //...
}
```

##### `void setIntegrationCycles(uint8_t integration_cycles)`
Sets the number of 2.4ms integration cycles used for one reading.


integration time = 2.4ms * integration cycles


Arguments:
* integration_cycles: number of 2.4ms integration_cycles per reading from 1-255


Example:
```
void setup()
{
    cs.begin();
    cs.setIntegrationCycles(10);
    //...
}
```


##### `void setGain(uint8_t gain)`
Sets the number of 2.4ms integration cycles used for one reading.


raw reading (unsigned 16-bit integer) = raw reading by TCS34725 onboard ADC * gain


Arguments:
* gain: multiplier applied to raw readings before being transmitted to sensor (can widen or reduce range of readings)
    * GAIN_1X: 1x gain
    * GAIN_4X: 4x gain
    * GAIN_16X: 16x gain
    * GAIN_60X: 60x gain

Example:
```
void setup()
{
    cs.begin();
    cs.setGain(GAIN_16X);
    //...
}
```

##### `void setRedRange(uint16_t low, uint16_t high)`
Sets the range of possible red values used for calibrating raw values.


Calibrated values returned by `readRedCal()` are calculated as such: red calibrated = (red raw - red low) / (red raw - red high)


If this function is not called, `readRedCal()` returns -1;

Arguments:
* low: lower bound of red readings
* high: upper bound of red readings

Example:
```
void setup()
{
    cs.begin();
    cs.setRedRange(50, 400);
    //...
}
```

##### `void setGreenRange(uint16_t low, uint16_t high)`
Sets the range of possible green values used for calibrating raw values.


Calibrated values returned by `readGreenCal()` are calculated as such: green calibrated = (green raw - green low) / (green raw - green high)


If this function is not called, `readGreenCal()` returns -1;

Arguments:
* low: lower bound of green readings
* high: upper bound of green readings

Example:
```
void setup()
{
    cs.begin();
    cs.setGreenRange(50, 400);
    //...
}
```

##### `void setBlueRange(uint16_t low, uint16_t high)`
Sets the range of possible blue values used for calibrating raw values.


Calibrated values returned by `readBlueCal()` are calculated as such: blue calibrated = (blue raw - blue low) / (blue raw - blue high)


If this function is not called, `readBlueCal()` returns -1;

Arguments:
* low: lower bound of blue readings
* high: upper bound of blue readings

Example:
```
void setup()
{
    cs.begin();
    cs.setBlueRange(50, 400);
    //...
}
```

##### `void setClearRange(uint16_t low, uint16_t high)`
Sets the range of possible clear values used for calibrating raw values.


Calibrated values returned by `readClearCal()` are calculated as such: clear calibrated = (clear raw - clear low) / (clear raw - clear high)


If this function is not called, `readClearCal()` returns -1;

Arguments:
* low: lower bound of clear readings
* high: upper bound of clear readings

Example:
```
void setup()
{
    cs.begin();
    cs.setClearRange(50, 400);
    //...
}
```

##### `uint16_t readRed()`
Returns raw red reading from sensor.

Returns: 
* raw red reading from sensor

Example:
```
void loop()
{
    int red = cs.readRed();
    //...
}
```

##### `uint16_t readGreen()`
Returns raw green reading from sensor.

Returns: 
* raw green reading from sensor

Example:
```
void loop()
{
    int green = cs.readGreen();
    //...
}
```

##### `uint16_t readBlue()`
Returns raw blue reading from sensor.

Returns: 
* raw blue reading from sensor

Example:
```
void loop()
{
    int blue = cs.readBlue();
    //...
}
```

##### `uint16_t readClear()`
Returns raw clear reading from sensor.

Returns: 
* raw clear reading from sensor

Example:
```
void loop()
{
    int clear = cs.readClear();
    //...
}
```

##### `double readRedCal()`
Returns calibrated red reading from sensor.

Calibrated values are calculated as such: red calibrated = (red raw - red low) / (red raw - red high)

Returns: 
* if `setRedRange()` has been called, calibrated red reading from sensor
* -1 otherwise

Example:
```
void loop()
{
    double red_cal = cs.readRedCal();
    //...
}
```

##### `double readGreenCal()`
Returns calibrated green reading from sensor.

Calibrated values are calculated as such: green calibrated = (green raw - green low) / (green raw - green high)

Returns: 
* if `setGreenRange()` has been called, calibrated green reading from sensor
* -1 otherwise

Example:
```
void loop()
{
    double green_cal = cs.readGreenCal();
    //...
}
```

##### `double readBlueCal()`
Returns calibrated blue reading from sensor.

Calibrated values are calculated as such: blue calibrated = (blue raw - blue low) / (blue raw - blue high)

Returns: 
* if `setBlueRange()` has been called, calibrated blue reading from sensor
* -1 otherwise

Example:
```
void loop()
{
    double blue_cal = cs.readBlueCal();
    //...
}
```

##### `double readClearCal()`
Returns calibrated clear reading from sensor.

Calibrated values are calculated as such: clear calibrated = (clear raw - clear low) / (clear raw - clear high)

Returns: 
* if `setClearRange()` has been called, calibrated clear reading from sensor
* -1 otherwise

Example:
```
void loop()
{
    double clear_cal = cs.readClearCal();
    //...
}
```