# EVNDistanceSensor
EVNDistanceSensor is a class used to interface EVN Alpha with our Distance Sensor Standard Peripheral, which uses a VL53L0X IC for distance ranging.

## Some Technical Details
This library is based on the VL53L0X library by Pololu. Huge thanks to Pololu for their hard work reverse-engineering ST Micro's API for the VL53L0X and offering it to the community. If you are looking to do a non-LEGO build with EVN, feel free to support them by purchasing their high-quality 6V motors and VL53L0X breakout boards (both EVN Alpha-compatible).

Compared to other libraries, the main benefit of EVNDistanceSensor is that instead of polling an I2C register to determine if a new reading is ready (which can take a while if you load 16 sensors), the time between sensor reads is measured and new data is only read when the required timing budget for a new reading has lapsed.

Another reason this wrapper library was created was to handle port selection and deselection.

Currently, only reading distance values and setting the measurement time for each reading is supported. 


Note: EVNDistanceSensor does port selection and deselection automatically, so users do not need to call `setPort()` on their `EVNAlpha` objects.

Note 2: The VL53L0X pulses 940nm Class 1 Laser infrared light to range distance. If your use case has restrictions of the use of infrared light, do take note of this.

## List of Functions
- [Constructor](#constructor)
- [begin()](#bool-begin)
- [setTimingBudget()](#void-settimingbudgetuint8t-timingbudgetms)
- [read()](#uint16t-read)

## Constructor
##### `EVNDistanceSensor(port, uint8_t timing_budget_ms = 33)`

Arguments:
* port: the I2C port the display is connected to (1-16, left to right)
* timing_budget_ms: timing budget for each reading, values can range from 20 to 200, default is 33

Example:
```
EVNDistanceSensor ds(5);
```
Another example:
```
EVNDistanceSensor ds(5, 33); //33ms timing budget
```

## Functions
##### `bool begin()`
Initializes distance sensor to start running in continuous mode.

Returns:
* `true` if successfully detected and initialized, `false` otherwise

Example:
```
void setup()
{
    ds.begin();
    //...
}
```

##### `void setTimingBudget(uint8_t timing_budget_ms)`
Sets the timing budget given for each reading.

Arguments:
* timing_budget: timing budget given for each reading (in ms)


Example:
```
void setup()
{
    ds.begin();
    ds.setTimingBudget(50);
    //...
}
```

##### `uint16_t read()`
Returns distance reading from sensor.

Returns: 
* distance reading (in mm)

Example:
```
void loop()
{
    int dist = ds.read();
    //...
}
```