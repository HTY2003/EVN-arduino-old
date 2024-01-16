# EVNCompassSensor
EVNCompassSensor is a class used to interface EVN Alpha with our Compass Sensor Standard Peripheral, which uses a HMC5883L or QMC5883L IC.

## Some Technical Details
This library supports both compass ICs, as QMC5883L chips can often be found on the same breakout boards as the HMC5883L, without any difference other than the tiny chip markings.

Compared to other libraries, the main benefit of EVNCompassSensor is that instead of polling an I2C register to determine if a new reading is ready (which can take a while if you load 16 sensors), the time between sensor reads is measured and new data is only read when the required sampling time for a new reading has lapsed. Another reason this wrapper library was created was to handle port selection and deselection.

Currently, only raw magnetometer readings and yaw calculation is supported. However, plans for simple hard iron calibration (and maybe even bit-less-simple soft iron calibration) is in the works.

With so much setup hassle, why don't we offer a gyroscope-based peripheral similar to the EV3's Gyro? While magnetometers require more calibration, they are capable of maintaining their heading without drifting over time like gyroscopes. This means the robot can do perfect 90 degree turns regardless of how long the program has been running.

Note: EVNCompassSensor does port selection and deselection automatically, so users do not need to call `setPort()` on their `EVNAlpha` objects.

Note 2: The Compass Sensor Standard Peripheral is quite sensitive to any motors running near it. Try to mount it a good distance away from any motors (higher is better).

## List of Functions
- [Constructor](#constructor)
- [begin()](#bool-begin)
- [read()](#double-read)
- [readRawX()](#double-readrawx)
- [readRawY()](#double-readrawy)
- [readRawZ()](#double-readrawz)

## Constructor
##### `EVNCompassSensor(port)`

Arguments:
* port: the I2C port the display is connected to (1-16, left to right)

Example:
```
EVNCompassSensor compass(5);
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
    compass.begin();
    //...
}
```

##### `double read()`
Returns yaw / heading of compass from 0-360 degrees.

Returns: 
* heading (in degrees)

Example:
```
void loop()
{
    double dist = compass.read();
    //...
}
```

##### `double readRawX()`
Returns X-axis magnetometer reading in Gauss.

Returns: 
* X-axis magnetometer reading (in Gauss)

Example:
```
void loop()
{
    double x = compass.readRawX();
    //...
}
```

##### `double readRawY()`
Returns Y-axis magnetometer reading in Gauss.

Returns: 
* Y-axis magnetometer reading (in Gauss)

Example:
```
void loop()
{
    double y = compass.readRawY();
    //...
}
```

##### `double readRawZ()`
Returns Z-axis magnetometer reading in Gauss.

Returns: 
* Z-axis magnetometer reading (in Gauss)

Example:
```
void loop()
{
    double z = compass.readRawZ();
    //...
}
```