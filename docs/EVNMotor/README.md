# EVNMotor
EVNMotor is a class used to interface EVN Alpha with LEGO EV3 and NXT motors, as well as gearmotors plugged into the 6-pin headers.

## Some Technical Details
EVNMotor uses timer-generated interrupts from the [RPi_PICO_TimerInterrupt](https://github.com/khoih-prog/RPI_PICO_TimerInterrupt) library to achieve consistent PID speed and position control. This means that when the motor is set to run at 30RPM, it will attempt to maintain speed regardless of the load on the motor. Using these functions, we have tried to replicate most of the standard functions EV3 users will be familiar with. We have provided PID presets for NXT Large Motors, EV3 Medium Motors and EV3 Large Motors, but the user may edit them if they wish to.

Note: for gearmotors plugged into the 6-pin headers, users will have to set additional parameters in `src/evn_motor_pids.h`, such as the rated RPM and PPR (pulses per revolution of their encoders).

Note 2: A gearmotor and a LEGO motor should not be simultaneously plugged into the same motor port, as the 6-pin headers are mapped to the LEGO ports.

Note 3: By default, EVNMotor objects will not run the motor until the user button is pressed. See EVNAlpha's README for more info.

## List of Functions
- [Constructor](#constructor)
- [begin()](#void-begin)
- [writePWM()](#void-writepwmdouble-pwm)
- [runSpeed()](#void-runspeeddouble-rpm)
- [runDegrees()](#void-rundegreesdouble-rpm-double-degrees-uint8t-stopaction-stopbrake-bool-wait-true)
- [runTime()](#void-runtimedouble-rpm-double-timems-uint8t-stopaction-stopbrake-bool-wait-true)
- [brake()](#void-brake)
- [coast()](#void-coast)
- [hold()](#void-hold)

## Constructor
`EVNMotor(uint8_t port, uint8_t motortype = EV3_LARGE, uint8_t motor_dir = DIRECT, uint8_t enc_dir = DIRECT)`

Arguments:
* port: integer from 1-4 indicating which port the motor is connected to

* motortype:
    * `EV3_LARGE` (default, EV3 Large Motor)
    * `EV3_MED`(EV3 Medium Motor)
    * `NXT_LARGE`(NXT Large Motor)
    * `CUSTOM_MOTOR` (Custom Motor)

* motor_dir: 
    * `DIRECT` (default)
    * `REVERSE` (swaps the 2 motor pins and 2 encoder pins to flip motor direction)

* enc_dir:
    * `DIRECT` (default)
    * `REVERSE` (only swaps the 2 encoder pins to change encoder direction, unnecessary for LEGO gearmotors)

Example:
```
EVNMotor motora(1);
EVNMotor motorb(2, EV3_MED);
EVNMotor motorc(3, NXT_LARGE, REVERSE);
EVNMotor motord(4, CUSTOM_MOTOR, DIRECT, REVERSE);
```

## Functions
##### `void begin()`
Initializes motor hardware. This function can be run in `void setup()` but it should be run in `void setup1()` for better performance.

Example:
```
void setup1()
{
    motor.begin();
}
```

#### `void writePWM(double pwm)`
Writes PWM value to motor. With this command, the motor's speed is not regulated using PID.

Arguments:
* pwm: floating point value from -1 to 1. To run in opposite direction, use negative values.

Example:
```
void loop()
{
    motor.writePWM(0.5);
    //...
}
```

##### `void runSpeed(double rpm)`
Sets motor to run at desired revolutions per minute (RPM) indefinitely, until next command is given.

Arguments:
* rpm: desired speed of motor in RPM. To run in opposite direction, use negative values.

Example:
```
void loop()
{
    motor.runSpeed(40);
    //...
}
```

##### `void runDegrees(double rpm, double degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true)`
Arguments:
* degrees: desired rotation of motor from current position (in degrees). To run in the opposite direction, use negative values
* stop_action: behaviour upon reaching desired position.
    * `STOP_BRAKE` (default, same behaviour as [brake()](#void-brake))
    * `STOP_COAST` (same behaviour as [coast()](#void-coast))
    * `STOP_HOLD` (same behaviour as [hold()](#void-hold))
* wait: whether program will wait for motor to complete its command (true), or proceed without waiting (false)

Example:
```
void loop()
{
    motora.runDegrees(360);
    //...
    motorb.runDegrees(720, STOP_COAST);
    //...
    motora.runDegrees(-360, STOP_HOLD, false);
    motorb.runDegrees(-360, STOP_HOLD);     //runDegrees on 2 motors concurrently
}
```

##### `void runTime(double rpm, double time_ms, uint8_t stop_action = STOP_BRAKE, bool wait = true)`
Arguments:
* rpm: desired rotation of motor from current position (in degrees). To run in opposite direction, use negative values
* time_ms: desired duration to run motor (in milliseconds)
* stop_action: behaviour after running for desired duration.
    * `STOP_BRAKE` (default, same behaviour as [brake()](#void-brake))
    * `STOP_COAST` (same behaviour as [coast()](#void-coast))
    * `STOP_HOLD` (same behaviour as [hold()](#void-hold))
* wait: whether program will wait for motor to complete its command (true), or proceed without waiting (false)

Example:
```
void loop()
{
    motora.runTime(20, 10000);
    //...
    motorb.runTime(30, 5000, STOP_COAST);
    //...
    motora.runTime(-20, 5000, STOP_HOLD, false);
    motorb.runTime(-20, 5000, STOP_HOLD);     //runTime on 2 motors concurrently
}
```

##### `void brake()`
Brings motor to a quick stop by effectively shorting the motor pins.

Example:
```
void loop()
{
    motor.brake();
    //...
}
```

##### `void coast()`
Brings motor to a gradual stop by effectively disconnecting the motor pins, allowing motor to slow down at its own rate.

Example:
```
void loop()
{
    motor.coast();
    //...
}
```

##### `void hold()`
Locks motor in its current encoder position. 
If the motor shaft is turned due to external forces, the motor will run to restore its encoder position.

Example:
```
void loop()
{
    motor.hold();
    //...
}
```