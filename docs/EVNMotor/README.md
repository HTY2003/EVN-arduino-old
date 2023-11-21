# EVNMotor
EVNMotor is a class used to interface EVN Alpha with LEGO EV3 and NXT motors, as well as gearmotors plugged into the 6-pin headers.

## Constructor
`EVNMotor(uint8_t port, uint8_t motortype = EV3_LARGE, uint8_t motor_dir = DIRECT, uint8_t enc_dir = DIRECT)`

Arguments:
* port: integer from 1-4 indicating which port the motor is connected to

* motortype: `EV3_LARGE` (default, EV3 Large Motor), `EV3_MED`(EV3 Medium Motor), `NXT_LARGE`(NXT Large Motor), or `CUSTOM_MOTOR` (Custom Motor)

* motor_dir: `DIRECT` (default) or `REVERSE`. `REVERSE` swaps the 2 motor pins and 2 encoder pins to change motor direction

* enc_dir: `DIRECT` (default) or `REVERSE`. `REVERSE` swaps the 2 encoder pins to change encoder direction. Unnecessary for LEGO gearmotors, but may be useful for gearmotors increasing PWM values leads to negative RPM with default settings.

Example:
```
EVNMotor motora(1);
EVNMotor motorb(2, EV3_MED);
EVNMotor motorc(3, NXT_LARGE, REVERSE);
EVNMotor motord(4, CUSTOM_MOTOR, DIRECT, REVERSE);
```

## Functions
##### `void begin()`
Initializes motor hardware for position + speed measurement, and speed control (below functions).

Example:
```
void setup1()
{
    motor.begin();
}
```


### Movement
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

##### `void runDegrees(double degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true)`
Arguments:
* degrees: desired rotation of motor from current position (in degrees). To run in the opposite direction, use negative values
* stop_action:  `STOP_BRAKE` (default), `STOP_COAST`, or `STOP_HOLD`
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
* time_ms: desired length of time to run motor (in milliseconds)
* stop_action:  `STOP_BRAKE` (default), `STOP_COAST`, or `STOP_HOLD`
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