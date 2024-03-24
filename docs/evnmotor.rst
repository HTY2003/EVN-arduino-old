``EVNMotor`` -- Motors with encoder feedback
============================================

Some Technical Details
----------------------

EVNMotor uses timer-generated interrupts to achieve consistent speed and position control. Using these functions, we have replicated most of the motor functions EV3 users will be familiar with. We have provided motor profiles for NXT Large Motors, EV3 Medium Motors and EV3 Large Motors, but the user may edit them if they wish to.

.. note::

    For gearmotors plugged into the 6-pin headers, users may have to set default parameters for the CUSTOM_MOTOR preset in `src/evn_motor_pids.h`, such as maximum RPM and encoder PPR (pulses per revolution).

.. note::

    By default, EVNMotor objects will not run the motor until the user button is pressed. See EVNAlpha's docs for more info.

.. warning::

    A gearmotor and a LEGO motor should not be simultaneously plugged into the same motor port, as the 6-pin headers are wired in parallel with the LEGO ports.


.. class:: EVNMotor(uint8_t port, uint8_t motortype = EV3_LARGE, uint8_t motor_dir = DIRECT, uint8_t enc_dir = DIRECT)
    
    :param port: Port the motor is connected to (1 - 4)
    
    :param motortype: Type of motor connected. Defaults to ``EV3_LARGE``

        * ``EV3_LARGE`` -- EV3 Large Servo Motor
        * ``EV3_MED`` -- EV3 Medium Servo Motor
        * ``NXT_LARGE`` -- NXT Large Servo Motor
        * ``CUSTOM_MOTOR`` -- Custom Gearmotor

    
    :param motor_dir: Swaps motor **and** encoder pins, flipping the direction of the motor. Defaults to ``DIRECT``

        * ``DIRECT`` -- Do not flip direction
        * ``REVERSE`` -- Flip direction

    :param enc_dir: Swaps encoder pins **only**. Not necessary for LEGO motors, but can be useful for non-LEGO gearmotors when the encoder input and motor output act in opposing directions. Defaults to ``DIRECT``

Example Usage:

.. code-block:: cpp

    EVNMotor motor1(1, EV3_MED);
    EVNMotor motor2(2, EV3_LARGE);
    EVNMotor motor3(3, NXT_LARGE, REVERSE);
    EVNMotor motor4(4, CUSTOM_MOTOR, DIRECT, REVERSE);

Functions
---------

.. function:: void begin()

    Initializes motor object. Call this function before calling the other EVNMotor functions.

.. note::
    For best performance, run this on the 2nd core using ``void setup1()``

Example Program:

.. code-block:: cpp

    EVNMotor motor(1, EV3_MED);

    void setup1()   //call on setup1() for best performance!
    {
        motor.begin();
    }

Measurements
""""""""""""

.. function:: double getPosition()

    :returns: Angular displacement of motor from its starting position, in degrees

.. function:: double getHeading()

    :returns: Motor position converted to range from 0-360 degrees

.. function:: void resetPosition()

    Reset starting position to motor's starting position.

.. function::   double getDPS()
                double getSpeed()

    :returns: Angular velocity of motor, in DPS (degrees per second)

.. function:: bool stalled()

    :returns: Boolean indicating when motor is stalled (unable to reach target velocity)

Example Usage:

.. code-block:: cpp

    double position = motor.getPosition();
    double heading = motor.getHeading();
    double speed = motor.getSpeed();
    
    motor.resetPosition();

Run Forever
"""""""""""

.. function:: void runPWM(double duty_cycle)

    Runs the motor at the given PWM duty cycle until a new command is called. Motor speed will vary with load torque applied.

    :param duty_cycle: duty cycle to run the motor at (floating point number from 0 to 1)

.. function::   void runDPS(double dps)
                void runSpeed(double dps)

    Runs the motor at the given angular velocity until a new command is called. Motor will attempt to maintain constant speed despite varying load torque.

    :param dps: Angular velocity to run the motor at (in DPS)

Example Usage:

.. code-block:: cpp

    //run motor at 100% duty cycle
    motor.runPWM(1);

    //run motor at 300DPS in the negative direction
    motor.runSpeed(-300);

Run by a Fixed Amount
"""""""""""""""""""""

.. function:: void runPosition(double dps, double position, uint8_t stop_action = STOP_BRAKE, bool wait = true)

    Run motor to the given motor shaft position, then performs the given stop action.

    :param dps: Angular velocity to run the motor at (in DPS)
    :param position: Position which the motor has to travel to (in degrees)
    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position
    
    :param wait: Block function from returning until command is finished

.. function:: void runAngle(double dps, double degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true)

    Run motor by the given angle (relative to its starting position), then performs the given stop action.

    :param dps: Angular velocity to run the motor at (in DPS)
    :param degrees: Angular displacement which the motor has to travel (in degrees)
    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position

    :param wait: Block function from returning until command is finished

.. function:: void runHeading(double dps, double heading, uint8_t stop_action = STOP_BRAKE, bool wait = true)

    Run motor to the specified motor shaft heading, then performs the given stop action.

    :param dps: Angular velocity to run the motor at (in DPS)
    :param time_ms: Heading which the motor has to travel to (0 - 360 degrees)
    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position

    :param wait: Block function from returning until command is finished

.. function:: void runTime(double dps, uint32_t time_ms, uint8_t stop_action = STOP_BRAKE, bool wait = true)

    Run motor for the given amount of time, then performs the given stop action.

    :param dps: Angular velocity to run the motor at (in DPS)
    :param time_ms: Time which the motor has to run for (in milliseconds)
    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position

    :param wait: Block function from returning until command is finished

.. function:: bool completed()

    :returns: Boolean indicating whether the motor has hit its target position / completed running for the set amount of time

Example Usage:

.. code-block:: cpp

    //run motor to a position of 180 degrees
    motor.runPosition(120, 180);

    //run motor at 120DPS in the negative direction for 1 second (1000ms)
    motor.runTime(-120, 1000, STOP_COAST);

    //run motor1 180 degrees in the negative direction from its current position
    motor1.runAngle(120, -180, STOP_HOLD, false);

    //at the same time, run motor2 to a heading of 75 degrees
    motor2.runHeading(120, 75, STOP_HOLD);

    //ensure that motor1 has completed before proceeding
    while (!motor1.completed());


Stopping
"""""""""

.. function::   void stop()
                void brake()

    Brakes the motor (slow decay).

.. function:: void coast()

    Coasts the motor (fast decay). Compared to `brake()`, motor comes to a stop more slowly.

.. function:: void hold()

    Hold the motor in its current position. Stops the motor shaft from moving freely.

Example Usage:

.. code-block:: cpp

    motor.stop();
    motor.brake();
    motor.coast();
    motor.hold();

Control Settings
""""""""""""""""

Docs to be added soon!

.. function:: void setPID(double p, double i, double d)

.. function:: void setAccel(double accel_dps_sq)

.. function:: void setDecel(double decel_dps_sq)

.. function:: void setMaxRPM(double max_rpm)

.. function:: void setPPR(uint32_t ppr)