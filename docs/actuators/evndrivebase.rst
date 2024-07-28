``EVNDrivebase``
================

Constructor
-----------

.. class:: EVNDrivebase(float wheel_dia, float axle_track, EVNMotor* motor_left, EVNMotor* motor_right);

    :param wheel_dia: Diameter of each wheel (in mm)

    :param axle_track: Distance between midpoint of both wheels (in mm)

    :param motor_left: Pointer to ``EVNMotor`` object for left motor

    :param motor_right: Pointer to ``EVNMotor`` object for right motor;

Example Usage:

.. code-block:: cpp

    EVNMotor left_motor(1, EV3_MED);
    EVNMotor right_motor(2, EV3_MED, REVERSE);
    //wheel diameter - 62mm, axle track - 178mm
    EVNDrivebase db(62, 178, &left_motor, &right_motor);


Functions
---------

.. function:: void begin();

    Initializes drivebase object. Call this function after calling begin() on the EVNMotor objects, but before calling any other EVNDrivebase functions.

.. note::
    This command should be run on the 2nd core using ``void setup1()``. 
    However, you can still call the movement functions in ``void loop()`` like a normal program.

Example Program:

.. code-block:: cpp

    EVNMotor left_motor(1, EV3_MED);
    EVNMotor right_motor(2, EV3_MED, REVERSE);
    EVNDrivebase db(62, 178, &left_motor, &right_motor);

    void setup1()
    {
        left_motor.begin();
        right_motor.begin();
        db.begin();
    }

Measurements
""""""""""""

.. function:: float getDistance()

    :returns: Distance travelled by drivebase (in mm)

.. function:: float getAngle()

    :returns: Angle turned by drivebase (in deg)

.. function:: float getHeading()

    :returns: Drivebase heading (ranges from 0 - 360deg)

.. function:: float getX()

    :returns: X coordinate of drivebase from origin (origin is the drivebase's position on startup)

.. function:: float getY()

    :returns: Y coordinate of drivebase from origin (origin is the drivebase's position on startup)

.. function:: void resetXY();

    Sets drivebase's position to be Origin (0, 0).

.. function:: float getDistanceToPoint(float x, float y);

    :returns: Euclidean distance between drivebase's XY position and target XY point


Move Forever
""""""""""""

.. function::   void drive(float speed, float turn_rate);
                void driveTurnRate(float speed, float turn_rate);

    Runs drivebase at the given speed and turn rate until a new command is called

    :param speed: velocity of drivebase (in mm/s)

    :param turn_rate: turning rate of drivebase (in deg/s)

.. function:: void driveRadius(float speed, float radius);

    Runs drivebase at the given speed and radius of turning until a new command is called

    :param speed: velocity of drivebase (in mm/s)

    :param radius: turning radius of drivebase (in mm)


Move by a Fixed Amount
""""""""""""""""""""""

.. function:: void straight(float speed, float distance, uint8_t stop_action = STOP_BRAKE, bool wait = true);

    Runs drivebase in a straight line for the specified distance, then performs given stop action

    :param speed: velocity of drivebase (in mm/s)

    :param distance: distance to travel (in mm)

    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position

    :param wait: Block function from returning until command is finished

.. function::   void curve(float speed, float radius, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true);
                void curveRadius(float speed, float radius, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true);

    Runs drivebase in a curve of specified radius until its heading has shifted by the given angle, then performs given stop action

    :param speed: velocity of drivebase (in mm/s)

    :param radius: turning radius of drivebase (in mm)

    :param angle: angle to travel by (in deg)

    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position

    :param wait: Block function from returning until command is finished

.. function:: void curveTurnRate(float speed, float turn_rate, float angle, uint8_t stop_action = STOP_BRAKE, bool wait = true);

    Runs drivebase at given speed and turn rate until its heading has shifted by the given angle, then runs specified stop action

    :param speed: velocity of drivebase (in mm/s)

    :param turn_rate: turning rate of drivebase (in deg/s)

    :param angle: angle to travel by (in deg)

    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position

    :param wait: Block function from returning until command is finished

.. function::   void turn(float turn_rate, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true);
                void turnDegrees(float turn_rate, float degrees, uint8_t stop_action = STOP_BRAKE, bool wait = true);

    Rotate drivebase on the spot by the given angle, then performs given stop action
    
    :param turn_rate: turning rate of drivebase (in deg/s)

    :param angle: angle to travel by (in deg)

    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position

    :param wait: Block function from returning until command is finished

.. function:: void turnHeading(float turn_rate, float heading, uint8_t stop_action = STOP_BRAKE, bool wait = true);

    Rotate drivebase on the spot to the given heading, then performs given stop action

    :param turn_rate: turning rate of drivebase (in deg/s)

    :param heading: heading to travel to (in deg)

    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position

    :param wait: Block function from returning until command is finished

.. function:: bool completed();

    :returns: Boolean indicating whether the drivebase's command has reached completion

Move to Point
""""""""""""""""
.. function:: void driveToXY(float speed, float turn_rate, float x, float y, uint8_t stop_action = STOP_BRAKE, bool restore_initial_heading = true);

    Rotates drivebase to face target XY position, drives forward to target, and rotates back to original heading

    :param speed: velocity of drivebase (in mm/s)

    :param turn_rate: turning rate of drivebase (in deg/s)

    :param x: X coordinate of target

    :param y: Y coordinate of target

    :param stop_action: Behaviour of the motor upon completing its command. Defaults to ``STOP_BRAKE``

        * ``STOP_BRAKE`` -- Brake (Slow decay)
        * ``STOP_COAST`` -- Coast (Fast decay)
        * ``STOP_HOLD`` -- Hold position

    :param wait: Block function from returning until command is finished

.. note:: This feature is experimental! Its behaviour may be changed in future versions.

Stopping
""""""""

.. function::   void stop();
                void brake();

    Brakes both drivebase motors (slow decay)

.. function:: void coast();
    
    Coast both drivebase motors (fast decay)


.. function:: void hold();
    
    Hold drivebase motors in their current positions

Control Settings
""""""""""""""""

To view the default PID and accel/decel values, look at ``src\evn_motor_defs.h`` in the Github repository.

.. function:: void setSpeedPID(float kp, float ki, float kd);

    Sets PID gain values for the speed controller (controls average drivebase speed).

    The error for the controller is the difference between the robot's target distance travelled (which increases over time) and the robot's current distance travelled.

    If your robot fails to consistently hit its desired speed, consider increasing kp. However, increasing it too much may cause the drivebase to jitter instead of moving smoothly.

    :param kp: Proportional gain
    :param ki: Integral gain
    :param kd: Derivative gain

.. function:: void setTurnRatePID(float kp, float ki, float kd);

    Sets PID gain values for the turn rate controller (controls rate of turning of drivebase).

    The error for the controller is the difference between the robot's target angle (which shifts over time if travelling in a curve) and the robot's current angle.

    This controller serves 2 purposes: to ensure the robot turns at the correct rate during movements, and to stop either motor if the other is stalled, essentially syncing their movement.

    If your robot jitters, consider lowering kp and kd. However, lowering kp and kd will mean that the motor sync will have a greater delay, making it less responsive.

    :param kp: Proportional gain
    :param ki: Integral gain
    :param kd: Derivative gain

.. function:: void setSpeedAccel(float speed_accel);

    Sets speed acceleration value for drivebase (in mm/s^2).

.. function:: void setSpeedDecel(float speed_decel);

    Sets speed acceleration value for drivebase (in mm/s^2).

.. function:: void setTurnRateAccel(float turn_rate_accel);

    Sets turn rate deceleration value for drivebase (in deg/s^2).

.. function:: void setTurnRateDecel(float turn_rate_decel);

    Sets turn rate deceleration value for drivebase (in deg/s^2).
