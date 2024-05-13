#include <EVNServo.h>

servo_state_t* EVNServo::servoArgs[];
bool EVNServo::servos_enabled[] = { false, false, false, false };;

EVNServo::EVNServo(uint8_t port, bool servo_dir, uint16_t range, double start_angle, uint16_t min_pulse_us, uint16_t max_pulse_us, double max_dps)
{
    _servo.servo = new Servo;
    _servo.servo_dir = servo_dir;
    _servo.range = range;
    _servo.min_pulse_us = min_pulse_us;
    _servo.max_pulse_us = max_pulse_us;
    _servo.dps = 0;
    _servo.max_dps = max(1, max_dps);
    _servo.angle = constrain(start_angle, 0, range);
    _servo.end_angle = 0;
    _servo.sweep = false;

    uint8_t portc = constrain(port, 1, 4);
    _servo.port = portc;

    switch (portc)
    {
    case 1:
        _servo.pin = SERVO_PORT_1;
        break;
    case 2:
        _servo.pin = SERVO_PORT_2;
        break;
    case 3:
        _servo.pin = SERVO_PORT_3;
        break;
    case 4:
        _servo.pin = SERVO_PORT_4;
        break;
    }
}

void EVNServo::begin()
{
    pinMode(_servo.pin, OUTPUT);
    _servo.servo->attach(_servo.pin, 200, 2800);

    double pulse = (double)(_servo.angle / _servo.range) * (double)(_servo.max_pulse_us - _servo.min_pulse_us);
    if (_servo.servo_dir == DIRECT)
        pulse = (double)_servo.min_pulse_us + pulse;
    else
        pulse = (double)_servo.max_pulse_us - pulse;
    _servo.servo->writeMicroseconds(pulse);

    attach_servo_interrupt(&_servo);
}

void EVNServo::write(double angle, uint16_t wait_time_ms, double dps)
{
    this->writeAngle(angle, wait_time_ms, dps);
}

void EVNServo::writeAngle(double angle, uint16_t wait_time_ms, double dps)
{
    if (motors_enabled())
    {
        if (dps == 0)
        {
            _servo.angle = constrain(angle, 0, _servo.range);
            double pulse = (double)(_servo.angle / _servo.range) * (double)(_servo.max_pulse_us - _servo.min_pulse_us);
            if (_servo.servo_dir == DIRECT)
                pulse = (double)_servo.min_pulse_us + pulse;
            else
                pulse = (double)_servo.max_pulse_us - pulse;
            this->writeMicroseconds(pulse);
        }
        else
        {
            _servo.end_angle = constrain(angle, 0, _servo.range);
            _servo.dps = min(fabs(dps), _servo.max_dps);
            _servo.sweep = true;
        }

        uint64_t start_time = millis();
        while ((millis() - start_time) < wait_time_ms)
        {
            if (!motors_enabled())
                break;
        }
    }
}

void EVNServo::writeMicroseconds(uint16_t pulse_us, uint16_t wait_time_ms)
{
    if (motors_enabled())
    {
        _servo.pulse = constrain(pulse_us, 200, 2800);
        _servo.sweep = false;

        uint64_t start_time = millis();
        while ((millis() - start_time) < wait_time_ms)
        {
            if (!motors_enabled())
                break;
        }
    }
}

void EVNServo::writeDutyCycle(double duty_cycle)
{
    if (motors_enabled())
    {
        double duty_cyclec = constrain(duty_cycle, -1, 1);
        double pulse = (1 - fabs(duty_cycle)) * 0.5 * (double)(_servo.max_pulse_us - _servo.min_pulse_us);

        if ((_servo.servo_dir == DIRECT) == (duty_cyclec < 0))
            pulse = (double)_servo.min_pulse_us + pulse;
        else
            pulse = (double)_servo.max_pulse_us - pulse;

        this->writeMicroseconds(pulse);
        _servo.sweep = false;
    }
}