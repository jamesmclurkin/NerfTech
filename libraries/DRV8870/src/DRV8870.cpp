#include "DRV8870.h"

DRV8870::DRV8870(int motor_pin_1, int motor_pin_2)
{
    this->_motor_pin_1 = motor_pin_1;
    this->_motor_pin_2 = motor_pin_2;
    pinMode(this->_motor_pin_1, OUTPUT);
    pinMode(this->_motor_pin_2, OUTPUT);
    this->coast();
}

void DRV8870::setSpeed(int motor_speed, int direction)
{
    int pwm = constrain(motor_speed, 0, MOTOR_SPEED_MAX);
    pwm = MOTOR_SPEED_MAX - pwm;
    if (direction == CLOCKWISE) {
        analogWrite(this->_motor_pin_1, pwm);
        analogWrite(this->_motor_pin_2, MOTOR_SPEED_MAX);
    }
    else if (direction == COUNTERCLOCKWISE) {
        analogWrite(this->_motor_pin_1, MOTOR_SPEED_MAX);
        analogWrite(this->_motor_pin_2, pwm);
    }
}

void DRV8870::brake(int motor_speed)
{
    int pwm = constrain(motor_speed, 0, MOTOR_SPEED_MAX);
    //motor_speed = MOTOR_SPEED_MAX - motor_speed;
    analogWrite(this->_motor_pin_1, pwm);
    analogWrite(this->_motor_pin_2, pwm);
}

void DRV8870::coast(void)
{
    analogWrite(this->_motor_pin_1, 0);
    analogWrite(this->_motor_pin_2, 0);
}

int DRV8870::version(void)
{
    return _version;
}
