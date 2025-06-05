#include "Tachometer.h"

void tachometerISR(void);

unsigned long tachReadTime;
unsigned long tachInterruptTime;
boolean tachRead = false;
int tachCount = 0;
int tachCountPeriod = 0;
long rpm = 0;

// Interrupt on rising edge on the tach pin.  two edges per rotation
void tachometerISR(void) {
  if(tachRead) {
    tachCountPeriod = tachCount;
    tachCount = 0;
    tachRead = false;
  }
  tachInterruptTime = millis();
  tachCount++;
}

long tachometerRPM (long counts) {
  long rpm = counts * TACH_READ_CONST;
  return rpm;
}


  if (currentTime > tachReadTime) {
    if (currentTime > (tachInterruptTime + TACH_READ_PERIOD)) {
      // It's been a long time since a tach interupt.  Clear the count
      tachCount = 0;
      tachCountPeriod = 0;
    } else {
      tachRead = true;
    }
    rpm = tachometerRPM(tachCountPeriod);
    tachReadTime += TACH_READ_PERIOD;
  }

Tachometer::Tachometer(int tach_pin)
{
  this->_tachometer_pin = tach_pin;
  pinMode(this->_tachometer_pin, INPUT);
  tachReadTime = millis();
}

void Tachometer::setSpeed(int motor_speed, int direction)
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

void Tachometer::brake(int motor_speed)
{
    int pwm = constrain(motor_speed, 0, MOTOR_SPEED_MAX);
    //motor_speed = MOTOR_SPEED_MAX - motor_speed;
    analogWrite(this->_motor_pin_1, pwm);
    analogWrite(this->_motor_pin_2, pwm);
}

void Tachometer::coast(void)
{
    analogWrite(this->_motor_pin_1, 0);
    analogWrite(this->_motor_pin_2, 0);
}

int Tachometer::version(void)
{
    return _version;
}
