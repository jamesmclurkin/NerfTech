#include <Arduino.h>
#include <Servo.h>
#include "MotorDriver.h"


#define BRUSHED_ESC_NEUTRAL               90
#define BRUSHED_ESC_FORWARD_MAX           160
#define BRUSHED_ESC_BRAKE                 20
#define BRUSHED_ESC_STARTUP_DELAY         4000

#define BRUSHLESS_ESC_NEUTRAL               0
#define BRUSHLESS_ESC_FORWARD_MAX           160
#define BRUSHLESS_ESC_BRAKE                 0
#define BRUSHLESS_ESC_STARTUP_DELAY         4000


MotorDriver::MotorDriver(int type, int pin_1)
{
  this->_pin_1 = pin_1;
  this->_pin_2 = -1;
  this->_type = type;
  this->startupTime = millis();

  switch(this->_type) {
    case MOTOR_TYPE_BRUSHED_ESC:{
      pinMode(this->_pin_1, OUTPUT);
      // init the esc.  we control it with a servo object
      this->servoESC.attach(this->_pin_1); // attaches the servo on pin 9 to the servo object
      this->servoESC.write(BRUSHED_ESC_NEUTRAL);  
      break;
    }
    case MOTOR_TYPE_BRUSHLESS_ESC:{
      pinMode(this->_pin_1, OUTPUT);
      // init the esc.  we control it with a servo object
      this->servoESC.attach(this->_pin_1); // attaches the servo on pin 9 to the servo object
      this->servoESC.write(BRUSHLESS_ESC_NEUTRAL);  
      break;
    }
    case MOTOR_TYPE_LOWSIDE_DRIVER:{
      pinMode(this->_pin_1, OUTPUT);
      analogWrite(this->_pin_1, 0);
      break;
    }
    case MOTOR_TYPE_DRV8870_HBRIDGE:{
      //TODO thoow some kind of error
      break;
    }
  }

}

MotorDriver::MotorDriver(int type, int pin_1, int pin_2)
{
  this->_pin_1 = pin_1;
  this->_pin_2 = pin_2;
  this->_type = type;
  this->startupTime = millis();

  switch(this->_type) {
    case MOTOR_TYPE_BRUSHED_ESC:
    case MOTOR_TYPE_BRUSHLESS_ESC:
    case MOTOR_TYPE_LOWSIDE_DRIVER:{
      //TODO thoow some kind of error
      break;
    }
    case MOTOR_TYPE_DRV8870_HBRIDGE:{
      pinMode(this->_pin_1, OUTPUT);
      pinMode(this->_pin_2, OUTPUT);
      this->coast();
      break;
    }
  }

}

void MotorDriver::forward(float speed)
{
  this->setVelocity(speed);
}

void MotorDriver::reverse(float speed)
{
  this->setVelocity(-speed);
}

#define PWM_MAX                 255

int pwmFromSpeed(float speed) {
  // map the speed (0.0 - 100.0) to a PWM value from 0-255
  int pwm = (int)(abs(speed) * PWM_MAX / 100);
  return pwm;
}

void MotorDriver::setVelocity(float speed)
{
  speed = constrain(speed, -100.0, 100.0);
  switch(this->_type) {
    case MOTOR_TYPE_BRUSHED_ESC:{
      if(millis() > this->startupTime + BRUSHED_ESC_STARTUP_DELAY) {
        if(speed > 0.0) {
          int s = (int)speed;
          int servoPos = map(s, 0, 100, BRUSHED_ESC_NEUTRAL, BRUSHED_ESC_FORWARD_MAX);
          this->servoESC.write(servoPos);  
        } else {
          this->servoESC.write(BRUSHED_ESC_NEUTRAL);  
        }
      }
      break;
    }
    case MOTOR_TYPE_BRUSHLESS_ESC:{
      if(millis() > this->startupTime + BRUSHLESS_ESC_STARTUP_DELAY) {
        if(speed > 0.0) {
          int s = (int)speed;
          int servoPos = map(s, 0, 100, BRUSHLESS_ESC_NEUTRAL, BRUSHLESS_ESC_FORWARD_MAX);
          this->servoESC.write(servoPos);  
        } else {
          this->servoESC.write(BRUSHLESS_ESC_NEUTRAL);  
        }
      }
      break;
    }
    case MOTOR_TYPE_LOWSIDE_DRIVER:{
      int pwm = pwmFromSpeed(speed);
      if (speed >= 0.0) {
          analogWrite(this->_pin_1, pwm);
      }
      else {
          analogWrite(this->_pin_1, 0);
      }
      break;
    }
    case MOTOR_TYPE_DRV8870_HBRIDGE:{
      int pwm = pwmFromSpeed(speed);
      // invert the duty cycle for braking while in the off mode
      pwm = PWM_MAX - pwm;

      if (speed >= 0.0) {
          analogWrite(this->_pin_1, pwm);
          analogWrite(this->_pin_2, PWM_MAX);
      }
      else {
          analogWrite(this->_pin_1, PWM_MAX);
          analogWrite(this->_pin_2, pwm);
      }
      break;
    }
  }
}

void MotorDriver::coast(void)
{
  switch(this->_type) {
    case MOTOR_TYPE_BRUSHED_ESC:{
      if(millis() > this->startupTime + BRUSHED_ESC_STARTUP_DELAY) {
        this->servoESC.write(BRUSHED_ESC_NEUTRAL);  
      }
      break;
    }
    case MOTOR_TYPE_BRUSHLESS_ESC:{
      if(millis() > this->startupTime + BRUSHLESS_ESC_STARTUP_DELAY) {
        this->servoESC.write(BRUSHLESS_ESC_NEUTRAL);  
      }
      break;
    }
    case MOTOR_TYPE_LOWSIDE_DRIVER:{
      analogWrite(this->_pin_1, 0);
      break;
    }
    case MOTOR_TYPE_DRV8870_HBRIDGE:{
      analogWrite(this->_pin_1, 0);
      analogWrite(this->_pin_2, 0);
      break;
    }
  }
}

void MotorDriver::brake(float strength)
{
  strength = constrain(strength, -100.0, 100.0);
  switch(this->_type) {
    case MOTOR_TYPE_BRUSHED_ESC:{
      if(millis() > this->startupTime + BRUSHED_ESC_STARTUP_DELAY) {
        if(strength > 0.0) {
          int s = (int)strength;
          int servoPos = map(s, 0, 100, BRUSHED_ESC_NEUTRAL, BRUSHED_ESC_BRAKE);
          this->servoESC.write(servoPos);  
        } else {
          this->servoESC.write(BRUSHED_ESC_NEUTRAL);  
        }
      }
      break;
    }
    case MOTOR_TYPE_BRUSHLESS_ESC:{
      if(millis() > this->startupTime + BRUSHLESS_ESC_STARTUP_DELAY) {
        this->servoESC.write(BRUSHED_ESC_NEUTRAL);  
      }
      break;
    }
    case MOTOR_TYPE_LOWSIDE_DRIVER:{
      analogWrite(this->_pin_1, 0);
      break;
    }
    case MOTOR_TYPE_DRV8870_HBRIDGE:{
      int pwm = pwmFromSpeed(strength);
      analogWrite(this->_pin_1, pwm);
      analogWrite(this->_pin_2, pwm);
      break;
    }
  }
}
