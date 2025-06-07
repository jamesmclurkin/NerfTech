/*
 * MotorDriver.h - Nerftech Tachometer library - Version 1.0.0
 *
 * Original library        (1.0.0)   by James McLurkin
 *
 * Text goes here
 *
 */

#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>
#include <Servo.h>


#define MOTOR_TYPE_BRUSHED_ESC          0
#define MOTOR_TYPE_BRUSHLESS_ESC        1
#define MOTOR_TYPE_LOWSIDE_DRIVER       2
#define MOTOR_TYPE_DRV8870_HBRIDGE      3

class MotorDriver
{
private:
  int _pin_1;
  int _pin_2;
  int _type;
  unsigned long startupTime;
  Servo servoESC;



public:
  /** Creates a MotorDriver interface
   *
   * @param pin_1 first pin.  use for single-pin motor controllers
   * @param pin_2 second pin.  use for H-Bridges
   * @param type the type of motor criver we're controlling
   *
   */
  MotorDriver(int type, int pin_1);
  MotorDriver(int type, int pin_1, int pin_2);


  void forward(float speed);

  void reverse(float speed);

  void setVelocity(float speed);

  void coast(void);

  void brake(float strength);

};

#endif // MOTORDRIVER_H
