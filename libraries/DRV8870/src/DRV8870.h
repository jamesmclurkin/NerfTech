/*
 * DRV8870.h - DRV8870 library for Wiring/Arduino - Version 1.0.0
 *
 * Original library        (1.0.0)   by Rodney Osodo.
 *
 * The moded the DRV8870 motor driver can run are listed below:
 *
 *    x_PWM1 x_PWM2    Mode
 *      0      0       Coast/Fast decay
 *      0      1       Reverse
 *      1      0       Forward
 *      1      1       Brake/slow decay
 */

#ifndef DRV8870_H
#define DRV8870_H

#define CLOCKWISE           1
#define COUNTERCLOCKWISE    0
#define MOTOR_SPEED_MAX     255

#include "Arduino.h"
#if defined(ESP32)
#include <analogWrite.h>
#endif
#if defined(ESP8266)
#include <analogWrite.h>
#endif

class DRV8870
{
private:
    // The library version number
    int _version = 2;

    // motor pin numbers. It can driver up to a maximum of 4 motors
    int _motor_pin_1;
    int _motor_pin_2;

public:
    /** Creates a DRV8870(H-bridge motor controller) control interface to drive 1 motor
     *
     * @param motor_pin_1 A PWM enabled pin, tied to the IN1 Logic input and controls state of OUT1
     * @param motor_pin_2 A PWM enabled pin, tied to the IN2 Logic input and controls state of OUT2
     *
     */
    DRV8870(int motor_pin_1, int motor_pin_2);

    /** Set the speed of the motor
     *
     * @param motor_speed The speed of the motor as a normalised value between 0 and max_speed
     */
    void setSpeed(int motor_speed, int direction);

    /** Brake the H-bridge.
     *
     * Defaults to 100% (max_speed).
     * @param brake_speed - Braking speed (power).
     *
     */
    void brake(int brake_speed);

    /** Coast the H-bridge
     *
     *
     */
    void coast(void);

    /** Returns the version of the library
     *
     *
     */
    int version(void);
};

#endif // DRV8870_H
