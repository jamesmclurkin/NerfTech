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

#ifndef TACHOMETER_H
#define TACHOMETER_H

#define TACH_READ_PERIOD          100
#define TACH_READ_CONST           (30000 / TACH_READ_PERIOD)

#include "Arduino.h"
#if defined(ESP32)
#include <analogWrite.h>
#endif
#if defined(ESP8266)
#include <analogWrite.h>
#endif

class Tachometer
{
private:
    // tachometer motor pin numbers
    int _tachometer_pin;

public:
    /** Creates a tqachometer control interface
     *
     * @param tachometer_pin An interrupt-enabled pin to read the counts from the tach
     *
     */
    Tachometer(int tachometer_pin);

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

#endif // TACHOMETER_H
