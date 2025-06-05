/*
 * Tachometer.h - Nerftech Tachometer library - Version 1.0.0
 *
 * Original library        (1.0.0)   by James McLurkin
 *
 * Text goes here
 *
 */

#ifndef TACHOMETER_H
#define TACHOMETER_H

#include "Arduino.h"

class Tachometer
{
public:
  /** Creates a tqachometer control interface
   *
   * @param tachometer_pin An interrupt-enabled pin to read the counts from the tach
   *
   */
  Tachometer(int tachometer_pin);

  /** Returns the current rpm
   *
   *
   */
  void update(unsigned long currentTime);

  /** Returns the current rpm
   *
   *
   */
  long rpm(void);
};

#endif // TACHOMETER_H
