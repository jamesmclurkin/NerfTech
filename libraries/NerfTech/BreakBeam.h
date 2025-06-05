/*
 * Tachometer.h - Nerftech Tachometer library - Version 1.0.0
 *
 * Original library        (1.0.0)   by James McLurkin
 *
 * Text goes here
 *
 */

#ifndef BREAKBEAM_H
#define BREAKBEAM_H

#include "Arduino.h"

class BreakBeam
{
public:
  /** Creates a tqachometer control interface
   *
   * @param pin An interrupt-enabled pin to read the counts from the tach
   *
   */
  BreakBeam(int pin);

  /** Returns the current rpm
   *
   *
   */
  void update(unsigned long currentTime);

  
  /** Returns the time in microseconds of the most recent dart
   *
   *
   */
  unsigned long getLastDartTimeUS(void);
  
  /** Returns the speed of the most recent dart
   *
   *
   */
  float getLastDartSpeedFPS(void);
  
  /** Returns the speed of the most recent dart, 0.0 if no new dart
   *
   *
   */
  float getNewDartSpeedFPS(void);
};

#endif // BREAKBEAM_H
