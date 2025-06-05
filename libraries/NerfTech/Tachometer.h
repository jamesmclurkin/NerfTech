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
#if defined(ESP32)
#include <analogWrite.h>
#endif
#if defined(ESP8266)
#include <analogWrite.h>
#endif

class Tachometer
{
private:
    // The library version number
    int _version = 2;
    
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

  /** Returns the version of the library
   *
   *
   */
  int version(void);
};

#endif // TACHOMETER_H
