#include <Arduino.h>
#include "Tachometer.h"

// Tach read period in milliseconds
#define TACH_READ_PERIOD          100
#define TACH_READ_CONST           (30000 / TACH_READ_PERIOD)


void tachometerISR(void);

int _pin_tachometer;
boolean _tachRead = false;
int _tachCount = 0;
int _tachCountPeriod = 0;

unsigned long _tachInterruptTime;
unsigned long _tachReadTime;


// Interrupt on rising edge on the tach pin.  two edges per rotation
void tachometerISR(void) {
  _tachCount++;
  _tachInterruptTime = millis();
  if(_tachRead) {
    _tachCountPeriod = _tachCount;
    _tachCount = 0;
    _tachRead = false;
  }
}


Tachometer::Tachometer(int pin)
{
  _pin_tachometer = pin;
  pinMode(_pin_tachometer, INPUT);
  attachInterrupt(digitalPinToInterrupt(_pin_tachometer), tachometerISR, RISING);

  _tachReadTime = millis();
}

void Tachometer::update(unsigned long currentTime)
{
  if (currentTime > _tachReadTime) {
    if (currentTime > (_tachInterruptTime + TACH_READ_PERIOD)) {
      // It's been a long time since a tach interupt.  Clear the count
      _tachCount = 0;
      _tachCountPeriod = 0;
    } else {
      _tachRead = true;
    }
    _tachReadTime += TACH_READ_PERIOD;
  }
}

long Tachometer::rpm(void)
{
  return (long)(_tachCountPeriod) * TACH_READ_CONST;
}
