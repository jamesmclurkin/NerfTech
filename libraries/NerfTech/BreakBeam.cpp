#include <Arduino.h>
#include "BreakBeam.h"

// breakbeam
#define BREAKBEAM_IDLE                0
#define BREAKBEAM_MEASURING           1

// max time in micro seconds
#define BREAKBEAM_TIME_MAX            10000
#define DART_LENGTH_MM                72.0
#define SENSOR_POS_MM                 10.0
#define MICRO_SECONDS_TO_SECONDS      1000000.0
#define MM_TO_FOOT    304.8

void breakbeamISR(void);

int _pin_breakbeam;

//unsigned long breakbeamReadTime;
boolean breakbeamState = BREAKBEAM_IDLE;
unsigned long breakbeamStartTime;
unsigned long breakbeamEndTime;
unsigned long breakbeamLastDartTime = 0;
boolean breakbeamNewDart = false;

// Interrupt on rising edge on the breakbeam pin.  Then falling edge.
void breakbeamEdgeMeasure (void) {
  attachInterrupt(digitalPinToInterrupt(_pin_breakbeam), breakbeamISR, RISING);
  breakbeamState = BREAKBEAM_MEASURING;
}

void breakbeamEdgeReset (void) {
  attachInterrupt(digitalPinToInterrupt(_pin_breakbeam), breakbeamISR, FALLING);
  breakbeamState = BREAKBEAM_IDLE;
}

void breakbeamISR(void) {
  if (breakbeamState == BREAKBEAM_IDLE) {
    breakbeamStartTime = micros();
    breakbeamEdgeMeasure();
  } else {
    // breakbeamState == BREAKBEAM_MEASURING:
    breakbeamEndTime = micros();
    breakbeamEdgeReset();
    breakbeamLastDartTime = breakbeamEndTime - breakbeamStartTime;
    //TODO: check for microsecond overflow here
    breakbeamNewDart = true;
  }
}

BreakBeam::BreakBeam(int pin)
{
  // init the dart breakbeam sensor
  _pin_breakbeam = pin;
  pinMode(_pin_breakbeam, INPUT);
  breakbeamEdgeReset();
}

void BreakBeam::update(unsigned long currentTime)
{
  // reset the breakbeam state after BREAKBEAM_TIME_MAX
  if (breakbeamState == BREAKBEAM_MEASURING) {
    unsigned long timeCurrent = micros();
    if(timeCurrent > (breakbeamStartTime + BREAKBEAM_TIME_MAX)) {
      breakbeamEdgeReset();
    }
  }
}

unsigned long BreakBeam::getLastDartTimeUS(void)
{
  return breakbeamLastDartTime;
}

float BreakBeam::getLastDartSpeedFPS(void)
{
  float speedMMS = DART_LENGTH_MM * MICRO_SECONDS_TO_SECONDS / (float)breakbeamLastDartTime;
  float speedFPS = speedMMS / MM_TO_FOOT;
  return speedFPS;
}

float BreakBeam::getNewDartSpeedFPS(void)
{
  if(breakbeamNewDart) {
    return this->getLastDartSpeedFPS();
  } else {
    return 0.0;
  }
}
