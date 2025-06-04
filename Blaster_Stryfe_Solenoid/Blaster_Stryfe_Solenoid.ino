// NerfComp control code for Stryfe Solenoid
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// global variables for main system status
#define HEARTBEAT_UPDATE_PERIOD         50
#define HEARTBEAT_PRINT_PERIOD          250
unsigned long heartbeatUpdateTime = 0;
unsigned long heartbeatPrintTime = 0;

Servo servoESC;

#define PIN_FLYWHEEL_ESC            10
#define PIN_TRIGGER_REV             9
#define PIN_TRIGGER_FIRE            7
#define PIN_SOLENOID                5

#define FLYWHEEL_MOTOR_ESC_NEUTRAL      90
#define FLYWHEEL_MOTOR_ESC_PRE_RUN_FULL 96
#define FLYWHEEL_MOTOR_ESC_REVUP        170
#define FLYWHEEL_MOTOR_ESC_RUN          150
#define FLYWHEEL_MOTOR_ESC_BRAKE        20

#define FLYWHEEL_STATE_NEUTRAL          0
#define FLYWHEEL_STATE_REV              1
#define FLYWHEEL_STATE_BRAKE            2
#define FLYWHEEL_STATE_BRAKE_TIME       2000

#define FIRE_REV_HOLD_DELAY             1000

int flywheelState = FLYWHEEL_STATE_NEUTRAL;
long flywheelStateTimer = 0;

#define PLUNGER_STATE_NEUTRAL           0
#define PLUNGER_STATE_ON                1
#define PLUNGER_STATE_DELAY             2

#define PLUNGER_STATE_ON_TIME           80
#define PLUNGER_STATE_OFF_TIME          150

int plungerState = PLUNGER_STATE_NEUTRAL;
long plungerStateTimer = 0;


boolean switchTriggerRevRead() {return digitalRead(PIN_TRIGGER_REV); }
boolean switchTriggerFireRead() {return !digitalRead(PIN_TRIGGER_FIRE); }
void plungerPWMSet(int val) {digitalWrite(PIN_SOLENOID, val); }



void setup() {
  // Setup the pins for internal sensing
  pinMode(PIN_TRIGGER_REV, INPUT_PULLUP);
  pinMode(PIN_TRIGGER_FIRE, INPUT_PULLUP);
  pinMode(PIN_SOLENOID, OUTPUT);
  plungerPWMSet(LOW);

  // init the servo
  servoESC.attach(PIN_FLYWHEEL_ESC); // attaches the servo on pin 9 to the servo object
  servoESC.write(FLYWHEEL_MOTOR_ESC_NEUTRAL);  
  

  // init the serial port for debugging output
  Serial.begin(115200);
  Serial.println(F(" NerfComp: Stryfe ver 0.1"));
  //Serial.print(F("   Free RAM:")); Serial.print(freeRam()); Serial.println(F(" bytes"));

  // reset the heartbeat time to avoid a bunch of initial updates
  heartbeatUpdateTime = millis();
  heartbeatPrintTime = heartbeatUpdateTime;
  //delay(4000);
}


//////// Loop ////////
boolean sp = true;
boolean triggerFireOld = false;
long fireTimeLast = 0;

void loop() {
  long currentTime = millis();

  // run the heartbeat routines
  boolean p = false;
  if (currentTime > heartbeatUpdateTime) {
    heartbeatUpdateTime += HEARTBEAT_UPDATE_PERIOD;
    if (currentTime > heartbeatPrintTime) {
      heartbeatPrintTime += HEARTBEAT_PRINT_PERIOD;
      p = true;
    }
    p = false;
    if (p) {Serial.print("hb ");}

    // print diagnostic switch status on serial port
    if (p) {Serial.print(" trigRev="); Serial.print(switchTriggerRevRead());}
    if (p) {Serial.print(" trigFire="); Serial.print(switchTriggerFireRead());}
    //if (p) {Serial.print(F(" jam=")); Serial.print(jamDoorRead());}
    if (p) {Serial.println("");}
  }

  // the Stryfe solenoid is a fully-auto blaster.
  // when the rev trigger is pulled, rev the flywheel.
  // brake for a while when the rev is off, then switch to neutral
  int ESCPos;
  switch (flywheelState) {
    case FLYWHEEL_STATE_REV: {
      if (!switchTriggerRevRead() && (currentTime >(fireTimeLast + FIRE_REV_HOLD_DELAY))) {
        flywheelState = FLYWHEEL_STATE_BRAKE;
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_BRAKE_TIME;
        Serial.println("brake");
      }
      ESCPos = FLYWHEEL_MOTOR_ESC_RUN;
      break;
    }
    case FLYWHEEL_STATE_BRAKE: {
      if (switchTriggerRevRead()) {
        flywheelState = FLYWHEEL_STATE_REV;
        Serial.println("rev");
      }
      if (currentTime > flywheelStateTimer) {
        flywheelState = FLYWHEEL_STATE_NEUTRAL;
        Serial.println("neutral");
      }
      ESCPos = FLYWHEEL_MOTOR_ESC_BRAKE;      
      break;
    }
    default:
    case FLYWHEEL_STATE_NEUTRAL: {
      if (switchTriggerRevRead()) {
        flywheelState = FLYWHEEL_STATE_REV;
        Serial.println("rev");
      }
      ESCPos = FLYWHEEL_MOTOR_ESC_NEUTRAL;
      break;
    }
  }
  servoESC.write(ESCPos);


  int solenoidPos = LOW;
  boolean triggerCurrent = switchTriggerFireRead();
  boolean triggerEdge = (triggerCurrent && !triggerFireOld);

  switch (plungerState) {
    case PLUNGER_STATE_RUN: {
      if (currentTime > plungerStateTimer) {
        plungerState = PLUNGER_STATE_DELAY;
        plungerStateTimer = currentTime + PLUNGER_STATE_OFF_TIME;
        solenoidPos = LOW;
        Serial.println("p-delay");
      } else{
        solenoidPos = HIGH;
      }
      break;
    }
    case PLUNGER_STATE_DELAY: {
      if (currentTime > plungerStateTimer) {
        if (triggerCurrent) {
          plungerState = PLUNGER_STATE_RUN;
          plungerStateTimer = currentTime + PLUNGER_STATE_ON_TIME;
          fireTimeLast = currentTime;
          Serial.println("p-auto");
          solenoidPos = HIGH;
        } else {        
          plungerState = PLUNGER_STATE_NEUTRAL;
          Serial.println("p-neutral");
          solenoidPos = LOW;
        }
      }
      break;
    }
    default:
    case PLUNGER_STATE_NEUTRAL: {
      if (triggerEdge) {
        plungerState = PLUNGER_STATE_RUN;
        plungerStateTimer = currentTime + PLUNGER_STATE_ON_TIME;
        fireTimeLast = currentTime;
        Serial.println("p-on");
        solenoidPos = HIGH;
      } else {
        solenoidPos = LOW;
      }
      break;
    }
  }
  plungerPWMSet(solenoidPos);
  triggerFireOld = triggerCurrent;

  delay(2);
}
