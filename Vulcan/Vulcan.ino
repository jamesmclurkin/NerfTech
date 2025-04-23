// NerfComp control code for Stryfe Solenoid
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

// global variables for main system status
#define HEARTBEAT_UPDATE_PERIOD         10
//#define HEARTBEAT_PRINT_PERIOD          250
#define HEARTBEAT_PRINT_PERIOD          50
unsigned long heartbeatUpdateTime = 0;
unsigned long heartbeatPrintTime = 0;

// flywheel globals
#define PIN_FLYWHEEL_ESC            10
//#define PIN_FIRE_MODE               9
#define PIN_TRIGGER_FIRE            7
#define PIN_PLUNGER_PWM             5

#define FLYWHEEL_MOTOR_ESC_NEUTRAL          90
#define FLYWHEEL_MOTOR_ESC_REV              160
#define FLYWHEEL_MOTOR_ESC_REV_HOLD         110
#define FLYWHEEL_MOTOR_ESC_BRAKE            20

#define FLYWHEEL_STATE_STARTUP_DELAY        0
#define FLYWHEEL_STATE_NEUTRAL              1
#define FLYWHEEL_STATE_REV_BEFORE_AUTO      2
#define FLYWHEEL_STATE_FIRE                 3
#define FLYWHEEL_STATE_REV_HOLD             4
#define FLYWHEEL_STATE_REREV                5
#define FLYWHEEL_STATE_BRAKE                6

#define FLYWHEEL_STATE_STARTUP_DELAY_TIME   4000
#define FLYWHEEL_STATE_REV_UP_TIME          600
#define FLYWHEEL_STATE_REREV_TIME           200
#define FLYWHEEL_STATE_REV_HOLD_TIME        3000
#define FLYWHEEL_STATE_BRAKE_TIME           600

Servo servoESC;
int flywheelState = FLYWHEEL_STATE_STARTUP_DELAY;


#define PWM_PERCENT(val)                  (((val) * 255) / 100)
#define PLUNGER_PWM_OFF                   0
#define PLUNGER_PWM_RUN                   PWM_PERCENT(60) 

#define LOOP_TIME                         5

// // plunger motor model constaqnts
// #define PLUNGER_PWM_MAX                   255
// #define PLUNGER_VEL_MAX                   255
// #define PLUNGER_VEL_STEP_UP               ((int)((6 * LOOP_TIME) / 5))
// #define PLUNGER_VEL_STEP_DOWN             ((int)((3 * LOOP_TIME) / 5))
// #define PLUNGER_VEL_STOP_THRESHOLD        PWM_PERCENT(60)
// #define PLUNGER_VEL_ITERM                 ((int)((3 * LOOP_TIME) / 5))
// int plungerVel = 0;

//boolean switchTriggerRevRead() {return digitalRead(PIN_TRIGGER_REV); }
boolean switchTriggerFireRead() {return !digitalRead(PIN_TRIGGER_FIRE); }
//boolean switchPlungerEndRead() {return !digitalRead(PIN_PLUNGER_END_SW); }
void plungerPWMSet(int val) {
  analogWrite(PIN_PLUNGER_PWM, val);
}



long flywheelStateTimer = 0;
long plungerStateTimer = 0;

void setup() {
  // Setup the pins for internal sensing
  //pinMode(PIN_TRIGGER_REV, INPUT_PULLUP);
  pinMode(PIN_TRIGGER_FIRE, INPUT_PULLUP);
  //pinMode(PIN_PLUNGER_END_SW, INPUT_PULLUP);
  pinMode(PIN_PLUNGER_PWM, OUTPUT);
  plungerPWMSet(0);

  // init the servo
  servoESC.attach(PIN_FLYWHEEL_ESC); // attaches the servo on pin 9 to the servo object
  servoESC.write(FLYWHEEL_MOTOR_ESC_NEUTRAL);  
  

  // init the serial port for debugging output
  Serial.begin(115200);
  Serial.println(F(" NerfComp: Vulcan Afterburner ver 0.1"));
  //Serial.print(F("   Free RAM:")); Serial.print(freeRam()); Serial.println(F(" bytes"));

  // reset the heartbeat time to avoid a bunch of initial updates
  heartbeatUpdateTime = millis();
  heartbeatPrintTime = heartbeatUpdateTime;
  flywheelStateTimer = heartbeatUpdateTime + FLYWHEEL_STATE_STARTUP_DELAY_TIME;
}


//////// Loop ////////
boolean sp = true;
boolean triggerFireOld = false;
boolean triggerRevOld = false;
boolean plungerEndSwitchOld = false;
int shotCounter = 0;
int stopSlowITermPWM = 0;

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
    //p = false;
    //if (p) {Serial.print("hb ");}

    // print diagnostic switch status on serial port
    //if (p) {Serial.print(" trigRev="); Serial.print(switchTriggerRevRead());}
    //if (p) {Serial.print(" trigFire="); Serial.print(switchTriggerFireRead());}
    //if (p) {Serial.print(F(" jam=")); Serial.print(jamDoorRead());}
    //if (p) {Serial.println("");}
  }

  // the Vulcan is a fully-auto blaster.
  // when the fire trigger is pulled, rev the flywheel, wait a few seconds
  // then start the plunger (belt) motor
  int ESCPos = FLYWHEEL_MOTOR_ESC_NEUTRAL;
  int plungerPWM = PLUNGER_PWM_OFF;

  boolean triggerFire = switchTriggerFireRead();
  boolean triggerFireEdge = (triggerFire && !triggerFireOld);


  switch (flywheelState) {
    case FLYWHEEL_STATE_STARTUP_DELAY: {
      ESCPos = FLYWHEEL_MOTOR_ESC_NEUTRAL;      
      plungerPWM = PLUNGER_PWM_OFF;
      if (currentTime > flywheelStateTimer) {
        flywheelState = FLYWHEEL_STATE_NEUTRAL;
        Serial.println("startup->neutral");
      }
      break;
    }
    case FLYWHEEL_STATE_NEUTRAL: {
      ESCPos = FLYWHEEL_MOTOR_ESC_NEUTRAL;
      plungerPWM = PLUNGER_PWM_OFF;
      if (triggerFire) {
        flywheelState = FLYWHEEL_STATE_REV_BEFORE_AUTO;
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_REV_UP_TIME;
        Serial.println("neutral->rev");
      }
      break;
    }
    case FLYWHEEL_STATE_REV_BEFORE_AUTO: {
      ESCPos = FLYWHEEL_MOTOR_ESC_REV;
      plungerPWM = PLUNGER_PWM_OFF;
      if (!triggerFire) {
        flywheelState = FLYWHEEL_STATE_REV_HOLD;
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_REV_HOLD_TIME;
        Serial.println("rev->revhold");
      }
      if (currentTime > flywheelStateTimer) {
        flywheelState = FLYWHEEL_STATE_FIRE;
        Serial.println("rev->fire");
      }
      break;
    }
    case FLYWHEEL_STATE_FIRE: {
      ESCPos = FLYWHEEL_MOTOR_ESC_REV;
      plungerPWM = PLUNGER_PWM_RUN;
      if (!triggerFire) {
        flywheelState = FLYWHEEL_STATE_REV_HOLD;
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_REV_HOLD_TIME;
        Serial.println("fire->revhold");
      }
      break;
    }
    case FLYWHEEL_STATE_REV_HOLD: {
      ESCPos = FLYWHEEL_MOTOR_ESC_REV_HOLD;
      plungerPWM = PLUNGER_PWM_OFF;
      if (triggerFire) {
        flywheelState = FLYWHEEL_STATE_REREV;
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_REREV_TIME;
        Serial.println("revhold->rerev");
      }
      if (currentTime > flywheelStateTimer) {
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_BRAKE_TIME;
        flywheelState = FLYWHEEL_STATE_BRAKE;
        Serial.println("revhold->brake");
      }
      break;
    }
    case FLYWHEEL_STATE_REREV: {
      ESCPos = FLYWHEEL_MOTOR_ESC_REV;
      plungerPWM = PLUNGER_PWM_OFF;
      if (!triggerFire) {
        flywheelState = FLYWHEEL_STATE_REV_HOLD;
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_REV_HOLD_TIME;
        Serial.println("rerev->revhold");
      }
      if (currentTime > flywheelStateTimer) {
        flywheelState = FLYWHEEL_STATE_FIRE;
        Serial.println("rerev->fire");
      }
      break;
    }
    case FLYWHEEL_STATE_BRAKE: {
      ESCPos = FLYWHEEL_MOTOR_ESC_BRAKE;      
      plungerPWM = PLUNGER_PWM_OFF;
      if (triggerFire) {
        flywheelState = FLYWHEEL_STATE_REV_BEFORE_AUTO;
        Serial.println("brake->rev");
      }
      if (currentTime > flywheelStateTimer) {
        flywheelState = FLYWHEEL_STATE_NEUTRAL;
        Serial.println("brake->neutral");
        Serial.println("");
      }
      break;
    }
    default: {
      flywheelState = FLYWHEEL_STATE_NEUTRAL;
      ESCPos = FLYWHEEL_MOTOR_ESC_NEUTRAL;
      plungerPWM = PLUNGER_PWM_OFF;
      break;
    }
  }
  servoESC.write(ESCPos);
  plungerPWMSet(plungerPWM);

  triggerFireOld = triggerFire;
  delay(LOOP_TIME);
}
