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
#define PIN_FLYWHEEL_ESC            5
#define PIN_TRIGGER_REV             9
#define PIN_TRIGGER_FIRE            7
#define PIN_PLUNGER_PWM             10
#define PIN_PLUNGER_END_SW          3

#define FLYWHEEL_MOTOR_ESC_NEUTRAL      90
#define FLYWHEEL_MOTOR_ESC_PRE_RUN_FULL 96
#define FLYWHEEL_MOTOR_ESC_REV          160
#define FLYWHEEL_MOTOR_ESC_REV_HOLD     110
#define FLYWHEEL_MOTOR_ESC_BRAKE        20

#define FLYWHEEL_STATE_STARTUP_DELAY    0
#define FLYWHEEL_STATE_NEUTRAL          1
#define FLYWHEEL_STATE_REV              2
#define FLYWHEEL_STATE_REV_HOLD         3
#define FLYWHEEL_STATE_BRAKE            4

#define FLYWHEEL_STATE_STARTUP_DELAY_TIME 3000
#define FLYWHEEL_STATE_REV_HOLD_TIME      1000
#define FLYWHEEL_STATE_BRAKE_TIME         2000

Servo servoESC;
int flywheelState = FLYWHEEL_STATE_NEUTRAL;



// plunger globals
#define PLUNGER_STATE_STARTUP_DELAY       0
#define PLUNGER_STATE_NEUTRAL             1
#define PLUNGER_STATE_RUN_AUTO                 2
#define PLUNGER_STATE_BEGIN_STOP          3
#define PLUNGER_STATE_RUN_TIMED           4
#define PLUNGER_STATE_RUN_SLOW            5
#define PLUNGER_STATE_RUN_TO_STOP         6
#define PLUNGER_STATE_STOP_SLOW           7
#define PLUNGER_STATE_STOP_FAST           8
#define PLUNGER_STATE_RUN_ONE             9
#define PLUNGER_STATE_STOP                10

int plungerState = PLUNGER_STATE_NEUTRAL;

#define PLUNGER_STATE_STARTUP_DELAY_TIME  FLYWHEEL_STATE_STARTUP_DELAY_TIME
#define PLUNGER_STATE_RUN_TIME_MAX        1000
#define PLUNGER_STATE_OFF_TIME            100

#define PLUNGER_PWM_BRAKE                 0
#define PLUNGER_PWM_OFF                   PLUNGER_PWM_BRAKE
#define PLUNGER_PWM_RUN_TIME              100
#define PLUNGER_PWM_RUN_TIME_AUTO         100

#define PWM_PERCENT(val)                  (((val) * 255) / 100)
#define PLUNGER_PWM_RUN_PERCENTAGE        90
#define PLUNGER_PWM_RUN                   PWM_PERCENT(PLUNGER_PWM_RUN_PERCENTAGE)
#define PLUNGER_PWM_RUN_SLOW_PERCENTAGE   40
#define PLUNGER_PWM_RUN_SLOW              PWM_PERCENT(PLUNGER_PWM_RUN_SLOW_PERCENTAGE)

#define SHOTCOUNTER_MAX                   2


#define LOOP_TIME                         5

#define PLUNGER_PWM_MAX                   255
#define PLUNGER_VEL_MAX                   255
#define PLUNGER_VEL_STEP_UP               ((int)((6 * LOOP_TIME) / 5))
#define PLUNGER_VEL_STEP_DOWN             ((int)((3 * LOOP_TIME) / 5))
#define PLUNGER_VEL_STOP_THRESHOLD        PWM_PERCENT(60)
#define PLUNGER_VEL_ITERM                 ((int)((3 * LOOP_TIME) / 5))
int plungerVel = 0;

boolean switchTriggerRevRead() {return digitalRead(PIN_TRIGGER_REV); }
boolean switchTriggerFireRead() {return !digitalRead(PIN_TRIGGER_FIRE); }
boolean switchPlungerEndRead() {return !digitalRead(PIN_PLUNGER_END_SW); }
void plungerPWMSet(int val) {
  analogWrite(PIN_PLUNGER_PWM, val);
}



long flywheelStateTimer = 0;
long plungerStateTimer = 0;

void setup() {
  // Setup the pins for internal sensing
  pinMode(PIN_TRIGGER_REV, INPUT_PULLUP);
  pinMode(PIN_TRIGGER_FIRE, INPUT_PULLUP);
  pinMode(PIN_PLUNGER_END_SW, INPUT_PULLUP);
  pinMode(PIN_PLUNGER_PWM, OUTPUT);
  plungerPWMSet(0);

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
  flywheelStateTimer = heartbeatUpdateTime + FLYWHEEL_STATE_STARTUP_DELAY_TIME;
  plungerStateTimer = heartbeatUpdateTime + PLUNGER_STATE_STARTUP_DELAY_TIME;
    //delay(4000);
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

  // the Stryfe solenoid is a fully-auto blaster.
  // when the rev trigger is pulled, rev the flywheel.
  // brake for a while when the rev is off, then switch to neutral
  int ESCPos;
  boolean triggerRev = switchTriggerRevRead();
  boolean triggerRevEdge = (triggerRev && !triggerRevOld);

  // For testing plunger FSM
  //triggerRev = false;

  switch (flywheelState) {
    case FLYWHEEL_STATE_STARTUP_DELAY: {
      if (currentTime > flywheelStateTimer) {
        flywheelState = FLYWHEEL_STATE_NEUTRAL;
        Serial.println("startup->neutral");
      }
      ESCPos = FLYWHEEL_MOTOR_ESC_NEUTRAL;      
      break;
    }
    case FLYWHEEL_STATE_NEUTRAL: {
      if (triggerRev) {
        flywheelState = FLYWHEEL_STATE_REV;
        Serial.println("neutral->rev");
      }
      ESCPos = FLYWHEEL_MOTOR_ESC_NEUTRAL;
      break;
    }
    case FLYWHEEL_STATE_REV: {
      if (!triggerRev) {
        flywheelState = FLYWHEEL_STATE_REV_HOLD;
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_REV_HOLD_TIME;
        Serial.println("rev->revhold");
      }
      ESCPos = FLYWHEEL_MOTOR_ESC_REV;
      break;
    }
    case FLYWHEEL_STATE_REV_HOLD: {
      if (triggerRev) {
        flywheelState = FLYWHEEL_STATE_REV;
        Serial.println("revhold->rev");
      }
      if (currentTime > flywheelStateTimer) {
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_BRAKE_TIME;
        flywheelState = FLYWHEEL_STATE_BRAKE;
        Serial.println("revhold->brake");
      }
      ESCPos = FLYWHEEL_MOTOR_ESC_REV_HOLD;
      break;
    }
    case FLYWHEEL_STATE_BRAKE: {
      if (triggerRev) {
        flywheelState = FLYWHEEL_STATE_REV;
        Serial.println("brake->rev");
      }
      if (currentTime > flywheelStateTimer) {
        flywheelState = FLYWHEEL_STATE_NEUTRAL;
        Serial.println("brake->neutral");
      }
      ESCPos = FLYWHEEL_MOTOR_ESC_BRAKE;      
      break;
    }
    default: {
      flywheelState = FLYWHEEL_STATE_NEUTRAL;
      ESCPos = FLYWHEEL_MOTOR_ESC_NEUTRAL;
      break;
    }
  }
  servoESC.write(ESCPos);


  int plungerPWM = PLUNGER_PWM_OFF;

  boolean triggerFire = switchTriggerFireRead();
  boolean triggerFireEdge = (triggerFire && !triggerFireOld);

  boolean plungerEndSwitch = switchPlungerEndRead();
  boolean plungerEndSwitchEdge = (plungerEndSwitch && !plungerEndSwitchOld);

 
  // if (triggerFireEdge) {
  //   if (shotCounter < SHOTCOUNTER_MAX) {
  //     shotCounter++;
  //   }
  // }

  switch (plungerState) {
    case PLUNGER_STATE_STARTUP_DELAY: {
      if (currentTime > plungerStateTimer) {
        plungerState = PLUNGER_STATE_NEUTRAL;
        Serial.println("p:startup->neutral");
      }
      plungerPWM = PLUNGER_PWM_OFF;
      break;
    }
    case PLUNGER_STATE_NEUTRAL: {
      plungerPWM = PLUNGER_PWM_OFF;
      if (triggerFire) {
        plungerStateTimer = currentTime + PLUNGER_PWM_RUN_TIME;
        plungerState = PLUNGER_STATE_RUN_AUTO;
        Serial.println("p:neutral->run");
      }
      break;
    }
    case PLUNGER_STATE_RUN_AUTO: {
      plungerPWM = PLUNGER_PWM_RUN;
      if ((currentTime > plungerStateTimer) && (!triggerFire)) {
        plungerPWM = PLUNGER_PWM_RUN_SLOW;
      }

      if(plungerEndSwitchEdge) {
        // the plunger just snapped back.
        if (triggerFire) {
          plungerStateTimer = currentTime + PLUNGER_PWM_RUN_TIME_AUTO;
          plungerState = PLUNGER_STATE_RUN_AUTO;
          Serial.print("p:run->runAuto           vel:");
          Serial.println(plungerVel, DEC);
        } else {
          // Check if the velocity is slow enough to stop
          if (plungerVel <= PLUNGER_VEL_STOP_THRESHOLD) {
            plungerState = PLUNGER_STATE_STOP;
            Serial.print("p:run->stop              vel:");
            Serial.println(plungerVel, DEC);
          } else {
            plungerState = PLUNGER_STATE_STOP_FAST;
            Serial.print("p:run->stopFast          vel:");
            Serial.println(plungerVel, DEC);
          }
        }
      }
      break;
    }
    case PLUNGER_STATE_STOP_FAST: {
      plungerPWM = PLUNGER_PWM_RUN_SLOW;
      if (!plungerEndSwitch) {
        plungerState = PLUNGER_STATE_STOP_SLOW;
        Serial.print("p:stopFast->stopSlow     vel:");
        Serial.println(plungerVel, DEC);
      }
      break;
    }
    case PLUNGER_STATE_STOP_SLOW: {
      plungerPWM = stopSlowITermPWM;
      stopSlowITermPWM += PLUNGER_VEL_ITERM;
      stopSlowITermPWM = constrain(stopSlowITermPWM, 0, 255);
      if(plungerEndSwitchEdge) {
        plungerState = PLUNGER_STATE_STOP;
        Serial.print("p:stopSlow->stop         vel:");
        Serial.println(plungerVel, DEC);
      }
      break;
    }
    case PLUNGER_STATE_STOP: {
      plungerPWM = 0;
      stopSlowITermPWM = 0;
      if(plungerVel == 0) {
        plungerState = PLUNGER_STATE_NEUTRAL;
        Serial.print("p:stop->neutral          vel:");
        Serial.println(plungerVel, DEC);
        Serial.println("");
      }
      break;
    }
    default: {
      plungerPWM = PLUNGER_PWM_OFF;
      plungerState = PLUNGER_STATE_NEUTRAL;
      break;
    }
  }

  plungerPWMSet(plungerPWM);
  if (plungerPWM > plungerVel) {
    if(PLUNGER_VEL_STEP_UP > 0) {
      plungerVel+= PLUNGER_VEL_STEP_UP;
    } else {
      plungerVel++;
    }
  } else if (plungerPWM < plungerVel) {
    if(PLUNGER_VEL_STEP_DOWN > 0) {
      plungerVel-= PLUNGER_VEL_STEP_DOWN;
    } else {
      plungerVel--;
    }
  }
  plungerVel = constrain(plungerVel, 0, 255);
  // if(p) {
  //   Serial.print("PWM:");
  //   Serial.print(plungerPWM, DEC);
  //   Serial.print(" vel:");
  //   Serial.println(plungerVel, DEC);
  // }

  triggerFireOld = triggerFire;
  plungerEndSwitchOld = plungerEndSwitch;
  delay(LOOP_TIME);
}
