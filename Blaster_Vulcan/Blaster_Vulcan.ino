// NerfComp control code for Stryfe Solenoid
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_DotStar.h>
#include <NerfTech.h>

// global variables for main system status
#define HEARTBEAT_UPDATE_PERIOD             10
#define HEARTBEAT_PRINT_PERIOD              250
#define LED_UPDATE_PERIOD                   20

unsigned long heartbeatUpdateTime = 0;
unsigned long heartbeatPrintTime = 0;
unsigned long LEDUpdateTime = 0;

// pin assignments
#define PIN_TRIGGER_MODE_SW                 9
#define PIN_TRIGGER_SW                      7

#define PIN_PUSHER_PWM                      12
#define PIN_PUSHER_BACK_SW                  10

#define PIN_FLYWHEEL_ESC                    5
#define PIN_BREAKBEAM                       1
#define PIN_FLYWHEEL_TACHOMETER             0

#define PIN_SAFETY_JAMDOOR_SW               3

#define PIN_MOTOR_TEMP                      (A2)

// flywheel motor constants
#define FLYWHEEL_MOTOR_SPEED_NEUTRAL        0
#define FLYWHEEL_MOTOR_SPEED_REV            100
#define FLYWHEEL_MOTOR_SPEED_REV_HOLD       75
#define FLYWHEEL_MOTOR_STRENGTH_BRAKE       100

#define FLYWHEEL_STATE_STARTUP_DELAY        0
#define FLYWHEEL_STATE_NEUTRAL              1
#define FLYWHEEL_STATE_REV_BEFORE_AUTO      2
#define FLYWHEEL_STATE_FIRE                 3
#define FLYWHEEL_STATE_REV_HOLD             4
#define FLYWHEEL_STATE_REREV                5
#define FLYWHEEL_STATE_BRAKE                6

#define FLYWHEEL_STATE_STARTUP_DELAY_TIME   4000
#define FLYWHEEL_STATE_REV_UP_TIME          800
#define FLYWHEEL_STATE_REREV_TIME           200
#define FLYWHEEL_STATE_REV_HOLD_TIME        1000
#define FLYWHEEL_STATE_BRAKE_TIME           600

int flywheelState = FLYWHEEL_STATE_STARTUP_DELAY;

#define PUSHER_MIN_RPM                       25000

#define PUSHER_SPEED_OFF                     0.0
#define PUSHER_SPEED_RUN                     50.0

#define LOOP_TIME                           5


//boolean switchTriggerRevRead() {return digitalRead(PIN_TRIGGER_REV); }
boolean switchTriggerFireRead() {return !digitalRead(PIN_TRIGGER_SW); }
//boolean switchPlungerEndRead() {return !digitalRead(PIN_PLUNGER_END_SW); }


// Dot Star
// Everything is defined in the Board Support Package
// DOTSTAR_NUM        number of onboard DotStars (typically just 1)
// PIN_DOTSTAR_DATA   onboard DotStar data pin
// PIN_DOTSTAR_CLK    onboard DotStar clock pin
Adafruit_DotStar dotStar(DOTSTAR_NUM, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLK, DOTSTAR_BRG);

Tachometer tach(PIN_FLYWHEEL_TACHOMETER);

BreakBeam breakbeam(PIN_BREAKBEAM);

MotorDriver flywheelMotor(MOTOR_TYPE_BRUSHED_ESC, PIN_FLYWHEEL_ESC);

MotorDriver pusherMotor(MOTOR_TYPE_LOWSIDE_DRIVER, PIN_PUSHER_PWM);


// custom code for hte Vulcan blaster
long flywheelStateTimer = 0;
long plungerStateTimer = 0;


void setup() {
  // Setup the pins for internal sensing
  //pinMode(PIN_TRIGGER_REV, INPUT_PULLUP);
  pinMode(PIN_TRIGGER_SW, INPUT_PULLUP);
  //pinMode(PIN_PLUNGER_END_SW, INPUT_PULLUP);

  
  // initialize the blinky LED as an output:
  pinMode(PIN_LED, OUTPUT);

  dotStar.begin(); // Initialize pins for output
  dotStar.setBrightness(80);
  dotStar.show();  // Turn all LEDs off ASAP

  // init the serial port for debugging output
  Serial.begin(115200);
  Serial.println(F(" NerfComp: Vulcan Afterburner ver 0.1"));
  //Serial.print(F("   Free RAM:")); Serial.print(freeRam()); Serial.println(F(" bytes"));

  // reset the heartbeat time to avoid a bunch of initial updates
  heartbeatUpdateTime = millis();
  heartbeatPrintTime = heartbeatUpdateTime;
  flywheelStateTimer = heartbeatUpdateTime + FLYWHEEL_STATE_STARTUP_DELAY_TIME;
}

// #ff8000
// #ff0000
#define COLOR_RED           0xFF
#define COLOR_GREEN         0x00
#define COLOR_BLUE          0x00

#define RGB_BRIGHT_MULT   1.1
#define RGB_BRIGHT_MAX    0.4
#define RGB_BRIGHT_MIN    0.05
float dotStarBrightness = RGB_BRIGHT_MIN;
boolean dotStarBrightnessAscending = true;


//////// Loop ////////
boolean sp = true;
boolean triggerFireOld = false;
boolean triggerRevOld = false;
boolean plungerEndSwitchOld = false;
int shotCounter = 0;
int stopSlowITermPWM = 0;

void loop() {
  long currentTime = millis();

  tach.update(currentTime);
  breakbeam.update(currentTime);


  // print dart stats if there is a new dart
  float dartSpeed = breakbeam.getNewDartSpeedFPS();
  if(dartSpeed > 0.0) {
    Serial.print("  dart:");
    Serial.print(breakbeam.getLastDartTimeUS(), DEC);
    Serial.print("us, ");
    Serial.print(dartSpeed, 1);
    Serial.print(" fps");
    Serial.println("");
  }


  // run the heartbeat routines
  boolean p = false;
  if (currentTime > heartbeatUpdateTime) {
    heartbeatUpdateTime += HEARTBEAT_UPDATE_PERIOD;
    if (currentTime > heartbeatPrintTime) {
      heartbeatPrintTime += HEARTBEAT_PRINT_PERIOD;
      p = true;
    }
    //p = false;
    if (p) {
      //Serial.print("hb ");
      //Serial.print("tach counts:");
      //Serial.print(tachCountPeriod, DEC);
      Serial.print(" RPM:");
      Serial.print(tach.rpm(), DEC);

      Serial.println("");
    }

    // print diagnostic switch status on serial port
    //if (p) {Serial.print(" trigRev="); Serial.print(switchTriggerRevRead());}
    //if (p) {Serial.print(" trigFire="); Serial.print(switchTriggerFireRead());}
    //if (p) {Serial.print(F(" jam=")); Serial.print(jamDoorRead());}
    //if (p) {Serial.println("");}
  }

  // the Vulcan is a fully-auto blaster.
  // when the fire trigger is pulled, rev the flywheel, wait a few seconds
  // then start the plunger (belt) motor

  boolean triggerFire = switchTriggerFireRead();
  boolean triggerFireEdge = (triggerFire && !triggerFireOld);

    // update LED
  if (currentTime > LEDUpdateTime) {
    if (triggerFire) {
      //dotStar.setPixelColor(0, 0x800000);
      if (tach.rpm() > PUSHER_MIN_RPM) {
        // #00FF00
        dotStar.setPixelColor(0, 0x00FF00);    
      } else {
        // #FFFF00
        dotStar.setPixelColor(0, 0xFFFF00);    
      }
    } else {
      if (tach.rpm() > PUSHER_MIN_RPM) {
        // #008000
        dotStar.setPixelColor(0, 0x004000);    
      } else {
        float dsRed = (float)COLOR_RED * dotStarBrightness;
        float dsGreen = (float)COLOR_GREEN * dotStarBrightness;
        float dsBlue = (float)COLOR_BLUE * dotStarBrightness;
        //dotStar.setPixelColor(0, (unsigned char)dsGreen, (unsigned char)dsRed, (unsigned char)dsBlue);
        dotStar.setPixelColor(0, (unsigned char)dsRed, (unsigned char)dsGreen, (unsigned char)dsBlue);
        //dotStar.setPixelColor(0, 0, (unsigned char)dsRed, 0);
        if(dotStarBrightnessAscending) {
          dotStarBrightness *= RGB_BRIGHT_MULT;
          if(dotStarBrightness > RGB_BRIGHT_MAX) {
            dotStarBrightness = RGB_BRIGHT_MAX;
            dotStarBrightnessAscending = false;
          }
        } else {
          dotStarBrightness /= RGB_BRIGHT_MULT;
          if(dotStarBrightness < RGB_BRIGHT_MIN) {
            dotStarBrightness = RGB_BRIGHT_MIN;
            dotStarBrightnessAscending = true;
          }
        }
      }
    }
    dotStar.show();
    LEDUpdateTime += LED_UPDATE_PERIOD;
  }


  switch (flywheelState) {
    case FLYWHEEL_STATE_STARTUP_DELAY: {
      flywheelMotor.coast();
      pusherMotor.coast();
      if (currentTime > flywheelStateTimer) {
        flywheelState = FLYWHEEL_STATE_NEUTRAL;
        Serial.println("startup->neutral");
      }
      break;
    }
    case FLYWHEEL_STATE_NEUTRAL: {
      flywheelMotor.coast();
      pusherMotor.coast();
      if (triggerFire) {
        flywheelState = FLYWHEEL_STATE_REV_BEFORE_AUTO;
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_REV_UP_TIME;
        Serial.println("neutral->rev");
      }
      break;
    }
    case FLYWHEEL_STATE_REV_BEFORE_AUTO: {
      flywheelMotor.forward(FLYWHEEL_MOTOR_SPEED_REV);
      pusherMotor.coast();
      if (!triggerFire) {
        flywheelState = FLYWHEEL_STATE_REV_HOLD;
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_REV_HOLD_TIME;
        Serial.println("rev->revhold");
      }
      //if (currentTime > flywheelStateTimer) {
      if (tach.rpm() > PUSHER_MIN_RPM) {
        flywheelState = FLYWHEEL_STATE_FIRE;
        Serial.println("rev->fire");
      }
      break;
    }
    case FLYWHEEL_STATE_FIRE: {
      flywheelMotor.forward(FLYWHEEL_MOTOR_SPEED_REV);
      pusherMotor.forward(PUSHER_SPEED_RUN);
      if (!triggerFire) {
        flywheelState = FLYWHEEL_STATE_REV_HOLD;
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_REV_HOLD_TIME;
        Serial.println("fire->revhold");
      }
      break;
    }
    case FLYWHEEL_STATE_REV_HOLD: {
      flywheelMotor.forward(FLYWHEEL_MOTOR_SPEED_REV_HOLD);
      pusherMotor.coast();
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
      flywheelMotor.forward(FLYWHEEL_MOTOR_SPEED_REV);
      pusherMotor.coast();
      if (!triggerFire) {
        flywheelState = FLYWHEEL_STATE_REV_HOLD;
        flywheelStateTimer = currentTime + FLYWHEEL_STATE_REV_HOLD_TIME;
        Serial.println("rerev->revhold");
      }
      if (tach.rpm() > PUSHER_MIN_RPM) {
      //if (currentTime > flywheelStateTimer) {
        flywheelState = FLYWHEEL_STATE_FIRE;
        Serial.println("rerev->fire");
      }
      break;
    }
    case FLYWHEEL_STATE_BRAKE: {
      flywheelMotor.brake(FLYWHEEL_MOTOR_STRENGTH_BRAKE);
      pusherMotor.coast();
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
      flywheelMotor.coast();
      pusherMotor.coast();
      break;
    }
  }
  triggerFireOld = triggerFire;
  delay(LOOP_TIME);
}
