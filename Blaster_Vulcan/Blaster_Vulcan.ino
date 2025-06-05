// NerfComp control code for Stryfe Solenoid
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_DotStar.h>
//#include <NerfTech.h>
#include <Tachometer.h>

// global variables for main system status
#define HEARTBEAT_UPDATE_PERIOD         10
#define HEARTBEAT_PRINT_PERIOD          250
#define LED_UPDATE_PERIOD               20

unsigned long heartbeatUpdateTime = 0;
unsigned long heartbeatPrintTime = 0;
unsigned long LEDUpdateTime = 0;

// pin assignments
#define PIN_PLUNGER_PWM             12
#define PIN_PLUNGER_BACK_SW         10

#define PIN_TRIGGER_MODE_SW         9
#define PIN_TRIGGER_SW              7

#define PIN_FLYWHEEL_ESC            5
#define PIN_BREAKBEAM               1
#define PIN_FLYWHEEL_TACHOMETER     0

#define PIN_SAFETY_JAMDOOR_SW       3

#define PIN_MOTOR_TEMP              (A2)

// flywheel motor constants
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
#define FLYWHEEL_STATE_REV_UP_TIME          800
#define FLYWHEEL_STATE_REREV_TIME           200
#define FLYWHEEL_STATE_REV_HOLD_TIME        1000
#define FLYWHEEL_STATE_BRAKE_TIME           600

Servo servoESC;
int flywheelState = FLYWHEEL_STATE_STARTUP_DELAY;

#define PLUNGER_MIN_RPM                     25000

#define PWM_PERCENT(val)                  (((val) * 255) / 100)
#define PLUNGER_PWM_OFF                   0
#define PLUNGER_PWM_RUN                   PWM_PERCENT(50)

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
boolean switchTriggerFireRead() {return !digitalRead(PIN_TRIGGER_SW); }
//boolean switchPlungerEndRead() {return !digitalRead(PIN_PLUNGER_END_SW); }

void plungerPWMSet(int val) {
  analogWrite(PIN_PLUNGER_PWM, val);
}



// breakbeam
#define BREAKBEAM_IDLE                0
#define BREAKBEAM_MEASURING           1

#define BREAKBEAM_TIME_MAX            10000
#define DART_LENGTH_MM                72.0
#define SENSOR_POS_MM                 10.0

void breakbeamISR(void);

unsigned long breakbeamReadTime;
unsigned long breakbeamStartTime;
unsigned long breakbeamEndTime;
unsigned long breakbeamTime = 0;
boolean breakbeamState = BREAKBEAM_IDLE;
#define MM_TO_FOOT    304.8

// Interrupt on rising edge on the breakbeam pin.  Then falling edge.
void breakbeamEdgeMeasure (void) {
  attachInterrupt(digitalPinToInterrupt(PIN_BREAKBEAM), breakbeamISR, RISING);
  breakbeamState = BREAKBEAM_MEASURING;
}

void breakbeamEdgeReset (void) {
  attachInterrupt(digitalPinToInterrupt(PIN_BREAKBEAM), breakbeamISR, FALLING);
  breakbeamState = BREAKBEAM_IDLE;
  breakbeamTime = 0;
}

void breakbeamISR(void) {
  if (breakbeamState == BREAKBEAM_IDLE) {
    breakbeamStartTime = micros();
    breakbeamEdgeMeasure();
  } else {
    // BREAKBEAM_MEASURING:
    breakbeamEndTime = micros();
    breakbeamEdgeReset();
    breakbeamTime = breakbeamEndTime - breakbeamStartTime;
  }
}

void breakbeamResetAfterMaxTime(void) {
  if (breakbeamState == BREAKBEAM_MEASURING) {
    unsigned long timeCurrent = micros();
    if(timeCurrent > (breakbeamStartTime + BREAKBEAM_TIME_MAX)) {
      breakbeamEdgeReset();
    }
  }
}

unsigned long breakbeamGetDartTime(void) {
  unsigned long val = breakbeamTime;
  if (val > 0) {
    breakbeamTime = 0;
  }
  return val;
}

float dartMicrosToFPS (int timeUS) {
  float speedFPS = -1.0;
  if (timeUS > 0) {
    float speedMMS = DART_LENGTH_MM * 1000000.0 / (float)timeUS;
    speedFPS = speedMMS / MM_TO_FOOT;
  }
  return speedFPS;
}

// Dot Star
// Everything is defined in the Board Support Package
// DOTSTAR_NUM        number of onboard DotStars (typically just 1)
// PIN_DOTSTAR_DATA   onboard DotStar data pin
// PIN_DOTSTAR_CLK    onboard DotStar clock pin
Adafruit_DotStar dotStar(DOTSTAR_NUM, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLK, DOTSTAR_BRG);



long flywheelStateTimer = 0;
long plungerStateTimer = 0;
Tachometer tach(PIN_FLYWHEEL_TACHOMETER);

void setup() {
  // Setup the pins for internal sensing
  //pinMode(PIN_TRIGGER_REV, INPUT_PULLUP);
  pinMode(PIN_TRIGGER_SW, INPUT_PULLUP);
  //pinMode(PIN_PLUNGER_END_SW, INPUT_PULLUP);
  pinMode(PIN_PLUNGER_PWM, OUTPUT);
  plungerPWMSet(0);

  // init the esc.  we control it with a servo object
  servoESC.attach(PIN_FLYWHEEL_ESC); // attaches the servo on pin 9 to the servo object
  servoESC.write(FLYWHEEL_MOTOR_ESC_NEUTRAL);  
  
  // init the dart breakbeam sensor
  pinMode(PIN_BREAKBEAM, INPUT);
  breakbeamEdgeReset();

  // initialize the LED as an output:
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

    // update breakbeam
  breakbeamResetAfterMaxTime();

  unsigned long dartTime = breakbeamGetDartTime();
  if(dartTime > 0) {
    int dt = (int)dartTime;
    Serial.print("  dart:");
    Serial.print(dartTime, DEC);
    Serial.print("us, ");
    Serial.print(dartMicrosToFPS(dt), 1);
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
  int ESCPos = FLYWHEEL_MOTOR_ESC_NEUTRAL;
  int plungerPWM = PLUNGER_PWM_OFF;

  boolean triggerFire = switchTriggerFireRead();
  boolean triggerFireEdge = (triggerFire && !triggerFireOld);

    // update tachometer & Fire LED
  if (currentTime > LEDUpdateTime) {
    if (triggerFire) {
      //dotStar.setPixelColor(0, 0x800000);
      if (tach.rpm() > PLUNGER_MIN_RPM) {
        // #00FF00
        dotStar.setPixelColor(0, 0x00FF00);    
      } else {
        // #FFFF00
        dotStar.setPixelColor(0, 0xFFFF00);    
      }
    } else {
      if (tach.rpm() > PLUNGER_MIN_RPM) {
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
      //if (currentTime > flywheelStateTimer) {
      if (tach.rpm() > PLUNGER_MIN_RPM) {
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
