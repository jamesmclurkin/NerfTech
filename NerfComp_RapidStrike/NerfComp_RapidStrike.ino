// NerfComp control code for RapidStrike
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>
#include <avr/pgmspace.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP23008.h>

#include <NerfComp.h>
#include <NerfCompDisplay.h>
#include <NerfCompIO.h>


//////// global variables for main system state ////////
uint8_t magazineType = MAGTYPE_EMPTY;
uint8_t magazineTypeIdx = 0;
int8_t roundCount = -1;
int16_t roundsPerMin = -1;
uint8_t roundsJamCount = 0;
float velocity = -1;
float voltageBatteryAvg = 0.0;
boolean jamDoorOpen = false;
boolean feedJam = false;

volatile unsigned long timeBarrelStart = 0;
volatile boolean timeBarrelStartFlag = false;

volatile unsigned long timeBarrelEnd = 0;
volatile boolean timeBarrelEndFlag = false;

unsigned long heartbeatUpdateTime = 0;
unsigned long heartbeatPrintTime = 0;


//////// configuration parameters for Rapidstrike ////////
#define PARAM_FLYWHEEL_MOTOR_ESC_RUN    0
#define PARAM_FLYWHEEL_REVUP_TIME_SEMI  1
#define PARAM_FLYWHEEL_REVUP_TIME_FULL  2
#define PARAM_PLUNGER_PWM_RUN_SPEED     3
#define PARAM_DART_LENGTH_MM            4
#define PARAM_DISPLAY_DIM               5
#define PARAM_INVERT_MAG                6
#define PARAM_RESET_ALL                 7

const ConfigParam configParams[] PROGMEM = {
  //012345678901234567890
//("   System Config:XXXX"));
  {"       Rev speed", FLYWHEEL_MOTOR_ESC_RUN,  100,  200, 5 }, // 0
  {"   Rev time semi", FLYWHEEL_REVUP_TIME_SEMI,  0, 1000, 50}, // 1
  {"   Rev time full", FLYWHEEL_REVUP_TIME_FULL,  0, 1000, 50}, // 2
  {"   Plunger speed", PLUNGER_PWM_RUN_SPEED,    50,  250, 5 }, // 3
  {"     Dart length", DART_LENGTH_MM,           10,  200, 1 }, // 4
  {"     Display dim", 0,                         0,    1, 1 }, // 5
  {"      Invert mag", 0,                         0,    1, 1 }, // 6
  {"    Reset config", 0,                         0,    1, 1 }, // 7
};

int8_t paramCount(void) {return sizeof(configParams)/sizeof(ConfigParam);}
int16_t paramValueDefault(uint8_t idx) {return (int16_t)pgm_read_word(&configParams[idx].valueDefault);}
int16_t paramValueMax(uint8_t idx)     {return (int16_t)pgm_read_word(&configParams[idx].valueMax);}
int16_t paramValueMin(uint8_t idx)     {return (int16_t)pgm_read_word(&configParams[idx].valueMin);}
int16_t paramValueStep(uint8_t idx)    {return (int16_t)pgm_read_word(&configParams[idx].valueStep);}
const char* paramName(uint8_t idx)     {return (const char*)&configParams[idx].name;}

int16_t paramReadDisplayDim(void)      {return paramRead(PARAM_DISPLAY_DIM);}
int16_t paramReadResetAll(void)        {return paramRead(PARAM_RESET_ALL);}
int16_t paramReadInvertMag(void)       {return paramRead(PARAM_INVERT_MAG);}


//////// setup ////////
void setup() {
  gpioInit();

  // init the serial port for debugging output
  Serial.begin(115200);
  Serial.println(F(" NerfComp: RapidStrike ver 0.7"));
  Serial.print(F("   Free RAM:")); Serial.print(freeRam()); Serial.println(F(" bytes"));

  // init the config Parameters
  paramInit();

  // init the LED Display
  displayInit();

  // reset the heartbeat time to avoid a bunch of initial updates
  heartbeatUpdateTime = millis();
  heartbeatPrintTime = heartbeatUpdateTime;
}


//////// Loop ////////
int plungerState = PLUNGER_STATE_IDLE;
unsigned long plungerStateTime;
unsigned long displayUpdateTime = 0;
boolean sp = true;

void loop() {
  static unsigned long roundTimePrev = 0;

  boolean displayUpdateEnable = true;
  boolean displayUpdateForce = false;

  // look for a dart at the end of the barrel
  if (timeBarrelEndFlag) {
    timeBarrelEndFlag = false;
    if (roundCount > 0) {
      Serial.print(F("round=")); Serial.print(roundCount, DEC);
      roundCount--;
      // compute time of flight in microseconds
      unsigned long time = timeBarrelEnd - timeBarrelStart;
      Serial.print(F(" time=")); Serial.print(time, DEC);

      // compute velocity in feet/sec. apply some sanity checks
      if (time > 0) {
        float velocityTemp = (DART_LENGTH_INCHES * 1000000) / (12 * (float) time);
        Serial.print(F(" vel=")); Serial.print(velocityTemp, 1);
        //if ((velocityTemp < VELOCITY_FPS_MIN) || (velocityTemp > VELOCITY_FPS_MAX)) {
        if ((velocityTemp > VELOCITY_FPS_MAX)) {
          // velocity error
          velocity = -1.0;
        } else {
          velocity = velocityTemp;
        }
      } else {
        // time <= 0.  time error => velocity error.
        velocity = -1.0;
      }
      // compute rounds per minute for bursts
      unsigned long roundTime = millis();
      if (roundTimePrev != 0) {
        unsigned long roundTimeDelta = roundTime - roundTimePrev;
        if (roundTimeDelta > 0) {
          // rounds per minute =
          unsigned long rpmTemp = 60000ul / roundTimeDelta;
          Serial.print(F(" delta=")); Serial.print(roundTimeDelta, DEC);
          Serial.print(F(",rpm=")); Serial.print(rpmTemp, DEC);
          if (rpmTemp > 60) {
            roundsPerMin = (int) rpmTemp;
          } else {
            // too slow a rate of fire to matter.  single shot
            // clear the rd/min display
            roundsPerMin = -1;
          }
        }
      }
      roundTimePrev = roundTime;
      Serial.println(F(""));
    }
  }


  // read the battery voltage every 100ms
  boolean p = false;
  if (millis() > heartbeatUpdateTime) {
    heartbeatUpdateTime += HEARTBEAT_UPDATE_PERIOD;
    if (millis() > heartbeatPrintTime) {
      heartbeatPrintTime += HEARTBEAT_PRINT_PERIOD;
      p = true;
    }
    //p = false;
    if (p) {Serial.print(F("hb "));}
    if (p) {Serial.print(F("  magbits=")); Serial.print(gpioGetMagSensorBits(), HEX); }
    if (p) {Serial.print(F("  uibits=")); Serial.print(displayGetButtonBits(), HEX); }
    if (p) {Serial.print(F("  rounds=")); Serial.print(roundCount, DEC); }


    // update the battery voltage
    batteryVoltageUpdate(p);

    // check the magazine type
    magazineTypeUpdate(p);

    // check the jam door
    jamDoorUpdate(p);

    // read the UI Buttons
    buttonUpdateEvents();
  }
//    if (p) {Serial.print(F(" mag=")); Serial.print(magazineSwitchRead());}
//    if (p) {Serial.print(F(" jam=")); Serial.print(jamDoorRead());}
//    if (p) {Serial.print(F(" rev=")); Serial.print(revTriggerRead());}
//    if (p) {Serial.print(F(" trig=")); Serial.print(triggerRead());}
//    if (p) {Serial.print(F(" plunger=")); Serial.print(plungerEndRead());}


  //if (magazineSwitchRead() && jamDoorRead()) {
  // Safety switches are ok.  process the rev and fire triggers

  int ESCPos = FLYWHEEL_MOTOR_ESC_NEUTRAL;
  // for semi-auto mode, when the trigger is pulled, rev the flywheel, then power the trigger
  switch (plungerState) {
  case PLUNGER_STATE_IDLE: {
    if (sp) {Serial.println(F("")); Serial.println(F(" s:idle")); sp = false; }
    plungerMotorPWM(MOTOR_DIR_BRAKE, PLUNGER_PWM_MAX);
    if (switchRevTriggerRead()) {
      ESCPos = FLYWHEEL_MOTOR_ESC_PRE_RUN_FULL;
    } else {
      ESCPos = FLYWHEEL_MOTOR_ESC_BRAKE;
    }
    if (switchTriggerRead() && STATE_DELAY(TRIGGER_DELAY_TIME)) {
      SET_PLUNGER_STATE(PLUNGER_STATE_FLYWHEEL_REVUP);
    }
    break;
  }
  case PLUNGER_STATE_FLYWHEEL_REVUP: {
    if (sp) {Serial.println(F(" s:revup")); sp = false;}
    plungerMotorPWM(MOTOR_DIR_BRAKE, PLUNGER_PWM_MAX);
    ESCPos = FLYWHEEL_MOTOR_ESC_REVUP;
    displayUpdateEnable = false;
    unsigned long revTime;
    if (switchRevTriggerRead()) {
      revTime = paramRead(PARAM_FLYWHEEL_REVUP_TIME_FULL);
    } else {
      revTime = paramRead(PARAM_FLYWHEEL_REVUP_TIME_SEMI);
    }
    if (STATE_DELAY(revTime)) {
      SET_PLUNGER_STATE(PLUNGER_STATE_CLEAR_END_SWITCH);
    }
    break;
  }
  case PLUNGER_STATE_CLEAR_END_SWITCH: {
    if (sp) {Serial.println(F(" s:clear")); sp = false;}
    plungerMotorPWM(MOTOR_DIR_FWD, paramRead(PARAM_PLUNGER_PWM_RUN_SPEED));
    ESCPos = paramRead(PARAM_FLYWHEEL_MOTOR_ESC_RUN);
    displayUpdateEnable = false;
    if (!switchPlungerStopRead()) {
      SET_PLUNGER_STATE(PLUNGER_STATE_RUN_PLUNGER);
      //displayUpdateForce = true;
    }
    if (STATE_DELAY(FEED_JAM_TIME)) {
      //SET_PLUNGER_STATE(PLUNGER_STATE_IDLE);
      feedJam = true;
    }
    break;
  }
  case PLUNGER_STATE_RUN_PLUNGER: {
    if (sp) {Serial.println(F(" s:run")); sp = false;}
    plungerMotorPWM(MOTOR_DIR_FWD, paramRead(PARAM_PLUNGER_PWM_RUN_SPEED));
    ESCPos = paramRead(PARAM_FLYWHEEL_MOTOR_ESC_RUN);
    displayUpdateEnable = false;
    if (switchPlungerStopRead() && STATE_DELAY(25)) {
      // plunger is at end.  stop it if we're in semi-auto
      if (!switchRevTriggerRead()) {
        // semi auto.  stop the plunger
        SET_PLUNGER_STATE(PLUNGER_STATE_WAIT_FOR_TRIGGGER_RELEASE);
      } else {
        SET_PLUNGER_STATE(PLUNGER_STATE_ROUND_DELAY);
      }
    }
    if (STATE_DELAY(FEED_JAM_TIME)) {
      //SET_PLUNGER_STATE(PLUNGER_STATE_IDLE);
      feedJam = true;
    }
    break;
  }
  case PLUNGER_STATE_ROUND_DELAY: {
    if (sp) {Serial.println(F(" s:delay")); sp = false;}
    plungerMotorPWM(MOTOR_DIR_BRAKE, PLUNGER_PWM_MAX);
    // full auto.  leave the flywheel spinning
    ESCPos = paramRead(PARAM_FLYWHEEL_MOTOR_ESC_RUN);
    displayUpdateEnable = false;
    if ((millis() - plungerStateTime) > ROUND_DELAY_TIME) {
      //displayUpdateForce = true;
      if (switchTriggerRead()) {
        // chamber another round
        SET_PLUNGER_STATE(PLUNGER_STATE_CLEAR_END_SWITCH);
      }
    }
    if (STATE_DELAY(FLYWHEEL_REVDOWN_TIME_FULL)) {
      // rev down the flywheel to idle speed until the trigger is pulled again
      SET_PLUNGER_STATE(PLUNGER_STATE_IDLE);
    }
    if (!switchRevTriggerRead()) {
      // semi auto.  stop the plunger
      SET_PLUNGER_STATE(PLUNGER_STATE_WAIT_FOR_TRIGGGER_RELEASE);
    }
    break;
  }
  case PLUNGER_STATE_WAIT_FOR_TRIGGGER_RELEASE: {
    if (sp) {Serial.println(F(" s:release")); sp = false;}
    plungerMotorPWM(MOTOR_DIR_BRAKE, PLUNGER_PWM_MAX);
    ESCPos = FLYWHEEL_MOTOR_ESC_BRAKE;
    displayUpdateEnable = false;
    if (!switchTriggerRead()) {
      SET_PLUNGER_STATE(PLUNGER_STATE_IDLE);
    }
    break;
  }
  }

  servoESCWrite(ESCPos);

  if (p) {Serial.println(F(""));}

  if (((millis() > (displayUpdateTime + DISPLAY_UPDATE_PERIOD))
      & displayUpdateEnable) || displayUpdateForce) {
    displayUpdateTime = millis();
    displayUpdateForce = false;
    displayUpdate();
  }
}
