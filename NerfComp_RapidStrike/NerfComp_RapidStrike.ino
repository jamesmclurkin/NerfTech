// NerfCommp control code

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Servo.h>
#include <avr/pgmspace.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP23008.h>

#define ARDUINO_NANO
//#define ARDUINO_PROMICRO

#include <NerfComp.h>
#include <NerfCompDisplay.h>


// global variables for main system stats
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

//void displayUpdate(void);

unsigned long heartbeatUpdateTime = 0;
unsigned long heartbeatPrintTime = 0;

// create a servo object to control the flywheel ESC
Servo servoESC;


// GPIO Port expander for mag type
Adafruit_MCP23008 GPIO_mag;

//////// configuration parameters ////////
const ConfigParam configParams[] PROGMEM = {
  //012345678901234567890
//("   System Config:XXXX"));
  {"       Rev speed", FLYWHEEL_MOTOR_ESC_RUN,  100,  200, 5 },
  {"   Rev time semi", FLYWHEEL_REVUP_TIME_SEMI,  0, 1000, 50},
  {"   Rev time full", FLYWHEEL_REVUP_TIME_FULL,  0, 1000, 50},
  {"   Plunger speed", PLUNGER_PWM_RUN_SPEED,    50,  250, 5 },
  {"     Dart length", DART_LENGTH_MM,           10,  200, 1 },
  {"     Display dim", 0,                         0,    1, 1 },
  {"Reset to default", 0,                         0,    1, 1 },
};

int8_t paramCount(void) {return sizeof(configParams)/sizeof(ConfigParam);}
int16_t paramValueDefault(uint8_t idx) {return (int16_t)pgm_read_word(&configParams[idx].valueDefault);}
int16_t paramValueMax(uint8_t idx)     {return (int16_t)pgm_read_word(&configParams[idx].valueMax);}
int16_t paramValueMin(uint8_t idx)     {return (int16_t)pgm_read_word(&configParams[idx].valueMin);}
int16_t paramValueStep(uint8_t idx)    {return (int16_t)pgm_read_word(&configParams[idx].valueStep);}
const char* paramName(uint8_t idx) {return (const char*)&configParams[idx].name;}


//////// I/O wrappers ////////
boolean switchMagSafetyRead() {return !digitalRead(PIN_SAFETY_MAG); }
boolean switchRevTriggerRead() {return !digitalRead(PIN_FLYWHEEL_TRIGGER); }
boolean switchJamDoorRead() {return !digitalRead(PIN_SAFETY_JAMDOOR); }
boolean switchTriggerRead() {return !digitalRead(PIN_PLUNGER_TRIGGER); }
boolean switchPlungerStopRead() {return !digitalRead(PIN_PLUNGER_END_SWITCH); }
boolean sensorBarrelRead() {return digitalRead(PIN_BARREL_START); }


uint8_t flipLowNibble(uint8_t val) {
  uint8_t rval = 0;
  if(val & 0x01) {rval |= 0x08;}
  if(val & 0x02) {rval |= 0x04;}
  if(val & 0x04) {rval |= 0x02;}
  if(val & 0x08) {rval |= 0x01;}
  return rval;
}

uint8_t magTypeReadBits() {
  uint8_t magTypeBits = MAGTYPE_EMPTY;
  if (switchMagSafetyRead()) {
    magTypeBits = GPIO_mag.readGPIO();
    //mask and invert
    magTypeBits = ((~magTypeBits) >> 4) & 0x0f;
    //flip if needed
    //magTypeBits = flipLowNibble(magTypeBits);
  }
  return magTypeBits;
}

uint8_t magazineTypeLookup(int magTypeBits) {
  uint8_t typeIdx = 0;
  uint8_t typeBitsTemp;

  do {
    //typeBitsTemp = pgm_read_byte(&magazineTypes[typeIdx].code);
    typeBitsTemp = magazineTypesGetCode(typeIdx);
    if ((typeBitsTemp == magTypeBits) || (typeBitsTemp == MAGTYPE_UNKNOWN)) {
      // found the correct magazine type, or reached the end
      // of the list.  break and return.
      break;
    }
    typeIdx++;
  } while (true);
  return typeIdx;
}


#define BARREL_STATE_START                  0
#define BARREL_STATE_END                    1
#define BARREL_TIME_DART_INTERVAL_MAX_US    10000L
#define BARREL_TIME_DART_LENGTH_MAX_US      1000L

volatile uint8_t barrelIRQState = BARREL_STATE_START;

void irqBarrelStart() {
  //debug_barrelStart = true;
  if (barrelIRQState == BARREL_STATE_START) {
    unsigned long time = micros();
    if (time > (timeBarrelStart + BARREL_TIME_DART_INTERVAL_MAX_US)) {
      // rising edge = new dart, and it's been long enough that it's not a glitch
      timeBarrelStart = time;
      barrelIRQState = BARREL_STATE_END;
      //debug_barrelStart_glitch = true;
    }
  }
}

void irqBarrelEnd() {
  //debug_barrelEnd = true;
  if (barrelIRQState == BARREL_STATE_END) {
    unsigned long time = micros();
    if (time > (timeBarrelStart + BARREL_TIME_DART_LENGTH_MAX_US)) {
      // falling edge = end of the dart, and it's been long enough that it's not a glitch
      timeBarrelEnd = time;
      timeBarrelEndFlag = true;
      barrelIRQState = BARREL_STATE_START;
      //debug_barrelEnd_glitch = true;
    }
  }
}


//////// motor control ////////

void plungerMotorPWM(int dir, int pwm) {
  int dirFwd, dirRev;


  switch (dir) {
  case MOTOR_DIR_FWD: {dirFwd = HIGH; dirRev = LOW;} break;
  case MOTOR_DIR_REV: {dirFwd = LOW; dirRev = HIGH;} break;
  case MOTOR_DIR_BRAKE: {dirFwd = HIGH; dirRev = HIGH; } break;
  case MOTOR_DIR_OFF: default: {dirFwd = LOW; dirRev = LOW; } break;
  }

  digitalWrite(PIN_PLUNGER_MOTOR_FWD, dirFwd);
  digitalWrite(PIN_PLUNGER_MOTOR_REV, dirRev);
  analogWrite(PIN_PLUNGER_MOTOR_PWM, pwm);
}


void plungerMotorInit(void) {
  pinMode(PIN_PLUNGER_MOTOR_FWD, OUTPUT);
  pinMode(PIN_PLUNGER_MOTOR_REV, OUTPUT);
  pinMode(PIN_PLUNGER_MOTOR_PWM, OUTPUT);
  plungerMotorPWM(MOTOR_DIR_OFF, 0);
}


//////// setup ////////
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


void setup() {
  // Setup the pins and interrupts for the barrel photo interrupters
  pinMode(PIN_BARREL_START, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BARREL_START), irqBarrelStart, RISING);

  pinMode(PIN_BARREL_END, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BARREL_END), irqBarrelEnd, FALLING);

  // Setup the pins for internal sensing
  pinMode(PIN_SAFETY_MAG, INPUT_PULLUP);
  pinMode(PIN_FLYWHEEL_TRIGGER, INPUT_PULLUP);
  pinMode(PIN_SAFETY_JAMDOOR, INPUT_PULLUP);
  pinMode(PIN_PLUNGER_TRIGGER, INPUT_PULLUP);
  pinMode(PIN_PLUNGER_END_SWITCH, INPUT_PULLUP);

  // init the port expanders for mag type and HUD buttons
  GPIO_mag.begin(0);      // GPIO magazine is on address 0
  for (int i = 0; i < 8; ++i) {
    GPIO_mag.pinMode(i, INPUT);
    GPIO_mag.pullUp(i, HIGH);  // turn on a 100K pullup internally
  }

  plungerMotorInit();

  // init the servo
  servoESC.attach(PIN_FLYWHEEL_ESC); // attaches the servo on pin 9 to the servo object
  servoESC.write(FLYWHEEL_MOTOR_ESC_NEUTRAL);

  // init the serial port for debugging output
  Serial.begin(115200);
  Serial.println(F(" NerfComp: RapidStrike ver 0.6"));
  Serial.print(F("   Free RAM:")); Serial.print(freeRam()); Serial.println(F(" bytes"));


  // init the config Parameters
  paramInit();

  // init the LED Display
  displayInit();

  // reset the heartbeat time to avoid a bunch of initial updates
  heartbeatUpdateTime = millis();
  heartbeatPrintTime = heartbeatUpdateTime;
}


int plungerState = PLUNGER_STATE_IDLE;
unsigned long plungerStateTime;
unsigned long displayUpdateTime = 0;

boolean sp = true;

void loop() {
  static int magazineTypeCounter;
  static boolean magazineNew = false;
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


  // read the motor voltage every 100ms
  boolean p = false;
  if (millis() > heartbeatUpdateTime) {
    heartbeatUpdateTime += HEARTBEAT_UPDATE_PERIOD;
    if (millis() > heartbeatPrintTime) {
      heartbeatPrintTime += HEARTBEAT_PRINT_PERIOD;
      p = true;
    }
    //p = false;
    if (p) {Serial.print(F("hb "));}
    if (p) {Serial.print(F("  magbits=")); Serial.print(GPIO_mag.readGPIO(), HEX); }
    if (p) {Serial.print(F("  uibits=")); Serial.print(displayGetButtonBits(), HEX); }
    if (p) {Serial.print(F("  rounds=")); Serial.print(roundCount, DEC); }


    // update the motor voltage
    int voltageBatteryRaw = analogRead(PIN_BATTERY_VOLTAGE);
    float voltageBatteryTemp = (float) voltageBatteryRaw * VOLTAGE_BATTERY_SCALER;

    if (p) {Serial.print(F("  vbat=")); Serial.print(voltageBatteryRaw, DEC); }
    if (p) {Serial.print(F(",")); Serial.print(voltageBatteryTemp, 2); }
    // add some sanity checks to the voltage
    if ((voltageBatteryTemp >= VOLTAGE_MIN) && (voltageBatteryTemp <= VOLTAGE_MAX)) {
      // compute a IIR low-pass filter
      voltageBatteryAvg = VOLTAGE_BATTERY_IIR_GAIN * voltageBatteryTemp + (1 - VOLTAGE_BATTERY_IIR_GAIN) * voltageBatteryAvg;
    }


    // check the magazine type
    uint8_t magazineTypeTemp = magTypeReadBits();
    if (p) {Serial.print(F("  mag=")); Serial.print(magazineTypeTemp, DEC); }
    if (magazineType != magazineTypeTemp) {
      magazineTypeCounter = 0;
      magazineNew = true;
    } else {
      if (magazineTypeCounter < MAGAZINE_TYPE_DELAY) {
        magazineTypeCounter++;
      } else {
        if (magazineNew) {
          magazineTypeIdx = magazineTypeLookup(magazineTypeTemp);
          //roundCount = pgm_read_byte(&magazineTypes[magazineTypeIdx].capacity);
          roundCount = magazineTypesGetCapacity(magazineTypeIdx);
          roundsJamCount = 0;
          magazineNew = false;
          Serial.println(F("")); Serial.print(F("(new magazine "));
          //SerialPrint_F(magazineTypes[magazineTypeIdx].name); Serial.println(F(")"));
          SerialPrint_F(magazineTypesGetName(magazineTypeIdx)); Serial.println(F(")"));          
        }
      }
    }
    magazineType = magazineTypeTemp;
    //if (p) {Serial.print(F(",")); SerialPrint_F(magazineTypes[magazineTypeIdx].name);}
    if (p) {Serial.print(F(",")); SerialPrint_F(magazineTypesGetName(magazineTypeIdx));}


    // check the jam door
    if ((magazineType != MAGTYPE_EMPTY) && (magazineTypeCounter == MAGAZINE_TYPE_DELAY)) {
      boolean jamDoorOpenTemp = switchJamDoorRead();
      if (p) {Serial.print(F(" jamdoor=")); Serial.print(jamDoorOpenTemp);}
      if (jamDoorOpen != jamDoorOpenTemp) {
        if (jamDoorOpenTemp) {
          // jam door has gone from closed to open.  inc the jam count
          // clear the feed jam indicator if it was set
          roundsJamCount++;
          feedJam = false;
        } else {
          // jam door has gone from open to closed.  decrement a round
          if (roundCount > 0) {
            roundCount--;
          }
        }
        jamDoorOpen = jamDoorOpenTemp;
      }
    } else {
      jamDoorOpen = false;
    }

    // read the UI Buttons
    buttonUpdateEvents();

//    if (p) {Serial.print(F(" mag=")); Serial.print(magazineSwitchRead());}
//    if (p) {Serial.print(F(" revTrig=")); Serial.print(revTriggerRead());}
//    if (p) {Serial.print(F(" jam=")); Serial.print(jamDoorRead());}
//    if (p) {Serial.print(F(" trig=")); Serial.print(triggerRead());}
//    if (p) {Serial.print(F(" plunger=")); Serial.print(plungerEndRead());}

}

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


  servoESC.write(ESCPos);

  if (p) {Serial.println(F(""));}

  if (((millis() > (displayUpdateTime + DISPLAY_UPDATE_PERIOD))
      & displayUpdateEnable) || displayUpdateForce) {
    displayUpdateTime = millis();
    displayUpdateForce = false;
    displayUpdate();
  }
}
