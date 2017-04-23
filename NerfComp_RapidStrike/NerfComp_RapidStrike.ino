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

void displayUpdate(void);

unsigned long heartbeatUpdateTime = 0;
unsigned long heartbeatPrintTime = 0;

// create a servo object to control the flywheel ESC
Servo servoESC;

typedef struct MagazineType {
  const uint8_t code;
  const char name[MAGAZINE_NAME_SIZE];
  const uint8_t capacity;
} MagazineType;


const MagazineType magazineTypes[] PROGMEM = {
    {MAGTYPE_EMPTY,   "----", -1},
    {MAGTYPE_CLIP_6,  "Clip6", 6},
    {MAGTYPE_CLIP_10, "Clip10", 10},
    {MAGTYPE_CLIP_12, "Clip12", 12},
    {MAGTYPE_CLIP_15, "Clip15", 15},
    {MAGTYPE_CLIP_18, "Clip18", 18 },
    {MAGTYPE_DRUM_18, "Drum18", 18},
    {MAGTYPE_DRUM_25, "Drum25", 25 },
    {MAGTYPE_DRUM_35, "Drum35", 35 },
    {MAGTYPE_UNKNOWN, "????", 10 }
};


// GPIO Port expander for mag type and screen UI
Adafruit_MCP23008 GPIO_mag;
Adafruit_MCP23008 GPIO_UI;

#define MENU_ITEM_NAME_SIZE 12

#define UI_SCREEN_HUD         0
#define UI_SCREEN_MENU        1
#define UI_SCREEN_CONFIG      2
#define UI_SCREEN_CONFIG_EDIT 3
#define UI_SCREEN_TIMER       4
#define UI_SCREEN_DIAGNOSTIC  5

typedef struct MenuItem {
  const char name[MENU_ITEM_NAME_SIZE];
  const uint8_t UIMode;
} MenuItem;

const MenuItem menuItems[] PROGMEM = {
   // 012345678901234567890
   //"Menu       "
    {"       HUD", UI_SCREEN_HUD},
    {"    Config", UI_SCREEN_CONFIG},
    {"     Timer", UI_SCREEN_TIMER},
    {"Diagnostic", UI_SCREEN_DIAGNOSTIC},
    {" MenuItem1", UI_SCREEN_DIAGNOSTIC},
    {" MenuItem2", UI_SCREEN_DIAGNOSTIC},
};



#define CONFIG_PARAM_NAME_SIZE 17

typedef struct ConfigParam {
  const char name[CONFIG_PARAM_NAME_SIZE];
  const int16_t valueDefault;
  const int16_t valueMin;
  const int16_t valueMax;
  const int16_t valueStep;
} ConfigParam;

#define PARAM_FLYWHEEL_MOTOR_ESC_RUN    0
#define PARAM_FLYWHEEL_REVUP_TIME_SEMI  1
#define PARAM_FLYWHEEL_REVUP_TIME_FULL  2
#define PARAM_PLUNGER_PWM_RUN_SPEED     3
#define PARAM_DART_LENGTH_MM            4
#define PARAM_DISPLAY_DIM               5
#define PARAM_RESET_ALL                 6

const ConfigParam configParams[] PROGMEM = {
    //012345678901234567890
  //("   System Config:XXXX"));
    {"       Rev speed", FLYWHEEL_MOTOR_ESC_RUN,  100,  200, 5 },
    {"   Rev time semi", FLYWHEEL_REVUP_TIME_SEMI,  0, 1000, 50},
    {"   Rev time full", FLYWHEEL_REVUP_TIME_FULL,  0, 1000, 50},
    {"   Plunger speed", PLUNGER_PWM_RUN_SPEED,    50,  250, 5 },
    {"     Dart sength", DART_LENGTH_MM,           10,  200, 1 },
    {"     Display dim", 0,                         0,    1, 1 },
    {"Reset to default", 0,                         0,    1, 1 },
};


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
    magTypeBits = flipLowNibble(magTypeBits);
  }
  return magTypeBits;
}

uint8_t magazineTypeLookup(int magTypeBits) {
  uint8_t typeIdx = 0;
  uint8_t typeBitsTemp;

  do {
    typeBitsTemp = pgm_read_byte(&magazineTypes[typeIdx].code);
    if ((typeBitsTemp == magTypeBits) || (typeBitsTemp == MAGTYPE_UNKNOWN)) {
      // found the correct magazine type, or reached the end
      // of the list.  break and return.
      break;
    }
    typeIdx++;
  } while (true);
  return typeIdx;
}


//#define GLITCH_CHECKS 4
//
//boolean glitchReject(int pin) {
//  boolean returnVal = true;
//  volatile int i;
//
//  for (i = 0; i < GLITCH_CHECKS; i++) {
//    if (digitalRead(pin) == LOW) {
//      // Pin is low again.  this was a glitch.  return false.
//      returnVal = false;
//      break;
//    } else {
//      // want ~500ns delay here, around 10 clock ticks
//      i++;
//      i--;
//      i++;
//      i--;
//    }
//  }
//  return returnVal;
//}
//
//void irqBarrelStart() {
//  // the dart crossed the first (chamber) barrel sensor
//  if (glitchReject(PIN_BARREL_START)) {
//    // good signal.  note the start time
//    timeBarrelStart = micros();
//  }
//}
//
//void irqBarrelEnd() {
//  // the dart crossed the last (muzzle) barrel sensor
//  if (glitchReject(PIN_BARREL_END)) {
//    // good signal.  note the end time, and set the display update flag
//    timeBarrelEnd = micros();
//    timeBarrelEndFlag = true;
//  }
//}

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


//////// user Interface ////////

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

uint8_t UIMode = UI_SCREEN_HUD;


#define BUTTON_BIT_SELECT       0x02
#define BUTTON_BIT_BACK         0x80
#define BUTTON_BIT_UP           0x04
#define BUTTON_BIT_DOWN         0x01

#define BUTTON_EVENT_NULL           0
#define BUTTON_EVENT_SHORT_SELECT   1
#define BUTTON_EVENT_SHORT_BACK     2
#define BUTTON_EVENT_SHORT_UP       3
#define BUTTON_EVENT_SHORT_DOWN     4

#define BUTTON_EVENT_LONG_SELECT    10

#define BUTTON_EVENT_RELEASE_SELECT 20

uint8_t buttonBitsOld1 = 0;
uint8_t buttonBitsOld2 = 0;
uint8_t buttonEvent = BUTTON_EVENT_NULL;

uint8_t _buttonRead() {
  uint8_t buttonBits;
  buttonBits = ~GPIO_UI.readGPIO();
  return buttonBits;
}

void buttonEventAdd(uint8_t e) {
  buttonEvent = e;
}

uint8_t buttonEventGet(void) {
  uint8_t val = buttonEvent;
  buttonEvent = BUTTON_EVENT_NULL;
  return val;
}

boolean buttonRisingEdge(uint8_t buttonBits, uint8_t buttonBitMask) {
  if ((!(buttonBitsOld2 & buttonBitMask)) &&
      (buttonBitsOld1 & buttonBitMask) &&
      (buttonBits & buttonBitMask)) {
    return true;
  } else {
    return false;
  }
}

void buttonUpdateEvents(void) {
  uint8_t buttonBits = _buttonRead();
  buttonBitsOld2 = buttonBitsOld1;
  buttonBitsOld1 = buttonBits;
  if (buttonRisingEdge(buttonBits, BUTTON_BIT_SELECT)) {buttonEventAdd(BUTTON_EVENT_SHORT_SELECT);}
  if (buttonRisingEdge(buttonBits, BUTTON_BIT_BACK))   {buttonEventAdd(BUTTON_EVENT_SHORT_BACK);}
  if (buttonRisingEdge(buttonBits, BUTTON_BIT_UP))     {buttonEventAdd(BUTTON_EVENT_SHORT_UP);}
  if (buttonRisingEdge(buttonBits, BUTTON_BIT_DOWN))   {buttonEventAdd(BUTTON_EVENT_SHORT_DOWN);}
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

void SerialPrint_F (const char * str) {
  char c;
  if (!str)
    return;
  while ((c = pgm_read_byte(str++)))
    Serial.print (c);
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
  GPIO_UI.begin(1);       // GPIO foor the UI buttons is on address 1
  for (int i = 0; i < 8; ++i) {
    GPIO_UI.pinMode(i, INPUT);
    GPIO_UI.pullUp(i, HIGH);  // turn on a 100K pullup internally
  }


  plungerMotorInit();

  // init the servo
  servoESC.attach(PIN_FLYWHEEL_ESC); // attaches the servo on pin 9 to the servo object
  servoESC.write(FLYWHEEL_MOTOR_ESC_NEUTRAL);

  // init the serial port for debugging output
  Serial.begin(115200);
  Serial.println(F(" NerfComp: RapidStrike ver 0.5"));
  Serial.print(F("   Free RAM:")); Serial.print(freeRam()); Serial.println(F(" bytes"));


  // init the config Parameters
  paramInit();

  // init the LED Display
  // generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C

  // Clear the display buffer and put up the splash screen
  display.clearDisplay();
  display.drawBitmap(0, 0, NerfLogoBitmap, NERF_LOGO_BITMAP_WIDTH, NERF_LOGO_BITMAP_HEIGHT, 1);
  display.display();

  // show the screen for a while
  delay(2000);

  // force an update to show  the initial data display
  displayUpdate();

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
    p = false;
    if (p) {Serial.print(F("hb "));}
    if (p) {Serial.print(F("  magbits=")); Serial.print(GPIO_mag.readGPIO(), HEX); }
    if (p) {Serial.print(F("  uibits=")); Serial.print(GPIO_UI.readGPIO(), HEX); }
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
          roundCount = pgm_read_byte(&magazineTypes[magazineTypeIdx].capacity);
          roundsJamCount = 0;
          magazineNew = false;
          Serial.println(F("")); Serial.print(F("(new magazine "));
          SerialPrint_F(magazineTypes[magazineTypeIdx].name); Serial.println(F(")"));
        }
      }
    }
    magazineType = magazineTypeTemp;
    if (p) {Serial.print(F(",")); SerialPrint_F(magazineTypes[magazineTypeIdx].name);}


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
      SET_PLUNGER_STATE(PLUNGER_STATE_IDLE);
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
      SET_PLUNGER_STATE(PLUNGER_STATE_IDLE);
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


//////// Display ////////

void printBit(uint8_t bit) {
  if (bit) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
    display.print(F("1"));
    display.setTextColor(WHITE);
  } else {
    display.setTextColor(WHITE);
    display.print(F("0"));
  }
}

//void displayPrint_F(Adafruit_SSD1306 d, const char * str) {
void displayPrint_P(const char * str) {
  char c;
  //char buffer[SCREEN_BUFFER_SIZE];

  if (!str)
    return;
//  strcpy_P(buffer, str);
//  d.print(buffer);
  while ((c = pgm_read_byte(str++)))
    display.print(c);
}


#define MENUITEM_COUNT (sizeof(menuItems)/sizeof(MenuItem))
#define MENUITEM_UIMODE(idx) ((uint8_t)pgm_read_byte(&menuItems[idx].UIMode))
#define MENUITEM_NAME(idx) ((const char*)&menuItems[idx].name)
#define SCREEN_MENU_ROWS 4

int8_t menuSelectIdx = 0;
int8_t menuScrollIdx = 0;

void displayScreenMenu() {
  // check the buttons
  int8_t highlightPos = menuSelectIdx - menuScrollIdx;
  switch (buttonEventGet()) {
  case BUTTON_EVENT_SHORT_SELECT: {
    // select the menu item cursor
    UIMode = MENUITEM_UIMODE(menuSelectIdx);
    break;
    }
  case BUTTON_EVENT_SHORT_BACK: {
    //Go back to menu screen
    UIMode = UI_SCREEN_HUD;
    break;
    }
  case BUTTON_EVENT_SHORT_UP: {
    // prev menuitem
    if (menuSelectIdx > 0) {
      menuSelectIdx--;
      highlightPos = menuSelectIdx - menuScrollIdx;
      if (highlightPos < 0) {
        menuScrollIdx--;
        highlightPos = menuSelectIdx - menuScrollIdx;
      }
    }
    break;
    }
  case BUTTON_EVENT_SHORT_DOWN: {
    // next parameter
    if (menuSelectIdx < (MENUITEM_COUNT - 1)) {
      menuSelectIdx++;
      highlightPos = menuSelectIdx - menuScrollIdx;
      if (highlightPos >= SCREEN_MENU_ROWS) {
        menuScrollIdx++;
        highlightPos = menuSelectIdx - menuScrollIdx;
      }
    }
    break;
    }
  }

  // draw the config screen
  //screenDrawTitle((const char *)F("System Config"));
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  for(uint8_t i = 0; i < SCREEN_MENU_ROWS; i++) {
    //Compute which screen line to highlight
    if (i == highlightPos) {
      display.setTextColor(BLACK, WHITE); // 'inverted' text
    } else {
      display.setTextColor(WHITE);
    }
    displayPrint_P(MENUITEM_NAME(menuScrollIdx + i));
    //display.println(F(""));
  }
}


#define SCREEN_UNDERLINE_POS  8
#define SCREEN_CONFIG_ROWS 6

void screenDrawTitle(const char * title) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // draw scren title
  display.setCursor(0, 0);
  displayPrint_P(title);
  display.drawLine(0, SCREEN_UNDERLINE_POS, display.width() - 1, SCREEN_UNDERLINE_POS, WHITE);
  display.setCursor(0, SCREEN_UNDERLINE_POS+2);
}

#define PARAM_COUNT (sizeof(configParams)/sizeof(ConfigParam))
#define PARAM_MAX(idx) ((int16_t)pgm_read_word(&configParams[configSelectIdx].valueMax))
#define PARAM_MIN(idx) ((int16_t)pgm_read_word(&configParams[configSelectIdx].valueMin))
#define PARAM_STEP(idx) ((int16_t)pgm_read_word(&configParams[configSelectIdx].valueStep))
#define PARAM_DEFAULT(idx) ((int16_t)pgm_read_word(&configParams[idx].valueStep))
#define PARAM_NAME(idx) ((const char*)&configParams[idx].name)

int8_t configSelectIdx = 0;
int8_t configScrollIdx = 0;

int16_t paramRead(uint8_t paramIdx) {
  uint16_t val;
  val = (uint16_t)EEPROM.read(paramIdx * sizeof(int16_t));
  val |= ((uint16_t)(EEPROM.read(paramIdx * sizeof(int16_t) + 1)) << 8);
  return (int16_t)val;
}

int16_t paramWrite(uint8_t paramIdx, int16_t val) {
  int16_t valRead = paramRead(paramIdx);
  if (valRead != val) {
    EEPROM.write(paramIdx * sizeof(int16_t), (uint8_t)((uint16_t)val & 0x00FF));
    EEPROM.write(paramIdx * sizeof(int16_t) + 1, (uint8_t)((uint16_t)val >> 8));
  }
}

void paramDefaultCheck(void) {
  if (paramRead(PARAM_RESET_ALL) != 0) {
    // parameters need to be initialiszed or reset.
    // Copy default values from flash.
    for(uint8_t i = 0; i < PARAM_COUNT; i++) {
      paramWrite(i, PARAM_DEFAULT(i));

      Serial.print(F(" param:"));
      SerialPrint_F(PARAM_NAME(i));
      Serial.print(F(" default="));
      Serial.print(PARAM_DEFAULT(i), DEC);
      Serial.print(F(" read="));
      Serial.println(paramRead(i), DEC);
    }
  }
}

void paramInit(void) {
  paramDefaultCheck();
}

void displayScreenConfig() {
  // check the buttons
  int8_t highlightPos = configSelectIdx - configScrollIdx;
  switch (buttonEventGet()) {
  case BUTTON_EVENT_SHORT_SELECT: {
    if (UIMode == UI_SCREEN_CONFIG_EDIT) {
      paramDefaultCheck();
      UIMode = UI_SCREEN_CONFIG;
    } else {
      //select the parameter unter the curcur for editing
      UIMode = UI_SCREEN_CONFIG_EDIT;
    }
    break;
    }
  case BUTTON_EVENT_SHORT_BACK: {
    //Go back to menu screen
    if (UIMode == UI_SCREEN_CONFIG_EDIT) {
      paramDefaultCheck();
      UIMode = UI_SCREEN_CONFIG;
    } else {
      UIMode = UI_SCREEN_MENU;
    }
    break;
    }
  case BUTTON_EVENT_SHORT_UP: {
    if (UIMode == UI_SCREEN_CONFIG_EDIT) {
      // increase parameter value
      int16_t paramTemp = paramRead(configSelectIdx);
      paramTemp += PARAM_STEP(configSelectIdx);
      if (paramTemp > PARAM_MAX(configSelectIdx)) {
        paramTemp = PARAM_MAX(configSelectIdx);
      }
      paramWrite(configSelectIdx, paramTemp);
    } else {
      // prev parameter
      if (configSelectIdx > 0) {
        configSelectIdx--;
        highlightPos = configSelectIdx - configScrollIdx;
        if (highlightPos < 0) {
          configScrollIdx--;
          highlightPos = configSelectIdx - configScrollIdx;
        }
      }
    }
    break;
    }
  case BUTTON_EVENT_SHORT_DOWN: {
    if (UIMode == UI_SCREEN_CONFIG_EDIT) {
      // decrease parameter value
      int16_t paramTemp = paramRead(configSelectIdx);
      paramTemp -= PARAM_STEP(configSelectIdx);
      if (paramTemp < PARAM_MIN(configSelectIdx)) {
        paramTemp = PARAM_MIN(configSelectIdx);
      }
      paramWrite(configSelectIdx, paramTemp);
    } else {
      // next parameter
      if (configSelectIdx < (PARAM_COUNT - 1)) {
        configSelectIdx++;
        highlightPos = configSelectIdx - configScrollIdx;
        if (highlightPos >= SCREEN_CONFIG_ROWS) {
          configScrollIdx++;
          highlightPos = configSelectIdx - configScrollIdx;
        }
      }
    }
    break;
    }
  }

  // draw the config screen
  screenDrawTitle((const char *)F("System Config"));
  for(uint8_t i = 0; i < SCREEN_CONFIG_ROWS; i++) {
    //Compute which screen line to highlight
    if ((UIMode == UI_SCREEN_CONFIG) && (i == highlightPos)) {
      display.setTextColor(BLACK, WHITE); // 'inverted' text
    } else {
      display.setTextColor(WHITE);
    }
    //displayPrint_P((const char*)&configParams[configScrollIdx + i].name);
    displayPrint_P(PARAM_NAME(configScrollIdx + i));
    display.print(F(":"));

    int16_t paramPrint = paramRead(configScrollIdx + i);
    if (i == highlightPos) {
      display.setTextColor(BLACK, WHITE); // 'inverted' text
    } else {
      display.setTextColor(WHITE);
    }
    display.print(paramPrint, DEC);
    if(paramPrint < 1000) {display.print(F(" "));}
    if(paramPrint < 100) {display.print(F(" "));}
    if(paramPrint < 10) {display.print(F(" "));}
    //display.println(F(""));
  }
}


void displayScreenDiag() {
  // dispay the diagnostic screen
  switch (buttonEventGet()) {
  case BUTTON_EVENT_SHORT_BACK: {
    //Go back to menu screen
    UIMode = UI_SCREEN_MENU;
    break;
    }
  }

  // draw screen
  screenDrawTitle((const char *)F("System Diagonstic"));

  display.print(F("Volt=")); display.println(voltageBatteryAvg, 1);
  display.print(F("Rev=")); printBit(switchRevTriggerRead()); display.print(F(" "));
  display.print(F("Tgr=")); printBit(switchTriggerRead()); display.print(F(" "));
  display.print(F("Brl=")); printBit(sensorBarrelRead()); display.println(F(""));
  display.print(F("Jam=")); printBit(switchJamDoorRead()); display.print(F(" "));
  display.print(F("Mag=")); printBit(switchMagSafetyRead()); display.println(F(""));
  display.print(F("MagBits="));
    uint8_t bits = magTypeReadBits();
    printBit(bitRead(bits, 0));
    printBit(bitRead(bits, 1));
    printBit(bitRead(bits, 2));
    printBit(bitRead(bits, 3));
    display.print(F("="));
    displayPrint_P(magazineTypes[magazineTypeIdx].name); display.println(F(""));
}


#define POSX_SEVEN_SEG_DIGIT_0  66
#define POSX_SEVEN_SEG_DIGIT_1  98
#define POSY_SEVEN_SEG_DIGIT  12

void displayScreenHUD() {
  switch (buttonEventGet()) {
  case BUTTON_EVENT_SHORT_SELECT: {
    //advance to config screen
    UIMode = UI_SCREEN_MENU;
    break;
    }
  }

  // Draw the HUD Display
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println(F("Rapid"));
  display.setTextSize(1);
  display.print(F("Mag:"));
  displayPrint_P(magazineTypes[magazineTypeIdx].name); display.println(F(""));
  display.print(F("Rd/m:"));
  if (roundsPerMin > 0) {
    display.println(roundsPerMin, DEC);
  } else {
    display.println(F("---"));
  }
  display.print(F("Ft/s:"));
  if (velocity >= 0.0) {
    display.println(velocity, 1);
  } else {
    display.println(F("---"));
  }
  display.print(F("Volt:"));
  display.println(voltageBatteryAvg, 1);

  display.print(F("JamCount:"));
  display.println(roundsJamCount, DEC);

  // draw the round digits.
  int digit0 = SEVEN_SEGMENT_BITMAP_DASH;
  int digit1 = SEVEN_SEGMENT_BITMAP_DASH;
  if ((roundCount >= 0) && (roundCount <= 99)) {
    digit0 = roundCount / 10;
    digit1 = roundCount % 10;
  }
  display.setCursor(86, 0);
  display.println(F("Rounds:"));
  display.drawBitmap(POSX_SEVEN_SEG_DIGIT_0, POSY_SEVEN_SEG_DIGIT,
      (uint8_t *) &(SevenSegmentBitMaps[digit0]),
      SEVEN_SEGMENT_BITMAP_WIDTH, SEVEN_SEGMENT_BITMAP_HEIGHT, 1);
  display.drawBitmap(POSX_SEVEN_SEG_DIGIT_1, POSY_SEVEN_SEG_DIGIT,
      (uint8_t *) &(SevenSegmentBitMaps[digit1]),
      SEVEN_SEGMENT_BITMAP_WIDTH, SEVEN_SEGMENT_BITMAP_HEIGHT, 1);

  // draw the jam door indicator or the feed jam indicator
  if (jamDoorOpen || feedJam) {
    //fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color),
    display.fillRoundRect(12, 20, 102, 23, 3, WHITE),
    display.drawRoundRect(12, 20, 102, 23, 3, BLACK),
    display.setTextSize(2);
    display.setCursor(16, 24);
    display.setTextColor(BLACK, WHITE); // 'inverted' text
    if (jamDoorOpen) {
      display.print(F("Jam:Door"));
    } else if (feedJam) {
      display.print(F("Jam:Feed"));
    }
  }
}


void displayUpdate() {
  display.dim(paramRead(PARAM_DISPLAY_DIM));

  switch (UIMode) {
  case UI_SCREEN_CONFIG:
  case UI_SCREEN_CONFIG_EDIT:
    displayScreenConfig();
    break;
  case UI_SCREEN_DIAGNOSTIC:
    displayScreenDiag();
    break;
  case UI_SCREEN_MENU:
    displayScreenMenu();
    break;
  default:
  case UI_SCREEN_HUD:
    displayScreenHUD();
    break;
  }
  display.display();
}
