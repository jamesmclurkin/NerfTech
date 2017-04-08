// NerfCommp control code

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP23008.h>
#include <Servo.h>
#include <avr/pgmspace.h>

#define ARDUINO_NANO
//#define ARDUINO_PROMICRO

#define SCREEN_ENABLE
#define SCREEN_GPIO_ENABLE
#define MAG_GPIO_ENABLE

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
#ifdef MAG_GPIO_ENABLE
Adafruit_MCP23008 GPIO_mag;
#endif
#ifdef SCREEN_GPIO_ENABLE
Adafruit_MCP23008 GPIO_UI;
#endif

#define CONFIG_PARAM_NAME_SIZE 17

typedef struct ConfigParam {
  const char name[CONFIG_PARAM_NAME_SIZE];
  const int16_t valueDefault;
  const int16_t valueMin;
  const int16_t valueMax;
  const int16_t valueStep;
} ConfigParam;

const ConfigParam configParams[] PROGMEM = {
    //012345678901234567890
  //("  System Config  XXXX"));
    {"       Rev speed", FLYWHEEL_MOTOR_ESC_RUN,  100,  200, 5 },
    {"   Rev time semi", FLYWHEEL_REVUP_TIME_SEMI,  0, 1000, 50},
    {"   Rev time full", FLYWHEEL_REVUP_TIME_FULL,  0, 1000, 50},
    {"   Plunger speed", PLUNGER_PWM_RUN_SPEED,    50,  250, 5 },
    {"     Dart sength", DART_LENGTH_MM,           10,  200, 5 },
    {"Reset to default", 0,                         0,    1, 1 },
};


//////// I/O wrappers ////////

boolean magazineSwitchRead() {return !digitalRead(PIN_SAFETY_MAG); }

boolean revTriggerRead() {return !digitalRead(PIN_FLYWHEEL_TRIGGER); }

boolean jamDoorRead() {return !digitalRead(PIN_SAFETY_JAMDOOR); }

boolean triggerRead() {return !digitalRead(PIN_PLUNGER_TRIGGER); }

boolean plungerEndRead() {return !digitalRead(PIN_PLUNGER_END_SWITCH); }

boolean barrelRead() {return digitalRead(PIN_BARREL_START); }

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
  if (magazineSwitchRead()) {
    #ifdef MAG_GPIO_ENABLE
    magTypeBits = GPIO_mag.readGPIO();
    #else
    uint8_t magTypeBits = 0xFF;
    #endif
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

#define BARREL_START                0
#define BARREL_END                  1
#define BARREL_INTER_DART_TIME_US   10000L
#define BARREL_DART_TIME_US         1000L

volatile uint8_t barrelIRQState = BARREL_START;

void irqBarrelStart() {
  //debug_barrelStart = true;
  if (barrelIRQState == BARREL_START) {
    unsigned long time = micros();
    if (time > (timeBarrelStart + BARREL_INTER_DART_TIME_US)) {
      // rising edge = new dart, and it's been long enough that it's not a glitch
      timeBarrelStart = time;
      barrelIRQState = BARREL_END;
      //debug_barrelStart_glitch = true;
    }
  }
}

void irqBarrelEnd() {
  //debug_barrelEnd = true;
  if (barrelIRQState == BARREL_END) {
    unsigned long time = micros();
    if (time > (timeBarrelStart + BARREL_DART_TIME_US)) {
      // falling edge = end of the dart, and it's been long enough that it's not a glitch
      timeBarrelEnd = time;
      timeBarrelEndFlag = true;
      barrelIRQState = BARREL_START;
      //debug_barrelEnd_glitch = true;
    }
  }
}


//////// user Interface ////////

#define UI_SCREEN_HUD         0
#define UI_SCREEN_DIAGNOSTIC  1
#define UI_SCREEN_CONFIG      2
#define UI_SCREEN_CONFIG_EDIT 3

#define OLED_RESET 4
#ifdef SCREEN_ENABLE
Adafruit_SSD1306 display(OLED_RESET);
#endif

uint8_t UIMode = UI_SCREEN_CONFIG;


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

// UI buttons
//boolean buttonUnpackUp(uint8_t buttonBits) { return buttonBits & BUTTON_UP_BIT; }
//boolean buttonUnpackDown(uint8_t buttonBits) { return buttonBits & BUTTON_DOWN_BIT; }
//boolean buttonUnpackSelect(uint8_t buttonBits) { return buttonBits & BUTTON_SELECT_BIT; }
//boolean buttonUnpackBack(uint8_t buttonBits) { return buttonBits & BUTTON_BACK_BIT; }

uint8_t _buttonRead() {
  uint8_t buttonBits;
#ifdef SCREEN_GPIO_ENABLE
  buttonBits = ~GPIO_UI.readGPIO();
#else
  buttonBits = 0x00;
#endif
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
#ifdef MAG_GPIO_ENABLE
  GPIO_mag.begin(0);      // GPIO magazine is on address 0
  for (int i = 0; i < 8; ++i) {
    GPIO_mag.pinMode(i, INPUT);
    GPIO_mag.pullUp(i, HIGH);  // turn on a 100K pullup internally
  }
#endif
#ifdef SCREEN_GPIO_ENABLE
  GPIO_UI.begin(1);       // GPIO foor the UI buttons is on address 1
  for (int i = 0; i < 8; ++i) {
    GPIO_UI.pinMode(i, INPUT);
    GPIO_UI.pullUp(i, HIGH);  // turn on a 100K pullup internally
  }
#endif


  plungerMotorInit();

  // init the servo
  servoESC.attach(PIN_FLYWHEEL_ESC); // attaches the servo on pin 9 to the servo object
  servoESC.write(FLYWHEEL_MOTOR_ESC_NEUTRAL);

  // init the serial port for debugging output
  Serial.begin(115200);
  Serial.println(F(" NerfComp: RapidStrike ver 0.5"));
  Serial.print(F("   Free RAM:")); Serial.print(freeRam()); Serial.println(F(" bytes"));


  // init the LED Display
#ifdef SCREEN_ENABLE

  // generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C

  // Clear the display buffer and put up the splash screen
  display.clearDisplay();
  display.drawBitmap(0, 0, NerfLogoBitmap, NERF_LOGO_BITMAP_WIDTH, NERF_LOGO_BITMAP_HEIGHT, 1);
  display.display();

  // show the screen for a while
  delay(2000);
#endif

  // force an update to show  the initial data display
  displayUpdate();

#define MAG_TYPE_TEST 5

//  Serial.print(F("Mag type Test:"));
//  SerialPrint_P(magazineTypes[MAG_TYPE_TEST].name);
//  Serial.print(F(","));
//  Serial.print(magazineTypes[MAG_TYPE_TEST].capacity, DEC);
//  Serial.print(F(","));
//  Serial.print(pgm_read_byte(&magazineTypes[MAG_TYPE_TEST].capacity), DEC);
//  Serial.println(F(""));

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
#ifdef MAG_GPIO_ENABLE
    if (p) {Serial.print(F("  magbits=")); Serial.print(GPIO_mag.readGPIO(), HEX); }
#endif
#ifdef SCREEN_GPIO_ENABLE
    if (p) {Serial.print(F("  uibits=")); Serial.print(GPIO_UI.readGPIO(), HEX); }
#endif
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
      boolean jamDoorOpenTemp = jamDoorRead();
      if (p) {Serial.print(F(" jamdoor=")); Serial.print(jamDoorOpenTemp);}
      if (jamDoorOpen != jamDoorOpenTemp) {
        if (jamDoorOpenTemp) {
          // jam door has gone from closed to open.  inc the jam count
          roundsJamCount++;
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
    if (revTriggerRead()) {
      ESCPos = FLYWHEEL_MOTOR_ESC_PRE_RUN_FULL;
    } else {
      ESCPos = FLYWHEEL_MOTOR_ESC_BRAKE;
    }
    if (triggerRead() && STATE_DELAY(TRIGGER_DELAY_TIME)) {
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
    if (revTriggerRead()) {
      revTime = FLYWHEEL_REVUP_TIME_FULL;
    } else {
      revTime = FLYWHEEL_REVUP_TIME_SEMI;
    }
    if (STATE_DELAY(revTime)) {
      SET_PLUNGER_STATE(PLUNGER_STATE_CLEAR_END_SWITCH);
    }
    break;
  }
  case PLUNGER_STATE_CLEAR_END_SWITCH: {
    if (sp) {Serial.println(F(" s:clear")); sp = false;}
    plungerMotorPWM(MOTOR_DIR_FWD, PLUNGER_PWM_RUN_SPEED);
    ESCPos = FLYWHEEL_MOTOR_ESC_RUN;
    displayUpdateEnable = false;
    if (!plungerEndRead()) {
      SET_PLUNGER_STATE(PLUNGER_STATE_RUN_PLUNGER);
      //displayUpdateForce = true;
    }
    break;
  }
  case PLUNGER_STATE_RUN_PLUNGER: {
    if (sp) {Serial.println(F(" s:run")); sp = false;}
    plungerMotorPWM(MOTOR_DIR_FWD, PLUNGER_PWM_RUN_SPEED);
    ESCPos = FLYWHEEL_MOTOR_ESC_RUN;
    displayUpdateEnable = false;
    if (plungerEndRead() && STATE_DELAY(25)) {
      // plunger is at end.  stop it if we're in semi-auto
      if (!revTriggerRead()) {
        // semi auto.  stop the plunger
        SET_PLUNGER_STATE(PLUNGER_STATE_WAIT_FOR_TRIGGGER_RELEASE);
      } else {
        SET_PLUNGER_STATE(PLUNGER_STATE_ROUND_DELAY);
      }
    }
    break;
  }
  case PLUNGER_STATE_ROUND_DELAY: {
    if (sp) {Serial.println(F(" s:delay")); sp = false;}
    plungerMotorPWM(MOTOR_DIR_BRAKE, PLUNGER_PWM_MAX);
    // full auto.  leave the flywheel spinning
    ESCPos = FLYWHEEL_MOTOR_ESC_RUN;
    displayUpdateEnable = false;
    if ((millis() - plungerStateTime) > ROUND_DELAY_TIME) {
      //displayUpdateForce = true;
      if (triggerRead()) {
        // chamber another round
        SET_PLUNGER_STATE(PLUNGER_STATE_CLEAR_END_SWITCH);
      }
    }
    if (STATE_DELAY(FLYWHEEL_REVDOWN_TIME_FULL)) {
      // rev down the flywheel to idle speed until the trigger is pulled again
      SET_PLUNGER_STATE(PLUNGER_STATE_IDLE);
    }
    if (!revTriggerRead()) {
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
    if (!triggerRead()) {
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

#define SCREEN_BUFFER_SIZE  40
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

#define SCREEN_UNDERLINE_POS  8
#define SCREEN_CONFIG_ROWS 3

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

int8_t selectIdx = 0;
int8_t scrollIdx = 0;
boolean p = true;

void displayScreenConfig() {
//  display.clearDisplay();
//  display.setTextSize(1);
//  display.setTextColor(WHITE);
//  // draw scren title
//  display.setCursor(0, 0);
//  display.print(F("System Config"));
//  display.drawLine(0, SCREEN_UNDERLINE_POS, display.width() - 1, SCREEN_UNDERLINE_POS, WHITE);
//  display.setCursor(0, SCREEN_UNDERLINE_POS+2);

  // check the buttons
  uint8_t paramCount = sizeof(configParams)/sizeof(ConfigParam);
  int8_t highlightPos = selectIdx - scrollIdx;
  switch (buttonEventGet()) {
  case BUTTON_EVENT_SHORT_SELECT: {
    //select the parameter unter the curcur for editing
    UIMode = UI_SCREEN_CONFIG_EDIT;
    break;
    }
  case BUTTON_EVENT_SHORT_BACK: {
    //Go back to disg screen
    UIMode = UI_SCREEN_DIAGNOSTIC;
    break;
    }
  case BUTTON_EVENT_SHORT_UP: {
    // prev parameter or increase value
    if (selectIdx > 0) {
      selectIdx--;
      highlightPos = selectIdx - scrollIdx;
      if (highlightPos < 0) {
        scrollIdx--;
        highlightPos = selectIdx - scrollIdx;
      }
    }
    p = true;
    break;
    }
  case BUTTON_EVENT_SHORT_DOWN: {
    // next parameter or decrease value
    if (selectIdx < (paramCount - 1)) {
      selectIdx++;
      highlightPos = selectIdx - scrollIdx;
      if (highlightPos >= SCREEN_CONFIG_ROWS) {
        scrollIdx++;
        highlightPos = selectIdx - scrollIdx;
      }
    }
    p = true;
    break;
    }
  }

  screenDrawTitle((const char *)F("System Config"));

  if (p) {
  Serial.print(F(" paramCount=")); Serial.print(paramCount, DEC);
  Serial.print(F(" selectIdx=")); Serial.print(selectIdx, DEC);
  Serial.print(F(" scrollIdx=")); Serial.print(scrollIdx, DEC);
  Serial.print(F(" highlightPos=")); Serial.print(highlightPos, DEC);
  Serial.println(F(""));
  p = false;
  }

  //Compute which screen line to highlight
  for(uint8_t i = 0; i < SCREEN_CONFIG_ROWS; i++) {
    if (i == highlightPos) {
      display.setTextColor(BLACK, WHITE); // 'inverted' text
    } else {
      display.setTextColor(WHITE);
    }
    displayPrint_P((const char*)&configParams[scrollIdx + i].name);
    //displayPrint_P(display, (const char*)F("       Rev speed"));
    display.print(F(":"));
    display.print((int16_t)pgm_read_word(&configParams[scrollIdx + i].valueDefault), DEC);
    display.println(F(""));
  }
  display.display();
}

//    // check the UI buttons
//    switch (UIMode) {
//    case UI_SCREEN_HUD:
//      if (buttonRisingEdge(BUTTON_BIT_SELECT)) {
//        UIMode = UI_SCREEN_DIAGNOSTIC;
//      }
//      break;
//    case UI_SCREEN_DIAGNOSTIC:
//      if (buttonRisingEdge(BUTTON_BIT_SELECT)) {
//        UIMode = UI_SCREEN_CONFIG;
//      }
//      if (buttonRisingEdge(BUTTON_BIT_BACK)) {
//        UIMode = UI_SCREEN_HUD;
//      }
//      break;
//    case UI_SCREEN_CONFIG:
//      break;
//    default:
//        break;
//    }


void displayScreenDiag() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.setCursor(0, 0);
  display.print(F("System Diagonstic  "));

  display.setTextColor(WHITE);
  display.print(F("Volt=")); display.println(voltageBatteryAvg, 1);
  display.print(F("SW:Rev=")); printBit(revTriggerRead()); display.print(F(" "));
  display.print(F("Jam=")); printBit(jamDoorRead()); display.print(F(" "));
  display.print(F("Mag=")); printBit(magazineSwitchRead()); display.println(F(""));
  display.print(F("MagBits="));
    uint8_t bits = magTypeReadBits();
    printBit(bitRead(bits, 0));
    printBit(bitRead(bits, 1));
    printBit(bitRead(bits, 2));
    printBit(bitRead(bits, 3));
    display.print(F("="));
    displayPrint_P(magazineTypes[magazineTypeIdx].name); display.println(F(""));
  display.print(F("Barrel=")); printBit(barrelRead()); display.println(F(""));
  display.display();
}


#define POSX_SEVEN_SEG_DIGIT_0  66
#define POSX_SEVEN_SEG_DIGIT_1  98
#define POSY_SEVEN_SEG_DIGIT  12

void displayScreenHUD() {
#ifdef SCREEN_ENABLE
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
  if (jamDoorOpen) {
    // draw the jam text inverted if the door is open
    display.setTextColor(BLACK, WHITE); // 'inverted' text
  }
  display.print(F("Jam:"));
  display.println(roundsJamCount, DEC);
  display.setTextColor(WHITE);

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

  display.display();
#endif
}

void displayUpdate() {
  switch (UIMode) {
  case UI_SCREEN_CONFIG:
  case UI_SCREEN_CONFIG_EDIT:
    displayScreenConfig();
    break;
  case UI_SCREEN_DIAGNOSTIC:
    displayScreenDiag();
    break;
  default:
  case UI_SCREEN_HUD:
    displayScreenHUD();
    break;
  }
}
