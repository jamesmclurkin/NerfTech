// NerfComp control code for Stryfe
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


// global variables for main system status
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


//////// configuration parameters for Stryfe_semi ////////
#define PARAM_FLYWHEEL_MOTOR_ESC_RUN    0
#define PARAM_DART_LENGTH_MM            1
#define PARAM_DISPLAY_DIM               2
#define PARAM_INVERT_MAG                3
#define PARAM_RESET_ALL                 4

const ConfigParam configParams[] PROGMEM = {
  //012345678901234567890
//("   System Config:XXXX"));
  {"       Rev speed", FLYWHEEL_MOTOR_ESC_RUN,  100,  200, 5 }, // 0
  {"     Dart length", DART_LENGTH_MM,           10,  200, 1 }, // 1
  {"     Display dim", 0,                         0,    1, 1 }, // 2
  {"      Invert mag", 0,                         0,    1, 1 }, // 3
  {"    Reset config", 0,                         0,    1, 1 }, // 4
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



void setup() {
  gpioInit();
  
  // init the serial port for debugging output
  Serial.begin(115200);
  Serial.println(F(" NerfComp: Stryfe ver 0.7"));
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
unsigned long displayUpdateTime = 0;
boolean sp = true;

void loop() {
  static uint8_t magazineTypeCounter;
  static boolean magazineNew = false;
  static unsigned long roundTimePrev = 0;

  boolean displayUpdateEnable = true;
  boolean displayUpdateForce = false;


//  debugPrint(&debug_barrelStart, "_");
//  debugPrint(&debug_barrelStart_glitch, "S");
//  debugPrint(&debug_barrelEnd, ".");
//  debugPrint(&debug_barrelEnd_glitch, "E");

  // reset the barrel IRQ state to look for new darts
  if (micros() > (timeBarrelStart + BARREL_INTER_DART_TIME_US)) {
    // It's been a long time.  Reset the barrel IRQ state to look for new darts,
    // in case  it's stuck in the state looking for the end of darts
    barrelIRQState = BARREL_START;
  }
      
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
        if ((velocityTemp < VELOCITY_FPS_MIN) || (velocityTemp > VELOCITY_FPS_MAX)) {
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
      roundTimePrev = roundTimeh;
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
    if (p) {Serial.print(F("  rounds=")); Serial.print(roundCount, DEC); }


    // update the motor voltage
    uint16_t voltageMotorRaw = analogRead(PIN_VOLTAGE_BATTERY);
    float voltageMotorTemp = (float) voltageMotorRaw * VOLTAGE_BATTERY_SCALER;

    if (p) {Serial.print(F("  vmot=")); Serial.print(voltageMotorRaw, DEC); }
    if (p) {Serial.print(F(",")); Serial.print(voltageMotorTemp, 2); }
    // add some sanity checks to the voltage
    if ((voltageMotorTemp >= VOLTAGE_MIN) && (voltageMotorTemp <= VOLTAGE_MAX)) {
      // compute a IIR low-pass filter
      if (voltageBatteryAvg == 0.0) {
        voltageBatteryAvg = voltageMotorTemp;
      } else {
        voltageBatteryAvg = (float)VOLTAGE_BATTERY_IIR_GAIN * voltageMotorTemp + (1.0 - (float)VOLTAGE_BATTERY_IIR_GAIN) * voltageBatteryAvg;
      }
      voltageBattery = voltageMotorTemp;
    }

    // check the magazine type
    uint8_t magTypeTemp = magTypeRead();
    if (p) {Serial.print(F("  mag=")); Serial.print(magTypeTemp, DEC); }
    if (magazineType != magTypeTemp) {
      magazineTypeCounter = 0;
      magazineNew = true;
    } else {
      if (magazineTypeCounter < MAGAZINE_TYPE_DELAY) {
        magazineTypeCounter++;
      } else {
        if (magazineNew) {
          magazineTypePtr = (MagazineType*)magazineTypeLookup(magTypeTemp);
          roundCount = magazineTypePtr->capacity;
          roundsJamCount = 0;
          magazineNew = false;
          Serial.println(F("")); Serial.print(F("(new magazine "));
          Serial.print(magazineTypePtr->name); Serial.println(F(")"));
        }
      }
    }
    magazineType = magTypeTemp;
    if (p) {Serial.print(F(",")); Serial.print(magazineTypePtr->name);}

    // check the jam door
    if ((magazineTypePtr->code != MAGTYPE_EMPTY) && (magazineTypeCounter == MAGAZINE_TYPE_DELAY)) {
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

    // print diagnostic switch status on serial port
    //if (p) {Serial.print(F(" mag=")); Serial.print(magazineSwitchRead());}
    //if (p) {Serial.print(F(" revTrig=")); Serial.print(revTriggerRead());}
    //if (p) {Serial.print(F(" jam=")); Serial.print(jamDoorRead());}

    if (p) {Serial.print(F(" maggpio=")); Serial.print(GPIO_mag.readGPIO());}


    // check the UI buttons
    uint8_t buttonBits = buttonRead();

    switch (UIMode) {
    case UI_SCREEN_HUD:
      if (buttonRisingEdge(buttonBits, BUTTON_SELECT_BIT)) {
        UIMode = UI_SCREEN_DIAGNOSTIC;
      }
      break;
    case UI_SCREEN_DIAGNOSTIC:
      if (buttonRisingEdge(buttonBits, BUTTON_BACK_BIT)) {
        UIMode = UI_SCREEN_HUD;
      }
      break;
    default:
        break;
    }

    buttonUpdateHistory(buttonBits);
  }

  //if (magazineSwitchRead() && jamDoorRead()) {
  // Safety switches are ok.  process the rev and fire triggers

  int ESCPos = FLYWHEEL_MOTOR_ESC_NEUTRAL;
  // Stryfe is a semi-auto blaster.  when the rev trigger is pulled, rev the flywheel.  Simple
  if (revTriggerRead()) {
    ESCPos = FLYWHEEL_MOTOR_ESC_RUN;
  } else {
    ESCPos = FLYWHEEL_MOTOR_ESC_BRAKE;
  }
  ESCServo.write(ESCPos);

  if (p) {
    uint8_t bits = magBitsRead();
    Serial.print(" mag=");
    Serial.print(bitRead(bits, 0));
    Serial.print(bitRead(bits, 1));
    Serial.print(bitRead(bits, 2));
    Serial.print(bitRead(bits, 3));
  }

  if (p) {Serial.println(F(""));}

  if (((millis() > (displayUpdateTime + DISPLAY_UPDATE_PERIOD))
      & displayUpdateEnable) || displayUpdateForce) {
    displayUpdateTime = millis();
    displayUpdateForce = false;
    displayUpdate();
  }
}


void printBit(uint8_t bit) {
  if (bit) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
    display.print("1");
    display.setTextColor(WHITE);
  } else {
    display.setTextColor(WHITE);
    display.print("0");
  }
}

// display functions
void displayScreenDiag() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.setCursor(0, 0);
  display.print("  System Diagonstic  ");

  display.setTextColor(WHITE);
  display.print("Volt:"); display.println(voltageBatteryAvg, 1);
  display.print("SW Rev:"); printBit(revTriggerRead()); display.print(" ");
  display.print("Jam:"); printBit(jamDoorRead()); display.print(" ");
  display.print("Mag:"); printBit(magazineSwitchRead()); display.println("");
  display.print("MagBits:");
    uint8_t bits = magBitsRead();
    printBit(bitRead(bits, 0));
    printBit(bitRead(bits, 1));
    printBit(bitRead(bits, 2));
    printBit(bitRead(bits, 3));
    display.print("=");
    display.println(magazineTypePtr->name);
  display.print("barrel:"); printBit(barrelRead()); display.println("");
  display.display();
}

#define POSX_SEVEN_SEG_DIGIT_0  66
#define POSX_SEVEN_SEG_DIGIT_1  98
#define POSY_SEVEN_SEG_DIGIT  12

void displayScreenHUD() {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println("Stryfe");
  display.setTextSize(1);
  display.print("Mag:");
  display.println(magazineTypePtr->name);
  display.print("Rd/m:");
  if (roundsPerMin > 0) {
    display.println(roundsPerMin, DEC);
  } else {
    display.println("---");

  }
  display.print("Ft/s:");
  if (velocity >= 0.0) {
    display.println(velocity, 1);
  } else {
    display.println("---");
  }
  display.print("Volt:");
  display.println(voltageBatteryAvg, 1);
  if (jamDoorOpen) {
    // draw the jam text inverted if the door is open
    display.setTextColor(BLACK, WHITE); // 'inverted' text
  }
  display.print("Jam:");
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
  display.println("Rounds:");
  display.drawBitmap(POSX_SEVEN_SEG_DIGIT_0, POSY_SEVEN_SEG_DIGIT,
      (uint8_t *) &(SevenSegmentBitMaps[digit0]),
      SEVEN_SEGMENT_BITMAP_WIDTH, SEVEN_SEGMENT_BITMAP_HEIGHT, 1);
  display.drawBitmap(POSX_SEVEN_SEG_DIGIT_1, POSY_SEVEN_SEG_DIGIT,
      (uint8_t *) &(SevenSegmentBitMaps[digit1]),
      SEVEN_SEGMENT_BITMAP_WIDTH, SEVEN_SEGMENT_BITMAP_HEIGHT, 1);

  display.display();
}

void displayUpdate() {
  switch (UIMode) {
  case UI_SCREEN_DIAGNOSTIC:
    displayScreenDiag();
    break;
  default:
  case UI_SCREEN_HUD:
    displayScreenHUD();
    break;
  }
}
