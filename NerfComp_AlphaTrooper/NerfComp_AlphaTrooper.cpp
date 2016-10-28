// NerfComp AlphaTrooper control code

// Do not remove the include below
#include "NerfComp_AlphaTrooper.h"
#include <SPI.h>
#include <Wire.h>
#include "libraries\Adafruit_GFX.h"
#include "libraries\Adafruit_SSD1306.h"
#include "libraries\Adafruit_MCP23008.h"
#include <avr/pgmspace.h>

////////////////////////////////////////////////////////////////////////////////

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#include "SevenSegmentBitmaps.h"
#include "NerfLogo.h"

#define PIN_BARREL_START        2
#define PIN_BARREL_END          3
#define PIN_JAMDOOR             6
#define PIN_VOLTAGE_BATTERY       A7

#define PIN_BUTTON_UP           10
#define PIN_BUTTON_DOWN         12
#define PIN_BUTTON_SELECT       11
#define PIN_BUTTON_BACK         8

#define BUTTON_UP_BIT           1
#define BUTTON_DOWN_BIT         2
#define BUTTON_SELECT_BIT       4
#define BUTTON_BACK_BIT         8

#define VOLTAGE_BATTERY_SCALER    (1/65.8)
#define VOLTAGE_BATTERY_IIR_GAIN  0.01

#define VELOCITY_FPS_MIN                20.0
#define VELOCITY_FPS_MAX                400.0
#define DART_LENGTH_INCHES              2.85

#define HEARTBEAT_UPDATE_PERIOD         50
#define HEARTBEAT_PRINT_PERIOD          500

#define VOLTAGE_MIN         0.0
#define VOLTAGE_MAX         15.0

#define PIN_MAGTYPE_BIT0       1
#define PIN_MAGTYPE_BIT1       2
#define PIN_MAGTYPE_BIT2       3
#define PIN_MAGTYPE_BIT3       4

#define MAGTYPE_EMPTY     0
#define MAGTYPE_CLIP_6    1
#define MAGTYPE_CLIP_10   4
#define MAGTYPE_CLIP_12   2
#define MAGTYPE_CLIP_15   3
#define MAGTYPE_CLIP_18   8
#define MAGTYPE_DRUM_18   9
#define MAGTYPE_DRUM_25   10
#define MAGTYPE_DRUM_35   12
#define MAGTYPE_UNKNOWN   16

#define MAGAZINE_TYPE_DELAY   3

#define DISPLAY_UPDATE_PERIOD 100


typedef struct MagazineType {
  const uint8_t code;
  const char* name;
  const uint8_t capacity;
} MagazineType;

MagazineType const magazineTypes[] = {
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


// global variables for main system status
uint8_t magazineType = MAGTYPE_EMPTY;
MagazineType* magazineTypePtr = (MagazineType*)&magazineTypes[MAGTYPE_EMPTY];
uint8_t roundCount = -1;
int16_t roundsPerMin = -1;
uint8_t roundsJamCount = 0;
float velocity = -1;
float voltageBattery = 0.0;
float voltageBatteryAvg = 0.0;
boolean jamDoorOpen = false;

volatile unsigned long timeBarrelStart = 0;
volatile boolean timeBarrelStartFlag = false;

volatile unsigned long timeBarrelEnd = 0;
volatile boolean timeBarrelEndFlag = false;

unsigned long heartbeatUpdateTime = 0;
unsigned long heartbeatPrintTime = 0;

// GPIO Port expanders for mag type and HUD Screen UI
Adafruit_MCP23008 magGPIO;
Adafruit_MCP23008 hudGPIO;

// forward function declatations
void displayUpdate(void);



int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// switches
boolean jamDoorRead() { return digitalRead(PIN_JAMDOOR); }

// magazine bits
uint8_t magBitsRead() {
  uint8_t bits = magGPIO.readGPIO();
  return ((~bits) >> 4) & 0x0f;
}

// UI buttons
boolean buttonUpRead() { return !digitalRead(PIN_BUTTON_UP); }
boolean buttonDownRead() { return !digitalRead(PIN_BUTTON_DOWN); }
boolean buttonSelectRead() { return !digitalRead(PIN_BUTTON_SELECT); }
boolean buttonBackRead() { return !digitalRead(PIN_BUTTON_BACK); }

boolean barrelRead() { return digitalRead(PIN_BARREL_START); }

MagazineType* magazineTypeLookup(int magTypeVal) {
  uint8_t typeIdx = 0;
  const MagazineType* tempPtr;

  do {
    tempPtr = &magazineTypes[typeIdx];
    if (tempPtr->code == magTypeVal) {
      // found the correct magazine type.  break and return.
      break;
    }
    typeIdx++;
  } while (tempPtr->code != MAGTYPE_UNKNOWN);
  return (MagazineType*)tempPtr;
}


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

uint8_t magTypeRead() {
  uint8_t magType;
  magType = magBitsRead();
  return magType;
}

// UI stuff
#define UI_SCREEN_HUD         0
#define UI_SCREEN_DIAGNOSTIC  1

uint8_t UIMode = UI_SCREEN_HUD;

uint8_t buttonBitsOld1 = 0;
uint8_t buttonBitsOld2 = 0;

uint8_t buttonRead() {
  uint8_t buttonBits = 0;
  if (buttonUpRead())     {buttonBits |= BUTTON_UP_BIT;}
  if (buttonDownRead())   {buttonBits |= BUTTON_DOWN_BIT;}
  if (buttonSelectRead()) {buttonBits |= BUTTON_SELECT_BIT;}
  if (buttonBackRead())   {buttonBits |= BUTTON_BACK_BIT;}
  return buttonBits;
}

boolean buttonUnpackUp(uint8_t buttonBits) { return buttonBits & BUTTON_UP_BIT; }
boolean buttonUnpackDown(uint8_t buttonBits) { return buttonBits & BUTTON_DOWN_BIT; }
boolean buttonUnpackSelect(uint8_t buttonBits) { return buttonBits & BUTTON_SELECT_BIT; }
boolean buttonUnpackBack(uint8_t buttonBits) { return buttonBits & BUTTON_BACK_BIT; }

boolean buttonRisingEdge(uint8_t buttonBits, uint8_t buttonBitMask) {
  if ((!(buttonBitsOld2 & buttonBitMask)) &&
      (buttonBitsOld1 & buttonBitMask) &&
      (buttonBits & buttonBitMask)) {
    return true;
  } else {
    return false;
  }
}

void buttonUpdateHistory(uint8_t buttonBits) {
  buttonBitsOld2 = buttonBitsOld1;
  buttonBitsOld1 = buttonBits;
}

void setup() {
  // Setup the pins and interrupts for the barrel photo interrupters
  pinMode(PIN_BARREL_START, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BARREL_START), irqBarrelStart, RISING);

  pinMode(PIN_BARREL_END, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BARREL_END), irqBarrelEnd, FALLING);

  // Setup the pins for magazine type sensors and jam door sensor
  pinMode(PIN_JAMDOOR, INPUT_PULLUP);

  pinMode(PIN_BUTTON_UP, INPUT_PULLUP);
  pinMode(PIN_BUTTON_DOWN, INPUT_PULLUP);
  pinMode(PIN_BUTTON_SELECT, INPUT_PULLUP);
  pinMode(PIN_BUTTON_BACK, INPUT_PULLUP);


  // init the port expanders for mag type and HUD buttons
  magGPIO.begin(0);      // use default address 0
  hudGPIO.begin(1);
  for (int i = 0; i < 8; ++i) {
    magGPIO.pinMode(i, INPUT);
    magGPIO.pullUp(i, HIGH);  // turn on a 100K pullup internally
    hudGPIO.pinMode(i, INPUT);
    hudGPIO.pullUp(i, HIGH);  // turn on a 100K pullup internally
  }

  // init the serial port for debugging output
  Serial.begin(115200);
  Serial.println(F("NerfComp: Styrfe ver 0.1"));
  Serial.print(F("   Free RAM:")); Serial.print(freeRam()); Serial.println(F(" bytes"));

  // init the LED Display
  // generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C

  // Clear the display buffer and put up the splash screen
  display.clearDisplay();
  display.drawBitmap(0, 0, NerfLogoBitmap, NERF_LOGO_BITMAP_WIDTH, NERF_LOGO_BITMAP_HEIGHT, 1);
  display.display();

  // show the splash screen for a while
  delay(2000);

  // force an update to show  the initial data display
  displayUpdate();

  // reset the heartbeat time to avoid a bunch of initial updates
  heartbeatUpdateTime = millis();
  heartbeatPrintTime = heartbeatUpdateTime;
}


unsigned long displayUpdateTime = 0;
boolean sp = true;

void debugPrint(volatile boolean* valPtr, char* text) {
    if (*valPtr) {
        Serial.print(text);
        *valPtr = false;
    }
}


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
    if (p) {Serial.print(F(" jam=")); Serial.print(jamDoorRead());}
    if (p) {Serial.print(F(" maggpio=")); Serial.print(magGPIO.readGPIO());}
    if (p) {Serial.print(F(" hudgpio=")); Serial.print(hudGPIO.readGPIO());}

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
  display.print("Jam:"); printBit(jamDoorRead()); display.println(" ");
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
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("AlphaTrooper");
  display.setTextColor(WHITE);
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
  displayScreenDiag();
//  switch (UIMode) {
//  case UI_SCREEN_DIAGNOSTIC:
//    displayScreenDiag();
//    break;
//  default:
//  case UI_SCREEN_HUD:
//    displayScreenHUD();
//    break;
//  }
}
