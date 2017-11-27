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
#include <NerfCompIO.h>

// GPIO Port expander for mag type
Adafruit_MCP23008 GPIO_mag;

// servo object to control the flywheel ESC
Servo servoESC;

uint8_t magazineTypeCounter;
boolean magazineNew = false;

//////// I/O wrappers ////////
boolean switchMagSafetyRead() {return !digitalRead(PIN_SAFETY_MAG); }
boolean switchRevTriggerRead() {return !digitalRead(PIN_FLYWHEEL_TRIGGER); }
boolean switchJamDoorRead() {return !digitalRead(PIN_SAFETY_JAMDOOR); }
boolean switchTriggerRead() {return !digitalRead(PIN_PLUNGER_TRIGGER); }
boolean switchPlungerStopRead() {return !digitalRead(PIN_PLUNGER_END_SWITCH); }
boolean sensorBarrelRead() {return digitalRead(PIN_BARREL_START); }


//////// jam door sensor ////////
void jamDoorUpdate(boolean p) {
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
}
  
//////// magazine type sensor ////////
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
    if (paramReadInvertMag()) {
      magTypeBits = flipLowNibble(magTypeBits);
    }
  }
  return magTypeBits;
}

uint8_t magazineTypeLookup(uint8_t magTypeBits) {
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

void magazineTypeUpdate(boolean p) {
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
}
  

//////// battery Voltage ////////
void batteryVoltageUpdate(boolean p) {
    int voltageBatteryRaw = analogRead(PIN_BATTERY_VOLTAGE);
    float voltageBatteryTemp = (float) voltageBatteryRaw * VOLTAGE_BATTERY_SCALER;

    if (p) {Serial.print(F("  vbat=")); Serial.print(voltageBatteryRaw, DEC); }
    if (p) {Serial.print(F(",")); Serial.print(voltageBatteryTemp, 2); }
    // add some sanity checks to the voltage
    if ((voltageBatteryTemp >= VOLTAGE_MIN) && (voltageBatteryTemp <= VOLTAGE_MAX)) {
      // compute a IIR low-pass filter
      voltageBatteryAvg = VOLTAGE_BATTERY_IIR_GAIN * voltageBatteryTemp + (1 - VOLTAGE_BATTERY_IIR_GAIN) * voltageBatteryAvg;
    }
}
  


//////// barrel sensor ////////
#define BARREL_STATE_START                  0
#define BARREL_STATE_END                    1
#define BARREL_TIME_DART_INTERVAL_MAX_US    10000L
#define BARREL_TIME_DART_LENGTH_MAX_US      1000L

volatile uint8_t barrelIRQState = BARREL_STATE_START;

volatile unsigned long timeBarrelStart = 0;
volatile boolean timeBarrelStartFlag = false;

volatile unsigned long timeBarrelEnd = 0;
volatile boolean timeBarrelEndFlag = false;

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


// look for a dart past the barrel sensor and update the rounds
void dartCheck(void) {
  static unsigned long roundTimePrev = 0;
  
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

void gpioInit(void) {
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
}

uint8_t gpioGetMagSensorBits(void) {
  return GPIO_mag.readGPIO();  
}

void servoESCWrite(int ESCPos) {
  servoESC.write(ESCPos);  
}