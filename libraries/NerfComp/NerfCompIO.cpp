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