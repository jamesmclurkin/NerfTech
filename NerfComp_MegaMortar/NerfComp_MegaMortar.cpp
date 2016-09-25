// NerfComp MegaMortar control code

// Do not remove the include below
#include "NerfComp_MegaMortar.h"
#include <SPI.h>
#include <Wire.h>
#include "libraries\Adafruit_GFX.h"
#include "libraries\Adafruit_SSD1306.h"
#include "libraries\Adafruit_MCP23008.h"
#include <Servo.h>
#include <avr/pgmspace.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#include "SevenSegmentBitmaps.h"
#include "NerfLogo.h"

#define PIN_BARREL_START            2
#define PIN_BARREL_END              3
#define PIN_MAGSWITCH               4
#define PIN_PUMPTRIGGER              5
#define PIN_JAMDOOR                 6
#define PIN_TRIGGER                 12
#define PIN_PLUNGER_END             10
#define PIN_VOLTAGE_BATTERY         A7
#define PIN_DRIVE_MOTOR_FWD         7
#define PIN_DRIVE_MOTOR_REV         8
#define PIN_DRIVE_MOTOR_PWM         11

#define VOLTAGE_BATTERY_SCALER      (1/65.8)
#define VOLTAGE_BATTERY_IIR_GAIN    0.01

#define MOTOR_DIR_FWD               0
#define MOTOR_DIR_REV               1
#define MOTOR_DIR_BRAKE             2
#define MOTOR_DIR_OFF               3

#define PIN_PUMP_ESC      9

#define ROUND_STATE_IDLE              0
#define ROUND_STATE_VALVE_OPEN    1
#define ROUND_STATE_CLEAR_END_SWITCH  2
#define ROUND_STATE_RUN_PLUNGER       3
#define ROUND_STATE_ROUND_DELAY       4
#define ROUND_STATE_WAIT_FOR_TRIGGGER_RELEASE 5

#define PLUNGER_PWM_RUN_SPEED           145
#define PLUNGER_PWM_MAX                 255

#define TRIGGER_DELAY_TIME              100
#define ROUND_DELAY_TIME                100

#define PUMP_ESC_NEUTRAL      90
#define PUMP_MOTOR_ESC_PRE_RUN_FULL 96
#define PUMP_MOTOR_ESC_REVUP        160
#define PUMP_MOTOR_ESC_RUN          140
#define PUMP_MOTOR_ESC_BRAKE        15

#define VELOCITY_FPS_MIN                20.0
#define VELOCITY_FPS_MAX                400.0
//#define BARREL_LENGTH_CM                117
#define BARREL_LENGTH_INCHES            4.6063

#define HEARTBEAT_UPDATE_PERIOD         50
#define HEARTBEAT_PRINT_PERIOD          250

#define VOLTAGE_MIN         0.0
#define VOLTAGE_MAX         15.0

#define PIN_MAGTYPE_BIT0       A0
#define PIN_MAGTYPE_BIT1       A1
#define PIN_MAGTYPE_BIT2       A2
#define PIN_MAGTYPE_BIT3       A3

#define MAGTYPE_EMPTY        0
#define MAGTYPE_UNKNOWN      16
#define MAGAZINE_TYPE_DELAY     3

#define DISPLAY_UPDATE_PERIOD 100

#define SET_ROUND_STATE(s)  roundState = s;roundStateTime = millis();sp=true
#define STATE_DELAY(t)      ((millis() - roundStateTime) > t)

// global variables for main system stats
int roundCount = -1;
int roundsPerMin = -1;
int roundsJamCount = 0;
float velocity = -1;
float voltageBattery = 0.0;
float voltageBatteryAvg = 0.0;
boolean jamDoorOpen = false;

volatile unsigned long timeBarrelStart = 0;
volatile boolean timeBarrelStartFlag = false;

volatile unsigned long timeBarrelEnd = 0;
volatile boolean timeBarrelEndFlag = false;

void displayUpdate(void);

unsigned long heartbeatUpdateTime = 0;
unsigned long heartbeatPrintTime = 0;

// create servo object to control the flywheel ESC
Servo servoESC;
Adafruit_MCP23008 magIO;


boolean magazineSwitchRead() { return magIO.digitalRead(0); }

boolean revTriggerRead() { return !digitalRead(PIN_PUMPTRIGGER); }

boolean jamDoorRead() { return !digitalRead(PIN_JAMDOOR); }

boolean triggerRead() { return !digitalRead(PIN_TRIGGER); }

boolean plungerEndRead() { return !digitalRead(PIN_PLUNGER_END); }


#define BARREL_START    0
#define BARREL_END      1
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


void driveMotorPWM(int dir, int pwm) {
  int dirFwd, dirRev;

  switch (dir) {
  case MOTOR_DIR_FWD: {dirFwd = HIGH; dirRev = LOW;} break;
  case MOTOR_DIR_REV: {dirFwd = LOW; dirRev = HIGH;} break;
  case MOTOR_DIR_BRAKE: {dirFwd = HIGH; dirRev = HIGH; } break;
  case MOTOR_DIR_OFF: default: {dirFwd = LOW; dirRev = LOW; } break;
  }

  digitalWrite(PIN_DRIVE_MOTOR_FWD, dirFwd);
  digitalWrite(PIN_DRIVE_MOTOR_REV, dirRev);
  analogWrite(PIN_DRIVE_MOTOR_PWM, pwm);
}


void plungerMotorInit(void) {
  pinMode(PIN_DRIVE_MOTOR_FWD, OUTPUT);
  pinMode(PIN_DRIVE_MOTOR_REV, OUTPUT);
  pinMode(PIN_DRIVE_MOTOR_PWM, OUTPUT);
  driveMotorPWM(MOTOR_DIR_OFF, 0);
}


int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void setup() {
  // Setup the pins and interrupts for the barrel photo interrupters
  pinMode(PIN_BARREL_START, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_BARREL_START), irqBarrelStart, RISING);

  pinMode(PIN_BARREL_END, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_BARREL_END), irqBarrelEnd, RISING);

  // Setup the pins for magazine type sensors and jam door sensor
  pinMode(PIN_MAGTYPE_BIT0, INPUT);
  pinMode(PIN_MAGTYPE_BIT1, INPUT);
  pinMode(PIN_MAGTYPE_BIT2, INPUT);
  pinMode(PIN_MAGTYPE_BIT3, INPUT);

  pinMode(PIN_MAGSWITCH, INPUT_PULLUP);
  pinMode(PIN_PUMPTRIGGER, INPUT_PULLUP);
  pinMode(PIN_JAMDOOR, INPUT_PULLUP);

  pinMode(PIN_TRIGGER, INPUT_PULLUP);
  pinMode(PIN_PLUNGER_END, INPUT_PULLUP);

  plungerMotorInit();

  // init the servo
  servoESC.attach(PIN_PUMP_ESC); // attaches the servo on pin 9 to the servo object
  servoESC.write(PUMP_ESC_NEUTRAL);


  magIO.begin(0);
  magIO.pinMode(0, INPUT);

  Serial.begin(115200);
  Serial.println(" NerfComp: MegaMortar ver 0.1");
  Serial.print(F("   Free RAM:")); Serial.print(freeRam()); Serial.println(F(" bytes"));

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


int roundState = ROUND_STATE_IDLE;
unsigned long roundStateTime;
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
      Serial.print("round="); Serial.print(roundCount, DEC);
      roundCount--;
      // compute time of flight in microseconds
      unsigned long time = timeBarrelEnd - timeBarrelStart;
      Serial.print(" time="); Serial.print(time, DEC);

      // compute velocity in feet/sec. apply some sanity checks
      if (time > 0) {
        float velocityTemp = (BARREL_LENGTH_INCHES * 1000000) / (12 * (float) time);
        Serial.print(" vel="); Serial.print(velocityTemp, 1);
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
          Serial.print(" delta="); Serial.print(roundTimeDelta, DEC);
          Serial.print(",rpm="); Serial.print(rpmTemp, DEC);
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
      Serial.println("");
    }
  }


  // heartbeat updates.  Slow, around 50ms
  boolean p = false;
  if (millis() > heartbeatUpdateTime) {
    heartbeatUpdateTime += HEARTBEAT_UPDATE_PERIOD;
    if (millis() > heartbeatPrintTime) {
      heartbeatPrintTime += HEARTBEAT_PRINT_PERIOD;
      p = true;
    }
    p = false;
    if (p) {Serial.print("hb ");}
    if (p) {Serial.print("  rounds="); Serial.print(roundCount, DEC); }


    // update the motor voltage
    int voltageMotorRaw = analogRead(PIN_VOLTAGE_BATTERY);
    float voltageMotorTemp = (float) voltageMotorRaw * VOLTAGE_BATTERY_SCALER;

    if (p) {Serial.print("  vmot="); Serial.print(voltageMotorRaw, DEC); }
    if (p) {Serial.print(","); Serial.print(voltageMotorTemp, 2); }
    // add some sanity checks to the voltage
    if ((voltageMotorTemp >= VOLTAGE_MIN) && (voltageMotorTemp <= VOLTAGE_MAX)) {
      // compute a IIR low-pass filter
      voltageBatteryAvg = VOLTAGE_BATTERY_IIR_GAIN * voltageMotorTemp + (1 - VOLTAGE_BATTERY_IIR_GAIN) * voltageBattery;
      voltageBattery = voltageMotorTemp;
    }
  }


  int ESCPos = PUMP_ESC_NEUTRAL;
  if (revTriggerRead()) {
    ESCPos = PUMP_MOTOR_ESC_RUN;
  } else {
    ESCPos = PUMP_MOTOR_ESC_BRAKE;
  }

  switch (roundState) {
  case ROUND_STATE_IDLE: {
    if (sp) {Serial.println(""); Serial.println(" s:idle"); sp = false; }
    driveMotorPWM(MOTOR_DIR_BRAKE, PLUNGER_PWM_MAX);
    // close the valve
    if (triggerRead() && STATE_DELAY(TRIGGER_DELAY_TIME)) {
      // fire a round.  open the valve
      SET_ROUND_STATE(ROUND_STATE_VALVE_OPEN);
    }
    break;
  }
  case ROUND_STATE_VALVE_OPEN: {
    if (sp) {Serial.println(" s:valve_open"); sp = false;}
    driveMotorPWM(MOTOR_DIR_BRAKE, PLUNGER_PWM_MAX);
    displayUpdateEnable = false;
    unsigned long valveTime = 200;
    if (STATE_DELAY(valveTime)) {
      SET_ROUND_STATE(ROUND_STATE_IDLE);
    }
    break;
  }
  }


  servoESC.write(ESCPos);

  if (p) {Serial.println("");}

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


void displayUpdate() {
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
  display.display();
}
