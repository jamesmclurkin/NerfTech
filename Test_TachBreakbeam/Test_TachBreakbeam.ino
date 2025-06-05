#include <Adafruit_DotStar.h>


#define PIN_PLUNGER_MOTOR_A       12
#define PIN_PLUNGER_BACK_SW       10

#define PIN_TRIGGER_MODE_SW       9
#define PIN_TRIGGER_SW            7

#define PIN_FLYWHEEL_ESC          5
#define PIN_BREAKBEAM             1
#define PIN_FLYWHEEL_TACHOMETER   0

#define PIN_SAFETY_JAMDOOR_SW     3

#define PIN_MOTOR_TEMP            (A2)



// plunger motor
//DRV8870 pusherMotor(PIN_PLUNGER_MOTOR_A, PIN_PLUNGER_MOTOR_B);


// heartbeat
#define HEART_BEAT_PERIOD         250

unsigned long heartbeatTime;

// tachometer
#define TACH_READ_PERIOD          100
#define TACH_READ_CONST           (30000 / TACH_READ_PERIOD)

void tachometerISR(void);

unsigned long _tachReadTime;
unsigned long _tachInterruptTime;
boolean _tachRead = false;
int _tachCount = 0;
int tachCountPeriod = 0;
long rpm = 0;

// Interrupt on rising edge on the tach pin.  two edges per rotation
void tachometerISR(void) {
  if(_tachRead) {
    tachCountPeriod = _tachCount;
    _tachCount = 0;
    _tachRead = false;
  }
  _tachInterruptTime = millis();
  _tachCount++;
}

long tachRPM (long counts) {
  long rpm = counts * TACH_READ_CONST;
  return rpm;
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


// GPIO Test
boolean buttonRead() {return !digitalRead(PIN_TRIGGER_SW); }

// Dot Star
// Everything is defined in the Board Support Package
// DOTSTAR_NUM        number of onboard DotStars (typically just 1)
// PIN_DOTSTAR_DATA   onboard DotStar data pin
// PIN_DOTSTAR_CLK    onboard DotStar clock pin
Adafruit_DotStar dotStar(DOTSTAR_NUM, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLK, DOTSTAR_BRG);

// Arduino structure
void setup() {
  // init the plunger motor
  //pusherMotor.coast();
  pinMode(PIN_PLUNGER_MOTOR_A, OUTPUT);
  digitalWrite(PIN_PLUNGER_MOTOR_A, LOW);


  // init the tachometer
  pinMode(PIN_FLYWHEEL_TACHOMETER, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_FLYWHEEL_TACHOMETER), tachometerISR, RISING);

  // init the dart breakbeam sensor
  pinMode(PIN_BREAKBEAM, INPUT);
  breakbeamEdgeReset();

  // try gpio on MOSI
  pinMode(PIN_TRIGGER_SW, INPUT_PULLUP);

  // initialize the LED as an output:
  pinMode(PIN_LED, OUTPUT);

  dotStar.begin(); // Initialize pins for output
  dotStar.setBrightness(80);
  dotStar.show();  // Turn all LEDs off ASAP

  // initialize serial communication:
  Serial.begin(115200);
  Serial.print("trigger init on pin ");
  Serial.println(PIN_TRIGGER_SW, DEC);

  heartbeatTime = millis();
  _tachReadTime = heartbeatTime;
}

// #ff8000
// #ff0000
#define COLOR_RED           0xFF
#define COLOR_GREEN         0x00
#define COLOR_BLUE          0x00

#define RGB_BRIGHT_MULT   1.15
#define RGB_BRIGHT_MAX    0.4
#define RGB_BRIGHT_MIN    0.05
float dotStarBrightness = RGB_BRIGHT_MIN;


// GPIO testing
boolean buttonOld = false;

void loop() {
  unsigned long timeCurrent = millis();

  // update breakbeam
  breakbeamResetAfterMaxTime();

  // heartbeat
  if (timeCurrent > heartbeatTime) {
    Serial.print("tach counts:");
    Serial.print(tachCountPeriod, DEC);
    Serial.print(" RPM:");
    Serial.print(rpm, DEC);

    Serial.println("");

    heartbeatTime += HEART_BEAT_PERIOD;
  }

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

  // update button and test motor
  boolean buttonCurrent = buttonRead();
  if (buttonCurrent && !buttonOld) {
    Serial.println("button!");
  }
  buttonOld = buttonCurrent;

  if (buttonCurrent) {
    //pusherMotor.setSpeed(255, CLOCKWISE);
    analogWrite(PIN_PLUNGER_MOTOR_A, 80);
  } else {
    //pusherMotor.brake(MOTOR_SPEED_MAX);
    analogWrite(PIN_PLUNGER_MOTOR_A, 0);
  }

  // update tachometer & Fire LED
  if (timeCurrent > _tachReadTime) {
    if (timeCurrent > (_tachInterruptTime + TACH_READ_PERIOD)) {
      // It's been a long time since a tach interupt.  Clear the count
      _tachCount = 0;
      tachCountPeriod = 0;
    } else {
      _tachRead = true;
    }
    rpm = tachRPM(tachCountPeriod);
    _tachReadTime += TACH_READ_PERIOD;


    if (buttonCurrent) {
      // #008000
      dotStar.setPixelColor(0, 0x800000);
    } else {
      float dsRed = (float)COLOR_RED * dotStarBrightness;
      float dsGreen = (float)COLOR_GREEN * dotStarBrightness;
      float dsBlue = (float)COLOR_BLUE * dotStarBrightness;
      dotStar.setPixelColor(0, (unsigned char)dsGreen, (unsigned char)dsRed, (unsigned char)dsBlue);
      //dotStar.setPixelColor(0, 0, (unsigned char)dsRed, 0);
      dotStarBrightness *= RGB_BRIGHT_MULT;
      if(dotStarBrightness > RGB_BRIGHT_MAX) {
        dotStarBrightness = RGB_BRIGHT_MIN;
      }
    }
    dotStar.show();
  }

  delay(5);
}
