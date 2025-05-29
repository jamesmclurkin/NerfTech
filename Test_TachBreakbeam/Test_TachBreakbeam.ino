#define PIN_TACHOMETER            4
#define PIN_BREAKBEAM             1
//#define PIN_LED                   13
#define PIN_MOTOR_TEMP            (A2)
// setup neopizel here


// heartbeat
#define HEART_BEAT_PERIOD         1000

unsigned long heartbeatTime;

// tachometer
#define TACH_READ_PERIOD          100
#define TACH_READ_CONST           (30000 / TACH_READ_PERIOD)

void tachometerISR(void);

unsigned long tachReadTime;
unsigned long tachInterruptTime;
boolean tachRead = false;
int tachCount = 0;
int tachCountPeriod = 0;
long rpm = 0;

// Interrupt on rising edge on the tach pin.  two edges per rotation
void tachometerISR(void) {
  if(tachRead) {
    tachCountPeriod = tachCount;
    tachCount = 0;
    tachRead = false;
  }
  tachInterruptTime = millis();
  tachCount++;
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

// Arduino structure
void setup() {
  // init the tachometer
  pinMode(PIN_TACHOMETER, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_TACHOMETER), tachometerISR, RISING);

  // init the dart breakbeam sensor
  pinMode(PIN_BREAKBEAM, INPUT);
  breakbeamEdgeReset();

  // initialize the LED as an output:
  pinMode(PIN_LED, OUTPUT);

  // initialize serial communication:
  Serial.begin(115200);
  Serial.print("Tach init on pin ");
  Serial.println(PIN_TACHOMETER, DEC);
  heartbeatTime = millis();
  tachReadTime = heartbeatTime;
}


void loop() {
  unsigned long timeCurrent = millis();

  // update tachometer
  if (timeCurrent > tachReadTime) {
    if (timeCurrent > (tachInterruptTime + TACH_READ_PERIOD)) {
      // It's been a long time since a tach interupt.  Clear the count
      tachCount = 0;
      tachCountPeriod = 0;
    } else {
      tachRead = true;
    }
    rpm = tachRPM(tachCountPeriod);
    tachReadTime += TACH_READ_PERIOD;
  }

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
}
