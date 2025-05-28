#define PIN_TACHOMETER            4
#define PIN_BREAKBEAM             1
//#define PIN_LED                   13
#define PIN_MOTOR_TEMP            (A2)
// setup neopizel here

void tachometerISR(void);

#define HEART_BEAT_PERIOD         250
#define TACH_READ_PERIOD          100
#define TACH_READ_CONST           (30000 / TACH_READ_PERIOD)

unsigned long heartbeatTime;
unsigned long tachReadTime;
unsigned long tachInterruptTime;

void setup() {
  // initialize the button pin as a input:
  pinMode(PIN_TACHOMETER, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_TACHOMETER), tachometerISR, RISING);

  // initialize the LED as an output:
  pinMode(PIN_LED, OUTPUT);

  // initialize serial communication:
  Serial.begin(115200);
  Serial.print("Tach init on pin ");
  Serial.println(PIN_TACHOMETER, DEC);
  heartbeatTime = millis();
  tachReadTime = heartbeatTime;
}

boolean tachRead = false;
int tachCount = 0;
int tachCountPeriod = 0;
long rpm = 0;

// rising edge on the tach pin.  two edges per rotation
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

void loop() {
  unsigned long timeCurrent = millis();
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

  if (timeCurrent > heartbeatTime) {
    Serial.print("tach counts:");
    Serial.print(tachCountPeriod, DEC);
    Serial.print(" RPM:");
    Serial.print(rpm, DEC);
    Serial.println("");
    heartbeatTime += HEART_BEAT_PERIOD;
  }
}
