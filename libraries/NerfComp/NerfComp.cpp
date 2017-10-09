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

uint8_t magazineTypesGetCode(uint8_t magazineTypeIdx) {
  return pgm_read_byte(&magazineTypes[magazineTypeIdx].code);
}

uint8_t magazineTypesGetCapacity(uint8_t magazineTypeIdx) {
  return pgm_read_byte(&magazineTypes[magazineTypeIdx].capacity);
}

const char * magazineTypesGetName(uint8_t magazineTypeIdx) {
  return magazineTypes[magazineTypeIdx].name;
}


void SerialPrint_F (const char * str) {
  char c;
  if (!str)
    return;
  while ((c = pgm_read_byte(str++)))
    Serial.print (c);
}

int16_t paramRead(uint8_t paramIdx) {
  uint16_t val;
  val = (uint16_t)EEPROM.read(paramIdx * sizeof(int16_t));
  val |= ((uint16_t)(EEPROM.read(paramIdx * sizeof(int16_t) + 1)) << 8);
  return (int16_t)val;
}

void paramWrite(uint8_t paramIdx, int16_t val) {
  int16_t valRead = paramRead(paramIdx);
  if (valRead != val) {
    EEPROM.write(paramIdx * sizeof(int16_t), (uint8_t)((uint16_t)val & 0x00FF));
    EEPROM.write(paramIdx * sizeof(int16_t) + 1, (uint8_t)((uint16_t)val >> 8));
  }
}

void paramDefaultCheck(void) {
  if (paramReadResetAll() != 0) {
    // parameters need to be initialiszed or reset.
    // Copy default values from flash.
    for(uint8_t i = 0; i < paramCount(); i++) {
      paramWrite(i, paramValueDefault(i));

      Serial.print(F(" param:"));
      SerialPrint_F(paramName(i));
      Serial.print(F(" default="));
      Serial.print(paramValueDefault(i), DEC);
      Serial.print(F(" read="));
      Serial.println(paramRead(i), DEC);
    }
  }
}

void paramInit(void) { 
  paramDefaultCheck();
}

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}