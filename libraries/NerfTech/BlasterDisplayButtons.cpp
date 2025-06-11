#include <Arduino.h>
#include <Wire.h>

#include <NerfTech.h>

#define BUTTON_UPDATE_TIME  10

BlasterDisplayButtons::BlasterDisplayButtons(void)
{
  GPIO_UI = Adafruit_MCP23008();
}

void BlasterDisplayButtons::begin(void)
{
  GPIO_UI.begin(1);       // GPIO foor the UI buttons is on address 1
  for (int i = 0; i < 8; ++i) {
    GPIO_UI.pinMode(i, INPUT);
    GPIO_UI.pullUp(i, HIGH);  // turn on a 100K pullup internally
  }

  updateTime = millis() + BUTTON_UPDATE_TIME;
}

void BlasterDisplayButtons::update(unsigned long currentTime)
{
}

uint8_t BlasterDisplayButtons::getButtonBits(void) {
  return GPIO_UI.readGPIO();  
}

void BlasterDisplayButtons::printButtonBits(void) {
  uint8_t byte = GPIO_UI.readGPIO();
  uint8_t mask = 0x80;
  for (int i = 0; i < 8; i++)
  {
    if (byte & mask) {
      Serial.print("1");
    } else {
      Serial.print("0");
    }
    mask = mask >> 1;
  }
  
}
