#include <Arduino.h>
#include <Wire.h>

#include <NerfTech.h>

///////// bitmapped graphics ////////
#include <BlasterDisplay_SevenSegmentLarge.h>
#include <BlasterDisplay_NerfLogo.h>

#define UI_SCREEN_HUD         0
#define UI_SCREEN_MENU        1
#define UI_SCREEN_CONFIG      2
#define UI_SCREEN_CONFIG_EDIT 3
#define UI_SCREEN_TIMER       4
#define UI_SCREEN_DIAGNOSTIC  5


BlasterDisplay::BlasterDisplay(int pin)
{
  this->display = Adafruit_SSD1306(OLED_RESET);
  this->UIMode = UI_SCREEN_HUD;

    // init the LED Display
  // generate the high voltage from the 3.3v line internally! (neat!)
  this->display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C
  
  // Clear the display buffer and put up the splash screen
  this->display.clearDisplay();
  this->display.drawBitmap(0, 0, NerfLogoBitmap, NERF_LOGO_BITMAP_WIDTH, NERF_LOGO_BITMAP_HEIGHT, 1);
  this->display.display();

  // show the splash screen for a while
  //delay(2000);

  // force an update to show  the initial data display
  // displayUpdate();

  // GPIO_UI.begin(1);       // GPIO foor the UI buttons is on address 1
  // for (int i = 0; i < 8; ++i) {
  //   GPIO_UI.pinMode(i, INPUT);
  //   GPIO_UI.pullUp(i, HIGH);  // turn on a 100K pullup internally
  // }

}

void BlasterDisplay::update(unsigned long currentTime)
{
}