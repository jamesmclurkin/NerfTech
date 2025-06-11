#include <Arduino.h>
#include <Wire.h>

#include <NerfTech.h>

///////// bitmapped graphics ////////
#include <BlasterDisplay_SevenSegmentLarge.h>
#include <BlasterDisplay_NerfLogo.h>

// #include <Fonts/DialogInput_plain_10.h>
// #include <Fonts/DialogInput_bold_10.h>

// #include <Fonts/Monospaced_plain_9.h>
// #include <Fonts/Monospaced_bold_9.h>

// #include <Fonts/Monospaced_plain_10.h>
// #include <Fonts/Monospaced_bold_10.h>

// #include <Fonts/Roboto_Light_9.h>
// #include <Fonts/Roboto_Black_9.h>

#include <Fonts/Roboto_Light_10.h>
#include <Fonts/Roboto_Light_11.h>

#include <Fonts/Roboto_Black_10.h>
#include <Fonts/Roboto_Black_11.h>
#include <Fonts/Roboto_Black_12.h>
#include <Fonts/Roboto_Black_13.h>

#define UI_SCREEN_HUD         0
#define UI_SCREEN_MENU        1
#define UI_SCREEN_CONFIG      2
#define UI_SCREEN_CONFIG_EDIT 3
#define UI_SCREEN_TIMER       4
#define UI_SCREEN_DIAGNOSTIC  5

#define SPLASH_SCREEN_TIME    1000
#define SCREEN_UPDATE_TIME    50
unsigned long updateTime;

#define PIN_OLED_RESET -1
#define I2C_CLOCK_SPEED_DEFAULT           400000
#define I2C_CLOCK_SPEED_DISPLAY           400000
#define DISPLAY_RES_HORIZONTAL            128
#define DISPLAY_RES_VERTICAL              64

// temp variables to dislpay until I figure out a "blaster status" class\
// blaster status
int roundsPerMin = 3;
float velocity = 152.6;
float voltageBatteryAvg = 11.8;
int roundsJamCount = 0;
int roundCount = 18;
boolean jamDoorOpen = false;
boolean feedJam = false;

BlasterDisplay::BlasterDisplay(void)
{
  //display = Adafruit_SSD1306(DISPLAY_RES_HORIZONTAL, DISPLAY_RES_VERTICAL, &Wire, OLED_RESET, 400000, 100000);
  display = Adafruit_SSD1306(DISPLAY_RES_HORIZONTAL, DISPLAY_RES_VERTICAL, &Wire, PIN_OLED_RESET, I2C_CLOCK_SPEED_DISPLAY, I2C_CLOCK_SPEED_DEFAULT);
  UIMode = UI_SCREEN_HUD;
}

void BlasterDisplay::begin(void)
{

  // init the LED Display
  // generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C
  
  // Clear the display buffer and put up the splash screen
  display.clearDisplay();
  display.drawBitmap(0, 0, NerfLogoBitmap, NERF_LOGO_BITMAP_WIDTH, NERF_LOGO_BITMAP_HEIGHT, 1);
  display.display();

  //updateTime = millis() + SPLASH_SCREEN_TIME;
  updateTime = SPLASH_SCREEN_TIME;
}

void BlasterDisplay::update(unsigned long currentTime, int rounds)
{
  roundCount = rounds;
  if (currentTime > updateTime) {
    updateTime += SCREEN_UPDATE_TIME;
    displayScreenHUD(currentTime, rounds);
  }
}

void BlasterDisplay::updateAndRedraw(unsigned long currentTime, int rounds)
{
  roundCount = rounds;
  displayScreenHUD(currentTime, rounds);
}

void BlasterDisplay::displayBit(uint8_t bit) {
  if (bit) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
    display.print("1");
    display.setTextColor(WHITE);
  } else {
    display.setTextColor(WHITE);
    display.print("0");
  }
}



#define POSX_SEVEN_SEG_DIGIT_0  66
#define POSX_SEVEN_SEG_DIGIT_1  98
#define POSY_SEVEN_SEG_DIGIT  12

void BlasterDisplay::displayScreenHUD(unsigned long currentTime, int rounds) {
  // Draw the HUD Display
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 9);
  display.setFont(&Roboto_Black_13);
  display.println("Vulcan");
  display.setCursor(0, 20);
  display.setFont(&Roboto_Light_11);
  display.print("mag:");
  //displayPrint_P(magazineTypes[magazineTypeIdx].name); display.println("");
  //displayPrint_P(magazineTypesGetName(magazineTypeIdx)); display.println("");
  display.print("rd/m:");
  if (roundsPerMin > 0) {
    display.println(roundsPerMin, DEC);
  } else {
    display.println("---");
  }
  display.print("ft/s:");
  if (velocity >= 0.0) {
    display.println(velocity, 1);
  } else {
    display.println("---");
  }
  
  display.print("volt:");
  display.println(voltageBatteryAvg, 1);

  // display.print("JamCount:");
  // display.println(roundsJamCount, DEC);

  // draw the round digits.
  int digit0 = SEVEN_SEGMENT_BITMAP_DASH;
  int digit1 = SEVEN_SEGMENT_BITMAP_DASH;
  if ((roundCount >= 0) && (roundCount <= 99)) {
    digit0 = roundCount / 10;
    digit1 = roundCount % 10;
  }
  display.setCursor(89, 7);
  display.print("rounds");
  display.setCursor(90, 10);
  display.drawBitmap(POSX_SEVEN_SEG_DIGIT_0, POSY_SEVEN_SEG_DIGIT,
      (uint8_t *) &(SevenSegmentBitMaps[digit0]),
      SEVEN_SEGMENT_BITMAP_WIDTH, SEVEN_SEGMENT_BITMAP_HEIGHT, 1);
  display.drawBitmap(POSX_SEVEN_SEG_DIGIT_1, POSY_SEVEN_SEG_DIGIT,
      (uint8_t *) &(SevenSegmentBitMaps[digit1]),
      SEVEN_SEGMENT_BITMAP_WIDTH, SEVEN_SEGMENT_BITMAP_HEIGHT, 1);

  // draw the jam door indicator or the feed jam indicator
  if (jamDoorOpen || feedJam) {
    //fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color),
    display.fillRoundRect(12, 20, 102, 23, 3, WHITE),
    display.drawRoundRect(12, 20, 102, 23, 3, BLACK),
    display.setTextSize(2);
    display.setCursor(16, 24);
    display.setTextColor(BLACK, WHITE); // 'inverted' text
    if (jamDoorOpen) {
      display.print("Jam:Door");
    } else if (feedJam) {
      display.print("Jam:Feed");
    }
  }
  display.display();
}
