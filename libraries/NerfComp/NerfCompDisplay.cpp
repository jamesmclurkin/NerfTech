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

///////// graphics ////////
#include <SevenSegmentBitmaps.h>
#include <NerfLogo.h>


#define MENU_ITEM_NAME_SIZE 12

#define UI_SCREEN_HUD         0
#define UI_SCREEN_MENU        1
#define UI_SCREEN_CONFIG      2
#define UI_SCREEN_CONFIG_EDIT 3
#define UI_SCREEN_TIMER       4
#define UI_SCREEN_DIAGNOSTIC  5

typedef struct MenuItem {
  const char name[MENU_ITEM_NAME_SIZE];
  const uint8_t UIMode;
} MenuItem;

const MenuItem menuItems[] PROGMEM = {
   // 012345678901234567890
   //"Menu       "
    {"       HUD", UI_SCREEN_HUD},
    {"    Config", UI_SCREEN_CONFIG},
    {"     Timer", UI_SCREEN_TIMER},
    {"Diagnostic", UI_SCREEN_DIAGNOSTIC},
};

//////// user Interface ////////

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_MCP23008 GPIO_UI;

uint8_t UIMode = UI_SCREEN_HUD;


#define BUTTON_BIT_SELECT       0x02
#define BUTTON_BIT_BACK         0x80
#define BUTTON_BIT_UP           0x04
#define BUTTON_BIT_DOWN         0x01

#define BUTTON_EVENT_NULL           0
#define BUTTON_EVENT_SHORT_SELECT   1
#define BUTTON_EVENT_SHORT_BACK     2
#define BUTTON_EVENT_SHORT_UP       3
#define BUTTON_EVENT_SHORT_DOWN     4

#define BUTTON_EVENT_LONG_SELECT    10

#define BUTTON_EVENT_RELEASE_SELECT 20

uint8_t buttonBitsOld1 = 0;
uint8_t buttonBitsOld2 = 0;
uint8_t buttonEvent = BUTTON_EVENT_NULL;

uint8_t _buttonRead() {
  uint8_t buttonBits;
  buttonBits = ~GPIO_UI.readGPIO();
  return buttonBits;
}

void buttonEventAdd(uint8_t e) {
  buttonEvent = e;
}

uint8_t buttonEventGet(void) {
  uint8_t val = buttonEvent;
  buttonEvent = BUTTON_EVENT_NULL;
  return val;
}

boolean buttonRisingEdge(uint8_t buttonBits, uint8_t buttonBitMask) {
  if ((!(buttonBitsOld2 & buttonBitMask)) &&
      (buttonBitsOld1 & buttonBitMask) &&
      (buttonBits & buttonBitMask)) {
    return true;
  } else {
    return false;
  }
}

void buttonUpdateEvents(void) {
  uint8_t buttonBits = _buttonRead();
  buttonBitsOld2 = buttonBitsOld1;
  buttonBitsOld1 = buttonBits;
  if (buttonRisingEdge(buttonBits, BUTTON_BIT_SELECT)) {buttonEventAdd(BUTTON_EVENT_SHORT_SELECT);}
  if (buttonRisingEdge(buttonBits, BUTTON_BIT_BACK))   {buttonEventAdd(BUTTON_EVENT_SHORT_BACK);}
  if (buttonRisingEdge(buttonBits, BUTTON_BIT_UP))     {buttonEventAdd(BUTTON_EVENT_SHORT_UP);}
  if (buttonRisingEdge(buttonBits, BUTTON_BIT_DOWN))   {buttonEventAdd(BUTTON_EVENT_SHORT_DOWN);}
}



//////// Display ////////

void printBit(uint8_t bit) {
  if (bit) {
    display.setTextColor(BLACK, WHITE); // 'inverted' text
    display.print(F("1"));
    display.setTextColor(WHITE);
  } else {
    display.setTextColor(WHITE);
    display.print(F("0"));
  }
}

//void displayPrint_F(Adafruit_SSD1306 d, const char * str) {
void displayPrint_P(const char * str) {
  char c;
  //char buffer[SCREEN_BUFFER_SIZE];

  if (!str)
    return;
//  strcpy_P(buffer, str);
//  d.print(buffer);
  while ((c = pgm_read_byte(str++)))
    display.print(c);
}


#define SCREEN_UNDERLINE_POS  8
#define SCREEN_CONFIG_ROWS 6

void screenDrawTitle(const char * title) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // draw scren title
  display.setCursor(0, 0);
  displayPrint_P(title);
  display.drawLine(0, SCREEN_UNDERLINE_POS, display.width() - 1, SCREEN_UNDERLINE_POS, WHITE);
  display.setCursor(0, SCREEN_UNDERLINE_POS+2);
}


#define MENUITEM_COUNT (sizeof(menuItems)/sizeof(MenuItem))
#define MENUITEM_UIMODE(idx) ((uint8_t)pgm_read_byte(&menuItems[idx].UIMode))
#define MENUITEM_NAME(idx) ((const char*)&menuItems[idx].name)
#define SCREEN_MENU_ROWS 7

int8_t menuSelectIdx = 0;
int8_t menuScrollIdx = 0;

void displayScreenMenu() {
  // check the buttons
  int8_t highlightPos = menuSelectIdx - menuScrollIdx;
  switch (buttonEventGet()) {
  case BUTTON_EVENT_SHORT_SELECT: {
    // select the menu item cursor
    UIMode = MENUITEM_UIMODE(menuSelectIdx);
    break;
    }
  case BUTTON_EVENT_SHORT_BACK: {
    //Go back to menu screen
    UIMode = UI_SCREEN_HUD;
    break;
    }
  case BUTTON_EVENT_SHORT_UP: {
    // prev menuitem
    if (menuSelectIdx > 0) {
      menuSelectIdx--;
      highlightPos = menuSelectIdx - menuScrollIdx;
      if (highlightPos < 0) {
        menuScrollIdx--;
        highlightPos = menuSelectIdx - menuScrollIdx;
      }
    }
    break;
    }
  case BUTTON_EVENT_SHORT_DOWN: {
    // next parameter
    if (menuSelectIdx < (MENUITEM_COUNT - 1)) {
      menuSelectIdx++;
      highlightPos = menuSelectIdx - menuScrollIdx;
      if (highlightPos >= SCREEN_MENU_ROWS) {
        menuScrollIdx++;
        highlightPos = menuSelectIdx - menuScrollIdx;
      }
    }
    break;
    }
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  screenDrawTitle((const char *)F("Main Menu"));

  uint8_t menuLinesToDraw = min(MENUITEM_COUNT, SCREEN_MENU_ROWS);
  for(uint8_t i = 0; i < menuLinesToDraw; i++) {
    //Compute which menu line to highlight
    if (i == highlightPos) {
      display.setTextColor(BLACK, WHITE); // 'inverted' text
    } else {
      display.setTextColor(WHITE);
    }
    displayPrint_P(MENUITEM_NAME(menuScrollIdx + i));
    display.println(F(""));
  }
}


int8_t configSelectIdx = 0;
int8_t configScrollIdx = 0;

void displayScreenConfig() {
  // check the buttons
  int8_t highlightPos = configSelectIdx - configScrollIdx;
  switch (buttonEventGet()) {
  case BUTTON_EVENT_SHORT_SELECT: {
    if (UIMode == UI_SCREEN_CONFIG_EDIT) {
      paramDefaultCheck();
      UIMode = UI_SCREEN_CONFIG;
    } else {
      //select the parameter unter the curcur for editing
      UIMode = UI_SCREEN_CONFIG_EDIT;
    }
    break;
    }
  case BUTTON_EVENT_SHORT_BACK: {
    //Go back to menu screen
    if (UIMode == UI_SCREEN_CONFIG_EDIT) {
      paramDefaultCheck();
      UIMode = UI_SCREEN_CONFIG;
    } else {
      UIMode = UI_SCREEN_MENU;
    }
    break;
    }
  case BUTTON_EVENT_SHORT_UP: {
    if (UIMode == UI_SCREEN_CONFIG_EDIT) {
      // increase parameter value
      int16_t paramTemp = paramRead(configSelectIdx);
      paramTemp += paramValueStep(configSelectIdx);
      if (paramTemp > paramValueMax(configSelectIdx)) {
        paramTemp = paramValueMax(configSelectIdx);
      }
      paramWrite(configSelectIdx, paramTemp);
    } else {
      // prev parameter
      if (configSelectIdx > 0) {
        configSelectIdx--;
        highlightPos = configSelectIdx - configScrollIdx;
        if (highlightPos < 0) {
          configScrollIdx--;
          highlightPos = configSelectIdx - configScrollIdx;
        }
      }
    }
    break;
    }
  case BUTTON_EVENT_SHORT_DOWN: {
    if (UIMode == UI_SCREEN_CONFIG_EDIT) {
      // decrease parameter value
      int16_t paramTemp = paramRead(configSelectIdx);
      paramTemp -= paramValueStep(configSelectIdx);
      if (paramTemp < paramValueMin(configSelectIdx)) {
        paramTemp = paramValueMin(configSelectIdx);
      }
      paramWrite(configSelectIdx, paramTemp);
    } else {
      // next parameter
      if (configSelectIdx < (paramCount() - 1)) {
        configSelectIdx++;
        highlightPos = configSelectIdx - configScrollIdx;
        if (highlightPos >= SCREEN_CONFIG_ROWS) {
          configScrollIdx++;
          highlightPos = configSelectIdx - configScrollIdx;
        }
      }
    }
    break;
    }
  }

  // draw the config screen
  screenDrawTitle((const char *)F("System Config"));
  for(uint8_t i = 0; i < SCREEN_CONFIG_ROWS; i++) {
    //Compute which screen line to highlight
    if ((UIMode == UI_SCREEN_CONFIG) && (i == highlightPos)) {
      display.setTextColor(BLACK, WHITE); // 'inverted' text
    } else {
      display.setTextColor(WHITE);
    }
    //displayPrint_P((const char*)&configParams[configScrollIdx + i].name);
    displayPrint_P(paramName(configScrollIdx + i));
    display.print(F(":"));

    int16_t paramPrint = paramRead(configScrollIdx + i);
    if (i == highlightPos) {
      display.setTextColor(BLACK, WHITE); // 'inverted' text
    } else {
      display.setTextColor(WHITE);
    }
    display.print(paramPrint, DEC);
    if(paramPrint < 1000) {display.print(F(" "));}
    if(paramPrint < 100) {display.print(F(" "));}
    if(paramPrint < 10) {display.print(F(" "));}
    //display.println(F(""));
  }
}


void displayScreenDiag() {
  // dispay the diagnostic screen
  switch (buttonEventGet()) {
  case BUTTON_EVENT_SHORT_BACK: {
    //Go back to menu screen
    UIMode = UI_SCREEN_MENU;
    break;
    }
  }

  // draw screen
  screenDrawTitle((const char *)F("System Diagonstic"));

  display.print(F("Volt=")); display.println(voltageBatteryAvg, 1);
  display.print(F("Rev=")); printBit(switchRevTriggerRead()); display.print(F(" "));
  display.print(F("Tgr=")); printBit(switchTriggerRead()); display.print(F(" "));
  display.print(F("Brl=")); printBit(sensorBarrelRead()); display.println(F(""));
  display.print(F("Jam=")); printBit(switchJamDoorRead()); display.print(F(" "));
  display.print(F("Mag=")); printBit(switchMagSafetyRead()); display.println(F(""));
  display.print(F("MagBits="));
    uint8_t bits = magTypeReadBits();
    printBit(bitRead(bits, 0));
    printBit(bitRead(bits, 1));
    printBit(bitRead(bits, 2));
    printBit(bitRead(bits, 3));
    display.print(F("="));
    //displayPrint_P(magazineTypes[magazineTypeIdx].name); display.println(F(""));
    displayPrint_P(magazineTypesGetName(magazineTypeIdx)); display.println(F(""));
  }


#define POSX_SEVEN_SEG_DIGIT_0  66
#define POSX_SEVEN_SEG_DIGIT_1  98
#define POSY_SEVEN_SEG_DIGIT  12

void displayScreenHUD() {
  switch (buttonEventGet()) {
  case BUTTON_EVENT_SHORT_SELECT: {
    //advance to config screen
    UIMode = UI_SCREEN_MENU;
    break;
    }
  }

  // Draw the HUD Display
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println(F("Rapid"));
  display.setTextSize(1);
  display.print(F("Mag:"));
  //displayPrint_P(magazineTypes[magazineTypeIdx].name); display.println(F(""));
  displayPrint_P(magazineTypesGetName(magazineTypeIdx)); display.println(F(""));
  display.print(F("Rd/m:"));
  if (roundsPerMin > 0) {
    display.println(roundsPerMin, DEC);
  } else {
    display.println(F("---"));
  }
  display.print(F("Ft/s:"));
  if (velocity >= 0.0) {
    display.println(velocity, 1);
  } else {
    display.println(F("---"));
  }
  display.print(F("Volt:"));
  display.println(voltageBatteryAvg, 1);

  display.print(F("JamCount:"));
  display.println(roundsJamCount, DEC);

  // draw the round digits.
  int digit0 = SEVEN_SEGMENT_BITMAP_DASH;
  int digit1 = SEVEN_SEGMENT_BITMAP_DASH;
  if ((roundCount >= 0) && (roundCount <= 99)) {
    digit0 = roundCount / 10;
    digit1 = roundCount % 10;
  }
  display.setCursor(86, 0);
  display.println(F("Rounds:"));
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
      display.print(F("Jam:Door"));
    } else if (feedJam) {
      display.print(F("Jam:Feed"));
    }
  }
}


void displayUpdate() {
  display.dim(paramReadDisplayDim());

  switch (UIMode) {
  case UI_SCREEN_CONFIG:
  case UI_SCREEN_CONFIG_EDIT:
    displayScreenConfig();
    break;
  case UI_SCREEN_DIAGNOSTIC:
    displayScreenDiag();
    break;
  case UI_SCREEN_MENU:
    displayScreenMenu();
    break;
  default:
  case UI_SCREEN_HUD:
    displayScreenHUD();
    break;
  }
  display.display();
}

void displayInit() {
  // init the LED Display
  // generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C
  
  // Clear the display buffer and put up the splash screen
  display.clearDisplay();
  display.drawBitmap(0, 0, NerfLogoBitmap, NERF_LOGO_BITMAP_WIDTH, NERF_LOGO_BITMAP_HEIGHT, 1);
  display.display();

  // show the splash screen for a while
  delay(2000);

  // force an update to show  the initial data display
  displayUpdate();

  GPIO_UI.begin(1);       // GPIO foor the UI buttons is on address 1
  for (int i = 0; i < 8; ++i) {
    GPIO_UI.pinMode(i, INPUT);
    GPIO_UI.pullUp(i, HIGH);  // turn on a 100K pullup internally
  }
}

uint8_t displayGetButtonBits(void) {
  return GPIO_UI.readGPIO();  
}
