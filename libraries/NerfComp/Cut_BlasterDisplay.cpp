#include <Arduino.h>
#include <Wire.h>

#include <NerfTech.h>

#define MENU_ITEM_NAME_SIZE 12

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
  screenDrawTitle((const char *)F("Config"));
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
  case BUTTON_EVENT_SHORT_UP: {
    // up = test rev motor
    break;
  }
  case BUTTON_EVENT_SHORT_SELECT: {
    // select = plunger dc motor
    break;
  }
  case BUTTON_EVENT_SHORT_DOWN: {
    // down = plunger servo motor
    break;
  }
  }

  // draw screen
  screenDrawTitle((const char *)F("Diagonstic"));

  display.print(F("Volt=")); display.println(voltageBatteryAvg, 1);
  display.print(F("Rev=")); printBit(switchRevTriggerRead()); display.print(F(" "));
  display.print(F("Tgr=")); printBit(switchTriggerRead()); display.print(F(" "));
  display.print(F("Brl=")); printBit(sensorBarrelRead()); display.println(F(""));
  display.print(F("Png=")); printBit(switchPlungerStopRead()); display.println(F(""));
  display.print(F("Jam=")); printBit(switchJamDoorRead()); display.print(F(" "));
  display.print(F("Mag=")); printBit(switchMagSafetyRead()); display.println(F(""));
  display.print(F("MagBits="));
    uint8_t bits = magTypeReadBits();
    printBit(bitRead(bits, 0));
    printBit(bitRead(bits, 1));
    printBit(bitRead(bits, 2));
    printBit(bitRead(bits, 3));
    display.print(F("="));
    displayPrint_P(magazineTypesGetName(magazineTypeIdx)); display.println(F(""));
  }



boolean updateEnable = true;
boolean updateForce = false;
unsigned long updateTime = 0;

void displayUpdateEnable(boolean val) {
  updateEnable = val;
}

void displayUpdateForce(boolean val) {
  updateForce = val;
}

void displayUpdate() {
  
  if (((millis() > (updateTime + DISPLAY_UPDATE_PERIOD)) & updateEnable) || updateForce) {
    updateTime = millis();
    
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
    updateForce = false;
  }
}

