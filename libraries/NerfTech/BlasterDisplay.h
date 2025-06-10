#ifndef BLASTERDISPLAY_h
#define BLASTERDISPLAY_h

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP23008.h>

// void displayUpdate(void);
// void displayInit(void);
// void displayUpdateEnable(boolean val);
// void displayUpdateForce(boolean val);

// void buttonUpdateEvents(void);
// uint8_t displayGetButtonBits(void);
    
// void paramInit(void);

class BlasterDisplay
{
private:
  Adafruit_SSD1306 display;
  Adafruit_MCP23008 GPIO_UI;

  uint8_t UIMode;
  void displayScreenHUD(unsigned long currentTime, int rounds);

public:
  /** Creates a Blaster Display
   *
   * @param pin_1 first pin.  use for single-pin motor controllers
   * @param pin_2 second pin.  use for H-Bridges
   * @param type the type of motor criver we're controlling
   *
   */

  BlasterDisplay(void);

  void begin(void);
  void update(unsigned long currentTime, int rounds);
  void updateAndRedraw(unsigned long currentTime, int rounds);

};

#endif // BLASTERDISPLAY_h