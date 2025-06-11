#ifndef BLASTERDISPLAYBUTTONS_h
#define BLASTERDISPLAYBUTTONS_h

#include <Adafruit_MCP23008.h>

// void buttonUpdateEvents(void);
// uint8_t displayGetButtonBits(void);
    
// void paramInit(void);

class BlasterDisplayButtons
{
private:
  Adafruit_MCP23008 GPIO_UI;
  unsigned long updateTime;

public:
  /** Creates a Blaster Display
   *
   *
   */

  BlasterDisplayButtons(void);

  void begin(void);
  void update(unsigned long currentTime);
  uint8_t getButtonBits(void);
  void printButtonBits(void);
};

#endif // BLASTERDISPLAYBUTTONS_h