#ifndef NerfCompDisplay_h
#define NerfCompDisplay_h

void displayUpdate(void);
void displayInit(void);
void displayUpdateEnable(boolean val);
void displayUpdateForce(boolean val);

void buttonUpdateEvents(void);
uint8_t displayGetButtonBits(void);
    
void paramInit(void);

#endif //NerfCompDisplay_h