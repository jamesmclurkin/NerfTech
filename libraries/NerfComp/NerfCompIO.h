#ifndef NerfCompIO_h
#define NerfCompIO_h

void gpioInit(void);
uint8_t gpioGetMagSensorBits(void);
uint8_t magazineTypeLookup(uint8_t magTypeBits);
void plungerMotorPWM(int dir, int pwm);
void servoESCWrite(int ESCPos);
void batteryVoltageUpdate(boolean p);    
void magazineTypeUpdate(boolean p);
void jamDoorUpdate(boolean p);
void dartCheck(void);

#endif // NerfCompIO_h