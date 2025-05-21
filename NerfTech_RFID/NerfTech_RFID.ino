/**************************************************************************/
/*!
    @file     readntag203.pde
    @author   KTOWN (Adafruit Industries)
    @license  BSD (see license.txt)

    This example will wait for any NTAG203 or NTAG213 card or tag,
    and will attempt to read from it.

    This is an example sketch for the Adafruit PN532 NFC/RFID breakout boards
    This library works with the Adafruit NFC breakout
      ----> https://www.adafruit.com/products/364

    Check out the links above for our tutorials and wiring diagrams
    These chips use SPI or I2C to communicate.

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!
*/
/**************************************************************************/
//#define PN532DEBUGPRINT

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

// If using the breakout with SPI, define the pins for SPI communication.
#define PN532_SCK  (2)
#define PN532_MOSI (3)
#define PN532_SS   (4)
#define PN532_MISO (5)

// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
#define PN532_IRQ   (2)
#define PN532_RESET (3)  // Not connected by default on the NFC Shield

// Uncomment just _one_ line below depending on how your breakout or shield
// is connected to the Arduino:

// Use this line for a breakout with a software SPI connection (recommended):
//Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

// Use this line for a breakout with a hardware SPI connection.  Note that
// the PN532 SCK, MOSI, and MISO pins need to be connected to the Arduino's
// hardware SPI SCK, MOSI, and MISO pins.  On an Arduino Uno these are
// SCK = 13, MOSI = 11, MISO = 12.  The SS line can be any digital IO pin.
//Adafruit_PN532 nfc(PN532_SS);

// Or use this line for a breakout or shield with an I2C connection:
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);


void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10); // for Leonardo/Micro/Zero

  Serial.println("Hello!");

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);

  Serial.println("Waiting for an ISO14443A Card ...");
}

#define UID_LENGTH                  7 
#define PAGE_LENGTH                 4
#define NEFTTECH_FIRST_PAGE         4
#define NEFTTECH_LAST_PAGE          20
#define NEFTTECH_STRING_COUNT       4
#define NEFTTECH_STRING_LEN         16

uint8_t uidPrev[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the previous UID

bool uidCompare(uint8_t* uid1, uint8_t* uid2) {
  for (uint8_t i = 0; i < UID_LENGTH; i++) {
    if (uid1[i] != uid2[i]) {
      return false;
    }
  }
  return true;  
}

void uidCopy(uint8_t* uidTo, uint8_t* uidFrom) {
  memcpy(uidTo, uidFrom, UID_LENGTH);
}

void uidClear(uint8_t* uid) {
  memset(uid, 0, UID_LENGTH);
}

void loop(void) {
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  char textData[NEFTTECH_STRING_COUNT][NEFTTECH_STRING_LEN];   // string data buffer

  // Wait for an NTAG203 card.  When one is found 'uid' will be populated with
  // the UID, and uidLength will indicate the size of the UUID (normally 7)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 50);

  if (!success) {     
    //Serial.println("nothing");
    uidClear(uidPrev);
  } else {
    if (uidLength == 7) {
      if (uidCompare(uid, uidPrev)) {
        //same card is still present.  Do nothing
        //Serial.println("Duplicate!");
      } else {
        // Display some basic information about the card
        Serial.println("Found an ISO14443A card!");
        Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
        Serial.print("  UID Value: ");
        nfc.PrintHex(uid, uidLength);
        Serial.println("");

        // We probably have an NTAG2xx card (though it could be Ultralight as well)
        Serial.println("Seems to be an NTAG2xx tag (7 byte UID)");
  
        // NTAG2x3 cards have 39*4 bytes of user pages (156 user bytes),
        // starting at page 4   
        // read pages 4-20 pages for NerfTech
        uint8_t data[(NEFTTECH_LAST_PAGE - NEFTTECH_FIRST_PAGE + 1) * PAGE_LENGTH];
        uint8_t j = 0;
        for (uint8_t i = NEFTTECH_FIRST_PAGE; i <= NEFTTECH_LAST_PAGE; i++) {
          success = nfc.ntag2xx_ReadPage(i, &data[j++ * PAGE_LENGTH]);
          if (!success) {
            Serial.println("tag read error");
            break;
          }
        }

        // Read, parse, and output the NerfTEch data
        if (success) {
          uint8_t i = 0;
          uint8_t stringNum = 0;
          uint8_t k;
          uint16_t size = 1;
          uint8_t len = 0;
          //bool done = false;

          while (i < size) {
            char d = data[i++];

            if (d == 0x03) {
              size = data[i++];
              // skip the odd number
              //Serial.print("size:");            
              //Serial.println(size, DEC);            
              i++;
            } else if (d == 1) {
              len = data[i] - 3;
              // Serial.print("string:");            
              // Serial.print(stringNum, DEC);            
              // Serial.print(" len:");            
              // Serial.println(len, DEC);            
              i += 5;
              for (k = 0; k < len; k++) {
                /* code */
                textData[stringNum][k] = data[i++];
              }
              textData[stringNum][k] = 0;
              stringNum++;
              // skip the odd number at the end
              i++;                
            }
          }
          for (stringNum = 0; stringNum < NEFTTECH_STRING_COUNT; stringNum++) {
            Serial.println(&textData[stringNum][0]);            
          }
        }
      }
      uidCopy(uidPrev, uid);
    }
    else
    {
      Serial.println("This doesn't seem to be an NTAG203 tag (UUID length != 7 bytes)!");
    }

    // // Wait a bit before trying again
    // Serial.println("\n\nSend a character to scan another tag!");
    // Serial.flush();
    // while (!Serial.available());
    // while (Serial.available()) {
    // Serial.read();
    // }
    // Serial.flush();
  }
  //Serial.println("waiting...");
  delay(250);
}
