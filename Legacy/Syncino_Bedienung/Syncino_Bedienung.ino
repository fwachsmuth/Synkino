// Check: https://github.com/greiman/SSD1306Ascii
/*
 * √ RFID einbauen (Hardware) 
 * √ fps mit Film speichern (kommen in den Titel)
 * 
 * Umstellen von RS232 auf RS485: http://www.gammon.com.au/forum/?id=11428
 * RFID einbauen
 * Vorwärts/Rückwärtskorrektur (0.1 Sek) 
 * Settings Menü (#blades, pitch mode)
 * Abspielsettings an Auge schicken 
 * Anlernen neuer Soundtracks
 * Buzzer einbauen
 * OLED auf SPI umstellen
 * Statemachine schreiben
 * 
 */



#include <SPI.h>
#include <PN532_SPI.h>
#include "PN532.h"

PN532_SPI pn532spi(SPI, 10);
PN532 nfc(pn532spi);

#define I2C_ADDRESS 0x3C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"

SSD1306AsciiAvrI2c oled;

boolean newData = false;
const byte numChars = 64;
char receivedChars[numChars];   // an array to store the received data
char tempChars[numChars];       // temporary array for use when parsing

// variables to hold the parsed data from serial
//char receivedChar;
char commandFromRemote[numChars] = {0};
char parameterFromRemote[numChars] = {0};


void setup(void) {
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(fixed_bold10x15);
  oled.clear();
  
  Serial.begin(9600);

  Serial1.begin(2400);
//  digitalWrite( 1, LOW );
//  pinMode( 1, INPUT ); // now we're tri-stated
//  u8g2.begin();

  nfc.begin();

  // configure board to read RFID tags
  nfc.SAMConfig();
}

void loop(void) {
//  rfidPoll();
 
 //digitalWrite(13, HIGH);
 checkDebugKey();
 recvWithStartEndMarkers();
  if (newData == true) {
      strcpy(tempChars, receivedChars);
          // this temporary copy is necessary to protect the original data
          //   because strtok() used in parseData() replaces the commas with \0
      parseData();
      showParsedData();
      executeCommandFromRemote();
      newData = false;
  }

}

void rfidPoll() {
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
    
  // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
  // 'uid' will be populated with the UID, and uidLength will indicate
  // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);
  //uint8_t PN532::mifareultralight_ReadPage (uint8_t page, uint8_t *buffer)

  
  if (success) {
    // Display some basic information about the card
    Serial.println("Found an ISO14443A card");
    Serial.print("  UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("  UID Value: ");
    nfc.PrintHex(uid, uidLength);
    Serial.println("");
    
    if (uidLength == 4)
    {
      // We probably have a Mifare Classic card ... 
      Serial.println("Seems to be a Mifare Classic card (4 byte UID)");
    
      // Now we need to try to authenticate it for read/write access
      // Try with the factory default KeyA: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
      Serial.println("Trying to authenticate block 4 with default KEYA value");
      uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    
    // Start with block 4 (the first block of sector 1) since sector 0
    // contains the manufacturer data and it's probably better just
    // to leave it alone unless you know what you're doing
      success = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keya);
    
      if (success)
      {
        Serial.println("Sector 1 (Blocks 4..7) has been authenticated");
        uint8_t data[16];
    
        // If you want to write something to block 4 to test with, uncomment
    // the following line and this text should be read back in a minute
        // data = { 'a', 'd', 'a', 'f', 'r', 'u', 'i', 't', '.', 'c', 'o', 'm', 0, 0, 0, 0};
        // success = nfc.mifareclassic_WriteDataBlock (4, data);
//    uint8_t mifareclassic_WriteDataBlock (uint8_t blockNumber, uint8_t *data);
//    uint8_t mifareclassic_FormatNDEF (void);
//    uint8_t mifareclassic_WriteNDEFURI (uint8_t sectorNumber, uint8_t uriIdentifier, const char *url);
//
//    // Mifare Ultralight functions
//    uint8_t mifareultralight_ReadPage (uint8_t page, uint8_t *buffer);
//    uint8_t mifareultralight_WritePage (uint8_t page, uint8_t *buffer);

        // Try to read the contents of block 4
        success = nfc.mifareclassic_ReadDataBlock(4, data);
    
        if (success)
        {
          // Data seems to have been read ... spit it out
          Serial.println("Reading Block 4:");
          nfc.PrintHexChar(data, 16);
          Serial.println("");
      
          // Wait a bit before reading the card again
          delay(1000);
        }
        else
        {
          Serial.println("Ooops ... unable to read the requested block.  Try another key?");
        }
      }
      else
      {
        Serial.println("Ooops ... authentication failed: Try another key?");
      }
    }
    
    if (uidLength == 7)
    {
      // We probably have a Mifare Ultralight card ...
      Serial.println("Seems to be a Mifare Ultralight tag (7 byte UID)");
    
      // Try to read the first general-purpose user page (#4)
      Serial.println("Reading page 4");
      uint8_t data[32];
      success = nfc.mifareultralight_ReadPage (4, data);
      if (success)
      {
        // Data seems to have been read ... spit it out
        nfc.PrintHexChar(data, 4);
        Serial.println("");
    
        // Wait a bit before reading the card again
        delay(1000);
      }
      else
      {
        Serial.println("Ooops ... unable to read the requested page!?");
      }
    }
  }
}

void showFPS() {  
//  oled.clear();
  oled.println(parameterFromRemote);
//  u8g2.firstPage();
//  do {
//    u8g2.setFont(u8g2_font_logisoso30_tr); // choose a suitable font
//    u8g2.drawStr(0,30,parameterFromRemote); // write something to the internal memory
//  } while ( u8g2.nextPage() );
}

void showText() {  
  oled.println(parameterFromRemote);
//   u8g2.firstPage();
//  do {
//    u8g2.setFont(u8g2_font_logisoso16_tr); // choose a suitable font
//    u8g2.drawStr(0,16,parameterFromRemote); // write something to the internal memory
//  } while ( u8g2.nextPage() );
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;     
        }
    }
}

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,"|");      // get the first part - the command string
    strcpy(commandFromRemote, strtokIndx); // copy it to commandFromRemote
 
    strtokIndx = strtok(NULL, "|"); // this continues where the previous call left off
    strcpy(parameterFromRemote, strtokIndx); // copy it to parameterFromRemote
}

void executeCommandFromRemote() {
  if (strcmp(commandFromRemote,"D30") == 0) {
    showFPS();
  } else if (strcmp(commandFromRemote,"D16") == 0) {
    showText();
  }
  newData = false;
}

void showParsedData() {
    Serial.print("Command: ");
    Serial.print(commandFromRemote);
    if (strcmp(parameterFromRemote,"") > 0) {
      Serial.print(" - Parameter: ");
      Serial.println(parameterFromRemote);
    } else {
      Serial.println("");
    }
}

void checkDebugKey() {
  if (Serial.available()) {
    char c = Serial.read();
   
    if (c == '1') {
      sendWithStartEndMarkers("NAME", "001");
    }
    if (c == '2') {
      sendWithStartEndMarkers("NAME", "002");
    }
    if (c == '3') {
      sendWithStartEndMarkers("NAME", "003");
    }
    if (c == '4') {
      sendWithStartEndMarkers("NAME", "004");
    }
    if (c == '5') {
      sendWithStartEndMarkers("NAME", "005");
    }
    if (c == '6') {
      sendWithStartEndMarkers("NAME", "006");
    }
    if (c == '7') {
      sendWithStartEndMarkers("NAME", "007");
    }
  }
}

void sendWithStartEndMarkers(const String& command, const String& parameter) {

  Serial1.print("<");
  Serial1.print(command);
  Serial1.print("|");
  Serial1.print(parameter);
  Serial1.println(">");

  Serial.println(parameter);

  delay(1);
}


