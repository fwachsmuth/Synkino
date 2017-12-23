// Check: https://github.com/greiman/SSD1306Ascii
/*
 * √ RFID einbauen (Hardware) 
 * √ fps mit Film speichern (kommen in den Titel)
 * √ Umstellen von RS232 auf RS485: http://www.gammon.com.au/forum/?id=11428
 * RFID einbauen
 * Vorwärts/Rückwärtskorrektur (0.1 Sek) 
 * Settings Menü (#blades, pitch mode)
 * Abspielsettings an Auge schicken 
 * Anlernen neuer Soundtracks
 * Buzzer einbauen
 * OLED auf SPI umstellen: http://forum.arduino.cc/index.php?topic=108542.0
 * Statemachine schreiben
 * 64 für FN durch define ersetzen
 * 
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

#include <RS485_non_blocking.h>
int fAvailable () { return Serial1.available (); }
int fRead () { return Serial1.read (); }
size_t fWrite (const byte what) { Serial1.write (what); }
RS485 myChannel (fRead, fAvailable, fWrite, 70);



// define recipient addresses, basically bitmasked
#define RCPT_EYE        1
#define RCPT_UI         2
#define RCPT_EYE_UI     3
#define RCPT_AUDIO      4
#define RCPT_EYE_AUDIO  5
#define RCPT_UI_AUDIO   6
#define RCPT_ALL        7

// definde commands to remote nodes

#define ADJUST_SPEED        10  // long
#define PLAY_TRACK          11
#define PREP_TRACK          12
#define PAUSE_ON            13
#define PAUSE_OFF           14
#define STOP_TRACK          15
#define NAME_TRACK          16  // string numericPrefix
#define ADJUST_VOLUME       17

#define SHOW_CURRENT_FPS    20
#define SHOW_TARGET_FPS     21
#define SHOW_NAME           22
#define SHOW_TOO_FAST       23
#define SHOW_TOO_SLOW       24
#define SHOW_IN_SYNC        25
#define CLS                 26
#define SHOW_TEXT           27
#define FOUND_NAME          28  // long result, string trackname

#define CONTINOUS_MEASURE   30
#define LONGTERM_MEASURE    31
#define RESET_MEASURE       32
#define SET_TARGETFPS       33
#define SET_BLADE_SEGMENTS  34



// tons of states for our state machines
#define STATE_WAIT_FOR_SCAN       10
#define STATE_REQUEST_FILENAME    11 
#define STATE_PREPARE_PLAYBACK    12
#define STATE_WAIT_FOR_PROJSTART  13
#define STATE_PROJ_PAUSED         14

#define BUTTON_IDLE        20
#define BUTTON_DOWN        21
#define BUTTON_DEBOUNCE1   22
#define BUTTON_WAIT        23
#define BUTTON_UP          24
#define BUTTON_DEBOUNCE2   25
#define BUTTON_IGNOREDOWN  26


const byte numChars = 64;  // Achtung: RS485 Objekt ggf anpassen!

// variables to hold the parsed data from serial
//char receivedChar;
char commandFromRemote[numChars] = {0};
char parameterFromRemote[numChars] = {0};

#define RS485_ENABLE_PIN  6

// the data we broadcast to other devices
struct {
  byte address;
  byte command;
  long parameter;
  char asciis[numChars];
} message;

char trackLongFileName[64];

void setup(void) {
  pinMode(RS485_ENABLE_PIN, OUTPUT);
  digitalWrite (RS485_ENABLE_PIN, LOW);

  Serial.begin(9600);
  Serial1.begin(9600);
  myChannel.begin();

  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(fixed_bold10x15);
  oled.clear();
  
//  digitalWrite( 1, LOW );
//  pinMode( 1, INPUT ); // now we're tri-stated
//  u8g2.begin();

  nfc.begin();

  // configure board to read RFID tags
  nfc.SAMConfig();
}
  
void loop(void) {

 checkDebugKey();
 listenOnBus();
 

}

void listenOnBus() {
  if (myChannel.update ()) {
    memset (&message, 0, sizeof message);
    int len = myChannel.getLength ();
    if (len > sizeof message)
      len = sizeof message;
    memcpy (&message, myChannel.getData (), len);
    processMessage ();
  }
}  

void processMessage () {
  if (message.address == RCPT_UI 
   || message.address == RCPT_EYE_UI 
   || message.address == RCPT_UI_AUDIO
   || message.address == RCPT_ALL ) {   // could just check for bit 1 here, too
    Serial.print("Adr: ");
    Serial.print(message.address);
    Serial.print(", Cmd: ");
    Serial.print(message.command);
    Serial.print(", Param: ");
    Serial.print(message.parameter);
    Serial.print(", Str: ");
    Serial.println(message.asciis);

    switch(message.command) {
      case FOUND_NAME:
        strcpy(trackLongFileName, message.asciis);
        sendMessage(RCPT_EYE_AUDIO, PREP_TRACK, NULL, trackLongFileName);
        // TODO: Titelnamen noch für Display aufbereiten und anzeigen
        // ist strcopy nötig?
        //sendMessage(RCPT_EYE, SET_TARGETFPS, targetFps, "");
        
        break;
      case ADJUST_VOLUME:
        break;
      default:
        Serial.print("ERROR Unknown command: ");
        Serial.println(message.command);
    }
  }
}

void sendMessage (byte rcpnt, byte cmnd, long prmtr, char prmtrstr[]) {
  memset (&message, 0, sizeof message);
  message.address = rcpnt;
  message.command = cmnd;
  message.parameter = prmtr;
  strcpy(message.asciis, prmtrstr);

  // now send it
  digitalWrite (RS485_ENABLE_PIN, HIGH);  // enable sending
  myChannel.sendMsg ((byte *) &message, sizeof message);
  while (!(UCSR1A & (1 << UDRE1)))  // Wait for empty transmit buffer
  UCSR1A |= 1 << TXC1;              // mark transmission not complete
  while (!(UCSR1A & (1 << TXC1)));  // Wait for the transmission to complete
  digitalWrite (RS485_ENABLE_PIN, LOW);  // disable sending
}

  
void rfidPoll() {
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID 
                                            // (4 or 7 bytes depending on ISO14443A card type)
    
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

void checkDebugKey() {
  if (Serial.available()) {
    char c = Serial.read();
   
    if (c == '1') {
      sendMessage(RCPT_AUDIO, NAME_TRACK, NULL, "001");
    }
    if (c == '2') {
      sendMessage(RCPT_AUDIO, NAME_TRACK, NULL, "002");
    }
    if (c == '3') {
      sendMessage(RCPT_AUDIO, NAME_TRACK, NULL, "003");
    }
    if (c == '4') {
      sendMessage(RCPT_AUDIO, NAME_TRACK, NULL, "004");
    }
    if (c == '5') {
      sendMessage(RCPT_AUDIO, NAME_TRACK, NULL, "005");
    }
    if (c == '6') {
      sendMessage(RCPT_AUDIO, NAME_TRACK, NULL, "006");
    }
    if (c == '7') {
      sendMessage(RCPT_AUDIO, NAME_TRACK, NULL, "007");
    }
  }
}



