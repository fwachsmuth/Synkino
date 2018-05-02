/*
 *  This is the frontend part of Synkino
 *  
 *  [ ] Mit FFFFF im EEPROM umgehen
 *  [ ] Make Display darker during Playback
 *  [ ] Get out of unsync-status
 *  
 *  [ ] Track number after editing Proj
 *  [ ] On Edit, Shutter Blade Position is wrong
 *  [ ] Verify what gets sent to AUDIO
 *  [ ] 8-3-1 default
 *  [ ] Compile and test patch 2.6 for wider upsampling trick
 *  
 *  [ ] Try http://arduino.land/Code/SmallSetup/
 *  [ ] Handle empty Projector List
 *  [ ] Limit to 8 Projectors
 *  [ ] Move Strings to PROGMEM
 *  [ ] Slow numeric selection?    
 *  [ ] waitForBttnRelease into the menu function?
 *      
 *  [ ] Add tick sounds to Menu :)
 *  [ ] Implement Extras Menu:
 *      [ ] Add Proportional On Measurement Option
 *      [ ] Configure Auto Power-Off Timeout
 *      [ ] Display Brightness
 *      [ ] Configure base Volume
 *  [ ] Implement Inc/Dec Sync Pos
 *  [ ] Implement end of track detection
 *  [ ] Implemet Reset
 *  
 *  PCB:
 *  - Offset ICSP Pins
 *  - Offset fTDI Pins
 *  - + mark on the backside
 *  - Display-Löcher minimal tiefer (1mm?)
 *  - Power On weiter nach rechts
 *  - Batteriefach
 *  - LEDs nicht ganz s nah an die Buchsen
 *  - Save Power: Disable Start mark Sensor after start...
 *  - Prep for EEPROM :)
 *  - Add Buzzer? 8.5x8.5 AAC801J-13 
 *  - C5 etwas kleiner?
 *    
 *  Change avrdude.conf in cd /Users/peaceman/Library/Arduino15/packages/arduino/tools/avrdude/6.3.0-arduino9/etc/ to burn 328 chips!
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Encoder.h>
#include <Wire.h>
#include <WireData.h>
#include <EEPROM.h>

// ---- Define the various Pins we use- --------------------------------------------
//
#define SPI_CS        10
#define SPI_DC        9
#define SPI_RESET     8
#define ENCODER_A     3
#define ENCODER_B     2
#define ENCODER_BTN   4
#define AUDIO_EN      6
#define POWER_OFF     7

// ---- Define the various Menu Screens --------------------------------------------
//
#define MAIN_MENU             1
#define PROJECTOR_ACTION_MENU 2
#define TRACK_SELECTION_MENU  3
#define SETTINGS_MENU         4
#define PLAYING_MENU          5
#define SHUTTER_MENU          6
#define STARTMARK_MENU        7
#define PID_MENU              8

// ---- Define the various Menu Item Positions -------------------------------------
//
#define MENU_ITEM_PROJECTOR       1
#define MENU_ITEM_SELECT_TRACK    2
#define MENU_ITEM_POWER_OFF       3
#define MENU_ITEM_EXTRAS          4

#define MENU_ITEM_NEW             1
#define MENU_ITEM_SELECT          2
#define MENU_ITEM_EDIT            3
#define MENU_ITEM_DELETE          4
#define MENU_ITEM_EXIT            5

#define MENU_ITEM_NAME            1
#define MENU_ITEM_SHUTTER_BLADES  2
#define MENU_ITEM_STARTMARK       3
#define MENU_ITEM_PID             4

#define MENU_ITEM_ONE             1
#define MENU_ITEM_TWO             2
#define MENU_ITEM_THREE           3
#define MENU_ITEM_FOUR            4
#define MENU_ITEM_FRAMES          1
#define MENU_ITEM_P               1  
#define MENU_ITEM_I               2
#define MENU_ITEM_D               3

#define MENU_ITEM_DEL_EEPROM      1
#define MENU_ITEM_DUMP            2

// ---- Define the I2C Commands ----------------------------------------------------
//
#define CMD_RESET               1   /* <---                   */
#define CMD_SET_SHUTTERBLADES   2   /* <--- (shutterBlades)   */
#define CMD_SET_STARTMARK       3   /* <--- (StartMarkOffset) */
#define CMD_SET_P               4   /* <--- (P-Value for PID) */
#define CMD_SET_I               5   /* <--- (I-Value for PID) */
#define CMD_SET_D               6   /* <--- (D-Value for PID) */
#define CMD_INC_FRAME           7   /* <---                   */
#define CMD_DEC_FRAME           8   /* <---                   */
#define CMD_LOAD_TRACK          9   /* <--- (trackId)         */

#define CMD_FOUND_FMT           10  /* ---> (fileFormat)      */
#define CMD_FOUND_FPS           11  /* ---> (fps)             */
#define CMD_CURRENT_AUDIOSEC    12  /* ---> (SecNo)           */
#define CMD_PROJ_PAUSE          13  /* --->                   */
#define CMD_PROJ_PLAY           14  /* --->                   */
#define CMD_FOUND_TRACKLENGTH   15  /* ---> (TrackLength)     */
#define CMD_OOSYNC              16  /* ---> (frameCount)      */
#define CMD_SHOW_ERROR          17  /* ---> (ErrorCode)       */
#define CMD_TRACK_LOADED        18  /* --->                   */
#define CMD_STARTMARK_HIT       19  /* --->                   */

// ---- Define the various States --------------------------------------------------
//
#define MAIN_MENU               1
#define SELECT_TRACK            2
#define WAIT_FOR_LOADING        3
#define TRACK_LOADED            4
#define SYNC_PLAY               5
#define GET_NAME                1
#define JUST_PRESSED            2
#define LONG_PRESSED            3

#define NEW                     0

// ---- Define some graphics -------------------------------------------------------
//
#define busybee_xbm_width  15
#define busybee_xbm_height 15
static const unsigned char busybee_xbm_bits[] U8X8_PROGMEM = {
   0x10, 0x00, 0x10, 0x3C, 0x00, 0x46, 0x60, 0x43, 0x63, 0x21, 0x98, 0x51,
   0xD8, 0x2A, 0x60, 0x07, 0xB8, 0x1A, 0xCC, 0x3F, 0x86, 0x06, 0x42, 0x7B,
   0x22, 0x1B, 0x52, 0x6A, 0x2C, 0x28 };

#define play_xbm_width  9
#define play_xbm_height 9
static const unsigned char play_xbm_bits[] U8X8_PROGMEM = {
   0x03, 0x00, 0x0F, 0x00, 0x3F, 0x00, 0xFF, 0x00, 0xFF, 0x01, 0xFF, 0x00,
   0x3F, 0x00, 0x0F, 0x00, 0x03, 0x00 };

#define pause_xbm_width  8
#define pause_xbm_height 8
static const unsigned char pause_xbm_bits[] U8X8_PROGMEM = {
   0xE7, 0xE7, 0xE7, 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };

#define sync_xbm_width 20
#define sync_xbm_height 9
static const unsigned char sync_xbm_bits[] U8X8_PROGMEM = {
   0xFE, 0xFF, 0x07, 0xFF, 0xFF, 0x0F, 0xA7, 0xDA, 0x0C, 0xBB, 0x52, 0x0F,
   0x77, 0x4B, 0x0F, 0x6F, 0x5B, 0x0F, 0x73, 0xDB, 0x0C, 0xFF, 0xFF, 0x0F,
   0xFE, 0xFF, 0x07 };

#define logo_xbm_width 96
#define logo_xbm_height 30
#define logo_xbm_x 14
#define logo_xbm_y 16
static const unsigned char logo_xbm_bits[] U8X8_PROGMEM = {
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0x03,
   0x00, 0x1F, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x0E,
   0xC0, 0x60, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x0E, 0x38, 0xE0, 0x30,
   0x30, 0x08, 0x80, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x0E, 0x38, 0xE0, 0x40,
   0x08, 0x14, 0x9F, 0xFF, 0xFF, 0xFF, 0xFF, 0x5F, 0x00, 0x00, 0x00, 0x80,
   0x04, 0x14, 0x8F, 0x0F, 0xEC, 0xDB, 0xEF, 0x4D, 0x4E, 0x20, 0x3C, 0xC0,
   0x04, 0x14, 0x87, 0xF7, 0xEF, 0x9B, 0xEF, 0x5D, 0xC4, 0x20, 0x42, 0xC0,
   0x02, 0x14, 0x8B, 0xFB, 0xDF, 0x9D, 0xEF, 0x2D, 0xC4, 0x20, 0x42, 0x80,
   0x02, 0x0E, 0x89, 0xFB, 0xDF, 0x5D, 0xEF, 0x3D, 0x44, 0x21, 0x81, 0x80,
   0x01, 0x05, 0x90, 0xFB, 0xBF, 0x5E, 0xEF, 0x1D, 0x44, 0x21, 0x81, 0x80,
   0x01, 0x05, 0x90, 0xF7, 0x7F, 0xDF, 0xEE, 0x15, 0x44, 0x22, 0x81, 0x80,
   0x81, 0x04, 0x90, 0x0F, 0x7F, 0xDF, 0xEE, 0x09, 0x44, 0x22, 0x81, 0x40,
   0x81, 0x1E, 0x90, 0xFF, 0x7E, 0xDF, 0xED, 0x19, 0x44, 0x24, 0x81, 0x40,
   0x81, 0x26, 0x90, 0xFF, 0x7D, 0xDF, 0xEB, 0x05, 0x44, 0x28, 0x81, 0x40,
   0x92, 0x24, 0x88, 0xFF, 0x7D, 0xDF, 0xEB, 0x2D, 0x44, 0x28, 0x81, 0x40,
   0x1A, 0x15, 0x88, 0xFF, 0x7D, 0xDF, 0xE7, 0x2D, 0x44, 0x30, 0x42, 0x40,
   0x1C, 0x1E, 0x84, 0xFF, 0x7E, 0xDF, 0xE7, 0x5D, 0x44, 0x30, 0x42, 0x40,
   0x1E, 0x05, 0x84, 0x03, 0x7F, 0xDF, 0xEF, 0x4D, 0x4E, 0x20, 0x3C, 0x40,
   0x1F, 0x05, 0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x40,
   0x00, 0x82, 0x81, 0xFF, 0xFF, 0xFF, 0xFF, 0xCF, 0x00, 0x00, 0x00, 0x40,
   0xC0, 0x60, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x43,
   0x00, 0x1F, 0x00, 0xFE, 0xFF, 0xFF, 0xFF, 0x3F, 0x02, 0x00, 0x00, 0x4E,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x70,
   0x00, 0x00, 0x2A, 0x40, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x40,
   0x00, 0x00, 0x21, 0x40, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x20,
   0x00, 0x00, 0x2B, 0x4B, 0x9D, 0x0D, 0x67, 0x07, 0x02, 0x00, 0x00, 0x10,
   0x00, 0x00, 0x29, 0xD5, 0x54, 0x14, 0x15, 0x05, 0x1C, 0x00, 0x00, 0x0C,
   0x00, 0x00, 0x49, 0x55, 0x5D, 0x54, 0x17, 0x07, 0xE0, 0x00, 0xC0, 0x03,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0xFF, 0x3F, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00
};

// ---- Define useful time constants and macros ------------------------------------
//
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)

// ---- Initialize Objects ---------------------------------------------------------
//
U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, SPI_CS, SPI_DC, SPI_RESET);
Encoder myEnc(ENCODER_A, ENCODER_B);

// ---- Initialize Consts and Vars -------------------------------------------------
//

const int myAddress = 0x07;     // Our i2c address here

const char *main_menu = 
  "Projector\n"
  "Select Track\n"
  "Power Off\n"
  "Extras";

const char *projector_action_menu =
  "New\n"
  "Select\n"
  "Edit\n"
  "Delete\n"
  "Exit";

const char *projector_config_menu =
  "Name\n"
  "Shutter Blades\n"
  "Start Mark\n"
  "PID";

const char *shutterblade_menu =
  "1\n"
  "2\n"
  "3\n"
  "4";

const char *extras_menu =
  "Del\n"
  "Dump";

const byte maxProjectorNameLength = 12;
const byte maxProjectorCount = 8;
char newProjectorName[maxProjectorNameLength + 1];
char projectorSelection_menu[maxProjectorNameLength * maxProjectorCount + maxProjectorCount]; 
//char errorMsg[24];

uint8_t mainMenuSelection               = MENU_ITEM_SELECT_TRACK;
uint8_t projectorActionMenuSelection    = MENU_ITEM_SELECT;
uint8_t projectorSelectionMenuSelection = 0;
uint8_t projectorConfigMenuSelection    = MENU_ITEM_NAME;
uint8_t shutterBladesMenuSelection      = MENU_ITEM_TWO;
uint8_t extrasMenuSelection             = 2;


volatile bool haveI2Cdata = false;
volatile uint8_t i2cCommand;
volatile long i2cParameter;

uint8_t myState = MAIN_MENU;

uint8_t fps = 0;
uint8_t fileType = 0;
uint8_t trackLoaded = 0;
uint8_t startMarkHit = 0;
uint8_t projectorPaused = 0;

unsigned long totalSeconds = 0;
uint8_t hours   = 0;
uint8_t minutes = 0;
uint8_t seconds = 0;

volatile unsigned long lastActivityMillies = millis();

int oosyncFrames = 0;
int lastEncRead;


byte new_p = 8;
byte new_i = 3;
byte new_d = 1;
byte newStartmarkOffset = 0;

byte lastProjectorUsed = 1;

/* Below is the EEPROM struct to save Projector Configs.
 *  Before the structs start, there are two header bytes:
 *  EEPROM.read(0, projectorCount);
 *  EEPROM.read(1, lastProjectorUsed);
 */
struct Projector {          // 19 Bytes per Projector
  byte  index;
  byte  shutterBladeCount;
  byte  startmarkOffset;
  byte  p;
  byte  i;
  byte  d;
  char  name[maxProjectorNameLength + 1];
};

// ---- Setup ---------------------------------------------------------------------
//
void setup(void) {
  pinMode(SPI_CS, OUTPUT);
  pinMode(SPI_DC, OUTPUT);
  pinMode(AUDIO_EN, OUTPUT);
  pinMode(POWER_OFF, OUTPUT);
  pinMode(ENCODER_BTN, INPUT);
  digitalWrite(SPI_CS, 0);
  digitalWrite(SPI_DC, 0);		
  digitalWrite(ENCODER_BTN, HIGH);
  digitalWrite(POWER_OFF, HIGH);
 
  myEnc.write(16000);

  Serial.begin(115200);

  Wire.begin(myAddress);
  Wire.onReceive(i2cReceive);
  Wire.onRequest(i2cRequest);
 
  u8g2.begin();  
  //u8g2.begin(/*Select=*/ 7, /*Right/Next=*/ A1, /*Left/Prev=*/ A2, /*Up=*/ A0, /*Down=*/ A3, /*Home/Cancel=*/ 5);
  u8g2.setFont(u8g2_font_helvR10_tr);

  u8g2.firstPage();
  do {
    u8g2.drawXBMP(logo_xbm_x, logo_xbm_y, logo_xbm_width, logo_xbm_height, logo_xbm_bits);
  } while ( u8g2.nextPage() );
  delay(2000);

  myState = MAIN_MENU;

  lastProjectorUsed = EEPROM.read(1);
  loadProjectorConfig(lastProjectorUsed);

  noInterrupts();
  // Setup Timer1 to periodically (every second) check if a Power-Off is due 
  TCCR1A = 0;                             // Clear registers
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 15624;                          // 1 Hz (16000000/((15624+1)*1024))
  TCCR1B |= (1 << WGM12);                 // CTC
  TCCR1B |= (1 << CS12) | (1 << CS10);    // Prescaler 1024
  TIMSK1 |= (1 << OCIE1A);                // Output Compare Match A Interrupt Enable
  interrupts();
  
}


// ---- Main Loop ------------------------------------------------------------------
//
void loop(void) {
  if (haveI2Cdata) {
    lastActivityMillies = millis(); 
    
    switch (i2cCommand) {   // Debug output
      case 10: Serial.println(F("CMD_FOUND_FMT"));
               Serial.println(i2cParameter);
               break;
      case 11: Serial.print(F("CMD_FOUND_FPS: "));
               Serial.println(i2cParameter);
               break;
      case 12: 
               Serial.print(F("CMD_CURRENT_AUDIOSEC: "));
               Serial.println(i2cParameter);
               break;
      case 13: Serial.println(F("CMD_PROJ_PAUSE"));
               break;
      case 14: Serial.println(F("CMD_PROJ_PLAY"));
               break;
      case 15: Serial.print(F("CMD_FOUND_TRACKLENGTH: "));
               Serial.println(i2cParameter);
               break;
      case 16: Serial.print(F("CMD_OOSYNC: "));
               Serial.println(i2cParameter);
               break;
      case 17: Serial.print(F("CMD_SHOW_ERROR: "));
               Serial.println(i2cParameter);
               break;
      case 18: Serial.println(F("CMD_TRACK_LOADED"));
               break;
      case 19: Serial.println(F("CMD_STARTMARK_HIT"));
               break;
      default: Serial.println(i2cCommand);
               Serial.println(i2cParameter);
    }
  
    switch (i2cCommand) {   // Feed the FSM
      case CMD_FOUND_FMT:
      break; 
      case CMD_FOUND_FPS:
        fps = i2cParameter;
      break; 
      case CMD_CURRENT_AUDIOSEC:
        hours   = numberOfHours(i2cParameter);
        minutes = numberOfMinutes(i2cParameter);
        seconds = numberOfSeconds(i2cParameter);
      break; 
      case CMD_PROJ_PAUSE:
        projectorPaused = 1;
      break; 
      case CMD_PROJ_PLAY:
        projectorPaused = 0;
      break; 
      case CMD_FOUND_TRACKLENGTH:
      break; 
      case CMD_OOSYNC:
        oosyncFrames = i2cParameter;
      break; 
      case CMD_SHOW_ERROR:
        switch (i2cParameter) { // see https://mpflaga.github.io/Arduino_Library-vs1053_for_SdFat/
          case 2:
            showError("File not found.","");
            waitForBttnRelease();
            myState = MENU_ITEM_SELECT_TRACK;
            break;
          case 16:
            showError("DSP Patch could","not be loaded."); 
          case 20:
            showError("No SD Card found.","");
            break;
          default:
            showError("Oops:", i2cParameter);
            //  1: Already playing track
            //  3: indicates that the DSP is in reset
            // 11: Failure of SdFat to initialize physical contact with the SdCard
            // 12: Failure of SdFat to start the SdCard's volume
            // 13: Failure of SdFat to mount the root directory on the volume of the SdCard
            // 14: Other than default values were found in the SCI_MODE register
            // 15: SCI_CLOCKF did not read back and verify the configured value
            break;
        }
      break; 
      case CMD_TRACK_LOADED:
        trackLoaded = 1;
      break;
      case  CMD_STARTMARK_HIT:
        startMarkHit = 1;
      break;
      default:
        Serial.println(i2cCommand);
        Serial.println(i2cParameter);
      break;
    }
  }
  haveI2Cdata = false;  

  Serial.println(millis() - lastActivityMillies);

  // State Machine ----------------------------------------------------------------
  //
  switch (myState) {
    case MAIN_MENU:
      mainMenuSelection = u8g2.userInterfaceSelectionList("Main Menu", MENU_ITEM_SELECT_TRACK, main_menu);
      waitForBttnRelease(); 
      
      switch (mainMenuSelection) {
        case MENU_ITEM_PROJECTOR:

          projectorActionMenuSelection = u8g2.userInterfaceSelectionList("Projector", MENU_ITEM_SELECT, projector_action_menu);
          waitForBttnRelease();
        
          if (projectorActionMenuSelection == MENU_ITEM_NEW) { 
            gatherProjectorData();
            saveProjector(NEW);
            
          } else if (projectorActionMenuSelection == MENU_ITEM_SELECT) {
            makeProjectorSelectionMenu();
            projectorSelectionMenuSelection = u8g2.userInterfaceSelectionList("Select Projector", lastProjectorUsed, projectorSelection_menu); 
            waitForBttnRelease();
            loadProjectorConfig(projectorSelectionMenuSelection);
            
          } else if (projectorActionMenuSelection == MENU_ITEM_EDIT) {
            makeProjectorSelectionMenu();
            projectorSelectionMenuSelection = u8g2.userInterfaceSelectionList("Edit Projector", lastProjectorUsed, projectorSelection_menu);
            waitForBttnRelease();
            loadProjectorConfig(projectorSelectionMenuSelection);
            gatherProjectorData();
            saveProjector(projectorSelectionMenuSelection);
            
          } else if (projectorActionMenuSelection == MENU_ITEM_DELETE) {
            byte currentProjectorCount = makeProjectorSelectionMenu();
            projectorSelectionMenuSelection = u8g2.userInterfaceSelectionList("Delete Projector", currentProjectorCount, projectorSelection_menu);
            waitForBttnRelease();
            Serial.print("DelMenu-Ausw: ");
            Serial.println(projectorSelectionMenuSelection);
            deleteProjector(projectorSelectionMenuSelection);

          } else if (projectorActionMenuSelection == MENU_ITEM_EXIT) {
            // nothing to do here            
          }
              
          break;
        case MENU_ITEM_SELECT_TRACK:
          myState = SELECT_TRACK;
          break;
        case MENU_ITEM_POWER_OFF:
          shutdownSelf();
          break;  
        case MENU_ITEM_EXTRAS:
          extrasMenuSelection = u8g2.userInterfaceSelectionList("Extras", MENU_ITEM_DEL_EEPROM, extras_menu);
          waitForBttnRelease();
          switch (extrasMenuSelection) {
            case MENU_ITEM_DEL_EEPROM:
              u8g2.setFont(u8g2_font_helvR08_tr);
              u8g2.setFontRefHeightAll();    /* this will add some extra space for the text inside the buttons */
              byte choice;
              choice = u8g2.userInterfaceMessage("Delete EEPROM", "Are you sure?", "", " Cancel \n Yes ");
              waitForBttnRelease();
              
              if (choice == 2) {
                for (int i = 0 ; i < EEPROM.length() ; i++) {
                  EEPROM.write(i, 0);
                }
              }
              u8g2.setFont(u8g2_font_helvR10_tr);
              u8g2.setFontRefHeightText();    
            break;
            case MENU_ITEM_DUMP:
              e2reader();
            break;
            default:
            break;
          }
          break;
        default:
          break;
      }
     break;
    case SELECT_TRACK:
      static int trackChosen;
      trackChosen = selectTrackScreen();
      if (trackChosen != 0) {
        tellAudioPlayer(CMD_LOAD_TRACK, trackChosen);
        myState = WAIT_FOR_LOADING;
      } else {
        myState = MAIN_MENU;
      }
      break;
    case WAIT_FOR_LOADING:
      drawBusyBee(90,10);
      if ((fps != 0) && (trackLoaded != 0)) {
        drawWaitForPlayingMenu(trackChosen, fps);
        myState = TRACK_LOADED;
        digitalWrite(AUDIO_EN, HIGH);
      } // Todo: Timeout und Error Handler
      break;
    case TRACK_LOADED:
      if (startMarkHit != 0) {
        myState = SYNC_PLAY;
      }
      break;
    case SYNC_PLAY:
      drawPlayingMenu(trackChosen, fps);
      break;
    default:
      break;
  }
}

void deleteProjector(byte thisProjector) {
  byte projectorCount;
  projectorCount = EEPROM.read(0);
  Projector aProjector;
  if (thisProjector == projectorCount) {
    Serial.print("Zeroing out (in theory): #");
    Serial.println(thisProjector);
  } else {
    for (byte i = thisProjector; i < projectorCount; i++) {
//      Serial.print("Moving #");
//      Serial.print(i + 1);
//      Serial.print(" to #");
//      Serial.println(i);
      EEPROM.get(i * sizeof(aProjector) + 2, aProjector);
      EEPROM.put((i - 1) * sizeof(aProjector) + 2, aProjector);
      e2reader();

    }
  }
  EEPROM.write(0, projectorCount - 1);
   e2reader();
  
//  Serial.println(lastProjectorUsed);
  if (thisProjector == lastProjectorUsed) {
    loadProjectorConfig(1);   // config 1
//    Serial.println("Switching active Projector to #1.");
  }
}

void saveProjector(byte thisProjector) {
  byte projectorCount;
  projectorCount = EEPROM.read(0);
//  Serial.println(projectorCount);
  
  Projector aProjector;
  aProjector.shutterBladeCount = shutterBladesMenuSelection;
  aProjector.startmarkOffset = newStartmarkOffset;
  aProjector.p = new_p;
  aProjector.i = new_i;
  aProjector.d = new_d;
  strcpy(aProjector.name, newProjectorName);

  if (thisProjector == NEW) {             // We have a NEW projector here
    EEPROM.write(0, projectorCount + 1);
    EEPROM.write(1, projectorCount + 1);
    aProjector.index = projectorCount + 1;
    EEPROM.put((projectorCount * sizeof(aProjector) + 2), aProjector);
    e2reader();

  } else {
    aProjector.index = thisProjector;
    EEPROM.write(1, thisProjector);
    EEPROM.put(((thisProjector - 1) * sizeof(aProjector) + 2), aProjector);
  e2reader();

  }
}

void gatherProjectorData() {
  handleProjectorNameInput();
  handleShutterbladeInput();
  handleStartmarkInput();
  handlePIDinput();
}

void loadProjectorConfig(uint8_t projNo) {
  Projector aProjector;
  EEPROM.get((projNo - 1) * sizeof(aProjector) + 2, aProjector);
  
  shutterBladesMenuSelection = aProjector.shutterBladeCount;
  newStartmarkOffset = aProjector.startmarkOffset;
  new_p = aProjector.p;
  new_i = aProjector.i;
  new_d = aProjector.d;
  
  strcpy(newProjectorName, aProjector.name);

  tellAudioPlayer(CMD_SET_SHUTTERBLADES, aProjector.shutterBladeCount);
  tellAudioPlayer(CMD_SET_STARTMARK, aProjector.startmarkOffset);
  tellAudioPlayer(CMD_SET_P, aProjector.p);   
  tellAudioPlayer(CMD_SET_I, aProjector.i);
  tellAudioPlayer(CMD_SET_D, aProjector.d);

  EEPROM.write(1, aProjector.index);
  e2reader();

}

byte makeProjectorSelectionMenu() {
  projectorSelection_menu[0] = 0;   // Empty this out and be prepared for an empty list
  byte projectorCount;
  projectorCount = EEPROM.read(0);
  lastProjectorUsed = EEPROM.read(1);
  
  Projector aProjector;
  for (byte i = 0; i < projectorCount; i++) {
    EEPROM.get(i * sizeof(aProjector) + 2, aProjector);
    strcat(projectorSelection_menu, aProjector.name);
    if (i < (projectorCount - 1)) {           // no \n for the last menu entry
      strcat(projectorSelection_menu, "\n");
    }
  }
  return projectorCount;
}

void waitForBttnRelease() {
  while (digitalRead(ENCODER_BTN) == 0) {};       // wait for button release
}
void handleProjectorNameInput() {
  char localChar;
  unsigned long newEncPosition;
  byte charIndex = 0;
  bool inputFinished = false;  
  unsigned long lastMillis;  
  bool firstUse = true;                        

  if (projectorActionMenuSelection == MENU_ITEM_EDIT) {
    if (strlen(newProjectorName) == maxProjectorNameLength) {
      const char lastChar = newProjectorName[maxProjectorNameLength - 1];

      newProjectorName[maxProjectorNameLength - 1] = '\0';
           if (lastChar >= 65 && lastChar <=  90) myEnc.write((lastChar - 65) * 2 + 16000);
      else if (lastChar >= 97 && lastChar <= 122) myEnc.write((lastChar - 97) * 2 + 16052);
      else if (lastChar >= 48 && lastChar <=  57) myEnc.write((lastChar - 48) * 2 + 16106);
      else    myEnc.write(16104);
    } else {
      myEnc.write(16126);         // to start with "Delete"
    }
    charIndex = strlen(newProjectorName);
    firstUse = false;
    for (byte i = charIndex + 1; i < maxProjectorNameLength; i++) {
      newProjectorName[i] = 0;
    }
  } else {
    for (byte i = 0; i < maxProjectorNameLength; i++) {
      newProjectorName[i] = 0;
    }
    myEnc.write(16000);           // to start with "A" 
  }
  
  while (charIndex < maxProjectorNameLength && !inputFinished) {
    while (digitalRead(ENCODER_BTN) == 1) {
//      newEncPosition = (myEnc.read() >> 1) % 64;
      newEncPosition = (myEnc.read() >> 2) % 64;
      /*
       * Chr : Ascii Code      | newEncPos | myEnc.write | #
       * ----:-----------------|-----------|-------------|---
       * A-Z : Ascii  65 -  90 |   0 - 25  |    0 -  50  | 26
       * a-z : Ascii  97 - 122 |  26 - 51  |   52 - 102  | 26
       * Spc : Ascii  32       |  52       |  104        |  1
       * 0-9 : Ascii  48 -  57 |  53 - 62  |  106 - 124  | 10
       * Del : Ascii 127       |  63       |  126        |  1
       * 
       */
           if (                        newEncPosition <= 25) localChar = newEncPosition + 65;
      else if (newEncPosition >= 26 && newEncPosition <= 51) localChar = newEncPosition + 71;
      else if (newEncPosition == 52                        ) localChar = 32;
      else if (newEncPosition >= 53 && newEncPosition <= 62) localChar = newEncPosition -  5;
      else    localChar = 127;
      lastMillis = millis();
      handleStringInputGraphically(GET_NAME, localChar, lastMillis, firstUse);
    }
    lastMillis = millis();
    while (digitalRead(ENCODER_BTN) == 0 && !inputFinished) {
      delay(50);
      inputFinished = handleStringInputGraphically(JUST_PRESSED, localChar, lastMillis, firstUse);
    }
    while (digitalRead(ENCODER_BTN) == 0 && inputFinished) {
       handleStringInputGraphically(LONG_PRESSED, localChar, 0, firstUse);
    }
    if (localChar == 127) {   // Delete
      charIndex--;            // Is it safe to become negative here?
      newProjectorName[charIndex] = 0;  
    } else if (!inputFinished) {
      newProjectorName[charIndex] = localChar;
      charIndex++;
      if (firstUse) myEnc.write(16052);   // switch to lower case 'a'
      firstUse = false;
    }
  }
  while (digitalRead(ENCODER_BTN) == 0) {}
  //inputFinished = false;
  newProjectorName[charIndex] = '\0';
}

void handleShutterbladeInput() {
  shutterBladesMenuSelection = u8g2.userInterfaceSelectionList("# Shutter Blades", shutterBladesMenuSelection, shutterblade_menu);
  waitForBttnRelease();
}

void handleStartmarkInput() {
  u8g2.userInterfaceInputValue("Start Mark Offset:", "", &newStartmarkOffset, 0, 255, 3, " Frames");
  waitForBttnRelease();
}

void handlePIDinput () {
  u8g2.userInterfaceInputValue("Proportional:", "", &new_p, 0, 99, 2, "");
  waitForBttnRelease();
  u8g2.userInterfaceInputValue("Integral:", "", &new_i, 0, 99, 2, "");
  waitForBttnRelease();
  u8g2.userInterfaceInputValue("Derivative:", "", &new_d, 0, 99, 2, "");
  waitForBttnRelease();
}

bool handleStringInputGraphically(byte action, char localChar, unsigned long lastMillis, bool firstUse) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvR10_tr);
    u8g2.setCursor(15,35);
    u8g2.print(newProjectorName);

    if (action == GET_NAME) {
      if (lastMillis % 300 > 150) {
        if      (localChar ==  32) u8g2.print("_");
        else if (localChar == 127) {  // Delete
          u8g2.setFont(u8g2_font_m2icon_9_tf);
          u8g2.print("a"); // https://github.com/olikraus/u8g2/wiki/fntpic/u8g2_font_m2icon_9_tf.png
          u8g2.setFont(u8g2_font_helvR10_tr);
        }
        else {
          u8g2.print(localChar);
        }
      }

      u8g2.setFont(u8g2_font_helvR08_tr);
      if (localChar ==  32) { 
        u8g2.drawStr(45, 55, "[Space]"); }
      else if (localChar == 127) { 
        u8g2.drawStr(34, 55, "[Delete last]"); }
      else {
        if (firstUse) { 
          u8g2.drawStr(16, 55, "[Turn and push Knob]"); }
        else { 
          u8g2.drawStr(14, 55, "[Long Press to Finish]"); }
      }
    } 

    else if (action == JUST_PRESSED) { 
      u8g2.setFont(u8g2_font_helvR08_tr);
      u8g2.drawStr(11, 55, "[Keep pressed to Save]"); 
      if (millis() - lastMillis > 1500) {
        return true;
      }
    }
    
    else if (action == LONG_PRESSED) { 
      u8g2.setFont(u8g2_font_helvR08_tr);
      u8g2.drawStr(41, 55, "[Saved!]"); 
    }
  u8g2.drawStr(19, 14, "Set Projector Name");
  u8g2.setFont(u8g2_font_helvR10_tr);
  } while ( u8g2.nextPage() );
}

void drawWaitForPlayingMenu(int trackNo, byte fps) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.setCursor(0,8);
    u8g2.print(newProjectorName);
    u8g2.setCursor(90,8);
    u8g2.print("Film ");
    if (trackNo < 10)  u8g2.print("0");
    if (trackNo < 100) u8g2.print("0");
    u8g2.print(trackNo);
    u8g2.setCursor(98,62);
    u8g2.print(fps);
    u8g2.print(" fps");
    u8g2.setFont(u8g2_font_helvR10_tr);
    u8g2.drawStr(27,28,"Waiting for");    
    u8g2.drawStr(16,46,"Projector Start");    
    u8g2.drawXBMP(60, 54, pause_xbm_width, pause_xbm_height, pause_xbm_bits);
  } while ( u8g2.nextPage() );
}

void drawPlayingMenu(int trackNo, byte fps) {
  u8g2.firstPage();

  unsigned long currentmillis = millis();
  do {
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.setCursor(0,8);
    u8g2.print(newProjectorName);
    u8g2.setCursor(90,8);
    u8g2.print("Film ");
    if (trackNo < 10)  u8g2.print("0");
    if (trackNo < 100) u8g2.print("0");
    u8g2.print(trackNo);
    u8g2.setCursor(98,62);
    u8g2.print(fps);
    u8g2.print(" fps");
    u8g2.setFont(u8g2_font_inb24_mn);
    u8g2.drawStr(20,36,":");
    u8g2.drawStr(71,36,":");
    u8g2.setCursor(4,40);
    u8g2.print(hours);
    u8g2.setCursor(35,40);
    if (minutes < 10) u8g2.print("0");
    u8g2.print(minutes);
    u8g2.setCursor(85,40);
    if (seconds < 10) u8g2.print("0");
    u8g2.print(seconds);
    
    if (projectorPaused == 1) {
      u8g2.drawXBMP(60, 54, pause_xbm_width, pause_xbm_height, pause_xbm_bits);
    } else {
      u8g2.drawXBMP(60, 54, play_xbm_width, play_xbm_height, play_xbm_bits);
    }

    if (oosyncFrames == 0) {
      u8g2.drawXBMP(2, 54, sync_xbm_width, sync_xbm_height, sync_xbm_bits);
    } else {
      if (currentmillis % 700 > 350) {
        u8g2.drawXBMP(2, 54, sync_xbm_width, sync_xbm_height, sync_xbm_bits);
      }
      u8g2.setFont(u8g2_font_helvR08_tr);
      u8g2.setCursor(24,62);
      if (oosyncFrames > 0) u8g2.print("+");
      u8g2.print(oosyncFrames);
    }
  } while ( u8g2.nextPage() );
}
void drawBusyBee(byte x, byte y) {
  u8g2.firstPage();
  do {
    u8g2.drawXBMP(x, y, busybee_xbm_width, busybee_xbm_height, busybee_xbm_bits);
    u8g2.setFont(u8g2_font_helvR10_tr);
    u8g2.drawStr(8,50,"Loading...");
  } while ( u8g2.nextPage() );
}
void tellAudioPlayer(byte command, long parameter) {
  Wire.beginTransmission(8); // This is the Audio Player
  wireWriteData(command);  
  wireWriteData(parameter);  
  Wire.endTransmission();    // stop transmitting
}

uint16_t selectTrackScreen() {
  int parentMenuEncPosition;
  parentMenuEncPosition = myEnc.read();
  int newEncPosition;
  int oldPosition;
// myEnc.write(16001);
  while (digitalRead(ENCODER_BTN) == 1) {     // adjust ### as long as button not pressed
    newEncPosition = myEnc.read();
//    newEncPosition = (newEncPosition >> 1) % 1000;
    newEncPosition = (newEncPosition >> 2) % 1000;
    if (newEncPosition != oldPosition) {
      oldPosition = newEncPosition;
      
      u8g2.firstPage();
      if (newEncPosition == 0) {
        do {
          u8g2.setFont(u8g2_font_helvR10_tr);
          u8g2.drawStr(18,35,"< Main Menu");
        } while ( u8g2.nextPage() );
      } else {
        do {
          u8g2.setFont(u8g2_font_inb46_mn);
          u8g2.setCursor(9, 55);
          if (newEncPosition < 10)  u8g2.print("0");
          if (newEncPosition < 100) u8g2.print("0");
          u8g2.print(newEncPosition);
        } while ( u8g2.nextPage() );
      }  
    }
  }
  waitForBttnRelease();
  oldPosition = 0;
  myEnc.write(parentMenuEncPosition);
  u8g2.setFont(u8g2_font_helvR10_tr);   // Only until we go to the PLAYING_MENU here
  return newEncPosition;
}

// This overwrites the weak function in u8x8_debounce.c
uint8_t u8x8_GetMenuEvent(u8x8_t *u8x8) {
//  int newEncPosition = myEnc.read() >> 1;
  int newEncPosition = myEnc.read() >> 2;
  static int oldEncPosition = 8000;
  int encoderBttn = digitalRead(ENCODER_BTN);

  if (newEncPosition < oldEncPosition) {
    oldEncPosition = newEncPosition;
    delay(50);
    return U8X8_MSG_GPIO_MENU_UP;
  } else if (newEncPosition > oldEncPosition) {
    oldEncPosition = newEncPosition;
    delay(50);
    return U8X8_MSG_GPIO_MENU_DOWN;
  } else if (encoderBttn == 0) {
    delay(50);
    return U8X8_MSG_GPIO_MENU_SELECT;
  } else {
    return 0;
  }
}

void i2cReceive (int howMany) {
  if (howMany >= (sizeof i2cCommand) + (sizeof i2cParameter)) {
     wireReadData(i2cCommand);   
     wireReadData(i2cParameter);   
     haveI2Cdata = true;     
   }  // end if have enough data
 }  // end of receive-ISR


void i2cRequest() {
  // wireWriteData(myData);
}


void e2reader(){
  char buffer[16];
  char valuePrint[4];
  byte value;
  unsigned int address;
  uint8_t trailingSpace = 2;     
  
  for(address = 0; address <= 127; address++){
    // read a byte from the current address of the EEPROM
    value = EEPROM.read(address);
 
    // add space between two sets of 8 bytes
    if(address % 8 == 0)
      Serial.print(F("  "));
 
    // newline and address for every 16 bytes
    if(address % 16 == 0){
      //print the buffer
      if(address > 0 && address % 16 == 0)
        printASCII(buffer);
 
      sprintf(buffer, "\n 0x%05X: ", address);
      Serial.print(buffer);
 
      //clear the buffer for the next data block
      memset (buffer, 32, 16);
    }
 
    // save the value in temporary storage
    buffer[address%16] = value;
 
    // print the formatted value
    sprintf(valuePrint, " %02X", value);
    Serial.print(valuePrint);
  }
 
  if(address % 16 > 0){
    if(address % 16 < 9)
      trailingSpace += 2;
 
    trailingSpace += (16 - address % 16) * 3;
  }
 
  for(int i = trailingSpace; i > 0; i--)
    Serial.print(F(" "));
 
  //last line of data and a new line
  printASCII(buffer);
  Serial.println();
}
 
void printASCII(char * buffer) {
  for(int i = 0; i < 16; i++){
    if(i == 8)
      Serial.print(" ");
 
    if(buffer[i] > 31 and buffer[i] < 127){
      Serial.print(buffer[i]);
    }else{
      Serial.print(F("."));
    }
  }
}

void shutdownSelf() {
  digitalWrite(POWER_OFF, LOW); 
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvR10_tr);
    u8g2.setCursor(25,35);
    u8g2.print("Good Bye.");
  } while ( u8g2.nextPage() );
  do {} while(true);
}

void showError(char * errorMsg1, char * errorMsg2) {
  u8g2.setFont(u8g2_font_helvR08_tr);
  u8g2.setFontRefHeightAll();    /* this will add some extra space for the text inside the buttons */
  byte choice;
  choice = u8g2.userInterfaceMessage("ERROR", errorMsg1, errorMsg2, " Okay ");
  waitForBttnRelease();
  
  if (choice == 2) {
  }
  u8g2.setFont(u8g2_font_helvR10_tr);
  u8g2.setFontRefHeightText();    
}

ISR(TIMER1_COMPA_vect) {                  // This gets called once every second
  if (myEnc.read() != lastEncRead) {  // See if there was any user activity
    lastActivityMillies = millis();
    lastEncRead = myEnc.read();
  }
  if (myState != SYNC_PLAY) {
    if (myState != TRACK_LOADED) {
      if (millis() - lastActivityMillies > 300000) {
        shutdownSelf();
      }
    }
  }
}

