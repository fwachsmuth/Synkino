/*
 * 
 *  This is the frontend part of Synkino
 * 
 * [ ] Startup wählt 3
 * 
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Encoder.h>
#include <Wire.h>
#include <WireData.h>


// ---- Define the various Pins we use- --------------------------------------------
//
#define SPI_CS        10
#define SPI_DC        9
#define SPI_RESET     8
#define ENCODER_A     2
#define ENCODER_B     3
#define ENCODER_BTN   4
#define BUZZER        6

// ---- Define the various Menu Screens --------------------------------------------
//
#define MAIN_MENU             1
#define PROJECTOR_MENU        2
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
#define MENU_ITEM_SETTINGS        3
#define MENU_ITEM_SHUTTER_BLADES  1
#define MENU_ITEM_STARTMARK       2
#define MENU_ITEM_PID             3
#define MENU_ITEM_ONE             1
#define MENU_ITEM_TWO             2
#define MENU_ITEM_THREE           3
#define MENU_ITEM_FOUR            4
#define MENU_ITEM_FRAMES          1
#define MENU_ITEM_P               1  
#define MENU_ITEM_I               2
#define MENU_ITEM_D               3

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
#define CMD_CURRENT_FRAME       12  /* ---> (frameNo)         */
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

// ---- Define some graphics -------------------------------------------------------
//
#define busybee_xbm_width 15
#define busybee_xbm_height 15
static const unsigned char busybee_xbm_bits[] U8X8_PROGMEM = {
   0x10, 0x00, 0x10, 0x3C, 0x00, 0x46, 0x60, 0x43, 0x63, 0x21, 0x98, 0x51,
   0xD8, 0x2A, 0x60, 0x07, 0xB8, 0x1A, 0xCC, 0x3F, 0x86, 0x06, 0x42, 0x7B,
   0x22, 0x1B, 0x52, 0x6A, 0x2C, 0x28 };

#define play_xbm_width 9
#define play_xbm_height 9
static const unsigned char play_xbm_bits[] U8X8_PROGMEM = {
   0x03, 0x00, 0x0F, 0x00, 0x3F, 0x00, 0xFF, 0x00, 0xFF, 0x01, 0xFF, 0x00,
   0x3F, 0x00, 0x0F, 0x00, 0x03, 0x00 };

#define pause_xbm_width 8
#define pause_xbm_height 8
static const unsigned char pause_xbm_bits[] U8X8_PROGMEM = {
   0xE7, 0xE7, 0xE7, 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };

#define sync_xbm_width 20
#define sync_xbm_height 9
static const unsigned char sync_xbm_bits[] U8X8_PROGMEM = {
   0xFE, 0xFF, 0x07, 0xFF, 0xFF, 0x0F, 0xA7, 0xDA, 0x0C, 0xBB, 0x52, 0x0F,
   0x77, 0x4B, 0x0F, 0x6F, 0x5B, 0x0F, 0x73, 0xDB, 0x0C, 0xFF, 0xFF, 0x0F,
   0xFE, 0xFF, 0x07 };

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
  "Settings";

const char *settings_menu =
  "Shutter Blades\n"
  "Start Mark\n"
  "PID Tuning\n"
  "Version";

const char *projector_menu =
  "New\n"
  "Change\n"
  "Edit\n"
  "Delete";

uint8_t currentMenuSelection = 2;
uint8_t prevMenuSelection = 0;

volatile bool haveI2Cdata = false;
volatile uint8_t i2cCommand;
volatile long i2cParameter;

uint8_t myState = MAIN_MENU;

uint8_t fps = 0;
uint8_t fileType = 0;
uint8_t trackLoaded = 0;
uint8_t startMarkHit = 0;

unsigned long totalSeconds = 0;
uint8_t hours   = 0;
uint8_t minutes = 0;
uint8_t seconds = 0;

// unsigned int oldPosition = 16000;   // some ugly hack to cope with 0->65535 when turning left

// ---- Setup ---------------------------------------------------------------------
//
void setup(void) {
  pinMode(SPI_CS, OUTPUT);
  pinMode(SPI_DC, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(ENCODER_BTN, INPUT);
  digitalWrite(SPI_CS, 0);
  digitalWrite(SPI_DC, 0);		
  digitalWrite(ENCODER_BTN, HIGH);

  myEnc.write(16000);
  // oldPosition = myEnc.read() >> 1;

  Serial.begin(115200);

  Wire.begin(myAddress);
  Wire.onReceive(i2cReceive);
  Wire.onRequest(i2cRequest);
 
  u8g2.begin();  
  //u8g2.begin(/*Select=*/ 7, /*Right/Next=*/ A1, /*Left/Prev=*/ A2, /*Up=*/ A0, /*Down=*/ A3, /*Home/Cancel=*/ 5);
  u8g2.setFont(u8g2_font_helvR14_tr);

  myState = MAIN_MENU;
}


// ---- Main Loop ------------------------------------------------------------------
//
void loop(void) {
  if (haveI2Cdata) {
    switch (i2cCommand) {   // Debug output
      case 10: Serial.println(F("CMD_FOUND_FMT"));
               Serial.println(i2cParameter);
               break;
      case 11: Serial.print(F("CMD_FOUND_FPS: "));
               Serial.println(i2cParameter);
               break;
      case 12: Serial.print(F("CMD_CURRENT_FRAME: "));
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
  
    switch (i2cCommand) {
      case CMD_FOUND_FMT:
      break; 
      case CMD_FOUND_FPS:
//        Serial.print(F("FPS sind "));
//        Serial.println(i2cParameter);
        fps = i2cParameter;
      break; 
      case CMD_CURRENT_FRAME:
        totalSeconds = i2cParameter / fps / 2;
        hours   = numberOfHours(totalSeconds);
        minutes = numberOfMinutes(totalSeconds);
        seconds = numberOfSeconds(totalSeconds);
      break; 
      case CMD_PROJ_PAUSE:
      break; 
      case CMD_PROJ_PLAY:
      break; 
      case CMD_FOUND_TRACKLENGTH:
      break; 
      case CMD_OOSYNC:
      break; 
      case CMD_SHOW_ERROR:
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

  // State Machine ----------------------------------------------------------------
  //
  switch (myState) {
    case MAIN_MENU:
      prevMenuSelection = currentMenuSelection;       // store previous menu selection
      currentMenuSelection = u8g2.userInterfaceSelectionList(NULL, currentMenuSelection, main_menu);
      while (digitalRead(ENCODER_BTN) == 0) {};       // wait for button release 
      switch (currentMenuSelection) {
        case MENU_ITEM_PROJECTOR:
          // go to Projector
          break;
        case MENU_ITEM_SELECT_TRACK:
          myState = SELECT_TRACK;
          break;
        case MENU_ITEM_SETTINGS:
          // go to Settings
          break;
        default:
          break;
      }
     break;
    case SELECT_TRACK:
      static int trackChosen;
      trackChosen = selectTrackScreen();
      tellAudioPlayer(CMD_LOAD_TRACK, trackChosen);
      myState = WAIT_FOR_LOADING;
      break;
    case WAIT_FOR_LOADING:
      drawBusyBee(90,10);
      if ((fps != 0) && (trackLoaded != 0)) {
        drawWaitForPlayingMenu(trackChosen, fps);
        myState = TRACK_LOADED;
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


void drawWaitForPlayingMenu(int trackNo, byte fps) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.setCursor(0,8);
    u8g2.print("Bauer t610");
    u8g2.setCursor(90,8);
    u8g2.print("Film ");
    if (trackNo < 10)  u8g2.print("0");
    if (trackNo < 100) u8g2.print("0");
    u8g2.print(trackNo);
    u8g2.setCursor(98,62);
    u8g2.print(fps);
    u8g2.print(" fps");
    u8g2.setFont(u8g2_font_helvR14_tr);
    u8g2.drawStr(17,28,"Waiting for");    
    u8g2.drawStr(0,46,"Projector Start");    
    u8g2.drawXBMP(60, 54, pause_xbm_width, pause_xbm_height, pause_xbm_bits);
    // u8g2.drawXBMP(2, 54, sync_xbm_width, sync_xbm_height, sync_xbm_bits);
  } while ( u8g2.nextPage() );
}

void drawPlayingMenu(int trackNo, byte fps) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_helvR08_tr);
    u8g2.setCursor(0,8);
    u8g2.print("Bauer t610");
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
    u8g2.drawXBMP(60, 54, play_xbm_width, play_xbm_height, play_xbm_bits);
    // u8g2.drawXBMP(60, 54, pause_xbm_width, pause_xbm_height, pause_xbm_bits);
    u8g2.drawXBMP(2, 54, sync_xbm_width, sync_xbm_height, sync_xbm_bits);
  } while ( u8g2.nextPage() );
}
void drawBusyBee(byte x, byte y) {
  u8g2.firstPage();
  do {
    u8g2.drawXBMP(x, y, busybee_xbm_width, busybee_xbm_height, busybee_xbm_bits);
    u8g2.setFont(u8g2_font_helvR14_tr);
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
  myEnc.write(16002);
  while (digitalRead(ENCODER_BTN) == 1) {     // adjust ### as long as button not pressed
    newEncPosition = myEnc.read();
    newEncPosition = (newEncPosition >> 1) % 1000;
    if (newEncPosition != oldPosition) {
      oldPosition = newEncPosition;
      
      u8g2.firstPage();
      if (newEncPosition == 0) {
        do {
          u8g2.setFont(u8g2_font_helvR14_tr);
          u8g2.drawStr(12,40,"< Main Menu");
        } while ( u8g2.nextPage() );
      } else {
        do {
          u8g2.setFont(u8g2_font_inb46_mn);
          u8g2.setCursor(8, 55);
          if (newEncPosition < 10)  u8g2.print("0");
          if (newEncPosition < 100) u8g2.print("0");
          u8g2.print(newEncPosition);
        } while ( u8g2.nextPage() );
      }  
    }
  }
  while (digitalRead(ENCODER_BTN) == 0) {};
  oldPosition = 0;
  myEnc.write(parentMenuEncPosition);
  u8g2.setFont(u8g2_font_helvR14_tr);   // Only until we go to the PLAYING_MENU here
  return newEncPosition;
}

// This overwrites the weak function in u8x8_debounce.c
uint8_t u8x8_GetMenuEvent(u8x8_t *u8x8) {
  int newEncPosition = myEnc.read() >> 1;
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

