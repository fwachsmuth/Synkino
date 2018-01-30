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
#define CMD_SHOW_PAUSE          13  /* --->                   */
#define CMD_SHOW_PLAY           14  /* --->                   */
#define CMD_FOUND_TRACKLENGTH   15  /* ---> (TrackLength)     */
#define CMD_OOSYNC              16  /* ---> (frameCount)      */
#define CMD_SHOW_ERROR          17  /* ---> (ErrorCode)       */

// ---- Define the various States --------------------------------------------------
//
#define MAIN_MENU               1
#define SELECT_TRACK            2
#define TRACK_LOADED            3

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
      case 13: Serial.println(F("CMD_SHOW_PAUSE"));
               break;
      case 14: Serial.println(F("CMD_SHOW_PLAY"));
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
      default: Serial.println(i2cCommand);
               Serial.println(i2cParameter);
    }
  
    switch (i2cCommand) {
      case CMD_FOUND_FMT:
      break; 
      case CMD_FOUND_FPS:
        Serial.print(F("FPS sind "));
        Serial.println(i2cParameter);
      break; 
      case CMD_CURRENT_FRAME:
      break; 
      case CMD_SHOW_PAUSE:
      break; 
      case CMD_SHOW_PLAY:
      break; 
      case CMD_FOUND_TRACKLENGTH:
      break; 
      case CMD_OOSYNC:
      break; 
      case CMD_SHOW_ERROR:
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
//    Serial.print("Before: ");
//    Serial.println(currentMenuSelection);
      currentMenuSelection = u8g2.userInterfaceSelectionList(NULL, currentMenuSelection, main_menu);
      while (digitalRead(ENCODER_BTN) == 0) {};       // wait for button release 
//    Serial.print("After: ");
//    Serial.println(currentMenuSelection);
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
      int trackChosen;
      trackChosen = selectTrackScreen();
      tellAudioPlayer(CMD_LOAD_TRACK, trackChosen);
      myState = TRACK_LOADED;
      break;
    case TRACK_LOADED:
      break;
    default:
      break;
  }


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
          u8g2.setCursor(8, 64);
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

