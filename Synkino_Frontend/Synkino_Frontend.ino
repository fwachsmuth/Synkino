/*
 * 
 *  This is the frontend part of Synkino
 * 
 * [ ] Exit von 2 ist hässlich
 * [ ] Startup wählt 3
 * 
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Encoder.h>

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

// ---- Initialize Objects ---------------------------------------------------------
//
U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, SPI_CS, SPI_DC, SPI_RESET);
Encoder myEnc(ENCODER_A, ENCODER_B);

// ---- Initialize Consts and Vars -------------------------------------------------
//
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

unsigned int oldPosition = 16000;   // some ugly hack to cope with 0->65535 when turning left

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
  oldPosition = myEnc.read() >> 1;

  Serial.begin(115200);

  u8g2.begin();  
  //u8g2.begin(/*Select=*/ 7, /*Right/Next=*/ A1, /*Left/Prev=*/ A2, /*Up=*/ A0, /*Down=*/ A3, /*Home/Cancel=*/ 5);
  u8g2.setFont(u8g2_font_helvR14_tr);

}


// ---- Main Loop ------------------------------------------------------------------
//
void loop(void) {
  
  prevMenuSelection = currentMenuSelection;       // store previous menu selection
                                                  // Now (blockingly) wait for anew selection
  Serial.print("Before: ");
  Serial.println(currentMenuSelection);
  currentMenuSelection = u8g2.userInterfaceSelectionList(
    NULL, /* Header would go here */
    currentMenuSelection, 
    main_menu);

  while (digitalRead(ENCODER_BTN) == 0) {};       // wait for button release 

  Serial.print("After: ");
  Serial.println(currentMenuSelection);

  switch (currentMenuSelection) {
    case 1:
      // go to Projector
    break;
    case 2:
      int trackChosen;
      trackChosen = selectTrackScreen();
    break;
    case 3:
      // go to Settings
    break;
    default:
    break;
  }
}

uint16_t selectTrackScreen() {
  int prevPosition;
  prevPosition = myEnc.read();
  int newPosition;
  myEnc.write(16002);
  while (digitalRead(ENCODER_BTN) == 1) {     // adjust ### as long as button not pressed
    newPosition = myEnc.read();
    newPosition = (newPosition >> 1) % 1000;
    if (newPosition != oldPosition) {
      oldPosition = newPosition;
      
      u8g2.firstPage();
      if (newPosition == 0) {
        do {
          u8g2.setFont(u8g2_font_helvR14_tr);
          u8g2.drawStr(12,40,"Select Track");
        } while ( u8g2.nextPage() );
      } else {
        do {
          u8g2.setFont(u8g2_font_inb46_mn);
          u8g2.setCursor(8, 64);
          if (newPosition < 10)  u8g2.print("0");
          if (newPosition < 100) u8g2.print("0");
          u8g2.print(newPosition);
        } while ( u8g2.nextPage() );
      }  
    }
  }
  myEnc.write(prevPosition);
  return newPosition;
}

// This overwrites the weak function in u8x8_debounce.c
uint8_t u8x8_GetMenuEvent(u8x8_t *u8x8) {
  int newEncPosition = myEnc.read() >> 1;
  static int oldEncPosition = 0;
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


