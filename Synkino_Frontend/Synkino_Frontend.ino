/*
 * 
 *  This is the frontend part of Synkino
 * 
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Encoder.h>

U8G2_SH1106_128X64_NONAME_1_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 10, /* dc=*/ 9, /* reset=*/ 8);
Encoder myEnc(2,3);

unsigned int oldPosition = 16000;

void setup(void) {
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(10, 0);
  digitalWrite(9, 0);		

  myEnc.write(16000);

  Serial.begin(115200);

  u8g2.begin();  
}

uint8_t m = 24;

void loop(void) {

  int newPosition = myEnc.read();
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

