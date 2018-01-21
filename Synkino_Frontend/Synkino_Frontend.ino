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
    do {
      u8g2.setFont(u8g2_font_inb46_mn);
      u8g2.setCursor(8, 64);
      u8g2.print(newPosition);
    } while ( u8g2.nextPage() );
  
    Serial.println(newPosition);
  }

  
//  char m_str[3];
//  strcpy(m_str, u8x8_u8toa(m, 2));		/* convert m to a string with two digits */
//  u8g2.firstPage();
//  do {
//    u8g2.setFont(u8g2_font_logisoso62_tn);
//    u8g2.drawStr(0,63,"9");
//    u8g2.drawStr(33,63,":");
//    u8g2.drawStr(50,63,m_str);
//  } while ( u8g2.nextPage() );
//  delay(1000);
//  m++;
//  if ( m == 60 )
//    m = 0;
}

