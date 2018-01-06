/* TODOs
 *  
 *  [ ] Find out why Schmitt Trigger sometimes oscillates
 *  [ ] Find out why sometimes just 77000 is sreceived
 *  [x] Find out why Playback sometimes occasionally stops / crashes (IRQ?)
 *  [ ] Detect or circumvent garbage received (not plausible)
 *  [ ] Allow watching two serials at once (Terminal?)
 *  
 *  
 *  
 *  
 *  This is the versionmanaged version!
 * 
 * √ Auto Start und Stop
 * √ Überläufe merken und abbauen
 * √ Watermarking LED
 * √ Display unterstützen
 * 
 * Vorwärts/Rückwärtskorrektur (0.1 Sek) 
 * I-Komponente bauen
 * Mute outrun on pause
 * ppm constrains dynamisch machen
 * Track Selection umziehen zum Bedienteil
 * wait for confirmation from audio module
 * pullups forlonger i12 cable runs!
 * 
 * Tastverhältnis messen
 * Bessren OpAmp einbauen
 * Lichtleiter
 *  
 *  
 */

#include <FreqMeasure.h>
#include <Wire.h>
#include <WireData.h>

// define OVERFLOWLED  

byte sollfps = 18;
byte segments = 2;          // Wieviele Segmente hat die Umlaufblende?
byte flickrate = sollfps * segments;

unsigned int projectorStopTimeoutMs = 250;
unsigned int projectorRunoutMs = 1000;

long ppmLo = -100000;
long ppmHi = 77000;

double impSum=0;
int impCount=0;

float frequency = 0;
float ppm = 0;
long ppmConstrained = 0;
long ppmRounded = 0;
long carryOverCorrection = 0;
long ppmRoundedPlusCarryOver = 0;



byte myState;
byte prevState;

const byte ISRPIN = 2;
volatile boolean projectorRunning = false;
boolean freqMeasurementStarted = false;


unsigned long lastMeasurementMillis = 0;
unsigned long currentMillis = 0;
unsigned long lastPausedMillis = 0;


void setup() {
//  pinMode(RS232LED, OUTPUT);
//  pinMode(OVERFLOWLED, OUTPUT);
  Serial.begin(115200);
  FreqMeasure.begin();
  Wire.begin(); // join i2c bus (address optional for master)
  //delay(500);  // Virtual coffee break.

}

void loop() {

  calculateCorrPpm();
  
//  if (myState != prevState) {
//    Serial.print("    --- State: ");
//    Serial.println(myState);
//    prevState = myState;
//  }
  if (FreqMeasure.available()) {
  // average several reading together
    impSum = impSum + FreqMeasure.read();
    impCount = impCount + 1;
    if (impCount > 30) {
      long frequency = 1000000 * FreqMeasure.countToFrequency(impSum / impCount);
      
//      Wire.beginTransmission(8); // transmit to device #8
//      wireWriteData(frequency);  
//      Wire.endTransmission();    // stop transmitting

      Serial.println(frequency);
      impSum = 0;
      impCount = 0;
    }
  }
}



//void runDetected() {  // ISR
//  projectorRunning = true;
//}

void startSyncedPlayback() {

// if (currentMillis - lastMeasurementMillis >= projectorStopTimeoutMs) {
  
//  detachInterrupt(digitalPinToInterrupt (ISRPIN));
//  FreqMeasure.begin();
//  freqMeasurementStarted = true;
//  //sendMessage(RCPT_AUDIO, ADJUST_VOLUME, NULL, "15");
//  //sendMessage(RCPT_AUDIO, PAUSE_OFF, NULL, "");
//  myState = RUNNING;
}

void calculateCorrPpm() {
  currentMillis = millis();
  if (FreqMeasure.available()) {
    lastMeasurementMillis = currentMillis;
 
    // average several reading together
    impSum = impSum + FreqMeasure.read();
    impCount = impCount + 1;
    if (impCount > 30) {
      frequency = FreqMeasure.countToFrequency(impSum / impCount);
      ppm = (1 - frequency / flickrate) * -480000;
      ppmRounded = ppm >= 0 ? (long)(ppm+0.5) : (long)(ppm-0.5);
      ppmRoundedPlusCarryOver = ppmRounded + carryOverCorrection;
      ppmConstrained = constrain(ppmRoundedPlusCarryOver, ppmLo, ppmHi);
//      Serial.print(frequency,5);
//      Serial.print(" Hz -> CorrVal = ");
//      Serial.print(ppmConstrained);
      if (ppmRoundedPlusCarryOver >= ppmHi) {
//        digitalWrite(OVERFLOWLED, HIGH);
        carryOverCorrection -= ppmHi - ppmRounded;
      } else if (ppmRoundedPlusCarryOver <= ppmLo) {
//        digitalWrite(OVERFLOWLED, HIGH);
        carryOverCorrection -= ppmLo - ppmRounded;       
      } else {
//        digitalWrite(OVERFLOWLED, LOW);
        carryOverCorrection = 0;
      }
      Serial.print("Übertrag ");
      Serial.println(carryOverCorrection);

      Wire.beginTransmission(8); // transmit to device #8
      wireWriteData(ppmConstrained);  
      Wire.endTransmission();    // stop transmitting

      Serial.println(ppmConstrained);

      
      //sendMessage(RCPT_AUDIO, ADJUST_SPEED, NULL, "");
//      //sendMessage(RCPT_UI, "SHOW_CURRENT_FPS", NULL, String(frequency / segments,4));
      impSum = 0;
      impCount = 0;
    }
  } else {
    if (currentMillis - lastMeasurementMillis >= projectorStopTimeoutMs) {
//      myState = PAUSE;
    }
  }
}


