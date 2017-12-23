/* TODOs
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

/* define recipient addresses, basically bitmasked  
#define RCPT_EYE        1
#define RCPT_UI         2
#define RCPT_EYE_UI     3
#define RCPT_AUDIO      4
#define RCPT_EYE_AUDIO  5
#define RCPT_UI_AUDIO   6
#define RCPT_ALL        7
*/
/* define commands to remote nodes

#define ADJUST_SPEED        10
#define PLAY_TRACK          11
#define PREP_TRACK          12
#define PAUSE_ON            13
#define PAUSE_OFF           14
#define STOP_TRACK          15
#define NAME_TRACK          16
#define ADJUST_VOLUME       17

#define SHOW_CURRENT_FPS    20
#define SHOW_TARGET_FPS     21
#define SHOW_NAME           22
#define SHOW_TOO_FAST       23
#define SHOW_TOO_SLOW       24
#define SHOW_IN_SYNC        25
#define CLS                 26
#define SHOW_TEXT           27
#define FOUND_NAME          28

#define CONTINOUS_MEASURE   30
#define LONGTERM_MEASURE    31
#define RESET_MEASURE       32
#define SET_TARGETFPS       33
#define SET_BLADE_SEGMENTS  34
*/

// States
#define IDLING            1
#define REQUEST_AUDIO     2
#define WAIT_FOR_RUN      3
#define RUNNING           4
#define PAUSE             5

#define RS232LED          12
#define OVERFLOWLED       11

byte sollfps = 18;
byte segments = 3;          // Wieviele Segmente hat die Umlaufblende?
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
  pinMode(RS232LED, OUTPUT);
  pinMode(OVERFLOWLED, OUTPUT);
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
//  if (FreqMeasure.available()) {
//  // average several reading together
//    impSum = impSum + FreqMeasure.read();
//    impCount = impCount + 1;
//    if (impCount > 10) {
//      long frequency = 1000000 * FreqMeasure.countToFrequency(impSum / impCount);
//      
//      Wire.beginTransmission(8); // transmit to device #8
//      wireWriteData(frequency);  
//      Wire.endTransmission();    // stop transmitting
//
//      Serial.println(frequency);
//      impSum = 0;
//      impCount = 0;
//    }
//  }
/*  switch(myState) {
    case IDLING:
      checkDebugKey();
    break;
    case REQUEST_AUDIO:
    break;
    case WAIT_FOR_RUN:
      if (projectorRunning && freqMeasurementStarted == false) {
        startSyncedPlayback();
      }
    break;
    case RUNNING:
      run();
    break;
    case PAUSE:
      pauseSyncedPlayback();
    break;
  }
  */
}



void runDetected() {  // ISR
  projectorRunning = true;
}

void prepareSyncedPlayback() {
  //sendMessage(RCPT_AUDIO, ADJUST_VOLUME, NULL, "15");
  attachInterrupt (digitalPinToInterrupt (ISRPIN), runDetected, CHANGE);  // attach interrupt handler
  // load selected track:
  //sendWithStartEndMarkers("PREP", "03 Giorgio by Moroder.m4a");
  // wait for confirmation from audio module
  myState = WAIT_FOR_RUN;
}

void startSyncedPlayback() {

// if (currentMillis - lastMeasurementMillis >= projectorStopTimeoutMs) {
  
  detachInterrupt(digitalPinToInterrupt (ISRPIN));
  FreqMeasure.begin();
  freqMeasurementStarted = true;
  //sendMessage(RCPT_AUDIO, ADJUST_VOLUME, NULL, "15");
  //sendMessage(RCPT_AUDIO, PAUSE_OFF, NULL, "");
  myState = RUNNING;
}

void pauseSyncedPlayback() {
  //sendMessage(RCPT_AUDIO, PAUSE_ON, NULL, "");
  //sendMessage(RCPT_AUDIO, ADJUST_VOLUME, NULL, "0");
  //sendMessage(RCPT_UI, SHOW_TEXT, NULL, "Stop");
  projectorRunning = false;

//  currentMillis = millis();  
//  lastPausedMillis = currentMillis();
  
  FreqMeasure.end();
  freqMeasurementStarted = false;
  attachInterrupt (digitalPinToInterrupt (ISRPIN), runDetected, CHANGE);  // attach interrupt handler
  impSum = 0;
  impCount = 0;
  myState = WAIT_FOR_RUN;
//  myState = IDLING;
}

//void 

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
        digitalWrite(OVERFLOWLED, HIGH);
        carryOverCorrection -= ppmHi - ppmRounded;
      } else if (ppmRoundedPlusCarryOver <= ppmLo) {
        digitalWrite(OVERFLOWLED, HIGH);
        carryOverCorrection -= ppmLo - ppmRounded;       
      } else {
        digitalWrite(OVERFLOWLED, LOW);
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
      myState = PAUSE;
    }
  }
}

void checkDebugKey() {
  if (Serial.available()) {
    char c = Serial.read();
   
    if (c == 'w') {
      //sendMessage(RCPT_AUDIO, PLAY_TRACK, NULL, "");
    }
    if (c == '3') {
      //sendMessage(RCPT_AUDIO, PREP_TRACK, NULL, "003");
      prepareSyncedPlayback();
    }
    if (c == '4') {
      //sendMessage(RCPT_AUDIO, PREP_TRACK, NULL, "004");
      prepareSyncedPlayback();
    }
    if (c == '9') {
      //sendMessage(RCPT_AUDIO, PREP_TRACK, NULL, "009");
      prepareSyncedPlayback();
    }
    if (c == 'p') {
      //sendMessage(RCPT_AUDIO, PAUSE_ON, NULL, "");
    }
    if (c == 'P') {
      //sendMessage(RCPT_AUDIO, PAUSE_OFF, NULL, "");
    }
    if (c == 's') {
      //sendMessage(RCPT_AUDIO, STOP_TRACK, NULL, "");
    }
    if (c == 'n') {
      //sendMessage(RCPT_AUDIO, NAME_TRACK, NULL, "");
    }
    if (c == 'r') {
      //sendMessage(RCPT_AUDIO, STOP_TRACK, NULL, "");
      //sendMessage(RCPT_AUDIO, ADJUST_VOLUME, 15, "");
      //sendMessage(RCPT_AUDIO, PREP_TRACK, NULL, "009-18-Synkino Testfilm.m4a");
    }
  }
}


