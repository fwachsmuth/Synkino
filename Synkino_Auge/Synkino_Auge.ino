/* TODOs
 *  
 *  [ ] Detect projector stops and stop the audio
 *  [x] Introduce Framecounter
 *  [ ] Don't send resyncs twice in a row but avoid delay too
 *  [ ] Schmitt Trigger needs a higher threshold since all dat noise
 *  [ ] add out of sync LED
 *  [ ] SPI vs i2c?
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

#define CMD_LOAD_TRACK    1
#define CMD_CORRECT_PPM   2
#define CMD_PLAY          3
#define CMD_PAUSE         4
#define CMD_STOP          5
#define CMD_SYNC_TO_FRAME 6

#define IDLING            1
#define LOAD_TRACK        2
#define TRACK_LOADED      3
#define RESET             4
#define PLAYING           5
#define PAUSED            6
#define SETTINGS_MENU     7

// define OVERFLOWLED  

byte sollfps = 18;
byte segments = 2;          // Wieviele Segmente hat die Umlaufblende?
byte startMarkOffset = 15;

byte flickrate = sollfps * segments;

const byte startMarkDetectorPin = 9;
const byte impDetectorPin = 8;
const byte impDetectorISRPIN = 2;

unsigned int projectorStopTimeoutMs = 250;
unsigned int projectorRunoutMs = 1000;      // ???

long ppmLo = -100000;
long ppmHi = 77000;

double impSum=0;
int impCount=0;

float frequency = 0;
float ppm = 0;
long ppmConstrained = 0;            // these should be local
long ppmRounded = 0;
long carryOverCorrection = 0;
long ppmRoundedPlusCarryOver = 0;

byte myState = 3;     // For now, let's assume (and make sure) the track is loaded 
byte prevState;

const byte ISRPIN = 2;
volatile boolean projectorRunning = false;
boolean freqMeasurementStarted = false;


unsigned long lastMeasurementMillis = 0;
unsigned long currentMillis = 0;
unsigned long lastPausedMillis = 0;
unsigned long playHeadStartMillis = 0;

volatile unsigned long totalImpCounter = 0;

void setup() {
//  pinMode(RS232LED, OUTPUT);
//  pinMode(OVERFLOWLED, OUTPUT);
  pinMode(impDetectorISRPIN, INPUT);
  Serial.begin(115200);
  Wire.begin(); // join i2c bus (address optional for master)
  //delay(500);  // Virtual coffee break.
  
}

void loop() {

  switch (myState) {
    case IDLING:
    break;
    case LOAD_TRACK:
    break;
    case TRACK_LOADED:
      waitForStartMark();
    break;
    case RESET:
    break;
    case PLAYING:
      calculateCorrPpm();
      considerResync();
    break;
    case PAUSED:
    break;
    case SETTINGS_MENU:
    break;
    default:
    break;
  }

  if (myState != prevState) {
    Serial.print("    --- State: ");
    Serial.println(myState);
    prevState = myState;
  }
  
}

void waitForStartMark() {
  if (digitalRead(startMarkDetectorPin) == HIGH) return;  // There is still leader
  static int impCountToStartMark = 0;
  static byte previousImpDetectorState = LOW;
  static byte impDetectorPinNow = 0;
  impDetectorPinNow = digitalRead(impDetectorPin);
  if (impDetectorPinNow != previousImpDetectorState) {
    impCountToStartMark++;
    // Serial.println(impCountToStartMark);
    previousImpDetectorState = impDetectorPinNow;
  }
  if (impCountToStartMark >= segments * 2 * startMarkOffset) {
    byte cmd = CMD_PLAY;
    long param = 0;
    Wire.beginTransmission(8); // transmit to device #8
    wireWriteData(cmd);  
    wireWriteData(param);  
    Wire.endTransmission();    // stop transmitting

    totalImpCounter = 0;
    attachInterrupt(digitalPinToInterrupt(impDetectorISRPIN), countISR, CHANGE);
    
    FreqMeasure.begin();
    impCountToStartMark = 0;
    playHeadStartMillis = millis();
    myState = PLAYING;
  }
}

void countISR() {
  totalImpCounter++;
}

void considerResync() {
  if (totalImpCounter % (sollfps * segments * 10) == 0) {    // every 5 seconds: 18 * 2 * 2 * 5
//if (((millis() - playHeadStartMillis) % 5000) == 0) {
    byte cmd = CMD_SYNC_TO_FRAME;
    long param = totalImpCounter / segments / 2;
    Wire.beginTransmission(8); // transmit to device #8
    wireWriteData(cmd);  
    wireWriteData(param);  
    Wire.endTransmission();    // stop transmitting

    Serial.println("Resync should happen now.");
  }
  // Every 10 seconds
  // transmit the total framecounter to audio-bob
  // soll-frame von bob: (0x1800) / 44100 * 16/15 * fps
 
  
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
    impSum = impSum + FreqMeasure.read();     // average several reading together
    impCount++;
    if (impCount > 30) {
      frequency = FreqMeasure.countToFrequency(impSum / impCount);
      ppm = (1 - frequency / flickrate) * -480000;
      ppmRounded = ppm >= 0 ? (long)(ppm+0.5) : (long)(ppm-0.5);
      ppmRoundedPlusCarryOver = ppmRounded + carryOverCorrection;
      ppmConstrained = constrain(ppmRoundedPlusCarryOver, ppmLo, ppmHi);
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
//      Serial.print("Übertrag ");
//      Serial.println(carryOverCorrection);
      Wire.beginTransmission(8); // transmit to device #8
      byte cmd = CMD_CORRECT_PPM;
      wireWriteData(cmd);  
      wireWriteData(ppmConstrained);  
      Serial.println(ppmConstrained);
      Wire.endTransmission();    // stop transmitting
      impSum = 0;
      impCount = 0;
    }
  } else {
    if (currentMillis - lastMeasurementMillis >= projectorStopTimeoutMs) {
//      myState = PAUSED;
    }
  }
}


