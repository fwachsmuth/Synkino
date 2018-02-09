/* TODOs
 *  This is the versionmanaged version!
 *   
 *  [ ] Detect projector stops and stop the audio
 *  [ ] add out of sync LED
 *  [ ] SPI vs i2c?
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

double freqMeasureSum=0;
int freqMeasureCount=0;

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
  static int freqMeasureCountToStartMark = 0;
  static byte previousImpDetectorState = LOW;
  static byte impDetectorPinNow = 0;
  impDetectorPinNow = digitalRead(impDetectorPin);
  if (impDetectorPinNow != previousImpDetectorState) {
    freqMeasureCountToStartMark++;
    // Serial.println(freqMeasureCountToStartMark);
    previousImpDetectorState = impDetectorPinNow;
  }
  if (freqMeasureCountToStartMark >= segments * 2 * startMarkOffset) {
    byte cmd = CMD_PLAY;
    long param = 0;
    Wire.beginTransmission(8); // transmit to device #8
    wireWriteData(cmd);  
    wireWriteData(param);  
    Wire.endTransmission();    // stop transmitting

    totalImpCounter = 0;
   // attachInterrupt(digitalPinToInterrupt(impDetectorISRPIN), countISR, CHANGE);
    
    FreqMeasure.begin();
    freqMeasureCountToStartMark = 0;
    playHeadStartMillis = millis();
    myState = PLAYING;
  }
}

void countISR() {
  totalImpCounter++;
}

void considerResync() {
  static unsigned long lastSyncedImpCounter = 0;
  unsigned long totalImpCounterNow;
  totalImpCounterNow = totalImpCounter;

  if (lastSyncedImpCounter != totalImpCounterNow) {
    if (totalImpCounterNow % (sollfps * segments * 2) == 0) {    // every 2 seconds: 18 * 2 * 2
      byte cmd = CMD_SYNC_TO_FRAME;
      long param = totalImpCounterNow / segments / 2;
      Wire.beginTransmission(8); // transmit to device #8
      wireWriteData(cmd);  
      wireWriteData(param);  
      Wire.endTransmission();    // stop transmitting
      lastSyncedImpCounter = totalImpCounterNow;
      Serial.println("Resync should happen now.");
    }
  }
}


void calculateCorrPpm() {
  currentMillis = millis();
  if (FreqMeasure.available()) {
    lastMeasurementMillis = currentMillis;
    freqMeasureSum = freqMeasureSum + FreqMeasure.read();     // average several reading together
    freqMeasureCount++;
    if (freqMeasureCount > 30) {                              // correct about every second
      frequency = FreqMeasure.countToFrequency(freqMeasureSum / freqMeasureCount);
      ppm = (1 - frequency / flickrate) * -480000;            // 512,000 * 15/16
      ppmRounded = ppm >= 0 ? (long)(ppm+0.5) : (long)(ppm-0.5);
      ppmRoundedPlusCarryOver = ppmRounded + carryOverCorrection;
      ppmConstrained = constrain(ppmRoundedPlusCarryOver, ppmLo, ppmHi);
      if (ppmRoundedPlusCarryOver >= ppmHi) {
        Serial.print(F("Projektor zu schnell, "));
//        digitalWrite(OVERFLOWLED, HIGH);
        carryOverCorrection -= ppmHi - ppmRounded;
      } else if (ppmRoundedPlusCarryOver <= ppmLo) {
        Serial.print(F("Projektor zu langsam, "));
//        digitalWrite(OVERFLOWLED, HIGH);
        carryOverCorrection -= ppmLo - ppmRounded;       
      } else {
//        digitalWrite(OVERFLOWLED, LOW);
        carryOverCorrection = 0;               
      }
      /*
       * The problem here that leads to drift is the carry-over routine. This doesn't work this way. 
       * Spliting a 15% increase into a 10% increase and a 5% increase (on the result of the previous 
       * calculation!), we end up with too much.
       * Go, read about PID controllers, dude.
       * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
       * http://playground.arduino.cc/Code/PIDLibrary
       * 
       */
      Serial.print(F("Korrektrur-Übertrag: "));
      Serial.println(carryOverCorrection);
      
//      Wire.beginTransmission(8); // transmit to device #8
//      byte cmd = CMD_CORRECT_PPM;
//      wireWriteData(cmd);  
//      wireWriteData(ppmConstrained);  
//      Serial.println(ppmConstrained);
//      Wire.endTransmission();    // stop transmitting
      freqMeasureSum = 0;
      freqMeasureCount = 0;
    }
  } else {
    if (currentMillis - lastMeasurementMillis >= projectorStopTimeoutMs) {
//      myState = PAUSED;
    }
  }
}


