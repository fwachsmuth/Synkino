/* TODOs
 * 
 * √ Auto Start und Stop
 * √ Überläufe merken und abbauen
 * √ Watermarking LED
 * √ Display unterstützen
 * √ Lowpass hinzufügen
 * 
 * State Machine umbaune und Reset erlauben
 * Vorwärts/Rückwärtskorrektur (0.1 Sek) 
 * Umstellen von RS232 auf RS485
 * I-Komponente bauen
 * Mute outrun on pause
 * Nicht pausieren bei dunklem Start (Bauer)
 * Startmarke
 * ppm constrains dynamisch machen
 * Track Selection umziehen zum Bedienteil
 * wait for confirmation from audio module
 * 
 * Tastverhältnis messen
 * Bessren OpAmp einbauen
 * Lichtleiter
 * Frequenzmessung mit drei Arduino machen
 * 
 *  
 *  
 */




/* FreqMeasure - Example with serial output
 * http://www.pjrc.com/teensy/td_libs_FreqMeasure.html
 *
 * This example code is in the public domain.
 */
#include "RS485_protocol.h"
#include <FreqMeasure.h>

void fWrite (const byte what) { Serial.write (what); }
int fAvailable () { return Serial.available (); }
int fRead () { return Serial.read (); } 



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

boolean newData = false;
const byte numChars = 64;
char receivedChars[numChars];   // an array to store the received data
char tempChars[numChars];       // temporary array for use when parsing

// variables to hold the parsed data from serial
//char receivedChar;
char commandFromRemote[numChars] = {0};
char parameterFromRemote[numChars] = {0};

byte myState;
byte prevState;

// States
#define IDLING            1
#define REQUEST_AUDIO     2
#define WAIT_FOR_RUN      3
#define RUNNING           4
#define PAUSE             5

#define RS232LED          12
#define OVERFLOWLED       11


const byte ISRPIN = 2;
volatile boolean projectorRunning = false;
boolean freqMeasurementStarted = false;

unsigned long lastMeasurementMillis = 0;
unsigned long currentMillis = 0;
unsigned long lastPausedMillis = 0;


void setup() {
  pinMode(RS232LED, OUTPUT);
  pinMode(OVERFLOWLED, OUTPUT);
  Serial1.begin(2400);

  digitalWrite( 1, LOW );
  pinMode( 1, INPUT ); // now we're tri-stated

  
  digitalWrite (ISRPIN, LOW);

  myState = IDLING;
  //Serial.begin(9600);
  //while (! Serial);
}

void loop() {
  if (myState != prevState) {
    Serial1.print("    --- State: ");
    Serial1.println(myState);
    prevState = myState;
  }

  
  switch(myState) {
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
}

void runDetected() {  // ISR
  projectorRunning = true;
}

void prepareSyncedPlayback() {
  sendWithStartEndMarkers("VOL", "15");
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
  sendWithStartEndMarkers("VOL", "15");
  sendWithStartEndMarkers("PAUSE", "OFF");
  myState = RUNNING;
}

void pauseSyncedPlayback() {
  sendWithStartEndMarkers("PAUSE", "ON");
  sendWithStartEndMarkers("VOL", "0");
  sendWithStartEndMarkers("D16", "Stop");
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

void run() {
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
      digitalWrite(RS232LED, HIGH);
      sendWithStartEndMarkers("ADJ", "");
      sendWithStartEndMarkers("D30", String(frequency / segments,4));
      digitalWrite(RS232LED, LOW);
      impSum = 0;
      impCount = 0;
    }
  } else {
    if (currentMillis - lastMeasurementMillis >= projectorStopTimeoutMs) {
      myState = PAUSE;
    }
  }
}

void sendWithStartEndMarkers(const String& command, const String& parameter) {
  Serial1.print("<");
  Serial1.print(command);
  Serial1.print("|");

  if (command == "ADJ") {
    Serial1.print(ppmConstrained);
  } else {
    Serial1.print(parameter);
  }

  Serial1.println(">");
  delay(1);
}

void checkDebugKey() {
  if (Serial.available()) {
    char c = Serial.read();
   
    if (c == 'w') {
      sendWithStartEndMarkers("PLAY", "");
    }
    if (c == '3') {
      sendWithStartEndMarkers("PREP", "004-18-Giorgio by Moroder.m4a");
      prepareSyncedPlayback();
    }
    if (c == '4') {
      sendWithStartEndMarkers("PREP", "009-18-Synkino Testfilm.m4a");
      prepareSyncedPlayback();
    }
    if (c == '5') {
      sendWithStartEndMarkers("PREP", "002-24-Mortel.m4a");
      prepareSyncedPlayback();
    }
    if (c == 'p') {
      sendWithStartEndMarkers("PAUSE", "ON");
    }
    if (c == 'P') {
      sendWithStartEndMarkers("PAUSE", "OFF");
    }
    if (c == 's') {
      sendWithStartEndMarkers("STOP", "");
    }
    if (c == 'n') {
      sendWithStartEndMarkers("NAME", "");
    }
    if (c == 'r') {
      sendWithStartEndMarkers("STOP", "");
      sendWithStartEndMarkers("VOL", "15");
      sendWithStartEndMarkers("PREP", "009-18-Synkino Testfilm.m4a");
    }
  }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

