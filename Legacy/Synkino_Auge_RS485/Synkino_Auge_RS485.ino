/* TODOs
 * 
 * √ Auto Start und Stop
 * √ Überläufe merken und abbauen
 * √ Watermarking LED
 * √ Display unterstützen
 * √ Lowpass hinzufügen
 * √ Umstellen von RS232 auf RS485
 * 
 * State Machine umbaune und Reset erlauben
 * Vorwärts/Rückwärtskorrektur (0.1 Sek) 
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
#include <RS485_non_blocking.h>
#include <FreqMeasure.h>

// define recipient addresses, basically bitmasked  
#define RCPT_EYE        1
#define RCPT_UI         2
#define RCPT_EYE_UI     3
#define RCPT_AUDIO      4
#define RCPT_EYE_AUDIO  5
#define RCPT_UI_AUDIO   6
#define RCPT_ALL        7

// definde commands to remote nodes

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




// States
#define IDLING            1
#define REQUEST_AUDIO     2
#define WAIT_FOR_RUN      3
#define RUNNING           4
#define PAUSE             5

#define RS232LED          12
#define OVERFLOWLED       11
#define RS485_ENABLE_PIN  6

// the data we broadcast to other devices
const byte numChars = 64; // Achtung: RS485 Objekt ggf anpassen!
struct {
  byte address;
  byte command;
  long parameter;
  char asciis[numChars];
} message;

int fAvailable () { return Serial1.available (); }
int fRead () { return Serial1.read (); }
size_t fWrite (const byte what) { Serial1.write (what); }
RS485 myChannel (fRead, fAvailable, fWrite, 70);


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


// variables to hold the parsed data from serial
//char receivedChar;
char commandFromRemote[numChars] = {0};
char parameterFromRemote[numChars] = {0};

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
  pinMode(RS485_ENABLE_PIN, OUTPUT);  
  
  Serial1.begin(9600);
  //while(!Serial1);
  myChannel.begin();
 
  digitalWrite (ISRPIN, LOW);

  myState = IDLING;
  //Serial.begin(9600);
  //while (! Serial);
}


void looptest()
{

  sendMessage (RCPT_EYE, 1, 31415927, "a");
  sendMessage (RCPT_UI, 2, 31415927, "b");
  sendMessage (RCPT_EYE_UI, 3, 31415927, "c");
  sendMessage (RCPT_AUDIO, 4, 31415927, "d");
  sendMessage (RCPT_EYE_AUDIO, 5, 31415927, "e");
  sendMessage (RCPT_UI_AUDIO, 6, 31415927, "f");
  sendMessage (RCPT_ALL, 7, 31415927, "abcdefghijklmnopqrstuvwxyz");

}

void loop() {
  listenOnBus();

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

void listenOnBus() {
  if (myChannel.update ()) {
    memset (&message, 0, sizeof message);
    int len = myChannel.getLength ();
    if (len > sizeof message)
      len = sizeof message;
    memcpy (&message, myChannel.getData (), len);
    processMessage ();
  }
}  

void processMessage () {
  if (message.address == RCPT_EYE 
   || message.address == RCPT_EYE_UI 
   || message.address == RCPT_EYE_AUDIO
   || message.address == RCPT_ALL ) {   // could just check for bit 0 here, too
    Serial.print("Adr: ");
    Serial.print(message.address);
    Serial.print(", Cmd: ");
    Serial.print(message.command);
    Serial.print(", Param: ");
    Serial.print(message.parameter);
    Serial.print(", Str: ");
    Serial.println(message.asciis);

    switch(message.command) {
      case ADJUST_SPEED:
        break;
      case ADJUST_VOLUME:
        break;
      default:
        Serial.print("ERROR Unknown command: ");
        Serial.println(message.command);
    }
  }
}

void sendMessage (byte rcpnt, byte cmnd, long prmtr, char prmtrstr[]) {
  memset (&message, 0, sizeof message);
  message.address = rcpnt;
  message.command = cmnd;
  message.parameter = prmtr;
  strcpy(message.asciis, prmtrstr);

  // now send it
  digitalWrite (RS485_ENABLE_PIN, HIGH);  // enable sending
  myChannel.sendMsg ((byte *) &message, sizeof message);
  while (!(UCSR1A & (1 << UDRE1)))  // Wait for empty transmit buffer
  UCSR1A |= 1 << TXC1;              // mark transmission not complete
  while (!(UCSR1A & (1 << TXC1)));  // Wait for the transmission to complete
  digitalWrite (RS485_ENABLE_PIN, LOW);  // disable sending
}

void runDetected() {  // ISR
  projectorRunning = true;
}

void prepareSyncedPlayback() {
  sendMessage(RCPT_AUDIO, ADJUST_VOLUME, NULL, "15");
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
  sendMessage(RCPT_AUDIO, ADJUST_VOLUME, NULL, "15");
  sendMessage(RCPT_AUDIO, PAUSE_OFF, NULL, "");
  myState = RUNNING;
}

void pauseSyncedPlayback() {
  sendMessage(RCPT_AUDIO, PAUSE_ON, NULL, "");
  sendMessage(RCPT_AUDIO, ADJUST_VOLUME, NULL, "0");
  sendMessage(RCPT_UI, SHOW_TEXT, NULL, "Stop");
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
      sendMessage(RCPT_AUDIO, ADJUST_SPEED, NULL, "");
//      sendMessage(RCPT_UI, "SHOW_CURRENT_FPS", NULL, String(frequency / segments,4));
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

void checkDebugKey() {
  if (Serial.available()) {
    char c = Serial.read();
   
    if (c == 'w') {
      sendMessage(RCPT_AUDIO, PLAY_TRACK, NULL, "");
    }
    if (c == '3') {
      sendMessage(RCPT_AUDIO, PREP_TRACK, NULL, "003");
      prepareSyncedPlayback();
    }
    if (c == '4') {
      sendMessage(RCPT_AUDIO, PREP_TRACK, NULL, "004");
      prepareSyncedPlayback();
    }
    if (c == '9') {
      sendMessage(RCPT_AUDIO, PREP_TRACK, NULL, "009");
      prepareSyncedPlayback();
    }
    if (c == 'p') {
      sendMessage(RCPT_AUDIO, PAUSE_ON, NULL, "");
    }
    if (c == 'P') {
      sendMessage(RCPT_AUDIO, PAUSE_OFF, NULL, "");
    }
    if (c == 's') {
      sendMessage(RCPT_AUDIO, STOP_TRACK, NULL, "");
    }
    if (c == 'n') {
      sendMessage(RCPT_AUDIO, NAME_TRACK, NULL, "");
    }
    if (c == 'r') {
      sendMessage(RCPT_AUDIO, STOP_TRACK, NULL, "");
      sendMessage(RCPT_AUDIO, ADJUST_VOLUME, 15, "");
      sendMessage(RCPT_AUDIO, PREP_TRACK, NULL, "009-18-Synkino Testfilm.m4a");
    }
  }
}


