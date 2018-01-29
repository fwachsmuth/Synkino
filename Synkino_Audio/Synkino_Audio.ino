/**
 * 
 * To Do:
 *  [ ] Aufschaukeln bei mp3 fixen
 *  
 *  [ ] 100n an den Encoderoutputs probieren (Prellschutz)
 * 
 *  [ ] Serial.print Fehler loswerden (Timer statt ISR?) USE_MP3_Polled 
 *  https://github.com/madsci1016/Sparkfun-MP3-Player-Shield-Arduino-Library/blob/master/SFEMP3Shield/SFEMP3ShieldConfig.h*  
 *  [ ] Corr Werte sind immer zwei mal gleich?
 *  [ ] Seltener PID Samplen?  
 *  [ ] Find & Load other files than m4a
 *  [ ] Anzeigen, wie lang das Delta in ms ist
 *  
 *  [ ] KiCad all this. Soon.
 *  [ ] load patch from EEPROM
 *  [ ] Document diffs to vs1053_SdFat.h
 *  [ ] decode delta-sigma line out
 *  [ ] Try/Switch to 15 MHz xtal
 *  [ ] Dynamic Sample Rate support?
 *  [ ] Forwards/Backwards Correction (Frame-wise)
 *  [ ] complete state machine here
 *  [ ] Projektor-Frequenzanzeige
 *  [ ] remove unused variables
 *  
 * 
 */
#include <PID_v1.h>

#include <SPI.h>
#include <FreeStack.h>
#include <vs1053_SdFat.h>
#include <Arduino.h>
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

//#define USE_MP3_REFILL_MEANS  USE_MP3_Timer1

const int myAddress = 0x08;   // Listen on the I2C Bus

uint8_t sollfps = 18;        
uint8_t segments = 2;              // Wieviele Segmente hat die Umlaufblende?
//uint8_t startMarkOffset = 15;      // Noris
uint8_t startMarkOffset = 53;      // Bauer t610
  
const float physicalSamplingrate = 41344;   // 44100 * 15/16 – to compensate the 15/16 Bit Resampler

#define impDetectorISRPIN  3
#define impDetectorPin 3
#define startMarkDetectorPin 5
int  pauseDetectedPeriod = (1000 / sollfps * 3);   // Duration of 3 single frames
unsigned int impToSamplerateFactor = physicalSamplingrate / sollfps / segments / 2;
int deltaToFramesDivider = physicalSamplingrate / sollfps;

#define numReadings   16
long readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
long total = 0;                 // the running total
long average = 0;                // the average


uint8_t myState = IDLING;     
uint8_t prevState;

volatile long ppmCorrection;
long lastPpmCorrection = 0;

volatile bool haveI2Cdata = false;
volatile uint8_t i2cCommand;
volatile long i2cParameter;

volatile unsigned long lastISRTime;

double Setpoint, Input, Output;

double Kp=8, Ki=3, Kd=1;  // PonM WINNER für 16 Readings, but with fixed int overflow

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// Below is not needed if interrupt driven. Safe to remove if not using.
#if defined(USE_MP3_REFILL_MEANS) && USE_MP3_REFILL_MEANS == USE_MP3_Timer1
  #include <TimerOne.h>
#elif defined(USE_MP3_REFILL_MEANS) && USE_MP3_REFILL_MEANS == USE_MP3_SimpleTimer
  #include <SimpleTimer.h>
#endif

// Instantiate SD and Player objects
SdFat sd;
vs1053 musicPlayer;

uint32_t  millis_prv;
long pitchrate = 0;
long frameToSync = 0;
unsigned long lastImpCounterHaltPos = 0;
unsigned long lastSampleCounterHaltPos = 0;

volatile unsigned long totalImpCounter = 0;



//------------------------------------------------------------------------------
void setup() {

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  
  pinMode(impDetectorISRPIN, INPUT);
  pinMode(startMarkDetectorPin, INPUT);

  Setpoint = 0;
  
  uint8_t result; //result code for initialization function

  Serial.begin(115200);

  Serial.print(F("F_CPU = "));
  Serial.println(F_CPU);
  Serial.print(F("Free RAM = ")); 
  Serial.println(FreeStack(), DEC);  // FreeStack() is provided by SdFat
  
  //Initialize the SdCard
  if(!sd.begin(SD_SEL, SPI_FULL_SPEED)) sd.initErrorHalt();
  // depending upon your SdCard environment, SPI_HAVE_SPEED may work better.
  if(!sd.chdir("/")) sd.errorHalt("sd.chdir");

  //Initialize the MP3 Player Shield

  result = musicPlayer.begin();
  //check result, see readme for error codes.
  if(result != 0) { 
    Serial.print(F("Error code: "));
    Serial.print(result);
    Serial.println(F(" when trying to start MP3 player"));
    if( result == 6 ) {
      Serial.println(F("Warning: patch file not found, skipping.")); // can be removed for space, if needed.
    }
  }

  Wire.begin(myAddress);
  Wire.onReceive(i2cReceive);
  Wire.onRequest(i2cRequest);

}

//------------------------------------------------------------------------------

void loop() {

  
  if (haveI2Cdata) {
    switch (i2cCommand) {   // Debug output
      case 1: Serial.print(F("CMD: Load Track: "));
              Serial.println(i2cParameter);
      break;
      case 2: Serial.print(F("CMD: Correct PPM: "));
              Serial.println(i2cParameter);
      break;
      case 3: Serial.println(F("CMD: Play"));
      break;
      case 4: Serial.println(F("CMD: Pause"));
      break;
      case 5: Serial.println(F("CMD: Stop"));
      break;
      case 6: Serial.print(F("CMD: Sync to Frame: "));
              Serial.println(i2cParameter);
      break;
      default:Serial.print(i2cCommand);
              Serial.println(i2cParameter);
    }

    switch (i2cCommand) {
      case CMD_PLAY: 
//        totalImpCounter = 0;
//        myPID.SetMode(AUTOMATIC);
//        myPID.SetOutputLimits(-77000, 77000);
//
//        attachInterrupt(digitalPinToInterrupt(impDetectorISRPIN), countISR, CHANGE);
//        
//        musicPlayer.resumeMusic();
//        clearSampleCounter();
//        Serial.println(F("Los geht's!"));
      break;
      case CMD_CORRECT_PPM:
//        if (i2cParameter != lastPpmCorrection) {
//          lastPpmCorrection = i2cParameter;
//          adjustSamplerate(i2cParameter);
//        }
     break;
    case CMD_LOAD_TRACK:
      Serial.println(loadTrackByNo(i2cParameter));
//      char paddedTrackNo[4];
//      sprintf(paddedTrackNo, "%03d", i2cParameter);
//      Serial.println(paddedTrackNo);
    break;
    case CMD_PAUSE:
    break;
    case CMD_STOP:
    break;
    case CMD_SYNC_TO_FRAME:
//    resyncPlayhead(i2cParameter);
    break;
    default:
    break;
    }
    
    haveI2Cdata = false;  
  }  // end if haveData

// Below is only needed if not interrupt driven. Safe to remove if not using.
#if defined(USE_MP3_REFILL_MEANS) \
    && ( (USE_MP3_REFILL_MEANS == USE_MP3_SimpleTimer) \
    ||   (USE_MP3_REFILL_MEANS == USE_MP3_Polled)      )

  musicPlayer.available();
#endif

  if(Serial.available()) {
    parse_menu(Serial.read()); // get command from serial input
  }


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
//      calculateCorrPpm();
//      considerResync();
      speedControlPID();
      checkIfStillRunning();
    break;
    case PAUSED:
      waitForResumeToPlay(lastImpCounterHaltPos);
    break;
    case SETTINGS_MENU:
    break;
    default:
    break;
  }

  if (myState != prevState) {
    switch (myState) {   // Debug output
      case 1: Serial.println(F("--- IDLING."));
      break;
      case 2: Serial.println(F("--- LOAD_TRACK"));
      break;
      case 3: Serial.println(F("--- TRACK_LOADED."));
      break;
      case 4: Serial.println(F("--- RESET"));
      break;
      case 5: Serial.println(F("--- PLAYING"));
      break;
      case 6: Serial.println(F("--- PAUSED"));
      break;
      case 7: Serial.println(F("--- SETTINGS_MENU"));
      break;
     }
    prevState = myState;
  }
    
}

uint8_t loadTrackByNo(int trackNo) {
  char trackName[11];
  char trackNameFound[11];
  for (uint8_t fpsGuess = 12; fpsGuess <= 25; fpsGuess++) {
    sprintf(trackName, "%03d-%d.m4a", trackNo, fpsGuess);  
    if (sd.exists(trackName)) {
      updateFpsDependencies(fpsGuess);
      strcpy(trackNameFound, trackName);
//      Serial.print(F("File exists and has ")); 
//      Serial.print(fpsGuess);
//      Serial.print(F(" fps:"));
//      Serial.println(trackName);
    }
  }
  uint8_t result;
  result = musicPlayer.playMP3(trackNameFound);
  if (result != 0) {
    Serial.print(F("Error code: "));
    Serial.print(result);
    Serial.println(F(" when trying to play track"));
  } else {
    Serial.println(F("Waiting for start mark..."));
  }
  musicPlayer.setVolume(3,3);
  musicPlayer.pauseMusic();
  enableResampler();
  while (musicPlayer.getState() != paused_playback) {}
  clearSampleCounter();
  myState = TRACK_LOADED;
  return result;
}

void updateFpsDependencies(uint8_t fps) {
  sollfps = fps;                                // redundant?
  pauseDetectedPeriod = (1000 / fps * 3);
  impToSamplerateFactor = physicalSamplingrate / fps / segments / 2;
  deltaToFramesDivider = physicalSamplingrate / fps;
}


void checkIfStillRunning() {
  static unsigned long prevTotalImpCounter2;
  static unsigned long lastImpMillis;
 
  if (totalImpCounter != prevTotalImpCounter2) {
    prevTotalImpCounter2 = totalImpCounter;
    lastImpMillis = millis();
  } else {
   // start countdown pauseDetectedPeriod
    if ((millis() - lastImpMillis) >= pauseDetectedPeriod) {
      lastSampleCounterHaltPos = Read32BitsFromSCI(0x1800);
      musicPlayer.pauseMusic();
      myPID.SetMode(MANUAL);
      lastImpCounterHaltPos = totalImpCounter;
      myState = PAUSED;
    }
  }
}



void speedControlPID(){
  static unsigned long prevTotalImpCounter;
  if (totalImpCounter != prevTotalImpCounter) {
    long actualSampleCount = Read32BitsFromSCI(0x1800);                 // 8.6ms Latenz here
    long desiredSampleCount = totalImpCounter * impToSamplerateFactor;
//    unsigned long latenz = millis() - lastISRTime;               // This had less positive imapct than expected
//    actualSampleCount = actualSampleCount + (latenz * 41);       // 41.344 samples per ms
    long delta = (actualSampleCount - desiredSampleCount);

    total = total - readings[readIndex];  // subtract the last reading
    readings[readIndex] = delta;          // read from the sensor:
    total = total + readings[readIndex];  // add the reading to the total:
    readIndex = readIndex + 1;            // advance to the next position in the array:
  
    if (readIndex >= numReadings) {       // if we're at the end of the array...
      readIndex = 0;                      // ...wrap around to the beginning:
    }
  
    average = total / numReadings;        // calculate the average
//    average = (total >> 4);

    Input = average;
    adjustSamplerate((long) Output);
  
    prevTotalImpCounter = totalImpCounter;        

    myPID.Compute();  // 9.2ms Latenz here

    // Serial.println(latenz); 


// Most of the delay here is from printing. One line of printing means one measurement
// every 2.3 imps, full CSV output is one ever ~6.3 imps.
// Printing the long seems super pricy

    Serial.print(average / deltaToFramesDivider);
    Serial.println(F(" Frames off"));
  }
}


void countISR() {
  totalImpCounter++;
//  lastISRTime = millis();
}

void waitForStartMark() {
  if (digitalRead(startMarkDetectorPin) == HIGH) return;  // There is still leader
  static int impCountToStartMark = 0;
  static uint8_t previousImpDetectorState = LOW;
  static uint8_t impDetectorPinNow = 0;
  impDetectorPinNow = digitalRead(impDetectorPin);
  if (impDetectorPinNow != previousImpDetectorState) {
    impCountToStartMark++;
    previousImpDetectorState = impDetectorPinNow;
  }
  if (impCountToStartMark >= segments * 2 * startMarkOffset) {

    totalImpCounter = 0;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-77000, 77000);

    attachInterrupt(digitalPinToInterrupt(impDetectorISRPIN), countISR, CHANGE);
    
    musicPlayer.resumeMusic();
    clearSampleCounter();
    Serial.println(F("Los geht's!"));

    impCountToStartMark = 0;
    myState = PLAYING;
  }
}

void waitForResumeToPlay(unsigned long impCounterStopPos) {
  if (totalImpCounter == impCounterStopPos) {
    return;
  } else {
    myPID.SetMode(AUTOMATIC);
    restoreSampleCounter(lastSampleCounterHaltPos);
    musicPlayer.resumeMusic();
    Serial.println(F("Weiter geht's!"));
    myState = PLAYING;
  }
}


void i2cReceive (int howMany) {
  if (howMany >= (sizeof i2cCommand) + (sizeof i2cParameter)) {
     wireReadData(i2cCommand);   
     wireReadData(i2cParameter);   
     haveI2Cdata = true;     
   }  // end if have enough data
 }  // end of receive-ISR


 

void i2cRequest()
{
  // FYI: The Atmel AVR 8 bit microcontroller provides for clock stretching while using
  // the ISR for the data request.  This is, by extension, happening here, since this
  // callback is called from the ISR.

  // Send the data.
//  wireWriteData(ppmCorrection);
}




//------------------------------------------------------------------------------
void parse_menu(uint8_t key_command) {

  uint8_t result; // result code from some function as to be tested at later time.

  Serial.print(F("Received Serial command: "));
  Serial.write(key_command);
  Serial.println(F(" "));

  //if s, stop the current track
  if(key_command == 's') {
    Serial.println(F("Stopping"));
    musicPlayer.stopTrack();

  //if 1-9, play corresponding track
  } else if(key_command >= '1' && key_command <= '9') {
    //convert ascii numbers to real numbers
    key_command = key_command - 48;

#if USE_MULTIPLE_CARDS
    sd.chvol(); // assign desired sdcard's volume.
#endif
    //tell the MP3 Shield to play a track
    result = musicPlayer.playTrack(key_command);

    //check result, see readme for error codes.
    if(result != 0) {
      Serial.print(F("Error code: "));
      Serial.print(result);
      Serial.println(F(" when trying to play track"));
    } else {

      Serial.println(F("Playing:"));

    }

  /* Alterativly, you could call a track by it's file name by using playMP3(filename);
  But you must stick to 8.1 filenames, only 8 characters long, and 3 for the extension */
  } else if(key_command == 'f' || key_command == 'F') {
    uint32_t offset = 0;
    if (key_command == 'F') {
      offset = 2000;
    }

    //create a string with the filename
    char trackName[] = "track003.m4a";

#if USE_MULTIPLE_CARDS
    sd.chvol(); // assign desired sdcard's volume.
#endif
    //tell the MP3 Shield to play that file
    result = musicPlayer.playMP3(trackName, offset);

    musicPlayer.setVolume(3,3);
    
    musicPlayer.pauseMusic();
    
    enableResampler();
 
    while (musicPlayer.getState() != paused_playback) {}
    clearSampleCounter();

    myState = TRACK_LOADED;

    Serial.println(F("Waiting for start mark..."));
    

    /*       
     *
     *  track001.mp3  tuuuut
     *  track002.m4a  Mortel
     *  track003.m4a  Der Himmel ist Blau wie noch nie
     *  track004.m4a  Giorgio by Moroder
     *  track005.m4a  Kid Francescoli
     *  track006.m4a  Der kleine Spatz
     *  track007.m4a  Bar in Amsterdam
     *  track008.m4a  Tatort
     *  track009.m4a  Xylophon
     *  
     */
    unsigned long t = millis();
    unsigned long currentmillis = 0;
    Serial.read();



    //check result, see readme for error codes.
    if(result != 0) {
      Serial.print(F("Error code: "));
      Serial.print(result);
      Serial.println(F(" when trying to play track"));
    }

  /* Display the files on the SdCard */
  } else if(key_command == 'd') {
    if(!musicPlayer.isPlaying()) {
      // prevent root.ls when playing, something locks the dump. but keeps playing.
      // yes, I have tried another unique instance with same results.
      // something about SdFat and its 500uint8_t cache.
      Serial.println(F("Files found (name date time size):"));
      sd.ls(LS_R | LS_DATE | LS_SIZE);
    } else {
      Serial.println(F("Busy playing files, try again later."));
    }
  }
}

//------------------------------------------------------------------------------
void adjustSamplerate(signed long ppm2) {
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x1e07);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, ppm2);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, ppm2 >> 16);
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x5b1c);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, 0);
  musicPlayer.Mp3WriteRegister(SCI_AUDATA, musicPlayer.Mp3ReadRegister(SCI_AUDATA));
}
void enableResampler() {
  // Enable 15-16 Resampler
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x1e09);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, 0x0080);
  Serial.println(F("15/16 resampling enabled."));
  
}

void clearSampleCounter() {
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x1800);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, 0);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, 0);
}
void clearErrorCounter() {
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x5a82);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, 0);
}

unsigned long Read32BitsFromSCI(unsigned short addr) {
  unsigned short msbV1, lsb, msbV2;
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, addr+1);
  msbV1 = (unsigned int)musicPlayer.Mp3ReadRegister(SCI_WRAM);
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, addr);
  lsb   = (unsigned long)musicPlayer.Mp3ReadRegister(SCI_WRAM);
  msbV2 = (unsigned int)musicPlayer.Mp3ReadRegister(SCI_WRAM);
  if (lsb < 0x8000U) {
    msbV1 = msbV2;
  }
  return ((unsigned long)msbV1 << 16) | lsb;
}

void restoreSampleCounter(unsigned long samplecounter) {
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x1800); // MSB
  musicPlayer.Mp3WriteRegister(SCI_WRAM, samplecounter);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, samplecounter >> 16);
}



//------------------------------------------------------------------------------



