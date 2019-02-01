/**
 * 
 * To Do / Ideas:
 *  [ ] Find & Load other files than m4a
 *  [ ] Aufschaukeln bei mp3 fixen
 *  [ ] Initiales Projektoranlaufen an Frontend melden
 *  
 *  [ ] 100n an den Encoderoutputs probieren (Prellschutz)
 * 
 *  [ ] Document diffs to vs1053_SdFat.h
 *  [ ] Try/Switch to 15 MHz xtal
 *  [ ] Projektor-Frequenzanzeige
 *  [ ] Update https://github.com/nickgammon/I2C_Anything
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
#include <extEEPROM.h>

// ---- Define the I2C Commands ----------------------------------------------------
//
#define CMD_RESET               1   /* <---                  √  */
#define CMD_SET_SHUTTERBLADES   2   /* <--- (shutterBlades)  √  */
#define CMD_SET_STARTMARK       3   /* <--- (StartMarkOffset)   */
#define CMD_SET_P               4   /* <--- (P-Value for PID)   */
#define CMD_SET_I               5   /* <--- (I-Value for PID)   */
#define CMD_SET_D               6   /* <--- (D-Value for PID)   */
#define CMD_SYNC_OFFSET         7   /* <--- (# of Frames)       */
#define CMD_LOAD_TRACK          8   /* <--- (trackId)        √  */

#define CMD_FOUND_FMT           10  /* ---> (fileFormat)        */
#define CMD_FOUND_FPS           11  /* ---> (fps)            √  */
#define CMD_CURRENT_AUDIOSEC    12  /* ---> (SecNo)          √  */
#define CMD_PROJ_PAUSE          13  /* --->                  √  */
#define CMD_PROJ_PLAY           14  /* --->                  √  */
#define CMD_FOUND_TRACKLENGTH   15  /* ---> (TrackLength)       */
#define CMD_OOSYNC              16  /* ---> (frameCount)     √  */
#define CMD_SHOW_ERROR          17  /* ---> (ErrorCode)      √  */
#define CMD_TRACK_LOADED        18  /* --->                  √  */
#define CMD_STARTMARK_HIT       19  /* --->                  √  */
#define CMD_DONE_PLAYING        20  /* --->                     */

// ---- Define the various States --------------------------------------------------
//
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
uint8_t shutterBlades = 2;         // Wieviele Segmente hat die Umlaufblende?
//uint8_t startMarkOffset = 15;    // Noris
uint8_t startMarkOffset = 53;      // Bauer t610
int16_t syncOffsetImps = 0;        
  
const float physicalSamplingrate = 41344;   // 44100 * 15/16 – to compensate the 15/16 Bit Resampler

#define impDetectorISRPIN  3
#define impDetectorPin 3
#define startMarkDetectorPin 5
int  pauseDetectedPeriod = (1000 / sollfps * 3);   // Duration of 3 single frames
unsigned int impToSamplerateFactor = physicalSamplingrate / sollfps / shutterBlades / 2;
int deltaToFramesDivider = physicalSamplingrate / sollfps;
unsigned int impToAudioSecondsDivider = sollfps * shutterBlades * 2;

#define numReadings   16
long readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
long total = 0;                 // the running total
long average = 0;                // the average


uint8_t myState = IDLING;   
uint8_t debugPrevState;   
uint8_t prevState;

volatile long ppmCorrection;
long lastPpmCorrection = 0;

volatile bool haveI2Cdata = false;
volatile uint8_t i2cCommand;
volatile long i2cParameter;

volatile unsigned long lastISRTime;
unsigned long lastInSyncMillis;


extEEPROM myEEPROM(kbits_256, 1, 64);

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

File pluginFile;
uint32_t eeAddress = 0; 
byte eeData[64] = {};
unsigned int pluginSize;




//------------------------------------------------------------------------------
void setup() {

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  
  pinMode(impDetectorISRPIN, INPUT);
  pinMode(startMarkDetectorPin, INPUT);

  Setpoint = 0;
  
  uint8_t result; //result code for initialization functions

  Serial.begin(115200);

  Serial.print(F("Free RAM = ")); 
  Serial.println(FreeStack(), DEC);  // FreeStack() is provided by SdFat

  Wire.begin(myAddress);
  Wire.setClock(400000L);

  Wire.onReceive(i2cReceive);
  Wire.onRequest(i2cRequest);

  // Initialize the SdCard
  //
  result = sd.begin(SD_SEL, SPI_FULL_SPEED);
  if(result != 1) {
    tellFrontend(CMD_SHOW_ERROR, result + 20);
    Serial.print(F("In SD.Begin: "));
    Serial.println(result);
    sd.initErrorHalt();
  }
  result = sd.chdir("/");

  if(result != 1) {
    tellFrontend(CMD_SHOW_ERROR, result + 30);
    Serial.print(F("In SD.chdir: "));
    Serial.println(result);
    sd.errorHalt("sd.chdir");
  }

  // Initialize the external EEPROM 
  //
  byte i2cStat = myEEPROM.begin(myEEPROM.twiClock400kHz);
  if ( i2cStat != 0 ) {
    Serial.println(F("Could not talk to I2C EEPROM."));
  }


  // Check for DSP Firmware availability
  //
  pluginFile = sd.open("synkino.fir");
  if (pluginFile) {
    Serial.println(F("Found new DSP Patch Package."));
    pluginSize = pluginFile.size();
    myEEPROM.write(eeAddress, pluginSize & 0xFF);     // LSB
    myEEPROM.write(eeAddress + 1, pluginSize >> 8);   // MSB
    eeAddress = 64; // to write full banks
    
    while (pluginFile.available()) {
      pluginFile.read(eeData, 64);

      byte i2cStat = myEEPROM.write(eeAddress, eeData, 64);
      
      if ( i2cStat != 0 ) {
        //there was a problem
        Serial.print(F("I2C Problem: "));
        if ( i2cStat == EEPROM_ADDR_ERR) {
          Serial.println(F("Wrong eeAddress"));
        } else {
          Serial.print(F("I2C error: "));
          Serial.print(i2cStat);
          Serial.println(F(""));
        }
      } else {
        eeAddress += 64;
      }
    }
    // rename DSP patches package
    if (!sd.exists("super.ctl")) {  // If this file exists, keep the synkino.fir file. Used to init all the PCBs.
      pluginFile.rename(sd.vwd(), "patches.053");
    }
  }   
  pluginFile.close();


  // Check for DSP Plugin availability
  //
  pluginFile = sd.open("patches.053");
  if (!pluginFile) {  // Let's re-create the file from EEPROM 
    eeAddress = 0; // Here are two bytes with the plugin size
    pluginSize = myEEPROM.read(eeAddress) + ((myEEPROM.read(eeAddress + 1) << 8) & 0xFF00);
    eeAddress = 64;
    pluginFile = sd.open("patches.053", FILE_WRITE);
    if (pluginFile) {
      for (eeAddress; eeAddress < (pluginSize + 64); eeAddress++) {
        pluginFile.write(myEEPROM.read(eeAddress));  
      }
    } else {
      // if the file didn't open, print an error:
      Serial.println(F("Could not write file patches.053"));
    }
  }
  pluginFile.close();

  


  //Initialize the MP3 Player Shield
  Serial.println("Initing DSP...");
  result = musicPlayer.begin();
  Serial.println("Done.");
  //check result, see readme for error codes.
  if(result != 0) { 
    tellFrontend(CMD_SHOW_ERROR, result + 10);
    Serial.print(F("In player.begin: "));
    Serial.print(result);
    if( result == 6 ) {
      Serial.println(F("ERROR: DSP patch file not found!")); 
    }
  }

}

//------------------------------------------------------------------------------

void loop() {
  if (haveI2Cdata) {
    switch (i2cCommand) {   // Debug output
      case 1: Serial.println(F("CMD_RESET"));
      break;
      case 2: Serial.print(F("CMD_SET_SHUTTERBLADES: "));
              Serial.println(i2cParameter);
      break;
      case 3: Serial.print(F("CMD_SET_STARTMARK: "));
              Serial.println(i2cParameter);
      break;
      case 4: Serial.print(F("CMD_SET_P: "));
              Serial.println(i2cParameter);
      break;
      case 5: Serial.print(F("CMD_SET_I: "));
              Serial.println(i2cParameter);
      break;
      case 6: Serial.print(F("CMD_SET_D: "));
              Serial.println(i2cParameter);
      break;
      case 7: Serial.print(F("CMD_SYNC_OFFSET: "));
              Serial.println(i2cParameter);
      break;
      case 8: Serial.print(F("CMD_LOAD_TRACK: "));
              Serial.println(i2cParameter);
      break;
      default:Serial.println(i2cCommand);
              Serial.println(i2cParameter);
    }

    switch (i2cCommand) {
      case CMD_RESET: 
        musicPlayer.stopTrack();
        myState = IDLING;
        //musicPlayer.vs_init();
      break;
      case CMD_SET_SHUTTERBLADES: 
        shutterBlades = i2cParameter;
        updateFpsDependencies(sollfps);
      break;
      case CMD_SET_STARTMARK: 
        startMarkOffset = i2cParameter;
      break;
      case CMD_SET_P: 
        Kp = i2cParameter;
      break;
      case CMD_SET_I: 
        Ki = i2cParameter;
      break;
      case CMD_SET_D: 
        Kd = i2cParameter;
        myPID.SetTunings(Kp, Ki, Kd);
      break;
      case CMD_SYNC_OFFSET: 
        syncOffsetImps = i2cParameter * shutterBlades * 2;
        // adjust frame counter
      break;
      case CMD_LOAD_TRACK: 
        loadTrackByNo(i2cParameter);
      break;
      default:
        Serial.println(i2cCommand);
        Serial.println(i2cParameter);
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

//  if(Serial.available()) {
//    parse_menu(Serial.read()); // get command from serial input
//  }

// State Machine ----------------------------------------------------------------
//
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
      sendCurrentAudioSec();
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

  if (myState != debugPrevState) {
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
    debugPrevState = myState;
  }
    
}

void tellFrontend(byte command, long parameter) {
  Wire.beginTransmission(7); // This is the Frontend
  wireWriteData(command);  
  wireWriteData(parameter);  
  Wire.endTransmission();    // stop transmitting
}

uint8_t loadTrackByNo(int trackNo) {
  char trackName[11]; 
  char trackNameFound[11];
  
  for (uint8_t fpsGuess = 12; fpsGuess <= 25; fpsGuess++) { 
    // look for m4a first
    sprintf(trackName, "%03d-%d.m4a", trackNo, fpsGuess);  
    if (sd.exists(trackName)) {
      updateFpsDependencies(fpsGuess);
      strcpy(trackNameFound, trackName);
      tellFrontend(CMD_FOUND_FPS, fpsGuess);
//      Serial.print(F("File exists and has ")); 
//      Serial.print(fpsGuess);
//      Serial.print(F(" fps:"));
//      Serial.println(trackName);
    }
  }

  for (uint8_t fpsGuess = 12; fpsGuess <= 25; fpsGuess++) {
    // look for ogg then
    sprintf(trackName, "%03d-%d.ogg", trackNo, fpsGuess);  
    if (sd.exists(trackName)) {
      updateFpsDependencies(fpsGuess);
      strcpy(trackNameFound, trackName);
      tellFrontend(CMD_FOUND_FPS, fpsGuess);
    }
  }
  
  for (uint8_t fpsGuess = 12; fpsGuess <= 25; fpsGuess++) {
    // look for mp3 then
    sprintf(trackName, "%03d-%d.mp3", trackNo, fpsGuess);  
    if (sd.exists(trackName)) {
      updateFpsDependencies(fpsGuess);
      strcpy(trackNameFound, trackName);
      tellFrontend(CMD_FOUND_FPS, fpsGuess);
    }
  }

  for (uint8_t fpsGuess = 12; fpsGuess <= 25; fpsGuess++) {
    // look for mp4 then
    sprintf(trackName, "%03d-%d.mp4", trackNo, fpsGuess);  
    if (sd.exists(trackName)) {
      updateFpsDependencies(fpsGuess);
      strcpy(trackNameFound, trackName);
      tellFrontend(CMD_FOUND_FPS, fpsGuess);
    }
  }


   for (uint8_t fpsGuess = 12; fpsGuess <= 25; fpsGuess++) {
    // look for wav then
    sprintf(trackName, "%03d-%d.wma", trackNo, fpsGuess);  
    if (sd.exists(trackName)) {
      updateFpsDependencies(fpsGuess);
      strcpy(trackNameFound, trackName);
      tellFrontend(CMD_FOUND_FPS, fpsGuess);
    }
  }
 
  
  
  uint8_t result;
  result = musicPlayer.playMP3(trackNameFound);
  if (result != 0) {
    tellFrontend(CMD_SHOW_ERROR, result);
    Serial.print(F("Playback-Error: "));
    Serial.println(result);
  } else {
    Serial.println(F("Waiting for start mark..."));
    musicPlayer.setVolume(3,3);
    musicPlayer.pauseMusic();
    enableResampler();
    while (musicPlayer.getState() != paused_playback) {}
    clearSampleCounter();
    myState = TRACK_LOADED;
    tellFrontend(CMD_TRACK_LOADED, 0);
  }
  return result;
}

void updateFpsDependencies(uint8_t fps) {
  sollfps = fps;                                // redundant?
  pauseDetectedPeriod = (1000 / fps * 3);
  impToSamplerateFactor = physicalSamplingrate / fps / shutterBlades / 2;
  deltaToFramesDivider = physicalSamplingrate / fps;
  impToAudioSecondsDivider = sollfps * shutterBlades * 2;  
}

void sendCurrentAudioSec() {
  static unsigned long prevSecCount;
  static unsigned long currentSecCount;
  currentSecCount = (totalImpCounter + syncOffsetImps) / impToAudioSecondsDivider; 
  
  if (currentSecCount > prevSecCount) {    
    tellFrontend(CMD_CURRENT_AUDIOSEC, currentSecCount); 
    prevSecCount = currentSecCount;
  }
}

void checkIfStillRunning() {
  static unsigned long prevTotalImpCounter2;
  static unsigned long lastImpMillis;
 
  if ((totalImpCounter + syncOffsetImps) != prevTotalImpCounter2) {
    prevTotalImpCounter2 = totalImpCounter + syncOffsetImps;
    lastImpMillis = millis();
  } else {
   // start countdown pauseDetectedPeriod
    if ((millis() - lastImpMillis) >= pauseDetectedPeriod) {
      lastSampleCounterHaltPos = Read32BitsFromSCI(0x1800);
      musicPlayer.pauseMusic();
      myPID.SetMode(MANUAL);
      lastImpCounterHaltPos = totalImpCounter + syncOffsetImps;
      tellFrontend(CMD_PROJ_PAUSE, 0);
      myState = PAUSED;
    }
  }
}



void speedControlPID(){
  static unsigned long prevTotalImpCounter;
  if (totalImpCounter + syncOffsetImps != prevTotalImpCounter) {
    long actualSampleCount = Read32BitsFromSCI(0x1800);                 // 8.6ms Latenz here
    long desiredSampleCount = (totalImpCounter + syncOffsetImps) * impToSamplerateFactor;
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
  
    prevTotalImpCounter = totalImpCounter + syncOffsetImps;        

    myPID.Compute();  // 9.2ms Latenz here

    // Serial.println(latenz); 
    // Most of the delay here is from printing. One line of printing means one measurement
    // every 2.3 imps, full CSV output is one ever ~6.3 imps.
    // Printing the long seems super pricy

    static unsigned int prevFrameOffset;
    int frameOffset = average / deltaToFramesDivider;
    if (frameOffset != prevFrameOffset) {
      tellFrontend(CMD_OOSYNC, frameOffset);
      prevFrameOffset = frameOffset;
    } 
    // Below is a hack to send a 0 every so often if everything is in sync – since occasionally signal gets lost on i2c due 
    // to too busy AVRs. Otherwise, the sync icon might not stop blinking in some cases.
    if ((frameOffset == 0) && (millis() > (lastInSyncMillis + 3000))) {
      tellFrontend(CMD_OOSYNC, 0);
      lastInSyncMillis = millis();
    }

    if (musicPlayer.isPlaying() == 0) { // and/or musicPlayer.getState() == 4
      tellFrontend(CMD_DONE_PLAYING, 0);
    }
  }
}


void countISR() {
  totalImpCounter++;
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
  
  if (impCountToStartMark >= shutterBlades * 2 * startMarkOffset) {
    totalImpCounter = 0;
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-77000, 77000);

    attachInterrupt(digitalPinToInterrupt(impDetectorISRPIN), countISR, CHANGE);
    
    musicPlayer.resumeMusic();
    clearSampleCounter();
    tellFrontend(CMD_STARTMARK_HIT, 0);
    Serial.println(F("Los geht's!"));

    impCountToStartMark = 0;
    myState = PLAYING;
  }
}

void waitForResumeToPlay(unsigned long impCounterStopPos) {
  if ((totalImpCounter + syncOffsetImps) == impCounterStopPos) {
    return;
  } else {
    myPID.SetMode(AUTOMATIC);
    restoreSampleCounter(lastSampleCounterHaltPos);
    musicPlayer.resumeMusic();
    tellFrontend(CMD_PROJ_PLAY, 0);
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

//  Serial.print(F("Received Serial command: "));
//  Serial.write(key_command);
//  Serial.println(F(" "));

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



