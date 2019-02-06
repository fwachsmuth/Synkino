/**
 * 
 *  Todos  
 *  [ ] Chart the PID with the very low control possibilties (on Bauer t610)
 *  
 *  Bugs:
 *  [ ] Bug: Manual Start -> Frame Offset destroys time display
 *  [ ] Manual Start isn't starting if no STartmark Sensor is connected or it still sees White
 *  [ ] Start Mark Offset 0 sometimes freezes screen
 *  [ ] First encoder knob push isn't always read?
 *  
 *  Features / Ideas
 *  [ ] Show File's Sampling Rate
 *  [ ] Show Projector Frquency
 *  
 *  Hygene
 *  [ ] Document diffs to vs1053_SdFat.h
 *  [ ] Try/Switch to 15 MHz xtal
 *  [ ] Cleanup state machine
 *  [ ] Remove PID options
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
//                                  Audio               Frontend
#define CMD_RESET               1   /* <---                   */
#define CMD_SET_SHUTTERBLADES   2   /* <--- (shutterBlades)   */
#define CMD_SET_STARTMARK       3   /* <--- (StartMarkOffset) */
#define CMD_SET_P               4   /* <--- (P-Value for PID) */
#define CMD_SET_I               5   /* <--- (I-Value for PID) */
#define CMD_SET_D               6   /* <--- (D-Value for PID) */
#define CMD_SYNC_OFFSET         7   /* <--- (# of Frames)     */
#define CMD_LOAD_TRACK          8   /* <--- (trackId)         */

#define CMD_FOUND_FMT           10  /* ---> (fileFormat)      */
#define CMD_FOUND_FPS           11  /* ---> (fps)             */
#define CMD_CURRENT_AUDIOSEC    12  /* ---> (SecNo)           */
#define CMD_PROJ_PAUSE          13  /* --->                   */
#define CMD_PROJ_PLAY           14  /* --->                   */
#define CMD_FOUND_TRACKLENGTH   15  /* ---> (TrackLength)     */
#define CMD_OOSYNC              16  /* ---> (frameCount)      */
#define CMD_SHOW_ERROR          17  /* ---> (ErrorCode)       */
#define CMD_TRACK_LOADED        18  /* --->                   */
#define CMD_STARTMARK_HIT       19  /* --->                   */
#define CMD_DONE_PLAYING        20  /* --->                   */
#define CMD_RESETAUDIO          21  /* --->                   */

// ---- Define the various States --------------------------------------------------
//
#define IDLING            1
#define TRACK_LOADED      3
#define RESET             4
#define PLAYING           5
#define PAUSED            6

const int myAddress = 0x08;        // Listen on the I2C Bus

uint8_t sollfps = 18;        
uint8_t shutterBlades = 2;         // Cutout count of the shutter blade
uint8_t startMarkOffset = 52;      // Bauer t610, example value
int32_t syncOffsetImps = 0;        

uint8_t sampleCountRegisterValid = true;  // It takes >8 KiB of data until the Ogg Samplecount Register
                                          // is valid. Disable the PID until then
  
float physicalSamplingrate = 44444;           // 44100 * 15/16 – to compensate the 15/16 Bit Resampler, but only if NOT ogg!
unsigned long sampleCountBaseLine = 0;    // The Ogg Sample Counter doesn't start at 0, probably due to Buffers.
                                          // This var stores the baseline right after having the file loaded.

#define impDetectorISRPIN  3
#define impDetectorPin 3
#define startMarkDetectorPin 5
int  pauseDetectedPeriod = (1000 / sollfps * 3);   // Duration of 3 single frames
unsigned int impToSamplerateFactor = physicalSamplingrate / sollfps / shutterBlades / 2;
int deltaToFramesDivider = physicalSamplingrate / sollfps;
unsigned int impToAudioSecondsDivider = sollfps * shutterBlades * 2;

#define numReadings   6
long readings[numReadings];      // the readings from the analog input
int readIndex = 0;               // the index of the current reading
long total = 0;                  // the running total
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
File audioFile;
uint32_t eeAddress = 0; 
byte eeData[64] = {};
unsigned int pluginSize;

unsigned long prevSecCount;
unsigned long currentSecCount;
unsigned long prevTotalImpCounter2;
unsigned long lastImpMillis;



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

  //Initialize the DSP
  Serial.println("Initing DSP...");
  result = musicPlayer.begin();
  Serial.println("Done.");
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
           resetAudio();
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
  }


// State Machine ----------------------------------------------------------------
//
  switch (myState) {
    case IDLING:
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
    default:
    break;
  }

  if (myState != debugPrevState) {
    switch (myState) {   // Debug output
      case 1: Serial.println(F("--- IDLING."));
      break;
      case 3: Serial.println(F("--- TRACK_LOADED."));
      break;
      case 4: Serial.println(F("--- RESET"));
      break;
      case 5: Serial.println(F("--- PLAYING"));
      break;
      case 6: Serial.println(F("--- PAUSED"));
      break;
    }
    debugPrevState = myState;
  }
    
}

//------------------------------------------------------------------------------
void tellFrontend(byte command, long parameter) {
  Wire.beginTransmission(7); // This is the Frontend
  wireWriteData(command);  
  wireWriteData(parameter);  
  Wire.endTransmission();    // stop transmitting
}

//------------------------------------------------------------------------------
uint8_t loadTrackByNo(int trackNo) {
  char trackName[11]; 
  char trackNameFound[11];
  bool fileExists = false; 
  uint8_t result;
  
  for (uint8_t fpsGuess = 12; fpsGuess <= 25; fpsGuess++) {
    sprintf(trackName, "%03d-%d.ogg", trackNo, fpsGuess);   // Mkaing up some possible filenames
    if (sd.exists(trackName)) {
      fileExists = true;
      updateFpsDependencies(fpsGuess);
      strcpy(trackNameFound, trackName);
      tellFrontend(CMD_FOUND_FPS, fpsGuess);
    }
  }

  if (!fileExists) {
    Serial.println(F("404 - File not found."));
    tellFrontend(CMD_SHOW_ERROR, 2);
    result = 2;   // File not found
  } else {
    result = musicPlayer.playMP3(trackNameFound);
    if (result != 0) {
      tellFrontend(CMD_SHOW_ERROR, result);
      Serial.print(F("Playback-Error: "));
      Serial.println(result);
    } else {
      musicPlayer.setVolume(254,254);
      musicPlayer.pauseMusic();
      musicPlayer.setVolume(4,4);
      Serial.println(F("Waiting for start mark..."));
      enableResampler();
  
      physicalSamplingrate = Read16BitsFromSCI(SCI_AUDATA) & 0xfffe;  // Mask the Mono/Stereo Bit
      updateFpsDependencies(sollfps);   // TODO: do it again, this time to adjust for sampling rates. Maybe betteer as a second function?
      
      Serial.println(physicalSamplingrate);
      
  //    Serial.print(F("HDAT1 (4F67) :"));                    // Would determine file type, see 9.6.9 in data sheet
  //    Serial.println(Read16BitsFromSCI(SCI_HDAT1), HEX);
      
      while (musicPlayer.getState() != paused_playback) {}
      clearSampleCounter();
      myState = TRACK_LOADED;
      tellFrontend(CMD_TRACK_LOADED, 0);
    }
  }
  return result;
}

//------------------------------------------------------------------------------
void updateFpsDependencies(uint8_t fps) {
  sollfps = fps;                                // redundant? sollfps is global, fps is local. Horrible.
  pauseDetectedPeriod = (1000 / fps * 3);
  sampleCountRegisterValid = true;           // It takes 8 KiB until the Ogg Sample Counter is valid for sure
  impToSamplerateFactor = physicalSamplingrate / fps / shutterBlades / 2;
  deltaToFramesDivider = physicalSamplingrate / fps;
  impToAudioSecondsDivider = sollfps * shutterBlades * 2;  

/*
//  Serial.print(F("FPS: "));
//  Serial.println(sollfps);
//  Serial.print(F("Phys. SR: "));
//  Serial.println(physicalSamplingrate);
//  Serial.print(F("Blades: "));
//  Serial.println(shutterBlades);
//  Serial.print(F("Offset: "));
//  Serial.println(startMarkOffset);
*/
}

//------------------------------------------------------------------------------
void sendCurrentAudioSec() {
  currentSecCount = (totalImpCounter + syncOffsetImps) / impToAudioSecondsDivider; 
  
  if (currentSecCount > prevSecCount) {    
    tellFrontend(CMD_CURRENT_AUDIOSEC, currentSecCount); 
    prevSecCount = currentSecCount;
  }

  if (currentSecCount >= 0 && !sampleCountRegisterValid) {  // might use imps here, not a hardcoded second?
    sampleCountRegisterValid = true;
//  Serial.println(F("1 sec is over."));
  }
}

//------------------------------------------------------------------------------
void checkIfStillRunning() {
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

//------------------------------------------------------------------------------
void speedControlPID() {
  static unsigned long prevTotalImpCounter;
  static unsigned long previousActualSampleCount;
  if (totalImpCounter + syncOffsetImps != prevTotalImpCounter) {
 
    if (sampleCountRegisterValid) {
      long actualSampleCount = Read32BitsFromSCI(0x1800) - sampleCountBaseLine;                 // 8.6ms Latenz here
      previousActualSampleCount = actualSampleCount;
      if (actualSampleCount < previousActualSampleCount) {
        actualSampleCount = previousActualSampleCount;
      }
      long desiredSampleCount = (totalImpCounter + syncOffsetImps) * impToSamplerateFactor;
      long delta = (actualSampleCount - desiredSampleCount);

/*
//   This puts nifty CSV to the Console, to graph PID results.  
//      Serial.print(F("Current Sample:\t"));
//      Serial.print(actualSampleCount);
//      Serial.print(F(","));
      Serial.print(desiredSampleCount);
      Serial.print(F(","));
//      Serial.print(F(","));
      Serial.println(delta);
//      Serial.print(F(","));
//      Serial.print(F(" Bitrate: "));
//      Serial.println(getBitrate());
*/
 
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
//    Serial.println((long) Output);
    
      prevTotalImpCounter = totalImpCounter + syncOffsetImps;        

      myPID.Compute();  // 9.2ms Latenz here
  
      static unsigned int prevFrameOffset;
      int frameOffset = average / deltaToFramesDivider;
      if (frameOffset != prevFrameOffset) {
        tellFrontend(CMD_OOSYNC, frameOffset);
        prevFrameOffset = frameOffset;
      } 
      // Below is a hack to send a 0 every so often if everything is in sync – since occasionally signal gets lost on i2c due 
      // to too busy AVRs. Otherwise, the sync icon might not stop blinking in some cases.
      if ((frameOffset == 0) && (millis() > (lastInSyncMillis + 500))) {
        tellFrontend(CMD_OOSYNC, 0);
        lastInSyncMillis = millis();
      }
    }
    if (musicPlayer.isPlaying() == 0) { // and/or musicPlayer.getState() == 4
      tellFrontend(CMD_DONE_PLAYING, 0);
      resetAudio();
    }
  }
}

//------------------------------------------------------------------------------
void countISR() {
  totalImpCounter++;
}

//------------------------------------------------------------------------------
void waitForStartMark() {
  if (digitalRead(startMarkDetectorPin) == HIGH) return;  // There is still leader
  static int impCountToStartMark = 0;
  static uint8_t previousImpDetectorState = LOW;
  static uint8_t impDetectorPinNow = 0;
  uint32_t highestCorrectionLimit;
  impDetectorPinNow = digitalRead(impDetectorPin);
  if (impDetectorPinNow != previousImpDetectorState) {
    impCountToStartMark++;
    previousImpDetectorState = impDetectorPinNow;
  }
  
  if (impCountToStartMark >= shutterBlades * 2 * startMarkOffset) {
    totalImpCounter = 0;
    myPID.SetMode(AUTOMATIC);

    Serial.print(F("File Sampling rate: "));
    Serial.print(physicalSamplingrate);

    if (physicalSamplingrate == 22050.00) {
      highestCorrectionLimit = 600000;
    } else if (physicalSamplingrate == 32000.00) {
        highestCorrectionLimit = 285000;
    } else if (physicalSamplingrate == 44100.00) {
        highestCorrectionLimit = 77000;
    } else if (physicalSamplingrate == 48000.00) {
        highestCorrectionLimit = 29000;
    } else {
        highestCorrectionLimit = 77000; // lets assume 44.2 kHz here
    }

    Serial.print(F("Max: "));
    Serial.println(highestCorrectionLimit);
    
    myPID.SetOutputLimits(-400000, highestCorrectionLimit); // -400k - 300k scheint zu gehen. testen.
    /*
     * 48.0  kHz is changeable from -400000 (73/440 = 17%) to 29000 (472/440 = 107% up)
     * 44.1  kHz is changeable from -400000 (73/440 = 17%) to 77000 (511/440 = 116% up) 
     * 32.0  kHz is changeable from -400000 (73/440 = 17%) to 285000 (702/440 = 159% up) 
     * 22.05 kHz is changeable from -400000 (73/440 = 17%) to 600000 (1000/440 = 227% up)
     * 
     * Approx Max Limit: (51200 - SR) * (47800 / SR) * 10 // clamped at 29000 & 600000
     * 
     */

    attachInterrupt(digitalPinToInterrupt(impDetectorISRPIN), countISR, CHANGE);
    
    musicPlayer.resumeMusic();
    
    clearSampleCounter();
    sampleCountBaseLine = Read32BitsFromSCI(0x1800);        
    
    Serial.print(F("Sample Count Baseline: "));
    Serial.println(sampleCountBaseLine);
    Serial.println(F("--------"));
    
    tellFrontend(CMD_STARTMARK_HIT, 0);
    Serial.println(F("Los geht's!"));

    impCountToStartMark = 0;
    myState = PLAYING;
  }
}

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
void i2cReceive (int howMany) {
  if (howMany >= (sizeof i2cCommand) + (sizeof i2cParameter)) {
     wireReadData(i2cCommand);   
     wireReadData(i2cParameter);   
     haveI2Cdata = true;     
   }  // end if have enough data
 }  // end of receive-ISR


//------------------------------------------------------------------------------
void adjustSamplerate(signed long ppm2) {
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x1e07);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, ppm2);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, ppm2 >> 16);
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x5b1c);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, 0);
  musicPlayer.Mp3WriteRegister(SCI_AUDATA, musicPlayer.Mp3ReadRegister(SCI_AUDATA));
}

//------------------------------------------------------------------------------
void enableResampler() {
  // Enable 15-16 Resampler
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x1e09);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, 0x0080);
  Serial.println(F("15/16 resampling enabled."));
  
}

//------------------------------------------------------------------------------
void clearSampleCounter() {
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x1800);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, 0);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, 0);
}

//------------------------------------------------------------------------------
void clearErrorCounter() {
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x5a82);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, 0);
}

unsigned int Read16BitsFromSCI(unsigned short addr) {
  return (unsigned int)musicPlayer.Mp3ReadRegister(addr);
}

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
void restoreSampleCounter(unsigned long samplecounter) {
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x1800); // MSB
  musicPlayer.Mp3WriteRegister(SCI_WRAM, samplecounter);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, samplecounter >> 16);
}

//------------------------------------------------------------------------------
uint16_t getBitrate() {
  return (musicPlayer.Mp3ReadWRAM(para_byteRate)>>7);
}

//------------------------------------------------------------------------------
//uint16_t getSamplerate() {
//  return (musicPlayer.Mp3ReadWRAM(SCI_AUDATA));
//}

void resetAudio() {
  detachInterrupt(digitalPinToInterrupt(impDetectorISRPIN));
  prevSecCount = 0;
  currentSecCount = 0;
  prevTotalImpCounter2 = 0;
  lastImpMillis = 0;

  sampleCountRegisterValid = true;

  for (int i = 0; i< numReadings; i++) {
    readings[i] = 0;
  }

//  prevTotalImpCounter = 0;

  syncOffsetImps = 0;
  totalImpCounter = 0;
  total = 0;
  average = 0;
  readIndex = 0;

  Input = 0;
  Output = 0;
  Setpoint = 0;

  //  sendCurrentAudioSec();
  
  tellFrontend(CMD_RESETAUDIO, 0);
  delay(100); // wait to let the audio switch go down
  musicPlayer.stopTrack();
  myState = IDLING;
  // mute audio
  // zero out seconds
  musicPlayer.vs_init();
  // zero out readings[]
  
}

