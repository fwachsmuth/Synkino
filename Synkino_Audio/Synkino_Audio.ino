/**
 * 
 * *  This is the versionmanaged version!
 * 
 * To Do:
 *  [ ] load patch from EEPROM
 *  [ ] Document diffs to vs1053_SdFat.h
 *  [ ] use vs1053 Interrupt oder Timer based?
 *  [ ] decode delta-sigma line out
 *  [ ] Try/Switch to 15 MHz xtal
 * 
 * 
 */

#include <SPI.h>
#include <FreeStack.h>
#include <SdFat.h>
#include <vs1053_SdFat.h>
#include <Wire.h>
#include <WireData.h>

#define CMD_LOAD_TRACK    1
#define CMD_CORRECT_PPM   2
#define CMD_PLAY          3
#define CMD_PAUSE         4
#define CMD_STOP          5
#define CMD_SYNC_TO_FRAME 6

const int myAddress = 0x08;
volatile long ppmCorrection;
long lastPpmCorrection = 0;

volatile bool haveI2Cdata = false;
volatile byte i2cCommand;
volatile long i2cParameter;


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


//------------------------------------------------------------------------------
void setup() {

  uint8_t result; //result code for initialization functions

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

  help();

  Wire.begin(myAddress);
  Wire.onReceive(i2cReceive);
  //Wire.onRequest(i2cRequest);

}

//------------------------------------------------------------------------------
void loop() {

  if (haveI2Cdata) {
    Serial.print ("Received Command = ");
    Serial.print (i2cCommand);  
    Serial.print (", Parameter = ");
    Serial.println (i2cParameter);  

    switch (i2cCommand) {
      case CMD_PLAY: 
        musicPlayer.resumeMusic();
        Serial.println("Los geht's!");
      break;
      case CMD_CORRECT_PPM:
        if (i2cParameter != lastPpmCorrection) {
          lastPpmCorrection = i2cParameter;
          adjustSamplerate(i2cParameter);
        }
     break;
    case CMD_LOAD_TRACK:
    break;
    case CMD_PAUSE:
    break;
    case CMD_STOP:
    break;
    case CMD_SYNC_TO_FRAME:
      // resyncPlayhead(i2cParameter);
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

  delay(50);
}

void resyncPlayHead() {
  // soll-frame von bob: (0x1800) / 44100 * 16/15 * fps
}

void old_i2cReceive(int byteCount) {
  wireReadData(ppmCorrection);
}

void i2cReceive (int howMany) {
  if (howMany >= (sizeof i2cCommand) + (sizeof i2cParameter)) {
     wireReadData(i2cCommand);   
     wireReadData(i2cParameter);   
     haveI2Cdata = true;     
   }  // end if have enough data
 }  // end of receive-ISR


 

//void i2cRequest()
//{
//  // FYI: The Atmel AVR 8 bit microcontroller provides for clock stretching while using
//  // the ISR for the data request.  This is, by extension, happening here, since this
//  // callback is called from the ISR.
//
//  // Send the data.
//  wireWriteData(ppmCorrection);
//}




//------------------------------------------------------------------------------
void parse_menu(byte key_command) {

  uint8_t result; // result code from some function as to be tested at later time.

  Serial.print(F("Received command: "));
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

  //if +/- to change volume
  } else if((key_command == '-') || (key_command == '+')) {
    union twobyte mp3_vol; // create key_command existing variable that can be both word and double byte of left and right.
    mp3_vol.word = musicPlayer.getVolume(); // returns a double uint8_t of Left and Right packed into int16_t

    if(key_command == '-') { // note dB is negative
      // assume equal balance and use byte[1] for math
      if(mp3_vol.byte[1] >= 254) { // range check
        mp3_vol.byte[1] = 254;
      } else {
        mp3_vol.byte[1] += 2; // keep it simpler with whole dB's
      }
    } else {
      if(mp3_vol.byte[1] <= 2) { // range check
        mp3_vol.byte[1] = 2;
      } else {
        mp3_vol.byte[1] -= 2;
      }
    }
    // push byte[1] into both left and right assuming equal balance.
    musicPlayer.setVolume(mp3_vol.byte[1], mp3_vol.byte[1]); // commit new volume
    Serial.print(F("Volume changed to -"));
    Serial.print(mp3_vol.byte[1]>>1, 1);
    Serial.println(F("[dB]"));

  //if < or > to change Play Speed
  } else if((key_command == '>') || (key_command == '<')) {
    uint16_t playspeed = musicPlayer.getPlaySpeed(); // create key_command existing variable
    // note playspeed of Zero is equal to ONE, normal speed.
    if(key_command == '>') { // note dB is negative
      // assume equal balance and use byte[1] for math
      if(playspeed >= 254) { // range check
        playspeed = 5;
      } else {
        playspeed += 1; // keep it simpler with whole dB's
      }
    } else {
      if(playspeed == 0) { // range check
        playspeed = 0;
      } else {
        playspeed -= 1;
      }
    }
    musicPlayer.setPlaySpeed(playspeed); // commit new playspeed
    Serial.print(F("playspeed to "));
    Serial.println(playspeed, DEC);

  /* Alterativly, you could call a track by it's file name by using playMP3(filename);
  But you must stick to 8.1 filenames, only 8 characters long, and 3 for the extension */
  } else if(key_command == 'f' || key_command == 'F') {
    uint32_t offset = 0;
    if (key_command == 'F') {
      offset = 2000;
    }

    //create a string with the filename
    char trackName[] = "track009.m4a";

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

    Serial.println("Warten auf Play-Command...");
//    musicPlayer.resumeMusic();


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
     *  
     *  
     */
    unsigned long t = millis();
    unsigned long currentmillis = 0;
//    while(currentmillis <= 10000000) {
//      currentmillis = millis() - t +1;
//      if (currentmillis % 10000 == 0) {
//        unsigned long sampleCount = Read32BitsFromSCI(0x1800);
//   
//        Serial.print(F("Sample count: "));
//        Serial.print(sampleCount * 16/15);  // compensate 15/16 resampler
//        Serial.print(F(" after "));
//    
//       
//        Serial.print(currentmillis);
//        Serial.print(F(" ms, so "));
//        Serial.print((float)(sampleCount * 16/15) / currentmillis * 1000);
//        Serial.println(F(" kHz in last 10 sec"));
//        
////        newPpmCorrection = ((float)44100 - ((float)(sampleCount) / currentmillis * 1000)) * 3 + initialPpmCorrection;
//        //adjustSamplerate(newPpmCorrection);
//        
//      }
//    }
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
      // something about SdFat and its 500byte cache.
      Serial.println(F("Files found (name date time size):"));
      sd.ls(LS_R | LS_DATE | LS_SIZE);
    } else {
      Serial.println(F("Busy playing files, try again later."));
    }

  /* Get and Display the Audio Information */
  } else if(key_command == 'i') {
    musicPlayer.getAudioInfo();

  } else if(key_command == 'p') {
    if( musicPlayer.getState() == playback) {
      musicPlayer.pauseMusic();
      Serial.println(F("Pausing"));
    } else if( musicPlayer.getState() == paused_playback) {
      musicPlayer.resumeMusic();
      Serial.println(F("Resuming"));
    } else {
      Serial.println(F("Not Playing!"));
    }

  } else if(key_command == 'S') {
    Serial.println(F("Current State of VS10xx is."));
    Serial.print(F("isPlaying() = "));
    Serial.println(musicPlayer.isPlaying());

    Serial.print(F("getState() = "));
    switch (musicPlayer.getState()) {
    case uninitialized:
      Serial.print(F("uninitialized"));
      break;
    case initialized:
      Serial.print(F("initialized"));
      break;
    case deactivated:
      Serial.print(F("deactivated"));
      break;
    case loading:
      Serial.print(F("loading"));
      break;
    case ready:
      Serial.print(F("ready"));
      break;
    case playback:
      Serial.print(F("playback"));
      break;
    case paused_playback:
      Serial.print(F("paused_playback"));
      break;
    case testing_memory:
      Serial.print(F("testing_memory"));
      break;
    case testing_sinewave:
      Serial.print(F("testing_sinewave"));
      break;
    }
    Serial.println();

   

#if !defined(__AVR_ATmega32U4__)
  } else if(key_command == 'm') {
      uint16_t teststate = musicPlayer.memoryTest();
    if(teststate == -1) {
      Serial.println(F("Unavailable while playing music or chip in reset."));
    } else if(teststate == 2) {
      teststate = musicPlayer.disableTestSineWave();
      Serial.println(F("Unavailable while Sine Wave Test"));
    } else {
      Serial.print(F("Memory Test Results = "));
      Serial.println(teststate, HEX);
      Serial.println(F("Result should be 0x83FF."));
      Serial.println(F("Reset is needed to recover to normal operation"));
    }

  } else if(key_command == 'R') {
    musicPlayer.stopTrack();
    musicPlayer.vs_init();
    Serial.println(F("Reseting VS10xx chip"));

  } else if(key_command == 'O') {
    musicPlayer.end();
    Serial.println(F("VS10xx placed into low power reset mode."));

  } else if(key_command == 'o') {
    musicPlayer.begin();
    Serial.println(F("VS10xx restored from low power reset mode."));

  } else if(key_command == 'D') {
    uint16_t diff_state = musicPlayer.getDifferentialOutput();
    Serial.print(F("Differential Mode "));
    if(diff_state == 0) {
      musicPlayer.setDifferentialOutput(1);
      Serial.println(F("Enabled."));
    } else {
      musicPlayer.setDifferentialOutput(0);
      Serial.println(F("Disabled."));
    }

  } else if(key_command == 'V') {
    musicPlayer.setVUmeter(1);
    Serial.println(F("Use \"No line ending\""));
    Serial.print(F("VU meter = "));
    Serial.println(musicPlayer.getVUmeter());
    Serial.println(F("Hit Any key to stop."));

    while(!Serial.available()) {
      union twobyte vu;
      vu.word = musicPlayer.getVUlevel();
      Serial.print(F("VU: L = "));
      Serial.print(vu.byte[1]);
      Serial.print(F(" / R = "));
      Serial.print(vu.byte[0]);
      Serial.println(" dB");
      delay(1000);
    }
    Serial.read();

    musicPlayer.setVUmeter(0);
    Serial.print(F("VU meter = "));
    Serial.println(musicPlayer.getVUmeter());

  } else if(key_command == 'M') {
    uint16_t monostate = musicPlayer.getMonoMode();
    Serial.print(F("Mono Mode "));
    if(monostate == 0) {
      musicPlayer.setMonoMode(1);
      Serial.println(F("Enabled."));
    } else {
      musicPlayer.setMonoMode(0);
      Serial.println(F("Disabled."));
    }
#endif

  } else if(key_command == 'h') {
    help();
    
  } else if(key_command == 'x') {
      pitchrate = pitchrate + 10000;
      adjustSamplerate(pitchrate);
      Serial.print(F("New Pitchrate: "));
      Serial.println(pitchrate);
    
  } else if(key_command == 'y') {
      pitchrate = pitchrate - 10000;
      adjustSamplerate(pitchrate);
      Serial.print(F("New Pitchrate: "));
      Serial.println(pitchrate);
   
  } else if(key_command == 'q') {
    // Resampler enable with rate compensation
    musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x1e09);
    musicPlayer.Mp3WriteRegister(SCI_WRAM, 0x0080);
    
  } else if(key_command == 'a') {
    // Read and print Sample Count Register
    unsigned long sampleCount = Read32BitsFromSCI(0x1800);
    Serial.println(sampleCount);
    
  }
  // print prompt after key stroke has been processed.
//  Serial.print(F("Time since last command: "));  
//  Serial.println((float) (millis() -  millis_prv)/1000, 2);  
//  millis_prv = millis();
//  Serial.print(F("Enter s,1-9,+,-,>,<,f,F,d,i,p,S"));
//#if !defined(__AVR_ATmega32U4__)
//  Serial.print(F(",m,R,k,O,o,D,V,M"));
//#endif
//  Serial.println(F(" or h:"));
}

//------------------------------------------------------------------------------
void adjustSamplerate(signed long ppm2) {
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x1e07);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, ppm2);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, ppm2 >> 16);
  /* oldClock4KHz = 0 forces adjustment calculation when rate checked. */
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x5b1c);
  musicPlayer.Mp3WriteRegister(SCI_WRAM, 0);
   /* Write to AUDATA or CLOCKF checks rate and recalculates adjustment. */
 
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
  musicPlayer.Mp3WriteRegister(SCI_WRAMADDR, 0x1801);
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




//------------------------------------------------------------------------------
void help() {
  Serial.println(F("COMMANDS:"));
  Serial.println(F(" [1-9] to play a track"));
  Serial.println(F(" [f] play track001.mp3 by filename example"));
  Serial.println(F(" [F] same as [f] but with initial skip of 2 second"));
  Serial.println(F(" [s] to stop playing"));
  Serial.println(F(" [d] display directory of SdCard"));
  Serial.println(F(" [+ or -] to change volume"));
  Serial.println(F(" [> or <] to increment or decrement play speed by 1 factor"));
  Serial.println(F(" [i] retrieve current audio information (partial list)"));
  Serial.println(F(" [p] to pause."));
  Serial.println(F(" [S] Show State of Device."));
#if !defined(__AVR_ATmega32U4__)
  Serial.println(F(" [m] perform memory test. reset is needed after to recover."));
  Serial.println(F(" [M] Toggle between Mono and Stereo Output."));
  Serial.println(F(" [k] Skip a predetermined number of ms in current track."));
  Serial.println(F(" [R] Resets and initializes VS10xx chip."));
  Serial.println(F(" [O] turns OFF the VS10xx into low power reset."));
  Serial.println(F(" [o] turns ON the VS10xx out of low power reset."));
  Serial.println(F(" [D] to toggle SM_DIFF between inphase and differential output"));
  Serial.println(F(" [V] Enable VU meter Test."));

  Serial.println(F(" [x] Pitch up"));
  Serial.println(F(" [y] Pitch down"));
  Serial.println(F(" [q] Enable 15/16 Resampler with rate compensation"));
  Serial.println(F(" [a] Print SampleCount Register"));
#endif
  Serial.println(F(" [h] this help"));
}


