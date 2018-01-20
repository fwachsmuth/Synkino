/**
 * 
 * This stripped down player allows configuring the PID with teh Processing Frontend Sketch.
 * 
 */
#include <PID_v1.h>

#include <SPI.h>
#include <FreeStack.h>
#include <vs1053_SdFat.h>
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

const int myAddress = 0x08;

const byte sollfps = 18;        // Todo: Read this from filename!
byte segments = 2;              // Wieviele Segmente hat die Umlaufblende?
byte startMarkOffset = 15;


const float physicalSamplingrate = 41344;   // 44100 * 15/16 – to compensate the 15/16 Bit Resampler

const byte impDetectorISRPIN = 3;
const byte impDetectorPin = 3;
const byte startMarkDetectorPin = 5;
const int  pauseDetectedPeriod = (1000 / sollfps * 3);   // Duration of 3 single frames
const int  impToSamplerateFactor = physicalSamplingrate / sollfps / segments / 2;

const int numReadings = 16;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
long total = 0;                  // the running total
int average = 0;                // the average


byte myState = IDLING;     
byte prevState;

volatile long ppmCorrection;
long lastPpmCorrection = 0;

volatile bool haveI2Cdata = false;
volatile byte i2cCommand;
volatile long i2cParameter;

volatile unsigned long lastISRTime;

double Setpoint, Input, Output;

double Kp=12, Ki=3, Kd=3;  // PonM WINNER für 8 Readings
//double Kp=12, Ki=3, Kd=1;  // PonM not so good für 16 Readings



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

unsigned long serialTime; //this will help us know when to talk with processing



//------------------------------------------------------------------------------
void setup() {

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  
  pinMode(impDetectorISRPIN, INPUT);
  pinMode(startMarkDetectorPin, INPUT);

  Setpoint = 0;
  
  uint8_t result; //result code for initialization function

  Serial.begin(9600);

//  Serial.print(F("F_CPU = "));
//  Serial.println(F_CPU);
//  Serial.print(F("Free RAM = ")); 
//  Serial.println(FreeStack(), DEC);  // FreeStack() is provided by SdFat
  
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

//  help();

  Wire.begin(myAddress);
  Wire.onReceive(i2cReceive);
  //Wire.onRequest(i2cRequest);

}

//------------------------------------------------------------------------------
void loop() {

  if (haveI2Cdata) {
    switch (i2cCommand) {   // Debug output
//      case 1: Serial.print(F("CMD: Load Track: "));
//              Serial.println(i2cParameter);
//      break;
//      case 2: Serial.print(F("CMD: Correct PPM: "));
//              Serial.println(i2cParameter);
//      break;
//      case 3: Serial.println(F("CMD: Play"));
//      break;
//      case 4: Serial.println(F("CMD: Pause"));
//      break;
//      case 5: Serial.println(F("CMD: Stop"));
//      break;
//      case 6: Serial.print(F("CMD: Sync to Frame: "));
//              Serial.println(i2cParameter);
//      break;
//      default:Serial.print(i2cCommand);
//              Serial.println(i2cParameter);
    }

    switch (i2cCommand) {
      case CMD_PLAY: 
        totalImpCounter = 0;
        myPID.SetMode(AUTOMATIC);
        myPID.SetOutputLimits(-77000, 77000);

        attachInterrupt(digitalPinToInterrupt(impDetectorISRPIN), countISR, CHANGE);
        
        musicPlayer.resumeMusic();
        clearSampleCounter();
//        Serial.println("Los geht's!");
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
    init_player(); // get command from serial input
  }


  switch (myState) {
    case IDLING:
      init_player();
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

      if(millis()>serialTime)
      {
        SerialReceive();
        SerialSend();
        serialTime+=500;
      }

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
//      case 1: Serial.println(F("--- IDLING."));
//      break;
//      case 2: Serial.println(F("--- LOAD_TRACK"));
//      break;
//      case 3: Serial.println(F("--- TRACK_LOADED."));
//      break;
//      case 4: Serial.println(F("--- RESET"));
//      break;
//      case 5: Serial.println(F("--- PLAYING"));
//      break;
//      case 6: Serial.println(F("--- PAUSED"));
//      break;
//      case 7: Serial.println(F("--- SETTINGS_MENU"));
//      break;
     }
    prevState = myState;
  }
    
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

//    unsigned long latenz = millis() - lastISRTime;
//    actualSampleCount = actualSampleCount + (latenz * 40);       // 41.344 samples per ms, rounding up here

    long delta = (actualSampleCount - desiredSampleCount);

    total = total - readings[readIndex];  // subtract the last reading
    readings[readIndex] = delta;          // read from the sensor:
    total = total + readings[readIndex];  // add the reading to the total:
    readIndex = readIndex + 1;            // advance to the next position in the array:
  
    if (readIndex >= numReadings) {       // if we're at the end of the array...
      readIndex = 0;                      // ...wrap around to the beginning:
    }
  
    average = total / numReadings;        // calculate the average
//    Serial.println(average);              // send it to the computer as ASCII digits


  
    Input = average;
    adjustSamplerate((long) Output);
  
    prevTotalImpCounter = totalImpCounter;        

    myPID.Compute();  // 9.2ms Latenz here

  }
}


void countISR() {
  totalImpCounter++;
//  lastISRTime = millis();
}

void waitForStartMark() {
  if (digitalRead(startMarkDetectorPin) == HIGH) return;  // There is still leader
  static int impCountToStartMark = 0;
  static byte previousImpDetectorState = LOW;
  static byte impDetectorPinNow = 0;
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
    Serial.println("Weiter geht's!");
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
void init_player() {
  uint8_t result; // result code from some function as to be tested at later time.
  uint32_t offset = 0;
  /*       
   *  track001.mp3  tuuuut
   *  track002.m4a  Mortel
   *  track003.m4a  Der Himmel ist Blau wie noch nie
   *  track004.m4a  Giorgio by Moroder
   *  track005.m4a  Kid Francescoli
   *  track006.m4a  Der kleine Spatz
   *  track007.m4a  Bar in Amsterdam
   *  track008.m4a  Tatort
   *  track009.m4a  Xylophon
   */
  char trackName[] = "track006.m4a";

  result = musicPlayer.playMP3(trackName, offset);
  musicPlayer.pauseMusic();

  musicPlayer.setVolume(3,3);
  enableResampler();

  while (musicPlayer.getState() != paused_playback) {}
  clearSampleCounter();

  myState = TRACK_LOADED;

  unsigned long t = millis();
  unsigned long currentmillis = 0;

  if(result != 0) {
    Serial.print(F("Error code: "));
    Serial.print(result);
    Serial.println(F(" when trying to play track"));
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
  // Serial.println(F("15/16 resampling enabled."));
  
}

void clearSampleCounter() {
  /* SCI WRAM is used to upload application programs and data 
  to instruction and data RAMs. The start address must be initialized 
  by writing to SCI WRAMADDR prior to the first write/read of SCI WRAM. 
  As 16 bits of data can be transferred with one SCI WRAM write/read, 
  and the instruction word is 32 bits long, two consecutive writes/reads 
  are needed for each instruction word. The byte order is big-endian 
  (i.e. most significant words first). After each full-word write/read, 
  the internal pointer is autoincremented. */
  
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


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output  
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while(Serial.available()&&index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    else if(index==1) Direct_Reverse = Serial.read();
    else foo.asBytes[index-2] = Serial.read();
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
  {
    Setpoint=double(foo.asFloat[0]);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
                                          //   value of "Input"  in most cases (as 
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the output if we are in 
    {                                     //   manual mode.  otherwise we'll get an
      Output=double(foo.asFloat[2]);      //   output blip, then the controller will 
    }                                     //   overwrite.
    
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    myPID.SetTunings(p, i, d);            //
    
    if(Auto_Man==0) myPID.SetMode(MANUAL);// * set the controller mode
    else myPID.SetMode(AUTOMATIC);             //
    
    if(Direct_Reverse==0) myPID.SetControllerDirection(DIRECT);// * set the controller Direction
    else myPID.SetControllerDirection(REVERSE);          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(Setpoint);   
  Serial.print(" ");
  Serial.print(Input);   
  Serial.print(" ");
  Serial.print(Output);   
  Serial.print(" ");
  Serial.print(myPID.GetKp());   
  Serial.print(" ");
  Serial.print(myPID.GetKi());   
  Serial.print(" ");
  Serial.print(myPID.GetKd());   
  Serial.print(" ");
  if(myPID.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
  if(myPID.GetDirection()==DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");
}

