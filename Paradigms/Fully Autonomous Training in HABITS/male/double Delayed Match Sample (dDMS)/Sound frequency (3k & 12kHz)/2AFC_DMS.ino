#include <string.h>
#include <SD.h>
#include <SPI.h> // SD card
// #include "MPR121.h"   // MPR121 Capacitive Touch Breakout
// #include <Wire.h>     // Already included in gpSMART_Habits
#include "HX711.h" // HX711 Weighting Amplifier
#include <WiFiEsp.h>
#include <WiFiEspUdp.h> // For WiFi

#include "gpSMART_Habits.h" // For State Machine

//******************************************************************************
// FlasherX -- firmware OTA update via Intel Hex file over serial or SD stream
//******************************************************************************
//
// Based on Flasher3 (Teensy 3.x) and Flasher4 (Teensy 4.x) by Jon Zeeff
//
// Jon Zeeff 2016, 2019, 2020 This code is in the public domain.
// Please retain my name in distributed copies, and let me know about any bugs
#include "FXUtil.h" // read_ascii_line(), hex file support
extern "C"
{
#include "FlashTxx.h" // TLC/T3x/T4x/TMM flash primitives
}
/* Connection: Teensy <<===Serial1===>> WiFi module <<===UDP===>> PC */

/****************************************************************************************************/
/********************************************** Public *********************************************/
/****************************************************************************************************/

/********** SD card **********/
const byte chipSelect = BUILTIN_SDCARD; // Teensy 3.5 & 3.6 & 4.1 on-board SD card
String string_tmp;
// String file_name[6] = {"Trial.txt" ,"event.txt" ,"cage_info.txt" ,"paraS.txt","Tevent.txt" ,"weight.txt"};

/********** WiFi **********/
char ssid[] = "HABITS";             // your network SSID (name) Brain&Brain-inspired Computing
char pass[] = "bcihabits";             // your network password BCIsLab514
int wifi_status = WL_IDLE_STATUS;     // the Wifi radio's status
uint16_t local_port = 52021;          // local port for message; local prot for file: (local_port + 1)
byte ip_bytes[] = {192, 168, 1, 100}; // Server IP
IPAddress serverIP(ip_bytes[0], ip_bytes[1], ip_bytes[2], ip_bytes[3]);
uint16_t serverPort = 52021; // init server Port
char packetBuffer[255];      // buffer to hold incoming packet
char sdBuffer[512];          // received buffer maximum size is 512
char outBuffer[1000];        // buffer to hold outcoming packet,UDP max 1472
byte ip_base = 100;
int cage_id = 101;
char task_name[40];
WiFiEspUDP Udp;

/********** HX711 Weight **********/
#define DOUT 7
#define CLK 8
HX711 scale;
long calibration_factor = 6470;        //
long weight_offset = 0;                //
float weight_value[30] = {0};          // weighting info in last 30 sec
unsigned long weight_timing[30] = {0}; // weighting info in last 30 sec
int weight_counter = 0;
int tare_counter = 0;
unsigned long last_weight_read_time = 0;

/********** Port Definition **********/
const byte switchPin = 4; // ToggerSwitch pin to start/pause experiment
const byte ledPin = 13;   // LED pin

// Events: record all the events happened during one loop
typedef struct
{
  int events_num = 0;
  unsigned long events_time[20] = {};
  byte events_id[20] = {}; /* 1: restart; 2: free reward; */
  int events_value[20] = {0};
} Events;
Events Ev;

/********** Other public **********/
static void FLASHLED(uint16_t duration_ms)
{
  digitalWrite(ledPin, HIGH);
  delay(duration_ms);
  digitalWrite(ledPin, LOW);
}

/********** gpSMART **********/
gpSMART smart;
extern TrialResult trial_res;
extern volatile bool smartFinished; // Has the system exited the matrix (final state)?
extern volatile bool smartRunning;  // 1 if state matrix is running
extern byte smartFlag[4];

// noise
const byte noisePin = 3;
byte LowBit;

/********** Trial related **********/
#define RECORD_TRIALS 100      // record recent 100 trials history
const byte recent_trials = 50; // Calculate performance for rencent 50 trials
// Define  RewardFlag struct.
typedef struct
{
  byte flag_L_water;
  byte flag_R_water;
  byte flag_M_water;
  unsigned int past_trials;
} RewardFlag;

typedef struct
{
  // Public
  unsigned int currTrialNum = 0;       // current trial number
  byte currProtocolIndex = 0;          // index of Protocol
  unsigned int currProtocolTrials = 0; // number of trials in current protocol
  byte currProtocolPerf = 0;           // performance: 0-100%
  byte TrialPresentMode = 0;           // 0"pattern",1"random",2"antiBias",3"fixed"
  byte ProtocolIndexHistory[RECORD_TRIALS] = {};
  byte TrialTypeHistory[RECORD_TRIALS] = {}; // 0 undef; 1 left; 2 right; 3 middle;
  byte OutcomeHistory[RECORD_TRIALS] = {};   // 0 no-response; 1 correct; 2 error; 3 others
  byte EarlyLickHistory[RECORD_TRIALS] = {}; // 0-no earlylick; 1-earlylick; 2-undef
  unsigned int totalRewardNum = 0;
  unsigned int retention_counter = 0;
  byte reward_left = 30;
  byte reward_right = 30;
  byte reward_middle = 30;
  byte high_light_intensity = 255;
  byte low_light_intensity = 1;
  unsigned long Trial_txt_position = 0;     // currently Trial file cursor position
  unsigned long Tevent_txt_position = 0;    // currently Tevent file cursor position
  RewardFlag GaveFreeReward = {0, 0, 0, 0}; // [freeReward flag L, R, M, past_trials] todo...

  // Task-specific
  int SamplePeriod = 100;
  int DelayPeriod = 500;
  int PreCueDelayPeriod = 500;
  int CueDelayPeriod = 200;
  int TimeOut = 1000;
  int AnswerPeriod = 10000;
  int ConsumptionPeriod = 750;
  int StopLickingPeriod = 1000;
  int EarlyLickPeriod = 100;
  int extra_TimeOut = 0;
} Parameters_behavior;
Parameters_behavior S;

/*******************Curriculum Training**************************/
typedef struct CurriculumParams
{ 
  float FastEMA = 0; // 
  float SlowEMA = 0; // 
  float LP = 0;
  float FastAlpha = 0.99; // average 100
  float SlowAlpha = 0.998; // average 1000
  float LPAlpha = 0.998;
  float NormSoftMax_T = 0.1; 
  unsigned int t = 1;                  // update step
  int currDL = 1;
} CurriculumParams;
CurriculumParams CP;

/********** Define OutputAction**********/
OutputAction LeftWaterOutput = {"DO1", 1};
OutputAction RightWaterOutput = {"DO2", 1};
OutputAction LowSoundOutput = {"tPWM2", 1}; // tPWM2 top sound
OutputAction HighSoundOutput = {"tPWM2", 2};
OutputAction CueOutput = {"tPWM2", 3};
OutputAction NoiseOutput = {"Flag1", 1};
OutputAction LeftSoundOutput = {"tPWM1", 2};  // tPWM1 left sound ,freq is 10KHz
OutputAction RightSoundOutput = {"tPWM3", 2}; // tPWM3 right sound ,freq is 10KHz
// Note: the freq setting is working for all regular PWM below
OutputAction LeftLightOutput = {"PWM1", S.low_light_intensity};  // left light, value 0-255
OutputAction RightLightOutput = {"PWM5", S.low_light_intensity}; // right light ,same as above

OutputAction BlueLightOutput = {"PWM4", S.high_light_intensity};  // Blue light...
OutputAction RedLightOutput = {"PWM2", S.high_light_intensity};   // red light...
OutputAction GreenLightOutput = {"PWM3", S.high_light_intensity}; // Green light...

// light intensity
OutputAction LeftLightOutputH = {"PWM1", S.high_light_intensity};  // High left light intensity
OutputAction RightLightOutputH = {"PWM5", S.high_light_intensity}; // High right light intensity

// sound frequency ;sound orients ;light orients ;reversal trials ;wavelength ;light intensity;

byte TrialType = 1;    // 0 undef; 1 left; 2 right; 3 middle;
byte TrialOutcome = 3; // 0 no-response; 1 correct; 2 error; 3 others
byte is_earlylick = 2; // 0-no earlylick; 1-earlylick; 2-undef
byte MaxSame = 3;
byte MinCorrect = 1; // minimum correct trials in the last 5 trials
byte Perf100 = 0;
byte EarlyLick100 = 0;
unsigned long last_reward_time = 0;
int timed_reward_count = 0;

int modality = 0; // sample stimulus : 0 is A ;1 is B

int CatchTrialProb = 2; // use a small number of trials as catch trials

byte pause_signal_PC = 0;
byte tare_flag = 1;
byte tare_length = 0;
bool paused = 1;
byte ledState = LOW;

/********** FlasherX **********/
#define FLASHERX_VERSION "FlasherX v2.3"
#define LARGE_ARRAY (0) // 1 = define large array to test large hex file

Stream *serial = &Serial;

#if (LARGE_ARRAY)
// nested arrays of integers to add code size for testing
#define A0                                               \
  {                                                      \
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 \
  } // 16  elements 64
#define A1                                                         \
  {                                                                \
    A0, A0, A0, A0, A0, A0, A0, A0, A0, A0, A0, A0, A0, A0, A0, A0 \
  } // 256 elements 1KB
#define A2                                                         \
  {                                                                \
    A1, A1, A1, A1, A1, A1, A1, A1, A1, A1, A1, A1, A1, A1, A1, A1 \
  } // 4K  elements 16KB
#define A3                                                         \
  {                                                                \
    A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2, A2 \
  } // 64K elements 256KB
#define A4                                                         \
  {                                                                \
    A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3, A3 \
  } // 1M  elements 4MB

// const variables reside in flash and get optimized out if never accessed
// use uint8_t -> 1MB, uint16_t -> 2MB, uint32_t -> 4MB, uint64_t -> 8MB)
PROGMEM const uint8_t a[16][16][16][16][16] = A4;
#endif

/****************************************************************************************************/
/********************************************** Setup() *********************************************/
/****************************************************************************************************/

void setup()
{
  delay(3000);          // for debug
  Serial.begin(115200); // initialize seiral for debugging
  pinMode(ledPin, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP); // low if switch on; hight if switch off

  /********** SD card ***********/
  if (!SD.begin(chipSelect))
  {
    Serial.println("SD Card failed, or not present");
    return; // don't do anything more:
  }
  else
  {
    Serial.println("SD is working...");
  }
  if (read_SD_cage_info() < 0)
  { // read cage_id, calibration_factor, weight_offset, ...
    // return;
    //  default values
    cage_id = 101;
    calibration_factor = 6470;
    weight_offset = 215911;
    sprintf(task_name, "TBD");
  }
  FLASHLED(200);

  /********** HX711 Weight ***********/
  scale.begin(DOUT, CLK);
  // scale.tare();  //Reset the scale to 0
  scale.set_offset(weight_offset);
  scale.set_scale(calibration_factor);
  FLASHLED(200);

  /********** gpSMART ***********/
  smart.init(2); // init smart with num_electrodes used
  byte PortEnabled[4] = {0, 0, 0, 0};
  smart.setDigitalInputsEnabled(PortEnabled);
  // tPWM1: left sound; tPWM2: top sound; tPWM3: right sound
  smart.setTruePWMFrequency(2, 1, 3000, 128);  // (low sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(2, 2, 12000, 128); // (high sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(2, 3, 6000, 128);  // (go cue) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  FLASHLED(200);

  // read parameters from SD Card to override S
  read_SD_para_S();
  read_Curriculum_Param_info();
  // write an artificial event to mark the restart of Arduino board
  Ev.events_num = 0;
  Ev.events_id[Ev.events_num] = 1; // restart;
  Ev.events_time[Ev.events_num] = Teensy3Clock.get();
  Ev.events_value[Ev.events_num] = -1;
  Ev.events_num = 1;
  write_SD_event(); // write event to file
  Ev.events_num = 0;
  // random seeds
  randomSeed(analogRead(0));

  /********** WiFi ***********/
  Serial1.begin(115200); // initialize serial1 for ESP module
  WiFi.init(&Serial1);   // initialize ESP module
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println("WiFi shield not present");
    return; // don't do anything more:
  }
  IPAddress cageIP(ip_bytes[0], ip_bytes[1], ip_bytes[2], ip_base + cage_id);
  WiFi.config(cageIP);
  // attempt to connect to WiFi network
  while (wifi_status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    wifi_status = WiFi.begin(ssid, pass); // Connect to WPA/WPA2 network
  }
  Serial.println("Connected to wifi");
  printWifiStatus();
  Udp.begin(local_port); // Listening on port: local_port
  // send 'join' Message to PC
  sprintf(outBuffer, "SE: Cage %d is online now; IP: %d.%d.%d.%d.%s", cage_id, ip_bytes[0], ip_bytes[1], ip_bytes[2], ip_base + cage_id, task_name); // Special message starts with 'S'
  udpPrint(outBuffer);
  Serial.println("UDP setup sending;");
  FLASHLED(200);

  digitalWrite(ledPin, HIGH); // Light up LED to indicate init finished

  /********** FlasherX ***********/

#if (LARGE_ARRAY) // if true, access array so it doesn't get optimized out
  serial->printf("Large Array -- %08lX\n", (uint32_t)&a[15][15][15][15][15]);
#endif
}

// test int
int testbyte = 0;

/****************************************************************************************************/
/********************************************** Loop() **********************************************/
/****************************************************************************************************/
void loop()
{

  // Check if the toggle switch is ON
  if (digitalRead(switchPin) == 0 && pause_signal_PC == 0)
  { // if yes, run the state matrix

    if (paused == 1)
    {
      paused = 0;
      digitalWrite(ledPin, HIGH);
      udpPrint("M: Program RESUME!!!");
      // in case SD card was removed and re-insert, need re-initilization
      SD.begin(chipSelect);
      read_SD_para_S();
      read_Curriculum_Param_info();
      // free reward to fill the lickport tube
      free_reward(100);
    }

    if (smartFinished)
    { // i.e., a trial is done

      smartFinished = false;
      digitalWrite(ledPin, HIGH);
      S.currTrialNum++;

      UpdateTrialOutcome();  // including white noise (if error) and inter-trial interval
      write_SD_trial_info(); // log trial info and event to SD
      SendTrialInfo2PC();    // send trial info to PC through UDP

      //////////// for next trial ///////////
      autoChangeProtocol(); // Change protocol and parameters based on performance;
      autoReward();         // Set Reward Flag if many wrongs in a row;
      trialSelection();     // determine TrialType;
      write_SD_para_S();    // write parameter S (updated after this trial) to SD card;
      write_Curriculum_Param_info();
      //////////// for next trial ///////////
    }
    else if (!smartRunning)
    {
      construct_matrix_and_Run();
      digitalWrite(ledPin, LOW); // Teensy led does not support PWM
    }

    // White noise for error trial or background stimuli
    if (smartFlag[0] == 1)
    {
      digitalWrite(noisePin, LowBit); // about 55 us/bit
      LowBit = random(2);
    }
    else
    {
      digitalWrite(noisePin, LOW);
    }

    if (millis() - last_reward_time > 3 * 3600000)
    { // there is no reward in last 3 hours
      // free reward to fill the lickport tube
      free_reward(50);
      last_reward_time = millis();
      udpPrint("M: No Reward in Last 3 Hours.");

      timed_reward_count++;

      if (timed_reward_count >= 4)
      { // in last 12 hours no reward
        timed_reward_count = 0;
        // more free reward to fill the lickport tube
        free_reward(100);
        last_reward_time = millis();
        udpPrint("E: No Reward in Last 12 Hours.");
      }
    }

    // read weight data every 1 sec
    if (millis() - last_weight_read_time > 1000)
    {
      last_weight_read_time = millis();
      weight_timing[weight_counter] = Teensy3Clock.get();
      weight_value[weight_counter] = scale.get_units(); // 65 us; if read every 100 ms
      weight_counter++;
      if (weight_counter >= 30)
      {
        // write to SD card // 1.5 ms?
        File dataFile = SD.open("weight.txt", FILE_WRITE);
        if (dataFile)
        {
          for (int i = 0; i < weight_counter; i++)
          {
            dataFile.print(weight_timing[i]);
            dataFile.print(" ");
            dataFile.println(weight_value[i]);
          }
        }
        else
        {
          Serial.println("Failed to open weight.txt");
        }
        dataFile.close();
        // tare the scale
        if (tare_counter >= 20)
        { // per 10 min tare the scale
          for (int i = 0; i < weight_counter; i++)
          {
            if (weight_value[i] >= 10)
            {
              tare_flag = 0;
              tare_length = 0;
              break;
            }
          }
          if (tare_flag)
          {
            tare_length++;
            if (tare_length >= 20)
            {                 // continuous 10 min weight data is below the tare threshold
              // scale.tare(10); // Reset the scale to 0
              // weight_offset = scale.get_offset();
              // write_SD_cage_info();
              tare_counter = 0; // Only after tared scale, the tare_counter will be reset to 0
              tare_length = 0;
            }
          }
          tare_flag = 1;
        }

        // write to PC
        String outBuffer_str = "W";
        for (int i = 0; i < weight_counter; i++)
        {
          outBuffer_str += String(weight_value[i], 2);
          outBuffer_str += ",";
        }
        outBuffer_str.toCharArray(outBuffer, 255);
        udpPrint(outBuffer);

        weight_counter = 0;
        tare_counter++;
      }
    }

    // write current event info
    if (Ev.events_num > 0)
    {
      write_SD_event();
      Ev.events_num = 0;
    }
  }

  else
  { // if the toggle switch is off
    if (paused == 0)
    {
      paused = 1; // execute only one time
      smart.Stop();
      udpPrint("M: Program PAUSED!!!");
    }
    // Flashing LED
    if (ledState == LOW)
    {
      ledState = HIGH;
    }
    else
    {
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);
    // stimulus valve check
    // auditory top(high) -> top(low) -> left -> right 
    // left
    analogWriteFrequency(2,3000);
    analogWrite(2, 128);
    analogWrite(5,10);
    delay(200);
    analogWriteFrequency(2,10000);
    analogWrite(2, 128);
    delay(200);
    analogWrite(2, 0);
    analogWrite(5,0);
    delay(500);
    // middle
    analogWriteFrequency(3,3000);
    analogWrite(3, 128);
    analogWrite(9,10);
    delay(200);
    analogWriteFrequency(3,10000);
    analogWrite(3, 128);
    delay(200);
    analogWrite(3, 0);
    analogWrite(9,0);
    delay(500);
  
    // right
    analogWriteFrequency(16,3000);
    analogWrite(16, 128);
    analogWrite(20,10);
    delay(200);
    analogWriteFrequency(16,10000);
    analogWrite(16, 128);
    delay(200);
    analogWrite(16, 0);
    analogWrite(20,0);
    delay(500);
  }

  /********** UDP communication with PC ***********/
  // int packetSize = Udp.parsePacket();
  if (Udp.parsePacket())
  {                              // receiving data from PC
    Udp.read(packetBuffer, 255); // int len = ...
    byte commandByte = packetBuffer[0];
    // T for Time calibration,
    switch (commandByte)
    {
    case 'T':                                   // Time calibration
      Teensy3Clock.set(atoi(&packetBuffer[1])); // str2int
      break;
    case 'P': // Pause the system
      pause_signal_PC = 1;
      break;
    case 'M': // resuMe the system
      pause_signal_PC = 0;
      break;
    case '0':       // tare/zero the scale
      scale.tare(); // Reset the scale to 0
      weight_offset = scale.get_offset();
      write_SD_cage_info();
      break;
    case 'C': // calibration weight
      calibration_factor = round(calibration_factor * scale.get_units() / 20);
      scale.set_scale(calibration_factor);
      write_SD_cage_info();
      break;
    case 'R': // set reward values for left, right and middle
    {
      String packetString = packetBuffer;
      int ind1 = packetString.indexOf(',');                            // finds location of first ,
      S.reward_left = packetString.substring(1, ind1).toInt();         // captures first data String
      int ind2 = packetString.indexOf(',', ind1 + 1);                  // finds location of second ,
      S.reward_right = packetString.substring(ind1 + 1, ind2).toInt(); // captures second data String
      int ind3 = packetString.indexOf(',', ind2 + 1);
      S.reward_middle = packetString.substring(ind2 + 1, ind3).toInt();
      write_SD_para_S();
      free_reward(S.reward_left);
      break;
    }
    case 'L': // set Light intensity value for low and high
    {
      String packetString = packetBuffer;
      int ind1 = packetString.indexOf(',');                                    // finds location of first ,
      S.low_light_intensity = packetString.substring(1, ind1).toInt();         // captures first data String
      int ind2 = packetString.indexOf(',', ind1 + 1);                          // finds location of second ,
      S.high_light_intensity = packetString.substring(ind1 + 1, ind2).toInt(); // captures second data String TODO bug
      write_SD_para_S();
      free_reward(S.reward_left);
      break;
    }
    case 'A': // read All info
      // send back all info in control panel, like reward values,
      sprintf(outBuffer, "A%d;%d;%d;%d;%d;%d;%.2f;%lu;%d;", S.reward_left, S.reward_right, S.reward_middle, S.low_light_intensity, S.high_light_intensity, S.currProtocolIndex, scale.get_units(), Teensy3Clock.get(), S.extra_TimeOut);
      udpPrint(outBuffer);
      break;
    case 'H': // 'S'pecical msg: 'H'andshake
      udpPrint("SH");
      break;
    case 'D':                  // communicate with PC about the SD files(currently this is just a hand shake)
      pause_signal_PC = 1;     // TODO
      sprintf(outBuffer, "D"); // TODO 待改成发送SD中存在的文件list
      udpPrint(outBuffer);
      break;
    case 'B': // remove selected SD file
    {
      char Remove_file[50];
      String packetStringSDRemove = packetBuffer;
      int indSDRemove = packetStringSDRemove.indexOf(',');                // finds location of first ,
      String Removefile = packetStringSDRemove.substring(1, indSDRemove); // captures first data String
      Removefile.toCharArray(Remove_file, 50);
      SD.remove(Remove_file);
      break;
    }
    case 'F': // SD file case .PC fetches the SD card file
    {
      pause_signal_PC = 1;
      String packetStringSD = packetBuffer;
      int indSD = packetStringSD.indexOf(',');            // finds location of first ,
      String fileSD = packetStringSD.substring(1, indSD); // captures first data String
      // Serial.println(~fileSD.compareTo("Trial&Tevent.txt"));
      if (fileSD.compareTo("Trial&Tevent.txt"))
      {
        Send_SD_file_2PC(fileSD);
      }
      else
      {
        Send_SD_file_2PC("Trial.txt");  // Trial.txt
        Send_SD_file_2PC("Tevent.txt"); // Tevent.txt
      }
      pause_signal_PC = 0;
      break;
    }
    case 'Z': // Protocol manual change
    {
      String packetString = packetBuffer;
      int ind1 = packetString.indexOf(',');                          // finds location of first ,
      S.currProtocolIndex = packetString.substring(1, ind1).toInt(); // captures first data String
      manualChangeProtocol();                                        // change manual protocol parameters
      write_SD_para_S();
      free_reward(S.reward_left);
      break;
    }
    case 'E': // Extra timeout reciever
    {
      String packetString = packetBuffer;
      int ind1 = packetString.indexOf(',');                      // finds location of first ,
      S.extra_TimeOut = packetString.substring(1, ind1).toInt(); // captures first data String
      write_SD_para_S();
      free_reward(S.reward_left);
      break;
    }
    case 'Y': // remote update firmware or receive the SD card file
    {
      int sdErr;
      String packetStringSDReceived = packetBuffer;
      int indSDReceived1 = packetStringSDReceived.indexOf(',');                                                // finds location of first ,
      String fileSDReceived = packetStringSDReceived.substring(1, indSDReceived1);                             // captures first data String which is file name
      int indSDReceived2 = packetStringSDReceived.indexOf(',', indSDReceived1 + 1);                            // finds location of second..
      unsigned long fileLength = packetStringSDReceived.substring(indSDReceived1 + 1, indSDReceived2).toInt(); // get the file size
      int expandedDot = fileSDReceived.indexOf('.');                                                           // note: the file name cannot include the character '.'
      String expandedName = fileSDReceived.substring(expandedDot + 1);

      // receive the file and save to SD card
      // pause the state machine
      pause_signal_PC = 1;
      sdErr = Receive_SD_file_fromPC(fileSDReceived, fileLength); // argument: fileName, fileLength
      // check the received file is complete and send the ACK to PC
      if (expandedName.compareTo("hex") == 0)
      {
        // update the firmware over the air .hex file
        // use the flashXer to update the firmware in the SD card automatically
        if (!sdErr)
        {
          FlasherXTrigger(fileSDReceived); // begin firmware update!
        }
      }
      else if (expandedName.compareTo("txt") == 0)
      {
      }
      // resume the state machine
      // pause_signal_PC = 0;
      break;
    }
    default: // never happen...
      udpPrint("E: UDP received some unusual packets!");
      break;
    }
  }

} // end of loop()

/****************************************************************************************************/
/********************************************** Functions *******************************************/
/****************************************************************************************************/

void construct_matrix_and_Run()
{
  OutputAction GreenLightOutputBackground = {"PWM3", S.low_light_intensity}; // Green light...
  OutputAction NoSampleOutput = {"Flag4" , 0}; // empty output

  //randomSeed(analogRead(39)); // anlog input as random seeds

  smart.EmptyMatrix(); // clear matrix at the begining of each trial

  switch (S.currProtocolIndex)
  {
  case 0:
  {                             // PROTOCOL 0: teaching animal to lick both side
    OutputAction SampleOutput; 
    OutputAction TestOutput;
    OutputAction RewardOutput;

    S.PreCueDelayPeriod = float(random(200, 500));
	  S.CueDelayPeriod    = float(random(200, 500));
  

    String LeftLickAction;
    String RightLickAction;
    String ActionAfterCue;
    float reward_dur = S.reward_left;

    modality = int(random(0 ,2)); // A is 3KHz ;B is 12KHz
  

    switch (TrialType) // determine left or right
    {
    case 1: // (AB) (BA) left
    {
      switch (modality)
      {
      case 0:
        {
          SampleOutput = LowSoundOutput;
          TestOutput   = HighSoundOutput;
        }
        break;
      case 1:
        {
          SampleOutput = HighSoundOutput;
          TestOutput   = LowSoundOutput;
        }
        break;
      
      default:
        break;
      }

      LeftLickAction = "Reward";
      RightLickAction = "AnswerPeriod"; // no error trial
      RewardOutput = LeftWaterOutput;
      reward_dur = S.reward_left;
    }
    break;
    case 2: //(AA) (BB) right
    {
      switch (modality)
      {
      case 0:
        {
          SampleOutput = LowSoundOutput;
          TestOutput   = LowSoundOutput;
        }
        break;
      case 1:
        {
          SampleOutput = HighSoundOutput;
          TestOutput   = HighSoundOutput;
        }
        break;
      
      default:
        break;
      }

      LeftLickAction = "AnswerPeriod"; // no error trial
      RightLickAction = "Reward";
      RewardOutput = RightWaterOutput;
      reward_dur = S.reward_right;
    }
    break;
    }

    if (random(100) < 60)
    { // free reward probability
      ActionAfterCue = "GiveFreeDrop";
    }
    else
    {
      ActionAfterCue = "ResponseCue";
    }

    // flashes bin condition
    StateTransition PreCueDelayPeriod_Cond[1] = {{"Tup", "SamplePeriod"}};
    StateTransition TrialStart_Cond[1] = {{"Tup", "PreCueDelayPeriod"}}; //
    StateTransition SamplePeriod_Cond[1]   = {{"Tup", "DelayPeriod"}};
    StateTransition DelayPeriod_Cond[1] = {{"Tup", "TestPeriod"}};
    StateTransition ResponseCue_Cond[1] = {{"Tup", "AnswerPeriod"}}; 
    StateTransition GiveFreeDrop_Cond[1] = {{"Tup", "ResponseCue"}};
    StateTransition AnswerPeriod_Cond[3] = {{"Lick1In", LeftLickAction}, {"Lick2In", RightLickAction}, {"Tup", "NoResponse"}};
    StateTransition Reward_Cond[1] = {{"Tup", "RewardConsumption"}};
    StateTransition Tup_Exit_Cond[1] = {{"Tup", "exit"}};
    StateTransition NoResponse_Cond[3] = {{"Lick1In", "exit"}, {"Lick2In", "exit"}, {"Tup", "exit"}};
    StateTransition ErrorTrial_Cond[1] = {{"Tup", "exit"}};
    StateTransition TestPeriod_Cond[1]     = {{"Tup" ,"CueDelayPeriod"}};
	StateTransition CueDelayPeriod_Cond[1]     = {{"Tup" ,ActionAfterCue}};

    OutputAction SamplePeriod_Output[1]     	   = {SampleOutput};
    OutputAction PreCueDelayPeriod_Output[1] = {GreenLightOutputBackground}; // low intensity green light as a trial start signal
    OutputAction ResponseCue_Output[1] = {CueOutput}; 
    OutputAction Reward_Output[1] = {RewardOutput};
    OutputAction NoOutput[0] = {};
    OutputAction ErrorOutput[1] = {NoiseOutput}; // not used
    OutputAction TestPeriod_Output[1] = {TestOutput};
    

    gpSMART_State states[13] = {};
    // visual flash states and conditions
    states[0] = smart.CreateState("TrialStart", 5, 1, TrialStart_Cond, 0, NoOutput); // msec
    states[1] = smart.CreateState("SamplePeriod", S.SamplePeriod, 1, SamplePeriod_Cond, 1, SamplePeriod_Output);
    states[2] = smart.CreateState("DelayPeriod", float(S.DelayPeriod), 1, DelayPeriod_Cond, 0, NoOutput);
    states[3] = smart.CreateState("ResponseCue", 100, 1, ResponseCue_Cond, 1, ResponseCue_Output); 
    states[4] = smart.CreateState("GiveFreeDrop", reward_dur, 1, GiveFreeDrop_Cond, 1, Reward_Output);
    states[5] = smart.CreateState("AnswerPeriod", S.AnswerPeriod, 3, AnswerPeriod_Cond, 0, NoOutput);
    states[6] = smart.CreateState("Reward", reward_dur, 1, Reward_Cond, 1, Reward_Output);
    states[7] = smart.CreateState("RewardConsumption", S.ConsumptionPeriod, 1, Tup_Exit_Cond, 0, NoOutput);
    states[8] = smart.CreateState("NoResponse", random(30, 60) * 60 * 1000, 3, NoResponse_Cond, 0, NoOutput); // 1-hr: 60*60*1000 msec
    // error trial state (not used state)
    states[9] = smart.CreateState("ErrorTrial", 500, 1, ErrorTrial_Cond, 1, ErrorOutput); // not used
    // pre-cue delay
    states[10] = smart.CreateState("PreCueDelayPeriod", S.PreCueDelayPeriod, 1, PreCueDelayPeriod_Cond, 1, PreCueDelayPeriod_Output);
    states[11] = smart.CreateState("TestPeriod" , S.SamplePeriod ,1 ,TestPeriod_Cond ,1 ,TestPeriod_Output);
	states[12] = smart.CreateState("CueDelayPeriod", float(S.CueDelayPeriod), 1, CueDelayPeriod_Cond, 0, NoOutput);
    // Predefine State sequence.
    for (int i = 0; i < 13; i++)
    {
      smart.AddBlankState(states[i].Name);
    }

    // Add a state to state machine.
    for (int i = 0; i < 13; i++)
    {
      smart.AddState(&states[i]);
    }

    // smart.PrintMatrix(); // for debug

    // Run the matrix
    smart.Run();
  }
  break;

  // Protocol 1: add error trials to give a feedback which mice can use it to lick following cue 
  case 1: // pattern
  case 2: // anti-bias
  {                             // PROTOCOL 0: teaching animal to lick both side
    OutputAction SampleOutput; 
    OutputAction TestOutput;
    OutputAction RewardOutput;

    S.PreCueDelayPeriod = float(random(200, 500));
  
	

    String LeftLickAction;
    String RightLickAction;
    String ActionAfterCue;
    float reward_dur = S.reward_left;

    modality = int(random(0 ,2)); // A is 3KHz ;B is 12KHz
  

    switch (TrialType) // determine left or right
    {
    case 1: // (AB) (BA) left
    {
      switch (modality)
      {
      case 0:
        {
          SampleOutput = LowSoundOutput;
          TestOutput   = HighSoundOutput;
        }
        break;
      case 1:
        {
          SampleOutput = HighSoundOutput;
          TestOutput   = LowSoundOutput;
        }
        break;
      
      default:
        break;
      }

      LeftLickAction = "Reward";
      RightLickAction = "ErrorTrial"; // no error trial
      RewardOutput = LeftWaterOutput;
      reward_dur = S.reward_left;
    }
    break;
    case 2: //(AA) (BB) right
    {
      switch (modality)
      {
      case 0:
        {
          SampleOutput = LowSoundOutput;
          TestOutput   = LowSoundOutput;
        }
        break;
      case 1:
        {
          SampleOutput = HighSoundOutput;
          TestOutput   = HighSoundOutput;
        }
        break;
      
      default:
        break;
      }

      LeftLickAction = "ErrorTrial"; // no error trial
      RightLickAction = "Reward";
      RewardOutput = RightWaterOutput;
      reward_dur = S.reward_right;
    }
    break;
    }

     // free reward
    if ((TrialType == 1 && S.GaveFreeReward.flag_L_water == 1) || (TrialType == 2 && S.GaveFreeReward.flag_R_water == 1))
    {
      ActionAfterCue = "GiveFreeDrop";
      if (TrialType == 1)
      {
        S.GaveFreeReward.flag_L_water = 0;
      }
      else
      {
        S.GaveFreeReward.flag_R_water = 0;
      }
    }
    else
    {
      ActionAfterCue = "ResponseCue";
    }

    // flashes bin condition
    StateTransition PreCueDelayPeriod_Cond[1] = {{"Tup", "SamplePeriod"}};
    StateTransition TrialStart_Cond[1] = {{"Tup", "PreCueDelayPeriod"}}; //
    StateTransition SamplePeriod_Cond[1]   = {{"Tup", "DelayPeriod"}};
    StateTransition DelayPeriod_Cond[1] = {{"Tup", "TestPeriod"}};
    StateTransition ResponseCue_Cond[1] = {{"Tup", "AnswerPeriod"}}; 
    StateTransition GiveFreeDrop_Cond[1] = {{"Tup", "ResponseCue"}};
    StateTransition AnswerPeriod_Cond[3] = {{"Lick1In", LeftLickAction}, {"Lick2In", RightLickAction}, {"Tup", "NoResponse"}};
    StateTransition Reward_Cond[1] = {{"Tup", "RewardConsumption"}};
    StateTransition Tup_Exit_Cond[1] = {{"Tup", "exit"}};
    StateTransition NoResponse_Cond[3] = {{"Lick1In", "exit"}, {"Lick2In", "exit"}, {"Tup", "exit"}};
    StateTransition ErrorTrial_Cond[1] = {{"Tup", "TimeOut"}};
    StateTransition TestPeriod_Cond[1]     = {{"Tup" ,"CueDelayPeriod"}};
	StateTransition CueDelayPeriod_Cond[1]     = {{"Tup" ,ActionAfterCue}};

    OutputAction SamplePeriod_Output[1]     	   = {SampleOutput};
    OutputAction PreCueDelayPeriod_Output[1] = {GreenLightOutputBackground}; // low intensity green light as a trial start signal
    OutputAction ResponseCue_Output[1] = {CueOutput}; // not used
    OutputAction Reward_Output[1] = {RewardOutput};
    OutputAction NoOutput[0] = {};
    OutputAction ErrorOutput[1] = {NoiseOutput}; // not used
    OutputAction TestPeriod_Output[1] = {TestOutput};
    

    gpSMART_State states[14] = {};
    // visual flash states and conditions
    states[0] = smart.CreateState("TrialStart", 5, 1, TrialStart_Cond, 0, NoOutput); // msec
    states[1] = smart.CreateState("SamplePeriod", S.SamplePeriod, 1, SamplePeriod_Cond, 1, SamplePeriod_Output);
    states[2] = smart.CreateState("DelayPeriod", float(S.DelayPeriod), 1, DelayPeriod_Cond, 0, NoOutput);
    states[3] = smart.CreateState("ResponseCue", 100, 1, ResponseCue_Cond, 1, ResponseCue_Output); // Not used
    states[4] = smart.CreateState("GiveFreeDrop", reward_dur, 1, GiveFreeDrop_Cond, 1, Reward_Output);
    states[5] = smart.CreateState("AnswerPeriod", S.AnswerPeriod, 3, AnswerPeriod_Cond, 0, NoOutput);
    states[6] = smart.CreateState("Reward", reward_dur, 1, Reward_Cond, 1, Reward_Output);
    states[7] = smart.CreateState("RewardConsumption", S.ConsumptionPeriod, 1, Tup_Exit_Cond, 0, NoOutput);
    states[8] = smart.CreateState("NoResponse", random(30, 60) * 60 * 1000, 3, NoResponse_Cond, 0, NoOutput); // 1-hr: 60*60*1000 msec
    // error trial state (not used state)
    states[9] = smart.CreateState("ErrorTrial", 500, 1, ErrorTrial_Cond, 1, ErrorOutput); 
    // pre-cue delay
    states[10] = smart.CreateState("PreCueDelayPeriod", S.PreCueDelayPeriod, 1, PreCueDelayPeriod_Cond, 1, PreCueDelayPeriod_Output);
    states[11] = smart.CreateState("TestPeriod" , S.SamplePeriod ,1 ,TestPeriod_Cond ,1 ,TestPeriod_Output);
    states[12]  = smart.CreateState("TimeOut",          S.TimeOut,            1, Tup_Exit_Cond,        0, NoOutput);
	states[13] = smart.CreateState("CueDelayPeriod", float(S.CueDelayPeriod), 1, CueDelayPeriod_Cond, 0, NoOutput);
    // Predefine State sequence.
    for (int i = 0; i < 14; i++)
    {
      smart.AddBlankState(states[i].Name);
    }

    // Add a state to state machine.
    for (int i = 0; i < 14; i++)
    {
      smart.AddState(&states[i]);
    }

    // smart.PrintMatrix(); // for debug

    // Run the matrix
    smart.Run();
  }
      break;
  case 3: //  antiBias  //  (samplePeriod delayPeriod)(1000 1000)
  case 4: //  antiBias (900 1500)
  case 5: //  antiBias (800 2000)
  case 6: //  antibias (700 2500)
  case 7: //  antibias (600 3000)
  case 8: //  antibias (500 3500)
  case 9://   antibias
  // Protocol 2: testing protocol random
  case 10: 
  {     
    OutputAction SampleOutput; 
    OutputAction TestOutput;
    OutputAction RewardOutput;

    S.PreCueDelayPeriod = float(random(200, 500));
    if(S.PreCueDelayPeriod == 105){ // cue tup period
      S.PreCueDelayPeriod = float(random(200 ,500));
    }
	
    // test
    if(S.currProtocolIndex == 10){
      Curriculum();
      // S.DelayPeriod = int(random(2 ,7)) * 500; // 1000ms ;1500ms ;2000ms ;2500ms ;3000ms
      // S.CueDelayPeriod = int(random(2 ,7)) * 500; // 1000ms ;1500ms ;2000ms ;2500ms ;3000ms
    }


    String LeftLickAction;
    String RightLickAction;
    String ActionAfterCue;
    float reward_dur = S.reward_left;

    modality = int(random(0 ,2)); // A is 3KHz ;B is 10KHz
  

    switch (TrialType) // determine left or right
    {
    case 1: // (AB) (BA) left
    {
      switch (modality)
      {
      case 0:
        {
          SampleOutput = LowSoundOutput;
          TestOutput   = HighSoundOutput;
        }
        break;
      case 1:
        {
          SampleOutput = HighSoundOutput;
          TestOutput   = LowSoundOutput;
        }
        break;
      
      default:
        break;
      }

      LeftLickAction = "Reward";
      RightLickAction = "ErrorTrial"; // no error trial
      RewardOutput = LeftWaterOutput;
      reward_dur = S.reward_left;
    }
    break;
    case 2: //(AA) (BB) right
    {
      switch (modality)
      {
      case 0:
        {
          SampleOutput = LowSoundOutput;
          TestOutput   = LowSoundOutput;
        }
        break;
      case 1:
        {
          SampleOutput = HighSoundOutput;
          TestOutput   = HighSoundOutput;
        }
        break;
      
      default:
        break;
      }

      LeftLickAction = "ErrorTrial"; // no error trial
      RightLickAction = "Reward";
      RewardOutput = RightWaterOutput;
      reward_dur = S.reward_right;
    }
    break;
    }

     // free reward
    if ((TrialType == 1 && S.GaveFreeReward.flag_L_water == 1) || (TrialType == 2 && S.GaveFreeReward.flag_R_water == 1))
    {
      ActionAfterCue = "GiveFreeDrop";
      if (TrialType == 1)
      {
        S.GaveFreeReward.flag_L_water = 0;
      }
      else
      {
        S.GaveFreeReward.flag_R_water = 0;
      }
    }
    else
    {
      ActionAfterCue = "ResponseCue";
    }

    // flashes bin condition
    StateTransition PreCueDelayPeriod_Cond[1] = {{"Tup", "SamplePeriod"}};
    StateTransition TrialStart_Cond[1] = {{"Tup", "PreCueDelayPeriod"}}; //
    StateTransition SamplePeriod_Cond[1]   = {{"Tup", "DelayPeriod"}};
    StateTransition DelayPeriod_Cond[3]    = {{"Lick1In", "EarlyLickDelay"}, {"Lick2In", "EarlyLickDelay"}, {"Tup", "TestPeriod"}};
    StateTransition ResponseCue_Cond[1] = {{"Tup", "AnswerPeriod"}}; 
    StateTransition GiveFreeDrop_Cond[1] = {{"Tup", "ResponseCue"}};
    StateTransition AnswerPeriod_Cond[3] = {{"Lick1In", LeftLickAction}, {"Lick2In", RightLickAction}, {"Tup", "NoResponse"}};
    StateTransition Reward_Cond[1] = {{"Tup", "RewardConsumption"}};
    StateTransition Tup_Exit_Cond[1] = {{"Tup", "exit"}};
    StateTransition NoResponse_Cond[3] = {{"Lick1In", "exit"}, {"Lick2In", "exit"}, {"Tup", "exit"}};
    StateTransition ErrorTrial_Cond[1] = {{"Tup", "TimeOut"}};
    StateTransition TestPeriod_Cond[1]     = {{"Tup" ,"CueDelayPeriod"}};
    StateTransition EarlyLickDelay_Cond[1] = {{"Tup", "DelayPeriod"}}; 
    StateTransition Tup_StopLicking_Cond[1] = {{"Tup", "StopLicking"}};
    StateTransition StopLicking_Cond[3] = {{"Lick1In", "StopLickingReturn"}, {"Lick2In", "StopLickingReturn"}, {"Tup", "exit"}}; //
	StateTransition CueDelayPeriod_Cond[3] = {{"Lick1In", "EarlyLickDelay"}, {"Lick2In", "EarlyLickDelay"} ,{"Tup", ActionAfterCue}}; //

    OutputAction SamplePeriod_Output[1]     	   = {SampleOutput};
    OutputAction PreCueDelayPeriod_Output[1] = {GreenLightOutputBackground}; // low intensity green light as a trial start signal
    OutputAction ResponseCue_Output[1] = {CueOutput}; 
    OutputAction Reward_Output[1] = {RewardOutput};
    OutputAction NoOutput[0] = {};
    OutputAction ErrorOutput[1] = {NoiseOutput}; 
    OutputAction TestPeriod_Output[1] = {TestOutput};
    

    gpSMART_State states[17] = {};
    // visual flash states and conditions
    states[0] = smart.CreateState("TrialStart", 5, 1, TrialStart_Cond, 0, NoOutput); // msec
    states[1] = smart.CreateState("SamplePeriod", S.SamplePeriod, 1, SamplePeriod_Cond, 1, SamplePeriod_Output);
    states[2] = smart.CreateState("DelayPeriod", float(S.DelayPeriod), 3, DelayPeriod_Cond, 0, NoOutput);
    states[3] = smart.CreateState("ResponseCue", 105, 1, ResponseCue_Cond, 1, ResponseCue_Output); // Not used
    states[4] = smart.CreateState("GiveFreeDrop", reward_dur, 1, GiveFreeDrop_Cond, 1, Reward_Output);
    states[5] = smart.CreateState("AnswerPeriod", S.AnswerPeriod, 3, AnswerPeriod_Cond, 0, NoOutput);
    states[6] = smart.CreateState("Reward", reward_dur, 1, Reward_Cond, 1, Reward_Output);
    states[7] = smart.CreateState("RewardConsumption", S.ConsumptionPeriod, 1, Tup_StopLicking_Cond, 0, NoOutput);
    states[8] = smart.CreateState("NoResponse", random(30, 60) * 60 * 1000, 3, NoResponse_Cond, 0, NoOutput); // 1-hr: 60*60*1000 msec
    // error trial state (not used state)
    states[9] = smart.CreateState("ErrorTrial", 500, 1, ErrorTrial_Cond, 1, ErrorOutput); 
    // pre-cue delay
    states[10] = smart.CreateState("PreCueDelayPeriod", S.PreCueDelayPeriod, 1, PreCueDelayPeriod_Cond, 1, PreCueDelayPeriod_Output);
    states[13] = smart.CreateState("TestPeriod" , S.SamplePeriod ,1 ,TestPeriod_Cond ,1 ,TestPeriod_Output);
    states[12]  = smart.CreateState("TimeOut",          S.TimeOut,            1, Tup_Exit_Cond,        0, NoOutput);
    states[11] = smart.CreateState("EarlyLickDelay",    S.EarlyLickPeriod,    1, EarlyLickDelay_Cond,  0, NoOutput);
    states[14] = smart.CreateState("StopLicking",       S.StopLickingPeriod,  3, StopLicking_Cond,     0, NoOutput);
    states[15] = smart.CreateState("StopLickingReturn", 10,                   1, Tup_StopLicking_Cond, 0, NoOutput);
	states[16] = smart.CreateState("CueDelayPeriod", float(S.CueDelayPeriod),   3, CueDelayPeriod_Cond, 0, NoOutput);
    // Predefine State sequence.
    for (int i = 0; i < 17; i++)
    {
      smart.AddBlankState(states[i].Name);
    }

    // Add a state to state machine.
    for (int i = 0; i < 17; i++)
    {
      smart.AddState(&states[i]);
    }

    // smart.PrintMatrix(); // for debug

    // Run the matrix
    smart.Run();
  }
  break;
  default:
    break;
  } // end for Switch(protocol)
}

void UpdateTrialOutcome()
{
  /* data will be stored in public variable 'trial_res', which includes:
    trial_res.nEvent:           number of event happened in last trial
    trial_res.eventTimeStamps[]: time stamps for each event
    trial_res.EventID[]:          event id for each event
    trial_res.nVisited:       number of states visited in last trial
    trial_res.stateVisited[]:   the states visited in last trail
  */

  TrialOutcome = 3; // 0 no-response; 1 correct; 2 error; 3 others
  // TODO
  for (int i = 0; i < trial_res.nVisited; i++)
  {
    if (trial_res.stateVisited[i] == 6 || trial_res.stateVisited[i] == 8 || trial_res.stateVisited[i] == 9)
    { // Reward || No Response || Error

      if (trial_res.stateVisited[i] == 6)
      {
        TrialOutcome = 1;
      }
      else if (trial_res.stateVisited[i] == 8)
      {
        TrialOutcome = 0;
      }
      else
      {
        TrialOutcome = 2;
      }
      break;
    }
  }

  if (TrialOutcome == 1)
  {
    S.totalRewardNum++;
    last_reward_time = millis(); // record the last reward time
    timed_reward_count = 0;
  }

  is_earlylick = 2;
  if (S.currProtocolIndex >= 3)
  {
    is_earlylick = 0;
    for (int i = 0; i < trial_res.nVisited; i++)
    {
      if (trial_res.stateVisited[i] == 11)
      { // earlylick delay
        is_earlylick = 1;
        break;
      }
    }
  }

  // do FIFO
  for (int i = 0; i < RECORD_TRIALS - 1; i++)
  {
    S.ProtocolIndexHistory[i] = S.ProtocolIndexHistory[i + 1];
    S.TrialTypeHistory[i] = S.TrialTypeHistory[i + 1]; //
    S.OutcomeHistory[i] = S.OutcomeHistory[i + 1];     //
    S.EarlyLickHistory[i] = S.EarlyLickHistory[i + 1]; //
  }
  // Keep record current trial info in the last position (RECORD_TRIALS-1) of the matrix
  S.ProtocolIndexHistory[RECORD_TRIALS - 1] = S.currProtocolIndex;
  S.OutcomeHistory[RECORD_TRIALS - 1] = TrialOutcome;
  S.TrialTypeHistory[RECORD_TRIALS - 1] = TrialType;
  S.EarlyLickHistory[RECORD_TRIALS - 1] = is_earlylick;

  byte Outcomes_sum = 0;
  for (int i = RECORD_TRIALS - recent_trials; i < RECORD_TRIALS; i++)
  {
    if (S.OutcomeHistory[i] == 1)
    {
      Outcomes_sum++;
    }
  }
  S.currProtocolPerf = round((float)Outcomes_sum / recent_trials * 100);
  S.currProtocolTrials++;

  Perf100 = 0;
  EarlyLick100 = 0;
  for (int i = 0; i < RECORD_TRIALS; i++)
  {
    if (S.OutcomeHistory[i] == 1)
    {
      Perf100++;
    }
    if (S.EarlyLickHistory[i] == 1)
    {
      EarlyLick100++;
    }
  }
}

void SendTrialInfo2PC()
{
  // UDP send trial info to PC
  String TrialInfo_str = "T";

  TrialInfo_str += String(S.currTrialNum);
  TrialInfo_str += ",";

  TrialInfo_str += String(TrialType);
  TrialInfo_str += ",";

  TrialInfo_str += String(S.currProtocolIndex);
  TrialInfo_str += ",";

  TrialInfo_str += String(TrialOutcome);
  TrialInfo_str += ",";

  TrialInfo_str += String(EarlyLick100);
  TrialInfo_str += ",";

  TrialInfo_str += String(Perf100);
  TrialInfo_str += ",";

  TrialInfo_str += String(S.currProtocolTrials);
  TrialInfo_str += ",";

  TrialInfo_str += String(S.currProtocolPerf);
  TrialInfo_str += ",";

  TrialInfo_str.toCharArray(outBuffer, 255);
  udpPrint(outBuffer);
}

void manualChangeProtocol()
{
}

void autoChangeProtocol()
{
  switch (S.currProtocolIndex)
  {
  case 0:
    if (S.currProtocolTrials > 1000 && Perf100 > 75)
    {                          
      S.currProtocolIndex = 1; // Advanced to Protocol 1: pattern left/right
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 10;
      S.CueDelayPeriod = 10;
      S.TimeOut = 2000;
      S.AnswerPeriod = 5000;
      S.ConsumptionPeriod = 750;
      //S.StopLickingPeriod   = 500;
      //S.EarlyLickPeriod     = 100;
      S.TrialPresentMode = 0; // 0"pattern",1"random",2"antiBias",3"fixed"

    }
    break;

  case 1:
    if (S.currProtocolTrials > 100 && Perf100 > 75)
    {
      S.currProtocolIndex = 2; // Advanced to Protocol 2: antiBias left/right
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 10;
      S.CueDelayPeriod = 10;
      S.TimeOut = 4000;
      S.AnswerPeriod = 3000;
      S.ConsumptionPeriod = 750;
      // S.StopLickingPeriod   = 500;
      // S.EarlyLickPeriod     = 100;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 2:
    if (S.currProtocolTrials > 100 && Perf100 > 75 )
    {
      S.currProtocolIndex = 3; // Advanced to Protocol 3: antiBias left/right w/ forced delay 0.5 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 200;
      S.CueDelayPeriod = 200;
      S.TimeOut = 4000;
      S.AnswerPeriod = 3000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 500;
      S.EarlyLickPeriod   = 100;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 3:
    if (S.currProtocolTrials > 100 && Perf100 > 75)
    {
      S.currProtocolIndex = 4; // Advanced to Protocol 4: antiBias left/right w/ forced delay 0.8 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.CueDelayPeriod = 400;
      S.DelayPeriod  = 400;
      S.TimeOut = 6000;
      S.AnswerPeriod = 3000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 800;
      S.EarlyLickPeriod   = 150;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 4:
    if (S.currProtocolTrials > 100 && Perf100 > 75)
    {
      S.currProtocolIndex = 5; // Advanced to Protocol 5: antiBias left/right w/ forced delay 1.0 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 500;
      S.CueDelayPeriod = 500;
      S.TimeOut = 6000;
      S.AnswerPeriod = 2000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 800;
      S.EarlyLickPeriod   = 200;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 5:
    if (S.currProtocolTrials > 100 && Perf100 > 75)
    {
      S.currProtocolIndex = 6; // Advanced to Protocol 6: antiBias left/right w/ forced delay 1.2 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 600;
      S.CueDelayPeriod = 600;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod   = 250;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 6:
    if (S.currProtocolTrials > 100 && Perf100 > 75)
    {
      S.currProtocolIndex = 7; // Advanced to Protocol 7: antibias left/right w/ forced delay 1.2 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 700;
      S.CueDelayPeriod = 700;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod   = 300;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 7: 
    if (S.currProtocolTrials > 100 && Perf100 > 75)
    {
      S.currProtocolIndex = 8; // advanced to Protocol reversal
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 800;
      S.CueDelayPeriod = 800;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod   = 300;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;
  case 8: 
    if (S.currProtocolTrials % 100 == 0 && Perf100 > 75)
    {
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;
      S.currProtocolIndex = 9;
      S.retention_counter = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 1000;
      S.CueDelayPeriod = 1000;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod   = 300;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;
  case 9: 
    if (S.currProtocolTrials % 100 == 0 && Perf100 > 75)
    {
      S.retention_counter++;
    }
    if (S.retention_counter == 10)
    {
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;
      S.retention_counter = 0;
      S.currProtocolIndex = 10; // testing protocol

      S.SamplePeriod = 500;
      S.CueDelayPeriod = 1000;
      S.DelayPeriod  = 1000;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod   = 300;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;
  default:
    break;
  }
}

void autoReward()
{
  S.GaveFreeReward.past_trials++;
  byte error_trials = 3; // consecutive 3 errors in a paticular trial type => free reward in next trial
  if (S.GaveFreeReward.past_trials >= error_trials)
  {
    byte n_RSideErrors = 0;
    byte n_LSideErrors = 0;
    for (int i = 0; i < error_trials; i++)
    {
      if (S.TrialTypeHistory[RECORD_TRIALS - error_trials + i] == 2 && S.OutcomeHistory[RECORD_TRIALS - error_trials + i] != 1)
      {
        n_RSideErrors++;
      }
      else if (S.TrialTypeHistory[RECORD_TRIALS - error_trials + i] == 1 && S.OutcomeHistory[RECORD_TRIALS - error_trials + i] != 1)
      {
        n_LSideErrors++;
      }
    }
    if (n_RSideErrors == error_trials)
    {
      S.GaveFreeReward.flag_R_water = 1;
      S.GaveFreeReward.flag_L_water = 0;
      S.GaveFreeReward.past_trials = 0;
    }
    else if (n_LSideErrors == error_trials)
    {
      S.GaveFreeReward.flag_R_water = 0;
      S.GaveFreeReward.flag_L_water = 1;
      S.GaveFreeReward.past_trials = 0;
    }
  }
}

void trialSelection()
{
  int switch_indicator = 0;
  switch (S.TrialPresentMode)
  {       // 0"pattern",1"random",2"antiBias",3"fixed"
  case 0: // Pattern: 3 left 3 right
    // switch trial types only when reaching MaxSame CORRECT trials
    for (int i = RECORD_TRIALS - 1; i >= 0; i--)
    {
      if (S.TrialTypeHistory[i] == S.TrialTypeHistory[RECORD_TRIALS - 1])
      {
        if (S.OutcomeHistory[i] == 1)
        {
          switch_indicator++;
        }
      }
      else
      {
        break;
      }
    }
    if (switch_indicator >= MaxSame)
    {
      if (S.TrialTypeHistory[RECORD_TRIALS - 1] == 1)
      {
        TrialType = 2;
      }
      else
      {
        TrialType = 1;
      }
    }
    else
    {
      TrialType = S.TrialTypeHistory[RECORD_TRIALS - 1];
    }
    break;

  case 1: // random
    TrialType = random(2) + 1;
    break;

  case 2:
  { // TrialPresentMode     'antiBias'
    float LeftTrialProb;
    if (S.currTrialNum > recent_trials)
    {
      byte correct_R_history = compare_array_sum(S.OutcomeHistory, 1, S.TrialTypeHistory, 2, RECORD_TRIALS - recent_trials, RECORD_TRIALS);
      byte correct_L_history = compare_array_sum(S.OutcomeHistory, 1, S.TrialTypeHistory, 1, RECORD_TRIALS - recent_trials, RECORD_TRIALS);
      byte incorrect_R_history = compare_array_sum(S.OutcomeHistory, 2, S.TrialTypeHistory, 2, RECORD_TRIALS - recent_trials, RECORD_TRIALS);
      byte incorrect_L_history = compare_array_sum(S.OutcomeHistory, 2, S.TrialTypeHistory, 1, RECORD_TRIALS - recent_trials, RECORD_TRIALS);
      if ((correct_R_history + correct_L_history) != 0)
      {
        float percent_R_corr = (float)correct_R_history / (float)(correct_R_history + correct_L_history);
        if ((incorrect_L_history + incorrect_R_history) != 0)
        {
          float percent_L_incorr = (float)incorrect_L_history / (float)(incorrect_L_history + incorrect_R_history);
          LeftTrialProb = (percent_R_corr + percent_L_incorr) * 0.5;
        }
        else
        {
          LeftTrialProb = 0.5;
        }
      }
      else
      {
        LeftTrialProb = 0.5;
      }
    }
    else
    {
      LeftTrialProb = 0.5;
    }

    //////////////////////////////////////
    for (int i = RECORD_TRIALS - 1; i >= RECORD_TRIALS - MaxSame; i--)
    {
      if (S.TrialTypeHistory[i] == S.TrialTypeHistory[RECORD_TRIALS - 1])
      {
        switch_indicator++;
      }
      else
      {
        break;
      }
    }
    if (switch_indicator == MaxSame)
    {
      if (S.TrialTypeHistory[RECORD_TRIALS - 1] == 1)
      {
        TrialType = 2;
      }
      else
      {
        TrialType = 1;
      }
    }
    else
    { // Haven't reached MaxSame limits yet, choose at random:
      if (random(100) <= (LeftTrialProb * 100))
      {
        TrialType = 1;
      }
      else
      {
        TrialType = 2;
      }
    }
    /////////////////////////////////////
    byte temp_left = 0;
    byte temp_left_correct = 0;
    // in the last '3' left trials,
    for (int i = RECORD_TRIALS - 1; i >= 0; i--)
    {
      if (S.TrialTypeHistory[i] == 1)
      { // left trial
        temp_left++;
        if (temp_left > 3)
        {
          break;
        }
        if (S.OutcomeHistory[i] == 1)
        { // correct
          temp_left_correct++;
        }
      }
    }
    if (temp_left_correct < MinCorrect)
    {
      TrialType = 1; // if there is less than 'Min_correct' correct trials, keep left
    }

    byte temp_right = 0;
    byte temp_right_correct = 0;
    // in the last '3' right trials,
    for (int i = RECORD_TRIALS - 1; i >= 0; i--)
    {
      if (S.TrialTypeHistory[i] == 2)
      { // right trial
        temp_right++;
        if (temp_right > 3)
        {
          break;
        }
        if (S.OutcomeHistory[i] == 1)
        { // correct
          temp_right_correct++;
        }
      }
    }
    if (temp_right_correct < MinCorrect)
    {
      TrialType = 2; // if there is no 'Min_correct' correct trials, then keep right, keep right
    }
    /////////////////////////////////////
  }
  break;

  case 3:
  { // TrialPresentMode     'fixed'
    TrialType = S.TrialTypeHistory[RECORD_TRIALS - 1];
  }
  break;
  default:
    break;
  }
}


void Curriculum(){
  // delay period training from 400 -> 1200 ; 5 difficulty levels
  float delayperiodLevel[7] = {0.5 ,1 ,2 ,3 ,4 ,5 ,8};
  float cuedelayperiodLevel[7] = {0.5 ,1 ,2 ,3 ,4 ,5 ,8}; // 其中1 -5 index 的值为选择的
  float StayorDecre[2] = {0};
  int StayorDecreFlag = 0;
  // 2: decrement step
    // 2.1 calc the FS EMA value
    float currReward;
    // including the EarlyLick trial
    // 200 - 1000 EMA 
    float corrected_FastEMA = 0;
    float corrected_SlowEMA = 0;
    float FSEMA = 0;
    float reweights = 0;

    if (TrialOutcome != 0 && TrialOutcome != 3){
      // get the trial outcome
      if (TrialOutcome == 1)
      {
        // correct trial
        currReward = 1;
      }
      else
      {
        currReward = 0;
      }

    CP.FastEMA = CP.FastEMA * CP.FastAlpha + (1.0 - CP.FastAlpha) * currReward; // Fast EMA 100 
    corrected_FastEMA = CP.FastEMA / (1.0 - pow(CP.FastAlpha, CP.t));
    
    CP.SlowEMA = CP.SlowEMA * CP.SlowAlpha + (1.0 - CP.SlowAlpha) * currReward; // Slow EMA N 其中Max N == 1000
    corrected_SlowEMA = CP.SlowEMA / (1.0 - pow(CP.SlowAlpha, CP.t));
    FSEMA = corrected_FastEMA - corrected_SlowEMA;

    // the first step : advance step
    if(CP.t >= 100){
      if(corrected_FastEMA > 0.8){
        if(CP.currDL < 5){
          CP.currDL++;
          CP.t = 1;
          CP.LP = 0;
          CP.FastEMA = 0;
          CP.SlowEMA = 0;
        }else{
          // S.currProtocolIndex = 11; 达到最后难度后的protocol
        }
      }
    }

    // ALP EMA
    CP.LP = CP.LP * CP.LPAlpha + (1.0 - CP.LPAlpha) * FSEMA; // Slow EMA N 其中Max N == 1000
    // reweight trials decay ; 对应 1- 10 的放大 ； 在该难度上训练的时间越长 ， stay 概率越低
    if(CP.t < 500){
      reweights = 1;
    }else if(CP.t < 5000){
      reweights = CP.t / 500;
    }else{
      reweights = 10;
    }
    // perf decay 对应 0 - 1 的缩小  ；perf 越高 ； stay 概率越大
    reweights = reweights * (1 - sigmoid_reweightCL(corrected_SlowEMA)); // 0.75
    FSEMA = CP.LP / (1.0 - pow(CP.LPAlpha, CP.t)) * reweights; 
    CP.t++;

    // 2.2 判断是否需要 回退
    if(CP.t >= 500 && CP.t % 500 == 0){
      StayorDecre[0] = FSEMA; // stay possiblity
      if(StayorDecre[0] < 0){ // 只有 perf 出现下降的时候才会进行判定是否要decrement
        StayorDecreFlag = normSoftMaxMin(2 ,StayorDecre ,CP.NormSoftMax_T ,0 ,0);
        if(StayorDecreFlag == 1){ 
          if(CP.currDL > 1){
            CP.currDL--;
            CP.t = 1;
            CP.LP = 0;
            CP.FastEMA = 0;
            CP.SlowEMA = 0;
          }
        }
      }
    }
    }
  // 3: output the delay period value
  S.DelayPeriod = delayperiodLevel[CP.currDL];
  S.CueDelayPeriod = cuedelayperiodLevel[CP.currDL];
  // add random 来模拟一种连续的过程
  S.DelayPeriod = exponent(S.DelayPeriod ,delayperiodLevel[CP.currDL - 1] ,delayperiodLevel[CP.currDL + 1]);
  S.CueDelayPeriod = exponent(S.CueDelayPeriod ,cuedelayperiodLevel[CP.currDL - 1] ,cuedelayperiodLevel[CP.currDL + 1]);
  write_Curriculum_model_info();

}

/**************************************************************************************************************/
/********************************************** SD related Functions *******************************************/
/**************************************************************************************************************/
int Send_SD_file_2PC(String fileID)
{
  char SD_msg[100];
  char SDfile[20];
  byte onset = 0;
  String("E: error opening " + fileID + " for SD read").toCharArray(SD_msg, 100);
  fileID.toCharArray(SDfile, 20);

  File dataFile = SD.open(SDfile, FILE_READ);
  if (dataFile)
  {
    if (fileID == "Trial.txt")
    {
      if (dataFile.seek(S.Trial_txt_position))
      {
        while (dataFile.available())
        {
          sprintf(outBuffer, "FA%dT#", cage_id); // T is the short of Trial
          for (int i = 0; i < 10; i++)
          {
            if (outBuffer[i] == '#')
            {
              onset = i + 1;
            }
          }
          dataFile.readBytesUntil('@', &outBuffer[onset], sizeof(outBuffer) - onset); // @ as a teminoral signal beyond fetch
          udpPrintSD(outBuffer);
          delay(100);
        }
      }
      // update Trial_txt_position
      S.Trial_txt_position = dataFile.position();
      write_SD_para_S();
    }
    else if (fileID == "Tevent.txt")
    {
      if (dataFile.seek(S.Tevent_txt_position))
      {
        while (dataFile.available())
        {
          sprintf(outBuffer, "FA%dE#", cage_id); // E is the short of Event
          for (int i = 0; i < 10; i++)
          {
            if (outBuffer[i] == '#')
            {
              onset = i + 1;
            }
          }
          dataFile.readBytesUntil('@', &outBuffer[onset], sizeof(outBuffer) - onset); // @ as a teminoral signal beyond fetch
          udpPrintSD(outBuffer);
          delay(100);
        }
      }
      // update Tevent_txt_position
      S.Tevent_txt_position = dataFile.position();
      write_SD_para_S();
    }
    else
    {
      while (dataFile.available())
      {
        dataFile.readBytesUntil('@', outBuffer, sizeof(outBuffer)); // @ as a teminoral signal beyond fetch
        udpPrintSD(outBuffer);
        delay(100);
      }
    }
    Serial.println("finish SD file " + fileID + " transmission");
    String("E: Success upload " + fileID + " file!").toCharArray(SD_msg, 100);
    udpPrint(SD_msg);
    udpPrintSD("FH");
  }
  else
  {
    Serial.println("E: error opening " + fileID + " for SD read");
    udpPrint(SD_msg);
    udpPrintSD("FH");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 0;
}

int Receive_SD_file_fromPC(String fileName, unsigned long fileLength)
{
  // receive the data from PC using Loop and UDP packets
  int err = 0;
  int read_bytes = 0;
  String SubSDString;

  char SD_msg[100];
  char SDfile[50];
  fileName.toCharArray(SDfile, 50);
  // check if there exist this file, if exist
  if (!(SD.exists(SDfile)))
  {
    // if this file not exists, create it
    File dataFile = SD.open(SDfile, FILE_WRITE);
    dataFile.close();
  }

  File dataFile = SD.open(SDfile, FILE_WRITE_BEGIN);
  if (dataFile)
  {
    // Serial.println("file in ");
    // Serial.println(fileLength);
    // Serial.println(fileName);
    while (true)
    {
      if (Udp.parsePacket())
      { // receiving file data
        read_bytes = Udp.read(sdBuffer, 512);
        if (read_bytes != 512)
        {
          // Serial.println("the end");
        }
        byte commandByte = sdBuffer[0];
        if (commandByte == 'K')
        {
          String sdString = sdBuffer;
          SubSDString = sdString.substring(1, read_bytes);
          dataFile.print(SubSDString);
        }
        else if (commandByte == 'E')
        { // end of transmisstion
          // check the length
          if (dataFile.size() < fileLength)
          {
            Serial.println("E:  the file downloaded is incomplete! " + fileName + " for SD write");
            String("E:  the file downloaded is incomplete! " + fileName + " for SD write").toCharArray(SD_msg, 100);
            udpPrint(SD_msg);
            err = 1;
          }
          else
          {
            Serial.println("E: Success download " + fileName + " file!");
            String("E: Success download " + fileName + " file!").toCharArray(SD_msg, 100);
            udpPrint(SD_msg);
          }
          // ack
          String("ES: received the end code!").toCharArray(SD_msg, 100);
          udpPrint(SD_msg);
          // Serial.println(dataFile.size());
          dataFile.close();
          break;
        }
      }
    }
  }
  else
  {
    Serial.println("E: error opening " + fileName + " for SD write");
    String("E: error opening " + fileName + " for SD write").toCharArray(SD_msg, 100);
    udpPrint(SD_msg);
    dataFile.close();
    err = 1;
  }

  return err;
}

int FlasherXTrigger(String HEX_FILE_NAME)
{
  // Begin firmware update using the FlasherX API
  // flash the LED to give a signal of firmware update
  char SDfile[50];
  HEX_FILE_NAME.toCharArray(SDfile, 50);
  for (int i = 0; i < 10; i++)
  {
    digitalWrite(ledPin, HIGH); // set the LED on
    delay(100);                 // delay
    digitalWrite(ledPin, LOW);  // set the LED off
    delay(100);
  }
  Serial.println("setupx");
  serial->printf("%s - %s %s\n", FLASHERX_VERSION, __DATE__, __TIME__);
  serial->printf("WARNING: this can ruin your device!\n");
  serial->printf("target = %s (%dK flash in %dK sectors)\n",
                 FLASH_ID, FLASH_SIZE / 1024, FLASH_SECTOR_SIZE / 1024);

  uint32_t buffer_addr, buffer_size;

  // create flash buffer to hold new firmware
  if (firmware_buffer_init(&buffer_addr, &buffer_size) == 0)
  {
    Serial.print("unable to create buffer\n");
    Serial.flush();
    for (;;)
    {
    }
  }

  serial->printf("created buffer = %1luK %s (%08lX - %08lX)\n",
                 buffer_size / 1024, IN_FLASH(buffer_addr) ? "FLASH" : "RAM",
                 buffer_addr, buffer_addr + buffer_size);

  // get user input to read from SD
  if (!SD.begin(chipSelect))
  {
    Serial.println("SD initialization failed");
    return 1;
  }
  File hexFile;
  Serial.println("SD initialization OK");
  hexFile = SD.open(SDfile, FILE_READ);
  if (!hexFile)
  {
    Serial.println("SD file open failed");
    return 1;
  }
  Serial.println("SD file open OK");
  // read hex file, write new firmware to flash, clean up, reboot
  update_firmware(&hexFile, serial, buffer_addr, buffer_size);

  // return from update_firmware() means error or user abort, so clean up and
  // reboot to ensure that static vars get boot-up initialized before retry
  Serial.print("erase FLASH buffer / free RAM buffer...\n");
  firmware_buffer_free(buffer_addr, buffer_size);
  serial->flush();
  REBOOT;
  return 0;
}

int write_SD_para_S()
{
  File dataFile = SD.open("paraS.txt", FILE_WRITE_BEGIN);
  if (dataFile)
  {
    // currTrialNum
    dataFile.print("currTrialNum = ");
    dataFile.println(S.currTrialNum);
    // currProtocolIndex
    dataFile.print("currProtocolIndex = ");
    dataFile.println(S.currProtocolIndex);
    // currProtocolTrials
    dataFile.print("currProtocolTrials = ");
    dataFile.println(S.currProtocolTrials);
    // currProtocolPerf
    dataFile.print("currProtocolPerf = ");
    dataFile.println(S.currProtocolPerf);
    // TrialPresentMode
    dataFile.print("TrialPresentMode = ");
    dataFile.println(S.TrialPresentMode);
    // ProtocolIndexHistory
    dataFile.print("ProtocolIndexHistory = ");
    for (int i = 0; i < RECORD_TRIALS; i++)
    {
      dataFile.print(S.ProtocolIndexHistory[i]);
      dataFile.print("; ");
    }
    dataFile.println();
    // TrialTypeHistory
    dataFile.print("TrialTypeHistory = ");
    for (int i = 0; i < RECORD_TRIALS; i++)
    {
      dataFile.print(S.TrialTypeHistory[i]);
      dataFile.print("; ");
    }
    dataFile.println();
    // OutcomeHistory
    dataFile.print("OutcomeHistory = ");
    for (int i = 0; i < RECORD_TRIALS; i++)
    {
      dataFile.print(S.OutcomeHistory[i]);
      dataFile.print("; ");
    }
    dataFile.println();

    // EarlyLickHistory
    dataFile.print("EarlyLickHistory = ");
    for (int i = 0; i < RECORD_TRIALS; i++)
    {
      dataFile.print(S.EarlyLickHistory[i]);
      dataFile.print("; ");
    }
    dataFile.println();

    // totalRewardNum
    dataFile.print("totalRewardNum = ");
    dataFile.println(S.totalRewardNum);
    // reward_left
    dataFile.print("reward_left = ");
    dataFile.println(S.reward_left);
    // reward_right
    dataFile.print("reward_right = ");
    dataFile.println(S.reward_right);
    // reward_middle
    dataFile.print("reward_middle = ");
    dataFile.println(S.reward_middle);
    // GaveFreeReward
    dataFile.print("GaveFreeReward.flag_L_water = ");
    dataFile.println(S.GaveFreeReward.flag_L_water);
    dataFile.print("GaveFreeReward.flag_R_water = ");
    dataFile.println(S.GaveFreeReward.flag_R_water);
    dataFile.print("GaveFreeReward.flag_M_water = ");
    dataFile.println(S.GaveFreeReward.flag_M_water);
    dataFile.print("GaveFreeReward.past_trials = ");
    dataFile.println(S.GaveFreeReward.past_trials);

    // SamplePeriod
    dataFile.print("SamplePeriod = ");
    dataFile.println(S.SamplePeriod);
    // DelayPeriod
    dataFile.print("DelayPeriod = ");
    dataFile.println(S.DelayPeriod);
    // TimeOut
    dataFile.print("TimeOut = ");
    dataFile.println(S.TimeOut);
    // AnswerPeriod
    dataFile.print("AnswerPeriod = ");
    dataFile.println(S.AnswerPeriod);
    // ConsumptionPeriod
    dataFile.print("ConsumptionPeriod = ");
    dataFile.println(S.ConsumptionPeriod);
    // StopLickingPeriod
    dataFile.print("StopLickingPeriod = ");
    dataFile.println(S.StopLickingPeriod);
    // EarlyLickPeriod
    dataFile.print("EarlyLickPeriod = ");
    dataFile.println(S.EarlyLickPeriod);
    // retention_counter
    dataFile.print("retention_counter = ");
    dataFile.println(S.retention_counter);
    // low_light_intensity
    dataFile.print("low_light_intensity = ");
    dataFile.println(S.low_light_intensity);
    // high_light_intensity
    dataFile.print("high_light_intensity = ");
    dataFile.println(S.high_light_intensity);
    // extra_TimeOut
    dataFile.print("extra_TimeOut = ");
    dataFile.println(S.extra_TimeOut);
    // Trial_txt_position
    dataFile.print("Trial_txt_position = ");
    dataFile.println(S.Trial_txt_position);
    // Tevent_txt_position
    dataFile.print("Tevent_txt_position = ");
    dataFile.println(S.Tevent_txt_position);
    // PreCueDelayPeriod
    dataFile.print("PreCueDelayPeriod = ");
    dataFile.println(S.PreCueDelayPeriod);
	// CueDelayPeriod
    dataFile.print("CueDelayPeriod = ");
    dataFile.println(S.CueDelayPeriod);
  }
  else
  {
    Serial.println("E: error opening paraS.txt for write");
    udpPrint("E: error opening paraS.txt for write");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 0;
}

int read_SD_para_S()
{
  File dataFile = SD.open("paraS.txt", FILE_READ);
  if (dataFile)
  {
    // currTrialNum
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.currTrialNum = string_tmp.toInt();
    // currProtocolIndex
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.currProtocolIndex = string_tmp.toInt();
    // currProtocolTrials
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.currProtocolTrials = string_tmp.toInt();
    // currProtocolPerf
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.currProtocolPerf = string_tmp.toInt();
    // TrialPresentMode
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.TrialPresentMode = string_tmp.toInt();
    // ProtocolIndexHistory
    string_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < RECORD_TRIALS; i++)
    {
      string_tmp = dataFile.readStringUntil(';');
      S.ProtocolIndexHistory[i] = string_tmp.toInt();
    }
    string_tmp = dataFile.readStringUntil('\n');
    // TrialTypeHistory
    string_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < RECORD_TRIALS; i++)
    {
      string_tmp = dataFile.readStringUntil(';');
      S.TrialTypeHistory[i] = string_tmp.toInt();
    }
    string_tmp = dataFile.readStringUntil('\n');
    // OutcomeHistory
    string_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < RECORD_TRIALS; i++)
    {
      string_tmp = dataFile.readStringUntil(';');
      S.OutcomeHistory[i] = string_tmp.toInt();
    }
    string_tmp = dataFile.readStringUntil('\n');
    // EarlyLickHistory
    string_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < RECORD_TRIALS; i++)
    {
      string_tmp = dataFile.readStringUntil(';');
      S.EarlyLickHistory[i] = string_tmp.toInt();
    }
    string_tmp = dataFile.readStringUntil('\n');

    // totalRewardNum
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.totalRewardNum = string_tmp.toInt();
    // reward_left
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.reward_left = string_tmp.toInt();
    // reward_right
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.reward_right = string_tmp.toInt();
    // reward_middle
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.reward_middle = string_tmp.toInt();

    // GaveFreeReward
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.GaveFreeReward.flag_L_water = string_tmp.toInt();
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.GaveFreeReward.flag_R_water = string_tmp.toInt();
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.GaveFreeReward.flag_M_water = string_tmp.toInt();
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.GaveFreeReward.past_trials = string_tmp.toInt();

    // SamplePeriod
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.SamplePeriod = string_tmp.toInt();
    // DelayPeriod
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.DelayPeriod = string_tmp.toInt();
    // TimeOut
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.TimeOut = string_tmp.toInt();
    // AnswerPeriod
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.AnswerPeriod = string_tmp.toInt();
    // ConsumptionPeriod
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.ConsumptionPeriod = string_tmp.toInt();
    // StopLickingPeriod
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.StopLickingPeriod = string_tmp.toInt();
    // EarlyLickPeriod
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.EarlyLickPeriod = string_tmp.toInt();
    // retention_counter
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.retention_counter = string_tmp.toInt();
    // low_light_intensity
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.low_light_intensity = string_tmp.toInt();
    // high_light_intensity
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.high_light_intensity = string_tmp.toInt();
    // extra_TimeOut
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.extra_TimeOut = string_tmp.toInt();
    // Trial_txt_position
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.Trial_txt_position = string_tmp.toInt();
    // Tevent_txt_position
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.Tevent_txt_position = string_tmp.toInt();
    // PreCueDelayPeriod
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.PreCueDelayPeriod = string_tmp.toInt();
	// CueDelayPeriod
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.CueDelayPeriod = string_tmp.toInt();
  }
  else
  {
    Serial.println("E: Error opening paraS.txt for read");
    udpPrint("E: Error opening paraS.txt for read");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 0;
}

// Write Fixation Events to SD card
int write_SD_event()
{
  File dataFile = SD.open("event.txt", FILE_WRITE);
  if (dataFile)
  {
    for (int i = 0; i < Ev.events_num; i++)
    {
      dataFile.print(Ev.events_time[i]);
      dataFile.print(" ");
      dataFile.print(Ev.events_id[i]);
      dataFile.print(" ");
      dataFile.println(Ev.events_value[i]);
    }
  }
  else
  {
    Serial.println("Error opening event.txt");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 0;
}

int read_SD_cage_info()
{
  // read file to identify the cage number
  File dataFile = SD.open("cage_info.txt", FILE_READ);
  if (dataFile)
  {
    dataFile.seek(0);
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n'); // 1st line: cage_id = xx
    cage_id = string_tmp.toInt();
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n'); // 2nd line: calibration_factor = xx
    calibration_factor = string_tmp.toInt();
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n'); // 3rd line: weight_offset  = xx
    weight_offset = string_tmp.toInt();
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n'); // 4th line: task = xx
    string_tmp.toCharArray(task_name, 40);
  }
  else
  {
    Serial.println("Can not open file: 'cage_info.txt'.");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 1;
}

int write_SD_cage_info()
{
  File dataFile = SD.open("cage_info.txt", FILE_WRITE_BEGIN);
  if (dataFile)
  {
    dataFile.print("cage_id = ");
    dataFile.println(cage_id);
    dataFile.print("calibration_factor = ");
    dataFile.println(calibration_factor);
    dataFile.print("weight_offset = ");
    dataFile.println(weight_offset);
  }
  else
  {
    Serial.println("Can not open file: 'cage_info.txt'.");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 1;
}

// Write Trial info to SD card // todo.. add sample delay is_earlylick etc.
int write_SD_trial_info()
{
  File dataFile = SD.open("Trial.txt", FILE_WRITE);
  if (dataFile)
  {
    // time
    dataFile.print(Teensy3Clock.get());
    dataFile.print(" ");

    // trial#
    dataFile.print(S.currTrialNum);
    dataFile.print(" ");

    dataFile.print(S.currProtocolIndex);
    dataFile.print(" ");

    dataFile.print(TrialType); 
    dataFile.print(" ");

    dataFile.print(TrialOutcome);
    dataFile.print(" ");

    dataFile.print(S.SamplePeriod);
    dataFile.print(" ");

    dataFile.print(S.DelayPeriod);
    dataFile.print(" ");

    dataFile.print(is_earlylick);
    dataFile.print(" ");

    dataFile.print(modality); 
    dataFile.print(" ");

    dataFile.print(S.CueDelayPeriod); 
    dataFile.print(" ");

    // state_visited
    dataFile.print(trial_res.nVisited);

    for (int i = 0; i < trial_res.nVisited; i++)
    {
      dataFile.print(" ");
      dataFile.print(trial_res.stateVisited[i]);
      dataFile.print(" ");
      dataFile.print(trial_res.stateTimeStamps[i]);
    }
    dataFile.println();
  }
  else
  {
    Serial.println("E: error opening Trial.txt'.");
    udpPrint("E: error opening Trial.txt");
    dataFile.close();
    return -1;
  }
  dataFile.close();

  // Tevent.txt
  dataFile = SD.open("Tevent.txt", FILE_WRITE);
  if (dataFile)
  {
    dataFile.print(S.currTrialNum);
    dataFile.print(" ");

    dataFile.print(trial_res.nEvent);
    for (int i = 0; i < trial_res.nEvent; i++)
    {
      dataFile.print(" ");
      dataFile.print(trial_res.EventID[i]);
      dataFile.print(" ");
      dataFile.print(trial_res.eventTimeStamps[i]);
    }
    dataFile.println();
  }
  else
  {
    Serial.println("E: error opening Tevent.txt");
    udpPrint("E: error opening Tevent.txt");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 0;
}

int sum_array(byte a[], int array_length)
{
  int res = 0;
  for (int i = 0; i < array_length; i++)
  {
    res = res + a[i];
  }
  return res;
}
int compare_array_sum(byte array1[], byte oprant1, byte array2[], byte oprant2, int start_ind, int end_ind)
{
  int num = 0;
  for (int i = start_ind; i < end_ind; i++)
  {
    if (array1[i] == oprant1 && array2[i] == oprant2)
    {
      num++;
    }
  }
  return num;
}
int compare_array_sum(byte array1[], byte oprant1, int start_ind, int end_ind)
{
  int num = 0;
  for (int i = start_ind; i < end_ind; i++)
  {
    if (array1[i] == oprant1)
    {
      num++;
    }
  }
  return num;
}

void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  // print your WiFi shield's IP address
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  // print the received signal strength
  Serial.print("Signal strength (RSSI):");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

void udpPrint(const char msg[])
{
  Udp.beginPacket(serverIP, serverPort);
  Udp.write(msg);
  Udp.endPacket();
}

void udpPrintSD(const char msg[])
{
  Udp.beginPacket(serverIP, serverPort + 1);
  Udp.write(msg);
  Udp.endPacket();
}

void free_reward(int rew_duration_ms)
{
  smart.ManualOverride("DO1", 1); // override valve
  smart.ManualOverride("DO2", 1); // override valve
  smart.ManualOverride("DO3", 1); // override valve
  delay(rew_duration_ms);
  smart.ManualOverride("DO1", 0);
  smart.ManualOverride("DO2", 0);
  smart.ManualOverride("DO3", 0);
}

/************************************************Curriculum training************************************************************************/
int normSoftMaxMin(int length , float *data ,float Gammar ,int mode ,int norm)
{
  float mode_flag;
  if(mode == 0){ // softMax
    mode_flag = 1.0;
  }else{ // softmin
    mode_flag = -1.0;
  }

  float minQ = 1000;
  float maxQ = 0;
  float temp_Q = 0;

  float sum = 0;
  int choicerandom = 0;
  float TrialProb[10] = {0};  // max length is 10
  int choicelist[10] = {0};
  
  int choice = 0;

  if(norm){
  }
  else{
    maxQ = 1;
    minQ = 0;
  }

	for (int j = 0; j < length; j++)
	{
    
	  TrialProb[j] = exp(mode_flag * (data[j] - minQ)/ (maxQ - minQ) / Gammar);
	  sum = sum + TrialProb[j];
	}
	
	for (int j = 0; j < length; j++)
	{
	  TrialProb[j] = TrialProb[j] / sum;
	}
	
	for (int loop = 0; loop < length; loop++)
	{
	  choicelist[loop] = 0;
	  for (int gg = 0; gg <= loop; gg++)
	  {
		choicelist[loop] += int(TrialProb[gg] * 1000);
	  }
	}
	choicerandom = int(random(0, choicelist[length-1]));
	for (int h = 0; h < length; h++)
	{
	  if (choicerandom < choicelist[h])
	  {
		choice = h;
		break;
	  }
	}
	return choice;
}


int read_Curriculum_Param_info()
{
  // read file to identify the cage number
  File dataFile = SD.open("curriculumparam.txt", FILE_READ);
  if (dataFile)
  {
    dataFile.seek(0);
    // FastEMA
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.FastEMA = string_tmp.toFloat();

    // SlowEMA
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.SlowEMA = string_tmp.toFloat();

    // LP 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.LP = string_tmp.toFloat();

    // currDL
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.currDL  = string_tmp.toInt();

    // FastAlpha 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.FastAlpha = string_tmp.toFloat();

    // SlowAlpha 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.SlowAlpha = string_tmp.toFloat();

    // LPAlpha 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.LPAlpha = string_tmp.toFloat();
    
    // NormSoftMax_T 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.NormSoftMax_T = string_tmp.toFloat();

    // t
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.t = string_tmp.toInt();
    
  }
  else
  {
    Serial.println("Can not open file: 'curriculumparam.txt'.");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 1;
}

int write_Curriculum_Param_info()
{
  File dataFile = SD.open("curriculumparam.txt", FILE_WRITE_BEGIN);
  if (dataFile)
  {
    // FastEMA
    dataFile.print("FastEMA = ");
    dataFile.print(CP.FastEMA ,6);
    dataFile.println();

    // SlowEMA
    dataFile.print("SlowEMA = ");
    dataFile.print(CP.SlowEMA ,6);
    dataFile.println();

    // LP
    dataFile.print("LP = ");
    dataFile.print(CP.LP ,6);
    dataFile.println();

    // currDL 
    dataFile.print("currDL = ");
    dataFile.println(CP.currDL);

    // FastAlpha 
    dataFile.print("FastAlpha = ");
    dataFile.println(CP.FastAlpha ,5);

    // SlowAlpha 
    dataFile.print("SlowAlpha = ");
    dataFile.print(CP.SlowAlpha ,6);
    dataFile.println();

    // LPAlpha 
    dataFile.print("LPAlpha = ");
    dataFile.println(CP.LPAlpha ,5);

    // NormSoftMax_T 
    dataFile.print("NormSoftMax_T = ");
    dataFile.println(CP.NormSoftMax_T ,5);

    // t
    dataFile.print("t = ");
    dataFile.print(CP.t);
    dataFile.println();
  }
  else
  {
    Serial.println("Can not open file: 'curriculumparam.txt'.");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 1;
}

// write_SD_model_info ;
int write_Curriculum_model_info()
{
  File dataFile = SD.open("curriculummodel.txt", FILE_WRITE);
  if (dataFile)
  {
    // trial#
    dataFile.print(S.currTrialNum);
    // iterations
    dataFile.print(" ");
    dataFile.print(CP.t);

    // 
    dataFile.print(" ");
    dataFile.print(CP.currDL);
    
    // FastEMA
    dataFile.print(" ");
    dataFile.print(CP.FastEMA ,6);
    
    // SlowEMA
    dataFile.print(" ");
    dataFile.print(CP.SlowEMA ,6);

    // LP
    dataFile.print(" ");
    dataFile.print(CP.LP ,6);
    dataFile.println();
  }
  else
  {
    Serial.println("E: error opening curriculummodel.txt'.");
    udpPrint("E: error opening curriculummodel.txt");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 1;
}

float sigmoid_reweightCL(float data){
  float reweightValue = 0.0;
  // remove the perf less than 0.5 in d2afc task 注意这个对于有reversal learning 的时候是不能使用的；对于reversal learning 需要将这个算法做一个翻转
  if(data < 0.5){ // range from 0.5 to 1
    reweightValue = 1.0 - data;
  }else{
    reweightValue = data;
  }
  // norm to 0-1
  reweightValue = float((reweightValue - 0.5) / 0.5);
  // sigmoid reweight
  reweightValue = 1.0/(1.0 + exp(-(reweightValue*15.0 - 4.0))); 
  // discterce
  reweightValue = int(reweightValue / 0.1) * 0.1; // 0.9 ~ 0
  return reweightValue;
}

int exponent(float beta, float min, float max)
{
  float u;
  float x;
  int period;
  for (;;)
  {
    u = float(float(random(0, 1000)) / float(1000));
    x = -beta * log(u);
    if (min < x && x < max)
    {
      period = int(x * 1000);
      return period;
    }
  }
}