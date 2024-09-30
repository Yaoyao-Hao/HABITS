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
  int SamplePeriod = 1000;
  int DelayPeriod = 500;
  int PreCueDelayPeriod = 500;
  int TimeOut = 1000;
  int AnswerPeriod = 10000;
  int ConsumptionPeriod = 750;
  int StopLickingPeriod = 1000;
  int EarlyLickPeriod = 100;
  int extra_TimeOut = 0;
  int bright_flag = 2; // 0 is dimmer ; 1 is brighter ;2 is no def(normal) 
} Parameters_behavior;
Parameters_behavior S;

/********** Define OutputAction**********/
OutputAction LeftWaterOutput = {"DO1", 1};
OutputAction RightWaterOutput = {"DO2", 1};
OutputAction LowSoundOutput = {"tPWM2", 1}; // tPWM2 top sound

// contrast aud cue
OutputAction HighSoundOutput         = {"tPWM2", 2}; // bigest contrast
OutputAction LowContrHighSoundOutput = {"tPWM2", 4}; // lowest contrast
OutputAction MidLowContrHighSoundOutput = {"tPWM2", 5}; // midlow
OutputAction MidHighContrHighSoundOutput = {"tPWM2", 6}; // midhigh
OutputAction AudContrastOutput[4] = {LowContrHighSoundOutput , MidLowContrHighSoundOutput ,MidHighContrHighSoundOutput ,HighSoundOutput};

OutputAction CueOutput = {"tPWM2", 3};
OutputAction NoiseOutput = {"Flag1", 1};
OutputAction LeftSoundOutput = {"tPWM1", 2};  // tPWM1 left sound 
OutputAction RightSoundOutput = {"tPWM3", 2}; // tPWM3 right sound 
// Note: the freq setting is working for all regular PWM below
OutputAction LeftLightOutput = {"PWM1", S.low_light_intensity};  // left light, value 0-255
OutputAction RightLightOutput = {"PWM5", S.low_light_intensity}; // right light ,same as above

OutputAction BlueLightOutput = {"PWM4", S.high_light_intensity};  // Blue light...
OutputAction RedLightOutput = {"PWM2", S.high_light_intensity};   // red light...
OutputAction GreenLightOutput = {"PWM3", S.high_light_intensity}; // Green light...

OutputAction BlueLightOutputBackground = {"PWM4", S.low_light_intensity};  // Blue light Background...
OutputAction RedLightOutputBackground = {"PWM2", S.low_light_intensity};   // red light Background...
OutputAction GreenLightOutputBackground = {"PWM3", S.low_light_intensity}; // Green light Background...
// light intensity
OutputAction LeftLightOutputH = {"PWM1", S.high_light_intensity};  // High left light intensity
OutputAction RightLightOutputH = {"PWM5", S.high_light_intensity}; // High right light intensity

// sound frequency ;sound orients ;light orients ;reversal trials ;wavelength ;light intensity;

byte TrialType = 1;         // 0 undef; 1 left; 2 right; 3 middle;
byte TrialTypeContrast = 4; // 4-20 event count; Using this value to calculate the event rate which is the criterion of trial outcome
byte TrialOutcome = 3;      // 0 no-response; 1 correct; 2 error; 3 others
byte is_earlylick = 2;      // 0-no earlylick; 1-earlylick; 2-undef
byte MaxSame = 3;
byte MinCorrect = 1; // minimum correct trials in the last 5 trials
byte Perf100 = 0;
byte EarlyLick100 = 0;
unsigned long last_reward_time = 0;
int timed_reward_count = 0;

int PoissonEvent[40] = {};
int modality = 0;
int NoOutputBinNum = 0;
int sample_bin = 1;                                                                                                                             // the basic duration of a single bin
int PulseTime = 20;                                                                                                                             // the duration of one pulse is 10ms
int IntervalPulse = 20;   
int PoissonRate = 0; // 4-20
int SampleBinNum = 40; // the duration of a bin is 40ms
int PoissonTotal = 25;
int P_value = 0; // P = lambda / N 


int CatchTrialProb = 10; // use a small number of trials as catch trials

int AudContrast = 0; // 1,2,3 ,4 -> 16 ,32 ,64 ,128 duty

byte pause_signal_PC = 0;
byte tare_flag = 1;
byte tare_length = 0;
bool paused = 1;
byte ledState = LOW;

/*******************Curriculum Training**************************/
typedef struct CurriculumParams
{
  int EvaPerf[5] = {0}; // 评估阶段 中得到的performance
  int LastEvaPerf[5] = {0};
  int ALP[5] = {0};
  int TargetTaskCounter[5] = {0};

  int CurrTrialModality = 0; // 当前trial 所选择的protocol indice 0 ,1 ,2 ;0 -> visual ;1 -> auditory ;2 -> multiModal
  int CurrTrialContrast = 0; // 0 -> (4 ,20) ; 1 -> (8 ,16)
  int CurrTrialLightIntensity = 0;// 0 -> 10(light intensity) ; 1 -> 50(light intensity) ; 2 -> undef
  
  int EvaNum_per_task = 100;
  int subtaskNum = 5;
  int currEvaTaskIndex = 0;  
  int currEvaNumIndex = 0;

  int updateALP_num = 2000;   // 1 : update the ALP distribution trial by trial
  int currHistory_step = 2000;   // recording the curr steps of a updateALP loop ;
  int t = 1;                  // update step
  bool evaFlag = false;
} CurriculumParams;
CurriculumParams CP;

// sub task (modality ,contrast ,lightIntensity) ( contrast is fixed to 4 vs 20)
const int trialParamsSpace[5][3] = {{0 ,0 ,0} ,{0 ,0 ,1} ,{1 ,0 ,2} ,{2 ,0 ,0} ,{2 ,0 ,1}};
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
  smart.setTruePWMFrequency(2, 2, 10000, 128); // (high sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(2, 3, 6500, 128);  // (go cue) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  //TODO
  smart.setTruePWMFrequency(2, 4, 10000, 4); // (high sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(2, 5, 10000, 8); // (high sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(2, 6, 10000, 16); // (high sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
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
  /* sample output 4 type:
   *    Sample_Output[1]     	   = {SampleOutput};    SampleOutput = {LowSoundOutput ,HighSoundOutput};
   *    Sample_Output[1]     	   = {SampleOutput};    SampleOutput = {LeftSoundOutput ,RightSoundOutput};
   *    Sample_Output[1]     	   = {SampleOutput};    SampleOutput = {LeftLightOutput ,RightLightOutput};
   *    Sample_Output[1]     	   = {SampleOutput};    SampleOutput = {BlueLightOutput ,RedLightOutput(as weak stimulus) ,GreenLightOutput};
   *    Sample_Output[2]     	   = {LeftLightOutputH ,RightLightOutputH};    Sample_Output[2]     	   = {LeftLightOutput ,RightLightOutput};
   */
  int LT_list[3] = {S.low_light_intensity ,S.low_light_intensity ,S.low_light_intensity};
  // auto change the light intensity PWM2 ,4 is light panel ports ;PWM1 ,5 is the left and right led ,respectively.
  if(CP.CurrTrialLightIntensity == 0){
    LeftLightOutput = {"PWM1", S.low_light_intensity};  // left light, value 0-255,TODO just 1 ,the light intensity is unaffordable(anlogWrite()command)
    RightLightOutput = {"PWM5", S.low_light_intensity}; // right light ,same as above
  }else if(CP.CurrTrialLightIntensity == 1){
    LeftLightOutput = {"PWM1", S.high_light_intensity};  // left light, value 0-255,TODO just 1 ,the light intensity is unaffordable(anlogWrite()command)
    RightLightOutput = {"PWM5", S.high_light_intensity}; // right light ,same as above
  }else{ // undef
    LeftLightOutput = {"PWM1", S.low_light_intensity};  // left light, value 0-255,TODO just 1 ,the light intensity is unaffordable(anlogWrite()command)
    RightLightOutput = {"PWM5", S.low_light_intensity}; // right light ,same as above
  }

  // auto change the modality
  modality = CP.CurrTrialModality;

  // OutputAction LeftLightOutputBackground = {"Flag2", 1}; // left LED noise light
  // OutputAction RightLightOutputBackground = {"Flag3", 1};
  OutputAction LeftLightOutputBackground = {"Flag3", 0};
  OutputAction RightLightOutputBackground = {"Flag3", 0};

  OutputAction BlueLightOutput = {"PWM4", S.high_light_intensity};           // Blue light...
  OutputAction RedLightOutput = {"PWM2", S.high_light_intensity};            // red light...
  OutputAction GreenLightOutput = {"PWM3", S.high_light_intensity};          // Green light...
  OutputAction BlueLightOutputBackground = {"PWM4", S.low_light_intensity};  // Blue light Background...
  OutputAction RedLightOutputBackground = {"PWM2", S.low_light_intensity};   // red light Background...
  OutputAction GreenLightOutputBackground = {"PWM3", S.low_light_intensity}; // Green light Background...

  OutputAction DelayNoOutput = GreenLightOutputBackground; // no output including pre delay and delay period TODO
  // light intensity
  OutputAction LeftLightOutputH = {"PWM1", S.high_light_intensity};  // High left light intensity
  OutputAction RightLightOutputH = {"PWM5", S.high_light_intensity}; // High right light intensity

  // randomSeed(analogRead(39)); // anlog input as random seeds

  // catch trial (alter the samplePeriod)
  NoOutputBinNum = 0;                                                     
  P_value = 0; // P = lambda / N 
                                                                                                                       // the interval of pulses is 15ms
  int BinaryFlash[40] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}; // 40 events maximum per 1000ms
  int BinaryClick[40] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
  // defaultly set NoOutput in the Sample period
  // random modality
  // int modality = int(random(3));
  // poisson click parameter

  /*********************CatchTrial************************/
  int SamplePeriodRandomValue = 0;
  if (S.currProtocolIndex == 8) // test protocol
  {
    // catch trials of sample period
    if (int(random(100)) < CatchTrialProb)
    {
      // 5% trials will have different SamplePeriod
      SamplePeriodRandomValue = int(random(4));
      switch (SamplePeriodRandomValue)
      {
      case 0:
        S.SamplePeriod = 400;
        break;
      case 1:
        S.SamplePeriod = 800;
        break;
      case 2:
        S.SamplePeriod = 1200;
        break;  
      case 3:
        S.SamplePeriod = 1600;
        break;
      default:
        break;
      }
    }
    else
    {
      S.SamplePeriod = 1000;
    }
    // catch trials of light intensity
    if (int(random(100)) < CatchTrialProb)
    {
      // 5% trials will have different SamplePeriod
      S.bright_flag = int(random(2));

      if(S.bright_flag == 0){
        LeftLightOutput = {"PWM1", int(LT_list[CP.CurrTrialLightIntensity] / 2)};  // left light, value 0-255,TODO just 1 ,the light intensity is unaffordable(anlogWrite()command)
        RightLightOutput = {"PWM5", int(LT_list[CP.CurrTrialLightIntensity] / 2)}; // right light ,same as above
      }else if(S.bright_flag == 1){
        LeftLightOutput = {"PWM1", int(LT_list[CP.CurrTrialLightIntensity] * 2)};  // left light, value 0-255,TODO just 1 ,the light intensity is unaffordable(anlogWrite()command)
        RightLightOutput = {"PWM5", int(LT_list[CP.CurrTrialLightIntensity] * 2)}; // right light ,same as above
      }else{ // undef
        LeftLightOutput = {"PWM1", LT_list[CP.CurrTrialLightIntensity]};  // left light, value 0-255,TODO just 1 ,the light intensity is unaffordable(anlogWrite()command)
        RightLightOutput = {"PWM5", LT_list[CP.CurrTrialLightIntensity]}; // right light ,same as above
      }

    }
    else
    {
      S.bright_flag = 2; // normal
    }
  }

  NoOutputBinNum = SampleBinNum - int(S.SamplePeriod / 40);
  /*******************************************************/
  // test
  Serial.print(CP.CurrTrialContrast); 
Serial.print(CP.CurrTrialLightIntensity); 
Serial.println(CP.CurrTrialModality); 


  smart.EmptyMatrix(); // clear matrix at the begining of each trial

  switch (S.currProtocolIndex)
  {
  case 0:
  {                             // PROTOCOL 0: teaching animal to lick both side
    OutputAction SampleOutputL; // low intensity
    OutputAction SampleOutputR;
    OutputAction AudSampleOutput;
    OutputAction SampleOutputBackgroundL;
    OutputAction SampleOutputBackgroundR;
    OutputAction AudSampleOutputBackground;
    OutputAction RewardOutput;

    SampleOutputL = LeftLightOutput;
    SampleOutputR = RightLightOutput;
    AudSampleOutput = HighSoundOutput;       // 10KHz freq clickes
    AudSampleOutputBackground = {"Flag3", 0}; // no output 
    SampleOutputBackgroundL = LeftLightOutputBackground;
    SampleOutputBackgroundR = RightLightOutputBackground;

    S.DelayPeriod = float(200); 
    S.PreCueDelayPeriod = float(random(100, 200));

    String LeftLickAction;
    String RightLickAction;
    String ActionAfterDelay;
    float reward_dur = S.reward_left;

    switch (TrialType)
    {
    case 1: // left low event rate
    {
      if(CP.CurrTrialContrast == 0){
        PoissonRate = 4;
      }else if(CP.CurrTrialContrast == 1){
        PoissonRate = 8;
      }

      LeftLickAction = "Reward";
      RightLickAction = "AnswerPeriod"; // no error trial
      RewardOutput = LeftWaterOutput;
      reward_dur = S.reward_left;
    }
      break;
    case 2: // right high event rate
    {
      if(CP.CurrTrialContrast == 0){
        PoissonRate = 20;
      }else if(CP.CurrTrialContrast == 1){
        PoissonRate = 16;
      }

      LeftLickAction = "AnswerPeriod"; // no error trial
      RightLickAction = "Reward";
      RewardOutput = RightWaterOutput;
      reward_dur = S.reward_right;
    }
      break;
    }
  
  int EventRate = 0; // using the real EventRate that presented to mice to judge the correct choice.
  while(EventRate != PoissonRate) // set the event rate to a constant
  {
    // sample in a binomial distribution to get the Contrast defined before
    TrialTypeContrast = 0;
    P_value = int(1000 * PoissonRate / PoissonTotal);
    for (int i = 0; i < 40; i ++)
    {
      if(i < NoOutputBinNum)
      {
        BinaryClick[i] = 0;
        BinaryFlash[i] = 0;
        PoissonEvent[i] = 2;
      }
      else if(i == NoOutputBinNum) // make sure the first event presents at the onset of sample period
      {
        BinaryClick[i] = 1;
        BinaryFlash[i] = 1;
        PoissonEvent[i] = 1;
        TrialTypeContrast++;
      }
      else
      {
        if (int(random(1000)) < P_value) // using 1000 as the cardinal number to increase resolution of P_value
        {
          BinaryClick[i] = 1;
          BinaryFlash[i] = 1;
          PoissonEvent[i] = 1;
          TrialTypeContrast++;
        }
        else
        {
          BinaryClick[i] = 0;
          BinaryFlash[i] = 0;
          PoissonEvent[i] = 0;
        } 
      }
    }
    // determine the real trialtype and trialContrast
    EventRate = int(round(float(TrialTypeContrast) / float(S.SamplePeriod) * 1000));
  }

    // modality choice
    // protocolIndex (0 ,1 ,2 ,3, 4 ,5 ,6 ,7) early training stage;modality is viusal & Audio. (8 ,9) is test trained stage;modality is audio and visual ,respectively.
    switch (modality)
    {
    case 0:
    {                         // visual only
      //TrialTypeContrast += 0; // encode the modality information into the TrialTypeContrast parameter
      for (int i = 0; i < 40; i++)
      {
        BinaryClick[i] = 0;
      }
      break;
    case 1:
    { // Auditory only
      //TrialTypeContrast += 20;
      for (int i = 0; i < 40; i++)
      {
        BinaryFlash[i] = 0;
      }
    }
    break;
    case 2:
    { // muti-modalities Synchronously
      //TrialTypeContrast += 40;
    }
    }
    break;
    }


    if (random(100) < 60)
    { // free reward probability
      ActionAfterDelay = "GiveFreeDrop";
    }
    else
    {
      ActionAfterDelay = "ResponseCue";
    }

    // flashes bin condition
    StateTransition SamplePeriod0_Cond[1] = {{"Tup", "SamplePeriodInterval0"}};
    StateTransition SamplePeriod1_Cond[1] = {{"Tup", "SamplePeriodInterval1"}};
    StateTransition SamplePeriod2_Cond[1] = {{"Tup", "SamplePeriodInterval2"}};
    StateTransition SamplePeriod3_Cond[1] = {{"Tup", "SamplePeriodInterval3"}};
    StateTransition SamplePeriod4_Cond[1] = {{"Tup", "SamplePeriodInterval4"}};
    StateTransition SamplePeriod5_Cond[1] = {{"Tup", "SamplePeriodInterval5"}};
    StateTransition SamplePeriod6_Cond[1] = {{"Tup", "SamplePeriodInterval6"}};
    StateTransition SamplePeriod7_Cond[1] = {{"Tup", "SamplePeriodInterval7"}};
    StateTransition SamplePeriod8_Cond[1] = {{"Tup", "SamplePeriodInterval8"}};
    StateTransition SamplePeriod9_Cond[1] = {{"Tup", "SamplePeriodInterval9"}};
    StateTransition SamplePeriod10_Cond[1] = {{"Tup", "SamplePeriodInterval10"}};
    StateTransition SamplePeriod11_Cond[1] = {{"Tup", "SamplePeriodInterval11"}};
    StateTransition SamplePeriod12_Cond[1] = {{"Tup", "SamplePeriodInterval12"}};
    StateTransition SamplePeriod13_Cond[1] = {{"Tup", "SamplePeriodInterval13"}};
    StateTransition SamplePeriod14_Cond[1] = {{"Tup", "SamplePeriodInterval14"}};
    StateTransition SamplePeriod15_Cond[1] = {{"Tup", "SamplePeriodInterval15"}};
    StateTransition SamplePeriod16_Cond[1] = {{"Tup", "SamplePeriodInterval16"}};
    StateTransition SamplePeriod17_Cond[1] = {{"Tup", "SamplePeriodInterval17"}};
    StateTransition SamplePeriod18_Cond[1] = {{"Tup", "SamplePeriodInterval18"}};
    StateTransition SamplePeriod19_Cond[1] = {{"Tup", "SamplePeriodInterval19"}};
    StateTransition SamplePeriod20_Cond[1] = {{"Tup", "SamplePeriodInterval20"}};
    StateTransition SamplePeriod21_Cond[1] = {{"Tup", "SamplePeriodInterval21"}};
    StateTransition SamplePeriod22_Cond[1] = {{"Tup", "SamplePeriodInterval22"}};
    StateTransition SamplePeriod23_Cond[1] = {{"Tup", "SamplePeriodInterval23"}};
    StateTransition SamplePeriod24_Cond[1] = {{"Tup", "SamplePeriodInterval24"}};
    StateTransition SamplePeriod25_Cond[1] = {{"Tup", "SamplePeriodInterval25"}};
    StateTransition SamplePeriod26_Cond[1] = {{"Tup", "SamplePeriodInterval26"}};
    StateTransition SamplePeriod27_Cond[1] = {{"Tup", "SamplePeriodInterval27"}};
    StateTransition SamplePeriod28_Cond[1] = {{"Tup", "SamplePeriodInterval28"}};
    StateTransition SamplePeriod29_Cond[1] = {{"Tup", "SamplePeriodInterval29"}};
    StateTransition SamplePeriod30_Cond[1] = {{"Tup", "SamplePeriodInterval30"}};
    StateTransition SamplePeriod31_Cond[1] = {{"Tup", "SamplePeriodInterval31"}};
    StateTransition SamplePeriod32_Cond[1] = {{"Tup", "SamplePeriodInterval32"}};
    StateTransition SamplePeriod33_Cond[1] = {{"Tup", "SamplePeriodInterval33"}};
    StateTransition SamplePeriod34_Cond[1] = {{"Tup", "SamplePeriodInterval34"}};
    StateTransition SamplePeriod35_Cond[1] = {{"Tup", "SamplePeriodInterval35"}};
    StateTransition SamplePeriod36_Cond[1] = {{"Tup", "SamplePeriodInterval36"}};
    StateTransition SamplePeriod37_Cond[1] = {{"Tup", "SamplePeriodInterval37"}};
    StateTransition SamplePeriod38_Cond[1] = {{"Tup", "SamplePeriodInterval38"}};
    StateTransition SamplePeriod39_Cond[1] = {{"Tup", "SamplePeriodInterval39"}};

    StateTransition SamplePeriodInterval0_Cond[1]      = {{"Tup" ,"SamplePeriod1"}};
		StateTransition SamplePeriodInterval1_Cond[1]      = {{"Tup" ,"SamplePeriod2"}};
		StateTransition SamplePeriodInterval2_Cond[1]      = {{"Tup" ,"SamplePeriod3"}};
		StateTransition SamplePeriodInterval3_Cond[1]      = {{"Tup" ,"SamplePeriod4"}};
		StateTransition SamplePeriodInterval4_Cond[1]      = {{"Tup" ,"SamplePeriod5"}};
		StateTransition SamplePeriodInterval5_Cond[1]      = {{"Tup" ,"SamplePeriod6"}};
		StateTransition SamplePeriodInterval6_Cond[1]      = {{"Tup" ,"SamplePeriod7"}};
		StateTransition SamplePeriodInterval7_Cond[1]      = {{"Tup" ,"SamplePeriod8"}};
		StateTransition SamplePeriodInterval8_Cond[1]      = {{"Tup" ,"SamplePeriod9"}};
		StateTransition SamplePeriodInterval9_Cond[1]      = {{"Tup" ,"SamplePeriod10"}};
		StateTransition SamplePeriodInterval10_Cond[1]      = {{"Tup" ,"SamplePeriod11"}};
		StateTransition SamplePeriodInterval11_Cond[1]      = {{"Tup" ,"SamplePeriod12"}};
		StateTransition SamplePeriodInterval12_Cond[1]      = {{"Tup" ,"SamplePeriod13"}};
		StateTransition SamplePeriodInterval13_Cond[1]      = {{"Tup" ,"SamplePeriod14"}};
		StateTransition SamplePeriodInterval14_Cond[1]      = {{"Tup" ,"SamplePeriod15"}};
		StateTransition SamplePeriodInterval15_Cond[1]      = {{"Tup" ,"SamplePeriod16"}};
		StateTransition SamplePeriodInterval16_Cond[1]      = {{"Tup" ,"SamplePeriod17"}};
		StateTransition SamplePeriodInterval17_Cond[1]      = {{"Tup" ,"SamplePeriod18"}};
		StateTransition SamplePeriodInterval18_Cond[1]      = {{"Tup" ,"SamplePeriod19"}};
		StateTransition SamplePeriodInterval19_Cond[1]      = {{"Tup" ,"SamplePeriod20"}};
		StateTransition SamplePeriodInterval20_Cond[1]      = {{"Tup" ,"SamplePeriod21"}};
		StateTransition SamplePeriodInterval21_Cond[1]      = {{"Tup" ,"SamplePeriod22"}};
		StateTransition SamplePeriodInterval22_Cond[1]      = {{"Tup" ,"SamplePeriod23"}};
		StateTransition SamplePeriodInterval23_Cond[1]      = {{"Tup" ,"SamplePeriod24"}};
		StateTransition SamplePeriodInterval24_Cond[1]      = {{"Tup" ,"SamplePeriod25"}};
		StateTransition SamplePeriodInterval25_Cond[1]      = {{"Tup" ,"SamplePeriod26"}};
		StateTransition SamplePeriodInterval26_Cond[1]      = {{"Tup" ,"SamplePeriod27"}};
		StateTransition SamplePeriodInterval27_Cond[1]      = {{"Tup" ,"SamplePeriod28"}};
		StateTransition SamplePeriodInterval28_Cond[1]      = {{"Tup" ,"SamplePeriod29"}};
		StateTransition SamplePeriodInterval29_Cond[1]      = {{"Tup" ,"SamplePeriod30"}};
		StateTransition SamplePeriodInterval30_Cond[1]      = {{"Tup" ,"SamplePeriod31"}};
		StateTransition SamplePeriodInterval31_Cond[1]      = {{"Tup" ,"SamplePeriod32"}};
		StateTransition SamplePeriodInterval32_Cond[1]      = {{"Tup" ,"SamplePeriod33"}};
		StateTransition SamplePeriodInterval33_Cond[1]      = {{"Tup" ,"SamplePeriod34"}};
		StateTransition SamplePeriodInterval34_Cond[1]      = {{"Tup" ,"SamplePeriod35"}};
		StateTransition SamplePeriodInterval35_Cond[1]      = {{"Tup" ,"SamplePeriod36"}};
		StateTransition SamplePeriodInterval36_Cond[1]      = {{"Tup" ,"SamplePeriod37"}};
		StateTransition SamplePeriodInterval37_Cond[1]      = {{"Tup" ,"SamplePeriod38"}};
		StateTransition SamplePeriodInterval38_Cond[1]      = {{"Tup" ,"SamplePeriod39"}};
		StateTransition SamplePeriodInterval39_Cond[1]      = {{"Tup" ,"DelayPeriod"}};

    StateTransition PreCueDelayPeriod_Cond[1] = {{"Tup", "SamplePeriod0"}};
    StateTransition TrialStart_Cond[1] = {{"Tup", "PreCueDelayPeriod"}}; //
    // StateTransition SamplePeriod_Cond[1]   = {{"Tup", "DelayPeriod"}};
    StateTransition DelayPeriod_Cond[3] = {{"Tup", ActionAfterDelay} ,{"Lick1In" ,"EarlyLickDelay"} ,{"Lick2In" ,"EarlyLickDelay"}};
    StateTransition ResponseCue_Cond[1] = {{"Tup", "AnswerPeriod"}};
    StateTransition GiveFreeDrop_Cond[1] = {{"Tup", "ResponseCue"}};
    StateTransition AnswerPeriod_Cond[3] = {{"Lick1In", LeftLickAction}, {"Lick2In", RightLickAction}, {"Tup", "NoResponse"}};
    StateTransition Reward_Cond[1] = {{"Tup", "RewardConsumption"}};
    StateTransition Tup_Exit_Cond[1] = {{"Tup", "exit"}};
    StateTransition NoResponse_Cond[3] = {{"Lick1In", "exit"}, {"Lick2In", "exit"}, {"Tup", "exit"}};
    StateTransition ErrorTrial_Cond[1] = {{"Tup", "exit"}};
    StateTransition EarlyLickDelay_Cond[1] = {{"Tup", "DelayPeriod"}}; // not used

    OutputAction FlashOutputL[3] = {SampleOutputBackgroundL, SampleOutputL, DelayNoOutput};
    OutputAction FlashOutputR[3] = {SampleOutputBackgroundR, SampleOutputR, DelayNoOutput};
    OutputAction AudOutput[3] = {AudSampleOutputBackground, AudSampleOutput, DelayNoOutput};
    // flash or click stimulus outputaction
    // set the distribution of flashes
    OutputAction SamplePeriod0_Output[3] = {FlashOutputL[BinaryFlash[0]], FlashOutputR[BinaryFlash[0]], AudOutput[BinaryClick[0]]};
    OutputAction SamplePeriod1_Output[3] = {FlashOutputL[BinaryFlash[1]], FlashOutputR[BinaryFlash[1]], AudOutput[BinaryClick[1]]};
    OutputAction SamplePeriod2_Output[3] = {FlashOutputL[BinaryFlash[2]], FlashOutputR[BinaryFlash[2]], AudOutput[BinaryClick[2]]};
    OutputAction SamplePeriod3_Output[3] = {FlashOutputL[BinaryFlash[3]], FlashOutputR[BinaryFlash[3]], AudOutput[BinaryClick[3]]};
    OutputAction SamplePeriod4_Output[3] = {FlashOutputL[BinaryFlash[4]], FlashOutputR[BinaryFlash[4]], AudOutput[BinaryClick[4]]};
    OutputAction SamplePeriod5_Output[3] = {FlashOutputL[BinaryFlash[5]], FlashOutputR[BinaryFlash[5]], AudOutput[BinaryClick[5]]};
    OutputAction SamplePeriod6_Output[3] = {FlashOutputL[BinaryFlash[6]], FlashOutputR[BinaryFlash[6]], AudOutput[BinaryClick[6]]};
    OutputAction SamplePeriod7_Output[3] = {FlashOutputL[BinaryFlash[7]], FlashOutputR[BinaryFlash[7]], AudOutput[BinaryClick[7]]};
    OutputAction SamplePeriod8_Output[3] = {FlashOutputL[BinaryFlash[8]], FlashOutputR[BinaryFlash[8]], AudOutput[BinaryClick[8]]};
    OutputAction SamplePeriod9_Output[3] = {FlashOutputL[BinaryFlash[9]], FlashOutputR[BinaryFlash[9]], AudOutput[BinaryClick[9]]};
    OutputAction SamplePeriod10_Output[3] = {FlashOutputL[BinaryFlash[10]], FlashOutputR[BinaryFlash[10]], AudOutput[BinaryClick[10]]};
    OutputAction SamplePeriod11_Output[3] = {FlashOutputL[BinaryFlash[11]], FlashOutputR[BinaryFlash[11]], AudOutput[BinaryClick[11]]};
    OutputAction SamplePeriod12_Output[3] = {FlashOutputL[BinaryFlash[12]], FlashOutputR[BinaryFlash[12]], AudOutput[BinaryClick[12]]};
    OutputAction SamplePeriod13_Output[3] = {FlashOutputL[BinaryFlash[13]], FlashOutputR[BinaryFlash[13]], AudOutput[BinaryClick[13]]};
    OutputAction SamplePeriod14_Output[3] = {FlashOutputL[BinaryFlash[14]], FlashOutputR[BinaryFlash[14]], AudOutput[BinaryClick[14]]};
    OutputAction SamplePeriod15_Output[3] = {FlashOutputL[BinaryFlash[15]], FlashOutputR[BinaryFlash[15]], AudOutput[BinaryClick[15]]};
    OutputAction SamplePeriod16_Output[3] = {FlashOutputL[BinaryFlash[16]], FlashOutputR[BinaryFlash[16]], AudOutput[BinaryClick[16]]};
    OutputAction SamplePeriod17_Output[3] = {FlashOutputL[BinaryFlash[17]], FlashOutputR[BinaryFlash[17]], AudOutput[BinaryClick[17]]};
    OutputAction SamplePeriod18_Output[3] = {FlashOutputL[BinaryFlash[18]], FlashOutputR[BinaryFlash[18]], AudOutput[BinaryClick[18]]};
    OutputAction SamplePeriod19_Output[3] = {FlashOutputL[BinaryFlash[19]], FlashOutputR[BinaryFlash[19]], AudOutput[BinaryClick[19]]};
    OutputAction SamplePeriod20_Output[3] = {FlashOutputL[BinaryFlash[20]], FlashOutputR[BinaryFlash[20]], AudOutput[BinaryClick[20]]};
    OutputAction SamplePeriod21_Output[3] = {FlashOutputL[BinaryFlash[21]], FlashOutputR[BinaryFlash[21]], AudOutput[BinaryClick[21]]};
    OutputAction SamplePeriod22_Output[3] = {FlashOutputL[BinaryFlash[22]], FlashOutputR[BinaryFlash[22]], AudOutput[BinaryClick[22]]};
    OutputAction SamplePeriod23_Output[3] = {FlashOutputL[BinaryFlash[23]], FlashOutputR[BinaryFlash[23]], AudOutput[BinaryClick[23]]};
    OutputAction SamplePeriod24_Output[3] = {FlashOutputL[BinaryFlash[24]], FlashOutputR[BinaryFlash[24]], AudOutput[BinaryClick[24]]};
    OutputAction SamplePeriod25_Output[3] = {FlashOutputL[BinaryFlash[25]], FlashOutputR[BinaryFlash[25]], AudOutput[BinaryClick[25]]};
    OutputAction SamplePeriod26_Output[3] = {FlashOutputL[BinaryFlash[26]], FlashOutputR[BinaryFlash[26]], AudOutput[BinaryClick[26]]};
    OutputAction SamplePeriod27_Output[3] = {FlashOutputL[BinaryFlash[27]], FlashOutputR[BinaryFlash[27]], AudOutput[BinaryClick[27]]};
    OutputAction SamplePeriod28_Output[3] = {FlashOutputL[BinaryFlash[28]], FlashOutputR[BinaryFlash[28]], AudOutput[BinaryClick[28]]};
    OutputAction SamplePeriod29_Output[3] = {FlashOutputL[BinaryFlash[29]], FlashOutputR[BinaryFlash[29]], AudOutput[BinaryClick[29]]};
    OutputAction SamplePeriod30_Output[3] = {FlashOutputL[BinaryFlash[30]], FlashOutputR[BinaryFlash[30]], AudOutput[BinaryClick[30]]};
    OutputAction SamplePeriod31_Output[3] = {FlashOutputL[BinaryFlash[31]], FlashOutputR[BinaryFlash[31]], AudOutput[BinaryClick[31]]};
    OutputAction SamplePeriod32_Output[3] = {FlashOutputL[BinaryFlash[32]], FlashOutputR[BinaryFlash[32]], AudOutput[BinaryClick[32]]};
    OutputAction SamplePeriod33_Output[3] = {FlashOutputL[BinaryFlash[33]], FlashOutputR[BinaryFlash[33]], AudOutput[BinaryClick[33]]};
    OutputAction SamplePeriod34_Output[3] = {FlashOutputL[BinaryFlash[34]], FlashOutputR[BinaryFlash[34]], AudOutput[BinaryClick[34]]};
    OutputAction SamplePeriod35_Output[3] = {FlashOutputL[BinaryFlash[35]], FlashOutputR[BinaryFlash[35]], AudOutput[BinaryClick[35]]};
    OutputAction SamplePeriod36_Output[3] = {FlashOutputL[BinaryFlash[36]], FlashOutputR[BinaryFlash[36]], AudOutput[BinaryClick[36]]};
    OutputAction SamplePeriod37_Output[3] = {FlashOutputL[BinaryFlash[37]], FlashOutputR[BinaryFlash[37]], AudOutput[BinaryClick[37]]};
    OutputAction SamplePeriod38_Output[3] = {FlashOutputL[BinaryFlash[38]], FlashOutputR[BinaryFlash[38]], AudOutput[BinaryClick[38]]};
    OutputAction SamplePeriod39_Output[3] = {FlashOutputL[BinaryFlash[39]], FlashOutputR[BinaryFlash[39]], AudOutput[BinaryClick[39]]};

    // OutputAction Sample_Output[1]     	   = {SampleOutput};
    OutputAction PreCueDelayPeriod_Output[1] = {GreenLightOutputBackground}; // low intensity green light as a pre cue
    OutputAction ResponseCue_Output[1] = {CueOutput};
    OutputAction Reward_Output[1] = {RewardOutput};
    OutputAction NoOutput[0] = {};
    OutputAction ErrorOutput[1] = {NoiseOutput};
    OutputAction DelayPeriod_Output[1] = {DelayNoOutput};

    gpSMART_State states[91] = {};
    // visual flash states and conditions
    states[0] = smart.CreateState("TrialStart", 5, 1, TrialStart_Cond, 0, NoOutput); // msec
    states[2] = smart.CreateState("DelayPeriod", S.DelayPeriod,  3, DelayPeriod_Cond, 0, NoOutput);
    states[3] = smart.CreateState("ResponseCue", 100, 1, ResponseCue_Cond, 1, ResponseCue_Output);
    states[4] = smart.CreateState("GiveFreeDrop", reward_dur, 1, GiveFreeDrop_Cond, 1, Reward_Output);
    states[5] = smart.CreateState("AnswerPeriod", S.AnswerPeriod, 3, AnswerPeriod_Cond, 0, NoOutput);
    states[6] = smart.CreateState("Reward", reward_dur, 1, Reward_Cond, 1, Reward_Output);
    states[7] = smart.CreateState("RewardConsumption", S.ConsumptionPeriod, 1, Tup_Exit_Cond, 0, NoOutput);
    states[8] = smart.CreateState("NoResponse", random(30, 60) * 60 * 1000, 3, NoResponse_Cond, 0, NoOutput); // 1-hr: 60*60*1000 msec
    // error trial state (not used state)
    states[9] = smart.CreateState("ErrorTrial", 500, 1, ErrorTrial_Cond, 1, ErrorOutput); // not used
    states[11] = smart.CreateState("EarlyLickDelay", S.EarlyLickPeriod, 1, EarlyLickDelay_Cond, 0, NoOutput); 
                                                                                          // flashes bin
    states[1] = smart.CreateState("SamplePeriod0", PulseTime * sample_bin, 1, SamplePeriod0_Cond, 3, SamplePeriod0_Output);
    states[10] = smart.CreateState("SamplePeriod1", PulseTime * sample_bin, 1, SamplePeriod1_Cond, 3, SamplePeriod1_Output);
    states[90] = smart.CreateState("SamplePeriod2", PulseTime * sample_bin, 1, SamplePeriod2_Cond, 3, SamplePeriod2_Output);
    states[12] = smart.CreateState("SamplePeriod3", PulseTime * sample_bin, 1, SamplePeriod3_Cond, 3, SamplePeriod3_Output);
    states[13] = smart.CreateState("SamplePeriod4", PulseTime * sample_bin, 1, SamplePeriod4_Cond, 3, SamplePeriod4_Output);
    states[14] = smart.CreateState("SamplePeriod5", PulseTime * sample_bin, 1, SamplePeriod5_Cond, 3, SamplePeriod5_Output);
    states[15] = smart.CreateState("SamplePeriod6", PulseTime * sample_bin, 1, SamplePeriod6_Cond, 3, SamplePeriod6_Output);
    states[16] = smart.CreateState("SamplePeriod7", PulseTime * sample_bin, 1, SamplePeriod7_Cond, 3, SamplePeriod7_Output);
    states[17] = smart.CreateState("SamplePeriod8", PulseTime * sample_bin, 1, SamplePeriod8_Cond, 3, SamplePeriod8_Output);
    states[18] = smart.CreateState("SamplePeriod9", PulseTime * sample_bin, 1, SamplePeriod9_Cond, 3, SamplePeriod9_Output);
    states[19] = smart.CreateState("SamplePeriod10", PulseTime * sample_bin, 1, SamplePeriod10_Cond, 3, SamplePeriod10_Output);
    states[20] = smart.CreateState("SamplePeriod11", PulseTime * sample_bin, 1, SamplePeriod11_Cond, 3, SamplePeriod11_Output);
    states[21] = smart.CreateState("SamplePeriod12", PulseTime * sample_bin, 1, SamplePeriod12_Cond, 3, SamplePeriod12_Output);
    states[22] = smart.CreateState("SamplePeriod13", PulseTime * sample_bin, 1, SamplePeriod13_Cond, 3, SamplePeriod13_Output);
    states[23] = smart.CreateState("SamplePeriod14", PulseTime * sample_bin, 1, SamplePeriod14_Cond, 3, SamplePeriod14_Output);
    states[24] = smart.CreateState("SamplePeriod15", PulseTime * sample_bin, 1, SamplePeriod15_Cond, 3, SamplePeriod15_Output);
    states[25] = smart.CreateState("SamplePeriod16", PulseTime * sample_bin, 1, SamplePeriod16_Cond, 3, SamplePeriod16_Output);
    states[26] = smart.CreateState("SamplePeriod17", PulseTime * sample_bin, 1, SamplePeriod17_Cond, 3, SamplePeriod17_Output);
    states[27] = smart.CreateState("SamplePeriod18", PulseTime * sample_bin, 1, SamplePeriod18_Cond, 3, SamplePeriod18_Output);
    states[28] = smart.CreateState("SamplePeriod19", PulseTime * sample_bin, 1, SamplePeriod19_Cond, 3, SamplePeriod19_Output);
    states[29] = smart.CreateState("SamplePeriod20", PulseTime * sample_bin, 1, SamplePeriod20_Cond, 3, SamplePeriod20_Output);
    states[30] = smart.CreateState("SamplePeriod21", PulseTime * sample_bin, 1, SamplePeriod21_Cond, 3, SamplePeriod21_Output);
    states[31] = smart.CreateState("SamplePeriod22", PulseTime * sample_bin, 1, SamplePeriod22_Cond, 3, SamplePeriod22_Output);
    states[32] = smart.CreateState("SamplePeriod23", PulseTime * sample_bin, 1, SamplePeriod23_Cond, 3, SamplePeriod23_Output);
    states[33] = smart.CreateState("SamplePeriod24", PulseTime * sample_bin, 1, SamplePeriod24_Cond, 3, SamplePeriod24_Output);
    states[34] = smart.CreateState("SamplePeriod25", PulseTime * sample_bin, 1, SamplePeriod25_Cond, 3, SamplePeriod25_Output);
    states[35] = smart.CreateState("SamplePeriod26", PulseTime * sample_bin, 1, SamplePeriod26_Cond, 3, SamplePeriod26_Output);
    states[36] = smart.CreateState("SamplePeriod27", PulseTime * sample_bin, 1, SamplePeriod27_Cond, 3, SamplePeriod27_Output);
    states[37] = smart.CreateState("SamplePeriod28", PulseTime * sample_bin, 1, SamplePeriod28_Cond, 3, SamplePeriod28_Output);
    states[38] = smart.CreateState("SamplePeriod29", PulseTime * sample_bin, 1, SamplePeriod29_Cond, 3, SamplePeriod29_Output);
    states[39] = smart.CreateState("SamplePeriod30", PulseTime * sample_bin, 1, SamplePeriod30_Cond, 3, SamplePeriod30_Output);
    states[40] = smart.CreateState("SamplePeriod31", PulseTime * sample_bin, 1, SamplePeriod31_Cond, 3, SamplePeriod31_Output);
    states[41] = smart.CreateState("SamplePeriod32", PulseTime * sample_bin, 1, SamplePeriod32_Cond, 3, SamplePeriod32_Output);
    states[42] = smart.CreateState("SamplePeriod33", PulseTime * sample_bin, 1, SamplePeriod33_Cond, 3, SamplePeriod33_Output);
    states[43] = smart.CreateState("SamplePeriod34", PulseTime * sample_bin, 1, SamplePeriod34_Cond, 3, SamplePeriod34_Output);
    states[44] = smart.CreateState("SamplePeriod35", PulseTime * sample_bin, 1, SamplePeriod35_Cond, 3, SamplePeriod35_Output);
    states[45] = smart.CreateState("SamplePeriod36", PulseTime * sample_bin, 1, SamplePeriod36_Cond, 3, SamplePeriod36_Output);
    states[46] = smart.CreateState("SamplePeriod37", PulseTime * sample_bin, 1, SamplePeriod37_Cond, 3, SamplePeriod37_Output);
    states[47] = smart.CreateState("SamplePeriod38", PulseTime * sample_bin, 1, SamplePeriod38_Cond, 3, SamplePeriod38_Output);
    states[48] = smart.CreateState("SamplePeriod39", PulseTime * sample_bin, 1, SamplePeriod39_Cond, 3, SamplePeriod39_Output);

    states[49] = smart.CreateState("SamplePeriodInterval0", IntervalPulse * sample_bin, 1, SamplePeriodInterval0_Cond, 0,   NoOutput);
    states[50] = smart.CreateState("SamplePeriodInterval1", IntervalPulse * sample_bin, 1, SamplePeriodInterval1_Cond, 0,   NoOutput);
    states[51] = smart.CreateState("SamplePeriodInterval2", IntervalPulse * sample_bin, 1, SamplePeriodInterval2_Cond, 0,   NoOutput);
    states[52] = smart.CreateState("SamplePeriodInterval3", IntervalPulse * sample_bin, 1, SamplePeriodInterval3_Cond, 0,   NoOutput);
    states[53] = smart.CreateState("SamplePeriodInterval4", IntervalPulse * sample_bin, 1, SamplePeriodInterval4_Cond, 0,   NoOutput);
    states[54] = smart.CreateState("SamplePeriodInterval5", IntervalPulse * sample_bin, 1, SamplePeriodInterval5_Cond, 0,   NoOutput);
    states[55] = smart.CreateState("SamplePeriodInterval6", IntervalPulse * sample_bin, 1, SamplePeriodInterval6_Cond, 0,   NoOutput);
    states[56] = smart.CreateState("SamplePeriodInterval7", IntervalPulse * sample_bin, 1, SamplePeriodInterval7_Cond, 0,   NoOutput);
    states[57] = smart.CreateState("SamplePeriodInterval8", IntervalPulse * sample_bin, 1, SamplePeriodInterval8_Cond, 0,   NoOutput);
    states[58] = smart.CreateState("SamplePeriodInterval9", IntervalPulse * sample_bin, 1, SamplePeriodInterval9_Cond, 0,   NoOutput);
    states[59] = smart.CreateState("SamplePeriodInterval10", IntervalPulse * sample_bin, 1, SamplePeriodInterval10_Cond, 0, NoOutput);
    states[60] = smart.CreateState("SamplePeriodInterval11", IntervalPulse * sample_bin, 1, SamplePeriodInterval11_Cond, 0, NoOutput);
    states[61] = smart.CreateState("SamplePeriodInterval12", IntervalPulse * sample_bin, 1, SamplePeriodInterval12_Cond, 0, NoOutput);
    states[62] = smart.CreateState("SamplePeriodInterval13", IntervalPulse * sample_bin, 1, SamplePeriodInterval13_Cond, 0, NoOutput);
    states[63] = smart.CreateState("SamplePeriodInterval14", IntervalPulse * sample_bin, 1, SamplePeriodInterval14_Cond, 0, NoOutput);
    states[64] = smart.CreateState("SamplePeriodInterval15", IntervalPulse * sample_bin, 1, SamplePeriodInterval15_Cond, 0, NoOutput);
    states[65] = smart.CreateState("SamplePeriodInterval16", IntervalPulse * sample_bin, 1, SamplePeriodInterval16_Cond, 0, NoOutput);
    states[66] = smart.CreateState("SamplePeriodInterval17", IntervalPulse * sample_bin, 1, SamplePeriodInterval17_Cond, 0, NoOutput);
    states[67] = smart.CreateState("SamplePeriodInterval18", IntervalPulse * sample_bin, 1, SamplePeriodInterval18_Cond, 0, NoOutput);
    states[68] = smart.CreateState("SamplePeriodInterval19", IntervalPulse * sample_bin, 1, SamplePeriodInterval19_Cond, 0, NoOutput);
    states[69] = smart.CreateState("SamplePeriodInterval20", IntervalPulse * sample_bin, 1, SamplePeriodInterval20_Cond, 0, NoOutput);
    states[70] = smart.CreateState("SamplePeriodInterval21", IntervalPulse * sample_bin, 1, SamplePeriodInterval21_Cond, 0, NoOutput);
    states[71] = smart.CreateState("SamplePeriodInterval22", IntervalPulse * sample_bin, 1, SamplePeriodInterval22_Cond, 0, NoOutput);
    states[72] = smart.CreateState("SamplePeriodInterval23", IntervalPulse * sample_bin, 1, SamplePeriodInterval23_Cond, 0, NoOutput);
    states[73] = smart.CreateState("SamplePeriodInterval24", IntervalPulse * sample_bin, 1, SamplePeriodInterval24_Cond, 0, NoOutput);
    states[74] = smart.CreateState("SamplePeriodInterval25", IntervalPulse * sample_bin, 1, SamplePeriodInterval25_Cond, 0, NoOutput);
    states[75] = smart.CreateState("SamplePeriodInterval26", IntervalPulse * sample_bin, 1, SamplePeriodInterval26_Cond, 0, NoOutput);
    states[76] = smart.CreateState("SamplePeriodInterval27", IntervalPulse * sample_bin, 1, SamplePeriodInterval27_Cond, 0, NoOutput);
    states[77] = smart.CreateState("SamplePeriodInterval28", IntervalPulse * sample_bin, 1, SamplePeriodInterval28_Cond, 0, NoOutput);
    states[78] = smart.CreateState("SamplePeriodInterval29", IntervalPulse * sample_bin, 1, SamplePeriodInterval29_Cond, 0, NoOutput);
    states[79] = smart.CreateState("SamplePeriodInterval30", IntervalPulse * sample_bin, 1, SamplePeriodInterval30_Cond, 0, NoOutput);
    states[80] = smart.CreateState("SamplePeriodInterval31", IntervalPulse * sample_bin, 1, SamplePeriodInterval31_Cond, 0, NoOutput);
    states[81] = smart.CreateState("SamplePeriodInterval32", IntervalPulse * sample_bin, 1, SamplePeriodInterval32_Cond, 0, NoOutput);
    states[82] = smart.CreateState("SamplePeriodInterval33", IntervalPulse * sample_bin, 1, SamplePeriodInterval33_Cond, 0, NoOutput);
    states[83] = smart.CreateState("SamplePeriodInterval34", IntervalPulse * sample_bin, 1, SamplePeriodInterval34_Cond, 0, NoOutput);
    states[84] = smart.CreateState("SamplePeriodInterval35", IntervalPulse * sample_bin, 1, SamplePeriodInterval35_Cond, 0, NoOutput);
    states[85] = smart.CreateState("SamplePeriodInterval36", IntervalPulse * sample_bin, 1, SamplePeriodInterval36_Cond, 0, NoOutput);
    states[86] = smart.CreateState("SamplePeriodInterval37", IntervalPulse * sample_bin, 1, SamplePeriodInterval37_Cond, 0, NoOutput);
    states[87] = smart.CreateState("SamplePeriodInterval38", IntervalPulse * sample_bin, 1, SamplePeriodInterval38_Cond, 0, NoOutput);
    states[88] = smart.CreateState("SamplePeriodInterval39", IntervalPulse * sample_bin, 1, SamplePeriodInterval39_Cond, 0, NoOutput);
    // pre-cue delay
    states[89] = smart.CreateState("PreCueDelayPeriod", S.PreCueDelayPeriod, 1, PreCueDelayPeriod_Cond, 1, PreCueDelayPeriod_Output);
    // Predefine State sequence.
    for (int i = 0; i < 91; i++)
    {
      smart.AddBlankState(states[i].Name);
    }

    // Add a state to state machine.
    for (int i = 0; i < 91; i++)
    {
      smart.AddState(&states[i]);
    }

    // smart.PrintMatrix(); // for debug

    // Run the matrix
    smart.Run();
  }
  break;
  // Protocol 1: add error trials to give a feedback which mice can use it to lick following cue
  case 1: // early training
  case 2: // antiBias
  {       // teach animal to lick according to cue
    OutputAction SampleOutputL;
    OutputAction SampleOutputR;
    OutputAction AudSampleOutput;
    OutputAction SampleOutputBackgroundL;
    OutputAction SampleOutputBackgroundR;
    OutputAction AudSampleOutputBackground;
    OutputAction RewardOutput;

    AudSampleOutput = HighSoundOutput;       // 10KHz freq clickes
    AudSampleOutputBackground = {"Flag3", 0}; // no output
    SampleOutputL = LeftLightOutput;
    SampleOutputR = RightLightOutput;
    SampleOutputBackgroundL = LeftLightOutputBackground;
    SampleOutputBackgroundR = RightLightOutputBackground;

    S.DelayPeriod = float(200); // 200-500 msec random delay
    S.PreCueDelayPeriod = float(random(100, 200));

    String LeftLickAction;
    String RightLickAction;
    String ActionAfterDelay;
    float reward_dur = S.reward_left;

    switch (TrialType)
    {
    case 1: // left low event rate
    {
      if(CP.CurrTrialContrast == 0){
        PoissonRate = 4;
      }else if(CP.CurrTrialContrast == 1){
        PoissonRate = 8;
      }

      LeftLickAction = "Reward";
      RightLickAction = "ErrorTrial"; // w/ error trial
      RewardOutput = LeftWaterOutput;
      reward_dur = S.reward_left;
    }
    break;
    case 2: // right high event rate
    {
      if(CP.CurrTrialContrast == 0){
        PoissonRate = 20;
      }else if(CP.CurrTrialContrast == 1){
        PoissonRate = 16;
      }

      LeftLickAction = "ErrorTrial"; // w/ error trial
      RightLickAction = "Reward";
      RewardOutput = RightWaterOutput;
      reward_dur = S.reward_right;
    }
    break;
    }

    int EventRate = 0; // using the real EventRate that presented to mice to judge the correct choice.
    while(EventRate != PoissonRate) // set the event rate to a constant
    {
    // sample in a binomial distribution to get the Contrast defined before
    TrialTypeContrast = 0;
    P_value = int(1000 * PoissonRate / PoissonTotal);
    for (int i = 0; i < 40; i ++)
    {
      if(i < NoOutputBinNum)
      {
        BinaryClick[i] = 0;
        BinaryFlash[i] = 0;
        PoissonEvent[i] = 2;
      }
      else if(i == NoOutputBinNum) // make sure the first event presents at the onset of sample period
      {
        BinaryClick[i] = 1;
        BinaryFlash[i] = 1;
        PoissonEvent[i] = 1;
        TrialTypeContrast++;
      }
      else
      {
        if (int(random(1000)) < P_value) // using 1000 as the cardinal number to increase resolution of P_value
        {
          BinaryClick[i] = 1;
          BinaryFlash[i] = 1;
          PoissonEvent[i] = 1;
          TrialTypeContrast++;
        }
        else
        {
          BinaryClick[i] = 0;
          BinaryFlash[i] = 0;
          PoissonEvent[i] = 0;
        } 
      }
    }
    // determine the real trialtype and trialContrast
    EventRate = int(round(float(TrialTypeContrast) / float(S.SamplePeriod) * 1000));
    }
    
    // modality choice
    switch (modality)
    {
    case 0:
    {                         // visual only
      //TrialTypeContrast += 0; // encode the modality information into the TrialTypeContrast parameter
      for (int i = 0; i < 40; i++)
      {
        BinaryClick[i] = 0;
      }
      break;
    case 1:
    { // Auditory only
     // TrialTypeContrast += 20;
      for (int i = 0; i < 40; i++)
      {
        BinaryFlash[i] = 0;
      }
    }
    break;
    case 2:
    { // muti-modalities Synchronously
     // TrialTypeContrast += 40;
    }
    }
    break;
    }

    // free reward
    if ((TrialType == 1 && S.GaveFreeReward.flag_L_water == 1) || (TrialType == 2 && S.GaveFreeReward.flag_R_water == 1))
    {
      ActionAfterDelay = "GiveFreeDrop";
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
      ActionAfterDelay = "ResponseCue";
    }
    // flashes bin condition
    StateTransition SamplePeriod0_Cond[1] = {{"Tup", "SamplePeriodInterval0"}};
    StateTransition SamplePeriod1_Cond[1] = {{"Tup", "SamplePeriodInterval1"}};
    StateTransition SamplePeriod2_Cond[1] = {{"Tup", "SamplePeriodInterval2"}};
    StateTransition SamplePeriod3_Cond[1] = {{"Tup", "SamplePeriodInterval3"}};
    StateTransition SamplePeriod4_Cond[1] = {{"Tup", "SamplePeriodInterval4"}};
    StateTransition SamplePeriod5_Cond[1] = {{"Tup", "SamplePeriodInterval5"}};
    StateTransition SamplePeriod6_Cond[1] = {{"Tup", "SamplePeriodInterval6"}};
    StateTransition SamplePeriod7_Cond[1] = {{"Tup", "SamplePeriodInterval7"}};
    StateTransition SamplePeriod8_Cond[1] = {{"Tup", "SamplePeriodInterval8"}};
    StateTransition SamplePeriod9_Cond[1] = {{"Tup", "SamplePeriodInterval9"}};
    StateTransition SamplePeriod10_Cond[1] = {{"Tup", "SamplePeriodInterval10"}};
    StateTransition SamplePeriod11_Cond[1] = {{"Tup", "SamplePeriodInterval11"}};
    StateTransition SamplePeriod12_Cond[1] = {{"Tup", "SamplePeriodInterval12"}};
    StateTransition SamplePeriod13_Cond[1] = {{"Tup", "SamplePeriodInterval13"}};
    StateTransition SamplePeriod14_Cond[1] = {{"Tup", "SamplePeriodInterval14"}};
    StateTransition SamplePeriod15_Cond[1] = {{"Tup", "SamplePeriodInterval15"}};
    StateTransition SamplePeriod16_Cond[1] = {{"Tup", "SamplePeriodInterval16"}};
    StateTransition SamplePeriod17_Cond[1] = {{"Tup", "SamplePeriodInterval17"}};
    StateTransition SamplePeriod18_Cond[1] = {{"Tup", "SamplePeriodInterval18"}};
    StateTransition SamplePeriod19_Cond[1] = {{"Tup", "SamplePeriodInterval19"}};
    StateTransition SamplePeriod20_Cond[1] = {{"Tup", "SamplePeriodInterval20"}};
    StateTransition SamplePeriod21_Cond[1] = {{"Tup", "SamplePeriodInterval21"}};
    StateTransition SamplePeriod22_Cond[1] = {{"Tup", "SamplePeriodInterval22"}};
    StateTransition SamplePeriod23_Cond[1] = {{"Tup", "SamplePeriodInterval23"}};
    StateTransition SamplePeriod24_Cond[1] = {{"Tup", "SamplePeriodInterval24"}};
    StateTransition SamplePeriod25_Cond[1] = {{"Tup", "SamplePeriodInterval25"}};
    StateTransition SamplePeriod26_Cond[1] = {{"Tup", "SamplePeriodInterval26"}};
    StateTransition SamplePeriod27_Cond[1] = {{"Tup", "SamplePeriodInterval27"}};
    StateTransition SamplePeriod28_Cond[1] = {{"Tup", "SamplePeriodInterval28"}};
    StateTransition SamplePeriod29_Cond[1] = {{"Tup", "SamplePeriodInterval29"}};
    StateTransition SamplePeriod30_Cond[1] = {{"Tup", "SamplePeriodInterval30"}};
    StateTransition SamplePeriod31_Cond[1] = {{"Tup", "SamplePeriodInterval31"}};
    StateTransition SamplePeriod32_Cond[1] = {{"Tup", "SamplePeriodInterval32"}};
    StateTransition SamplePeriod33_Cond[1] = {{"Tup", "SamplePeriodInterval33"}};
    StateTransition SamplePeriod34_Cond[1] = {{"Tup", "SamplePeriodInterval34"}};
    StateTransition SamplePeriod35_Cond[1] = {{"Tup", "SamplePeriodInterval35"}};
    StateTransition SamplePeriod36_Cond[1] = {{"Tup", "SamplePeriodInterval36"}};
    StateTransition SamplePeriod37_Cond[1] = {{"Tup", "SamplePeriodInterval37"}};
    StateTransition SamplePeriod38_Cond[1] = {{"Tup", "SamplePeriodInterval38"}};
    StateTransition SamplePeriod39_Cond[1] = {{"Tup", "SamplePeriodInterval39"}};

    StateTransition SamplePeriodInterval0_Cond[1]      = {{"Tup" ,"SamplePeriod1"}};
		StateTransition SamplePeriodInterval1_Cond[1]      = {{"Tup" ,"SamplePeriod2"}};
		StateTransition SamplePeriodInterval2_Cond[1]      = {{"Tup" ,"SamplePeriod3"}};
		StateTransition SamplePeriodInterval3_Cond[1]      = {{"Tup" ,"SamplePeriod4"}};
		StateTransition SamplePeriodInterval4_Cond[1]      = {{"Tup" ,"SamplePeriod5"}};
		StateTransition SamplePeriodInterval5_Cond[1]      = {{"Tup" ,"SamplePeriod6"}};
		StateTransition SamplePeriodInterval6_Cond[1]      = {{"Tup" ,"SamplePeriod7"}};
		StateTransition SamplePeriodInterval7_Cond[1]      = {{"Tup" ,"SamplePeriod8"}};
		StateTransition SamplePeriodInterval8_Cond[1]      = {{"Tup" ,"SamplePeriod9"}};
		StateTransition SamplePeriodInterval9_Cond[1]      = {{"Tup" ,"SamplePeriod10"}};
		StateTransition SamplePeriodInterval10_Cond[1]      = {{"Tup" ,"SamplePeriod11"}};
		StateTransition SamplePeriodInterval11_Cond[1]      = {{"Tup" ,"SamplePeriod12"}};
		StateTransition SamplePeriodInterval12_Cond[1]      = {{"Tup" ,"SamplePeriod13"}};
		StateTransition SamplePeriodInterval13_Cond[1]      = {{"Tup" ,"SamplePeriod14"}};
		StateTransition SamplePeriodInterval14_Cond[1]      = {{"Tup" ,"SamplePeriod15"}};
		StateTransition SamplePeriodInterval15_Cond[1]      = {{"Tup" ,"SamplePeriod16"}};
		StateTransition SamplePeriodInterval16_Cond[1]      = {{"Tup" ,"SamplePeriod17"}};
		StateTransition SamplePeriodInterval17_Cond[1]      = {{"Tup" ,"SamplePeriod18"}};
		StateTransition SamplePeriodInterval18_Cond[1]      = {{"Tup" ,"SamplePeriod19"}};
		StateTransition SamplePeriodInterval19_Cond[1]      = {{"Tup" ,"SamplePeriod20"}};
		StateTransition SamplePeriodInterval20_Cond[1]      = {{"Tup" ,"SamplePeriod21"}};
		StateTransition SamplePeriodInterval21_Cond[1]      = {{"Tup" ,"SamplePeriod22"}};
		StateTransition SamplePeriodInterval22_Cond[1]      = {{"Tup" ,"SamplePeriod23"}};
		StateTransition SamplePeriodInterval23_Cond[1]      = {{"Tup" ,"SamplePeriod24"}};
		StateTransition SamplePeriodInterval24_Cond[1]      = {{"Tup" ,"SamplePeriod25"}};
		StateTransition SamplePeriodInterval25_Cond[1]      = {{"Tup" ,"SamplePeriod26"}};
		StateTransition SamplePeriodInterval26_Cond[1]      = {{"Tup" ,"SamplePeriod27"}};
		StateTransition SamplePeriodInterval27_Cond[1]      = {{"Tup" ,"SamplePeriod28"}};
		StateTransition SamplePeriodInterval28_Cond[1]      = {{"Tup" ,"SamplePeriod29"}};
		StateTransition SamplePeriodInterval29_Cond[1]      = {{"Tup" ,"SamplePeriod30"}};
		StateTransition SamplePeriodInterval30_Cond[1]      = {{"Tup" ,"SamplePeriod31"}};
		StateTransition SamplePeriodInterval31_Cond[1]      = {{"Tup" ,"SamplePeriod32"}};
		StateTransition SamplePeriodInterval32_Cond[1]      = {{"Tup" ,"SamplePeriod33"}};
		StateTransition SamplePeriodInterval33_Cond[1]      = {{"Tup" ,"SamplePeriod34"}};
		StateTransition SamplePeriodInterval34_Cond[1]      = {{"Tup" ,"SamplePeriod35"}};
		StateTransition SamplePeriodInterval35_Cond[1]      = {{"Tup" ,"SamplePeriod36"}};
		StateTransition SamplePeriodInterval36_Cond[1]      = {{"Tup" ,"SamplePeriod37"}};
		StateTransition SamplePeriodInterval37_Cond[1]      = {{"Tup" ,"SamplePeriod38"}};
		StateTransition SamplePeriodInterval38_Cond[1]      = {{"Tup" ,"SamplePeriod39"}};
		StateTransition SamplePeriodInterval39_Cond[1]      = {{"Tup" ,"DelayPeriod"}};

    StateTransition PreCueDelayPeriod_Cond[1] = {{"Tup", "SamplePeriod0"}};
    StateTransition TrialStart_Cond[1] = {{"Tup", "PreCueDelayPeriod"}}; //

    StateTransition EarlyLickDelay_Cond[1] = {{"Tup", "DelayPeriod"}}; 
    StateTransition DelayPeriod_Cond[3] = {{"Tup", ActionAfterDelay} ,{"Lick1In" ,"EarlyLickDelay"} ,{"Lick2In" ,"EarlyLickDelay"}};
    StateTransition ResponseCue_Cond[1] = {{"Tup", "AnswerPeriod"}};
    StateTransition GiveFreeDrop_Cond[1] = {{"Tup", "ResponseCue"}};
    StateTransition AnswerPeriod_Cond[3] = {{"Lick1In", LeftLickAction}, {"Lick2In", RightLickAction}, {"Tup", "NoResponse"}};
    StateTransition Reward_Cond[1] = {{"Tup", "RewardConsumption"}};
    StateTransition Tup_Exit_Cond[1] = {{"Tup", "exit"}};
    StateTransition NoResponse_Cond[3] = {{"Lick1In", "exit"}, {"Lick2In", "exit"}, {"Tup", "exit"}};
    StateTransition ErrorTrial_Cond[1] = {{"Tup", "TimeOut"}};

    OutputAction FlashOutputL[3] = {SampleOutputBackgroundL, SampleOutputL, DelayNoOutput};
    OutputAction FlashOutputR[3] = {SampleOutputBackgroundR, SampleOutputR, DelayNoOutput};
    OutputAction AudOutput[3] = {AudSampleOutputBackground, AudSampleOutput, DelayNoOutput};
    // flash or click stimulus outputaction
    // set the distribution of flashes
    OutputAction SamplePeriod0_Output[3] = {FlashOutputL[BinaryFlash[0]], FlashOutputR[BinaryFlash[0]], AudOutput[BinaryClick[0]]};
    OutputAction SamplePeriod1_Output[3] = {FlashOutputL[BinaryFlash[1]], FlashOutputR[BinaryFlash[1]], AudOutput[BinaryClick[1]]};
    OutputAction SamplePeriod2_Output[3] = {FlashOutputL[BinaryFlash[2]], FlashOutputR[BinaryFlash[2]], AudOutput[BinaryClick[2]]};
    OutputAction SamplePeriod3_Output[3] = {FlashOutputL[BinaryFlash[3]], FlashOutputR[BinaryFlash[3]], AudOutput[BinaryClick[3]]};
    OutputAction SamplePeriod4_Output[3] = {FlashOutputL[BinaryFlash[4]], FlashOutputR[BinaryFlash[4]], AudOutput[BinaryClick[4]]};
    OutputAction SamplePeriod5_Output[3] = {FlashOutputL[BinaryFlash[5]], FlashOutputR[BinaryFlash[5]], AudOutput[BinaryClick[5]]};
    OutputAction SamplePeriod6_Output[3] = {FlashOutputL[BinaryFlash[6]], FlashOutputR[BinaryFlash[6]], AudOutput[BinaryClick[6]]};
    OutputAction SamplePeriod7_Output[3] = {FlashOutputL[BinaryFlash[7]], FlashOutputR[BinaryFlash[7]], AudOutput[BinaryClick[7]]};
    OutputAction SamplePeriod8_Output[3] = {FlashOutputL[BinaryFlash[8]], FlashOutputR[BinaryFlash[8]], AudOutput[BinaryClick[8]]};
    OutputAction SamplePeriod9_Output[3] = {FlashOutputL[BinaryFlash[9]], FlashOutputR[BinaryFlash[9]], AudOutput[BinaryClick[9]]};
    OutputAction SamplePeriod10_Output[3] = {FlashOutputL[BinaryFlash[10]], FlashOutputR[BinaryFlash[10]], AudOutput[BinaryClick[10]]};
    OutputAction SamplePeriod11_Output[3] = {FlashOutputL[BinaryFlash[11]], FlashOutputR[BinaryFlash[11]], AudOutput[BinaryClick[11]]};
    OutputAction SamplePeriod12_Output[3] = {FlashOutputL[BinaryFlash[12]], FlashOutputR[BinaryFlash[12]], AudOutput[BinaryClick[12]]};
    OutputAction SamplePeriod13_Output[3] = {FlashOutputL[BinaryFlash[13]], FlashOutputR[BinaryFlash[13]], AudOutput[BinaryClick[13]]};
    OutputAction SamplePeriod14_Output[3] = {FlashOutputL[BinaryFlash[14]], FlashOutputR[BinaryFlash[14]], AudOutput[BinaryClick[14]]};
    OutputAction SamplePeriod15_Output[3] = {FlashOutputL[BinaryFlash[15]], FlashOutputR[BinaryFlash[15]], AudOutput[BinaryClick[15]]};
    OutputAction SamplePeriod16_Output[3] = {FlashOutputL[BinaryFlash[16]], FlashOutputR[BinaryFlash[16]], AudOutput[BinaryClick[16]]};
    OutputAction SamplePeriod17_Output[3] = {FlashOutputL[BinaryFlash[17]], FlashOutputR[BinaryFlash[17]], AudOutput[BinaryClick[17]]};
    OutputAction SamplePeriod18_Output[3] = {FlashOutputL[BinaryFlash[18]], FlashOutputR[BinaryFlash[18]], AudOutput[BinaryClick[18]]};
    OutputAction SamplePeriod19_Output[3] = {FlashOutputL[BinaryFlash[19]], FlashOutputR[BinaryFlash[19]], AudOutput[BinaryClick[19]]};
    OutputAction SamplePeriod20_Output[3] = {FlashOutputL[BinaryFlash[20]], FlashOutputR[BinaryFlash[20]], AudOutput[BinaryClick[20]]};
    OutputAction SamplePeriod21_Output[3] = {FlashOutputL[BinaryFlash[21]], FlashOutputR[BinaryFlash[21]], AudOutput[BinaryClick[21]]};
    OutputAction SamplePeriod22_Output[3] = {FlashOutputL[BinaryFlash[22]], FlashOutputR[BinaryFlash[22]], AudOutput[BinaryClick[22]]};
    OutputAction SamplePeriod23_Output[3] = {FlashOutputL[BinaryFlash[23]], FlashOutputR[BinaryFlash[23]], AudOutput[BinaryClick[23]]};
    OutputAction SamplePeriod24_Output[3] = {FlashOutputL[BinaryFlash[24]], FlashOutputR[BinaryFlash[24]], AudOutput[BinaryClick[24]]};
    OutputAction SamplePeriod25_Output[3] = {FlashOutputL[BinaryFlash[25]], FlashOutputR[BinaryFlash[25]], AudOutput[BinaryClick[25]]};
    OutputAction SamplePeriod26_Output[3] = {FlashOutputL[BinaryFlash[26]], FlashOutputR[BinaryFlash[26]], AudOutput[BinaryClick[26]]};
    OutputAction SamplePeriod27_Output[3] = {FlashOutputL[BinaryFlash[27]], FlashOutputR[BinaryFlash[27]], AudOutput[BinaryClick[27]]};
    OutputAction SamplePeriod28_Output[3] = {FlashOutputL[BinaryFlash[28]], FlashOutputR[BinaryFlash[28]], AudOutput[BinaryClick[28]]};
    OutputAction SamplePeriod29_Output[3] = {FlashOutputL[BinaryFlash[29]], FlashOutputR[BinaryFlash[29]], AudOutput[BinaryClick[29]]};
    OutputAction SamplePeriod30_Output[3] = {FlashOutputL[BinaryFlash[30]], FlashOutputR[BinaryFlash[30]], AudOutput[BinaryClick[30]]};
    OutputAction SamplePeriod31_Output[3] = {FlashOutputL[BinaryFlash[31]], FlashOutputR[BinaryFlash[31]], AudOutput[BinaryClick[31]]};
    OutputAction SamplePeriod32_Output[3] = {FlashOutputL[BinaryFlash[32]], FlashOutputR[BinaryFlash[32]], AudOutput[BinaryClick[32]]};
    OutputAction SamplePeriod33_Output[3] = {FlashOutputL[BinaryFlash[33]], FlashOutputR[BinaryFlash[33]], AudOutput[BinaryClick[33]]};
    OutputAction SamplePeriod34_Output[3] = {FlashOutputL[BinaryFlash[34]], FlashOutputR[BinaryFlash[34]], AudOutput[BinaryClick[34]]};
    OutputAction SamplePeriod35_Output[3] = {FlashOutputL[BinaryFlash[35]], FlashOutputR[BinaryFlash[35]], AudOutput[BinaryClick[35]]};
    OutputAction SamplePeriod36_Output[3] = {FlashOutputL[BinaryFlash[36]], FlashOutputR[BinaryFlash[36]], AudOutput[BinaryClick[36]]};
    OutputAction SamplePeriod37_Output[3] = {FlashOutputL[BinaryFlash[37]], FlashOutputR[BinaryFlash[37]], AudOutput[BinaryClick[37]]};
    OutputAction SamplePeriod38_Output[3] = {FlashOutputL[BinaryFlash[38]], FlashOutputR[BinaryFlash[38]], AudOutput[BinaryClick[38]]};
    OutputAction SamplePeriod39_Output[3] = {FlashOutputL[BinaryFlash[39]], FlashOutputR[BinaryFlash[39]], AudOutput[BinaryClick[39]]};

    // OutputAction Sample_Output[1]     	   = {SampleOutput};
    OutputAction PreCueDelayPeriod_Output[1] = {GreenLightOutputBackground}; // low intensity green light as a pre cue
    OutputAction ResponseCue_Output[1] = {CueOutput};
    OutputAction Reward_Output[1] = {RewardOutput};
    OutputAction NoOutput[0] = {};
    OutputAction ErrorOutput[1] = {NoiseOutput};
    OutputAction DelayPeriod_Output[1] = {DelayNoOutput}; // red light

    gpSMART_State states[92] = {};
    states[0] = smart.CreateState("TrialStart", 5, 1, TrialStart_Cond, 0, NoOutput); // msec
                                                                                     // states[1]  = smart.CreateState("SamplePeriod",      S.SamplePeriod,       1, SamplePeriod_Cond,    1, Sample_Output);
    states[2] = smart.CreateState("DelayPeriod", float(S.DelayPeriod), 3, DelayPeriod_Cond, 0, NoOutput);
    states[3] = smart.CreateState("ResponseCue", 100, 1, ResponseCue_Cond, 1, ResponseCue_Output);
    states[4] = smart.CreateState("GiveFreeDrop", reward_dur, 1, GiveFreeDrop_Cond, 1, Reward_Output);
    states[5] = smart.CreateState("AnswerPeriod", S.AnswerPeriod, 3, AnswerPeriod_Cond, 0, NoOutput);
    states[6] = smart.CreateState("Reward", reward_dur, 1, Reward_Cond, 1, Reward_Output);
    states[7] = smart.CreateState("RewardConsumption", S.ConsumptionPeriod, 1, Tup_Exit_Cond, 0, NoOutput);
    states[8] = smart.CreateState("NoResponse", random(30, 60) * 60 * 1000, 3, NoResponse_Cond, 0, NoOutput); // 1-hr: 60*60*1000 msec
    states[9] = smart.CreateState("ErrorTrial", 500, 1, ErrorTrial_Cond, 1, ErrorOutput);
    states[10] = smart.CreateState("TimeOut", S.TimeOut + S.extra_TimeOut, 1, Tup_Exit_Cond, 0, NoOutput);

    states[11] = smart.CreateState("EarlyLickDelay", S.EarlyLickPeriod, 1, EarlyLickDelay_Cond, 0, NoOutput); // not used
    // flashes bin
    states[1] = smart.CreateState("SamplePeriod0", PulseTime * sample_bin, 1, SamplePeriod0_Cond, 3, SamplePeriod0_Output);
    states[12] = smart.CreateState("SamplePeriod1", PulseTime * sample_bin, 1, SamplePeriod1_Cond, 3, SamplePeriod1_Output);
    states[13] = smart.CreateState("SamplePeriod2", PulseTime * sample_bin, 1, SamplePeriod2_Cond, 3, SamplePeriod2_Output);
    states[14] = smart.CreateState("SamplePeriod3", PulseTime * sample_bin, 1, SamplePeriod3_Cond, 3, SamplePeriod3_Output);
    states[15] = smart.CreateState("SamplePeriod4", PulseTime * sample_bin, 1, SamplePeriod4_Cond, 3, SamplePeriod4_Output);
    states[16] = smart.CreateState("SamplePeriod5", PulseTime * sample_bin, 1, SamplePeriod5_Cond, 3, SamplePeriod5_Output);
    states[17] = smart.CreateState("SamplePeriod6", PulseTime * sample_bin, 1, SamplePeriod6_Cond, 3, SamplePeriod6_Output);
    states[18] = smart.CreateState("SamplePeriod7", PulseTime * sample_bin, 1, SamplePeriod7_Cond, 3, SamplePeriod7_Output);
    states[19] = smart.CreateState("SamplePeriod8", PulseTime * sample_bin, 1, SamplePeriod8_Cond, 3, SamplePeriod8_Output);
    states[20] = smart.CreateState("SamplePeriod9", PulseTime * sample_bin, 1, SamplePeriod9_Cond, 3, SamplePeriod9_Output);
    states[21] = smart.CreateState("SamplePeriod10", PulseTime * sample_bin, 1, SamplePeriod10_Cond, 3, SamplePeriod10_Output);
    states[22] = smart.CreateState("SamplePeriod11", PulseTime * sample_bin, 1, SamplePeriod11_Cond, 3, SamplePeriod11_Output);
    states[23] = smart.CreateState("SamplePeriod12", PulseTime * sample_bin, 1, SamplePeriod12_Cond, 3, SamplePeriod12_Output);
    states[24] = smart.CreateState("SamplePeriod13", PulseTime * sample_bin, 1, SamplePeriod13_Cond, 3, SamplePeriod13_Output);
    states[25] = smart.CreateState("SamplePeriod14", PulseTime * sample_bin, 1, SamplePeriod14_Cond, 3, SamplePeriod14_Output);
    states[26] = smart.CreateState("SamplePeriod15", PulseTime * sample_bin, 1, SamplePeriod15_Cond, 3, SamplePeriod15_Output);
    states[27] = smart.CreateState("SamplePeriod16", PulseTime * sample_bin, 1, SamplePeriod16_Cond, 3, SamplePeriod16_Output);
    states[28] = smart.CreateState("SamplePeriod17", PulseTime * sample_bin, 1, SamplePeriod17_Cond, 3, SamplePeriod17_Output);
    states[29] = smart.CreateState("SamplePeriod18", PulseTime * sample_bin, 1, SamplePeriod18_Cond, 3, SamplePeriod18_Output);
    states[30] = smart.CreateState("SamplePeriod19", PulseTime * sample_bin, 1, SamplePeriod19_Cond, 3, SamplePeriod19_Output);
    states[31] = smart.CreateState("SamplePeriod20", PulseTime * sample_bin, 1, SamplePeriod20_Cond, 3, SamplePeriod20_Output);
    states[32] = smart.CreateState("SamplePeriod21", PulseTime * sample_bin, 1, SamplePeriod21_Cond, 3, SamplePeriod21_Output);
    states[33] = smart.CreateState("SamplePeriod22", PulseTime * sample_bin, 1, SamplePeriod22_Cond, 3, SamplePeriod22_Output);
    states[34] = smart.CreateState("SamplePeriod23", PulseTime * sample_bin, 1, SamplePeriod23_Cond, 3, SamplePeriod23_Output);
    states[35] = smart.CreateState("SamplePeriod24", PulseTime * sample_bin, 1, SamplePeriod24_Cond, 3, SamplePeriod24_Output);
    states[36] = smart.CreateState("SamplePeriod25", PulseTime * sample_bin, 1, SamplePeriod25_Cond, 3, SamplePeriod25_Output);
    states[37] = smart.CreateState("SamplePeriod26", PulseTime * sample_bin, 1, SamplePeriod26_Cond, 3, SamplePeriod26_Output);
    states[38] = smart.CreateState("SamplePeriod27", PulseTime * sample_bin, 1, SamplePeriod27_Cond, 3, SamplePeriod27_Output);
    states[39] = smart.CreateState("SamplePeriod28", PulseTime * sample_bin, 1, SamplePeriod28_Cond, 3, SamplePeriod28_Output);
    states[40] = smart.CreateState("SamplePeriod29", PulseTime * sample_bin, 1, SamplePeriod29_Cond, 3, SamplePeriod29_Output);
    states[41] = smart.CreateState("SamplePeriod30", PulseTime * sample_bin, 1, SamplePeriod30_Cond, 3, SamplePeriod30_Output);
    states[42] = smart.CreateState("SamplePeriod31", PulseTime * sample_bin, 1, SamplePeriod31_Cond, 3, SamplePeriod31_Output);
    states[43] = smart.CreateState("SamplePeriod32", PulseTime * sample_bin, 1, SamplePeriod32_Cond, 3, SamplePeriod32_Output);
    states[44] = smart.CreateState("SamplePeriod33", PulseTime * sample_bin, 1, SamplePeriod33_Cond, 3, SamplePeriod33_Output);
    states[45] = smart.CreateState("SamplePeriod34", PulseTime * sample_bin, 1, SamplePeriod34_Cond, 3, SamplePeriod34_Output);
    states[46] = smart.CreateState("SamplePeriod35", PulseTime * sample_bin, 1, SamplePeriod35_Cond, 3, SamplePeriod35_Output);
    states[47] = smart.CreateState("SamplePeriod36", PulseTime * sample_bin, 1, SamplePeriod36_Cond, 3, SamplePeriod36_Output);
    states[48] = smart.CreateState("SamplePeriod37", PulseTime * sample_bin, 1, SamplePeriod37_Cond, 3, SamplePeriod37_Output);
    states[49] = smart.CreateState("SamplePeriod38", PulseTime * sample_bin, 1, SamplePeriod38_Cond, 3, SamplePeriod38_Output);
    states[50] = smart.CreateState("SamplePeriod39", PulseTime * sample_bin, 1, SamplePeriod39_Cond, 3, SamplePeriod39_Output);

    states[51] = smart.CreateState("SamplePeriodInterval0", IntervalPulse * sample_bin, 1, SamplePeriodInterval0_Cond, 0,   NoOutput);
    states[52] = smart.CreateState("SamplePeriodInterval1", IntervalPulse * sample_bin, 1, SamplePeriodInterval1_Cond, 0,   NoOutput);
    states[53] = smart.CreateState("SamplePeriodInterval2", IntervalPulse * sample_bin, 1, SamplePeriodInterval2_Cond, 0,   NoOutput);
    states[54] = smart.CreateState("SamplePeriodInterval3", IntervalPulse * sample_bin, 1, SamplePeriodInterval3_Cond, 0,   NoOutput);
    states[55] = smart.CreateState("SamplePeriodInterval4", IntervalPulse * sample_bin, 1, SamplePeriodInterval4_Cond, 0,   NoOutput);
    states[56] = smart.CreateState("SamplePeriodInterval5", IntervalPulse * sample_bin, 1, SamplePeriodInterval5_Cond, 0,   NoOutput);
    states[57] = smart.CreateState("SamplePeriodInterval6", IntervalPulse * sample_bin, 1, SamplePeriodInterval6_Cond, 0,   NoOutput);
    states[58] = smart.CreateState("SamplePeriodInterval7", IntervalPulse * sample_bin, 1, SamplePeriodInterval7_Cond, 0,   NoOutput);
    states[59] = smart.CreateState("SamplePeriodInterval8", IntervalPulse * sample_bin, 1, SamplePeriodInterval8_Cond, 0,   NoOutput);
    states[60] = smart.CreateState("SamplePeriodInterval9", IntervalPulse * sample_bin, 1, SamplePeriodInterval9_Cond, 0,   NoOutput);
    states[61] = smart.CreateState("SamplePeriodInterval10", IntervalPulse * sample_bin, 1, SamplePeriodInterval10_Cond, 0, NoOutput);
    states[62] = smart.CreateState("SamplePeriodInterval11", IntervalPulse * sample_bin, 1, SamplePeriodInterval11_Cond, 0, NoOutput);
    states[63] = smart.CreateState("SamplePeriodInterval12", IntervalPulse * sample_bin, 1, SamplePeriodInterval12_Cond, 0, NoOutput);
    states[64] = smart.CreateState("SamplePeriodInterval13", IntervalPulse * sample_bin, 1, SamplePeriodInterval13_Cond, 0, NoOutput);
    states[65] = smart.CreateState("SamplePeriodInterval14", IntervalPulse * sample_bin, 1, SamplePeriodInterval14_Cond, 0, NoOutput);
    states[66] = smart.CreateState("SamplePeriodInterval15", IntervalPulse * sample_bin, 1, SamplePeriodInterval15_Cond, 0, NoOutput);
    states[67] = smart.CreateState("SamplePeriodInterval16", IntervalPulse * sample_bin, 1, SamplePeriodInterval16_Cond, 0, NoOutput);
    states[68] = smart.CreateState("SamplePeriodInterval17", IntervalPulse * sample_bin, 1, SamplePeriodInterval17_Cond, 0, NoOutput);
    states[69] = smart.CreateState("SamplePeriodInterval18", IntervalPulse * sample_bin, 1, SamplePeriodInterval18_Cond, 0, NoOutput);
    states[70] = smart.CreateState("SamplePeriodInterval19", IntervalPulse * sample_bin, 1, SamplePeriodInterval19_Cond, 0, NoOutput);
    states[71] = smart.CreateState("SamplePeriodInterval20", IntervalPulse * sample_bin, 1, SamplePeriodInterval20_Cond, 0, NoOutput);
    states[72] = smart.CreateState("SamplePeriodInterval21", IntervalPulse * sample_bin, 1, SamplePeriodInterval21_Cond, 0, NoOutput);
    states[73] = smart.CreateState("SamplePeriodInterval22", IntervalPulse * sample_bin, 1, SamplePeriodInterval22_Cond, 0, NoOutput);
    states[74] = smart.CreateState("SamplePeriodInterval23", IntervalPulse * sample_bin, 1, SamplePeriodInterval23_Cond, 0, NoOutput);
    states[75] = smart.CreateState("SamplePeriodInterval24", IntervalPulse * sample_bin, 1, SamplePeriodInterval24_Cond, 0, NoOutput);
    states[76] = smart.CreateState("SamplePeriodInterval25", IntervalPulse * sample_bin, 1, SamplePeriodInterval25_Cond, 0, NoOutput);
    states[77] = smart.CreateState("SamplePeriodInterval26", IntervalPulse * sample_bin, 1, SamplePeriodInterval26_Cond, 0, NoOutput);
    states[78] = smart.CreateState("SamplePeriodInterval27", IntervalPulse * sample_bin, 1, SamplePeriodInterval27_Cond, 0, NoOutput);
    states[79] = smart.CreateState("SamplePeriodInterval28", IntervalPulse * sample_bin, 1, SamplePeriodInterval28_Cond, 0, NoOutput);
    states[80] = smart.CreateState("SamplePeriodInterval29", IntervalPulse * sample_bin, 1, SamplePeriodInterval29_Cond, 0, NoOutput);
    states[81] = smart.CreateState("SamplePeriodInterval30", IntervalPulse * sample_bin, 1, SamplePeriodInterval30_Cond, 0, NoOutput);
    states[82] = smart.CreateState("SamplePeriodInterval31", IntervalPulse * sample_bin, 1, SamplePeriodInterval31_Cond, 0, NoOutput);
    states[83] = smart.CreateState("SamplePeriodInterval32", IntervalPulse * sample_bin, 1, SamplePeriodInterval32_Cond, 0, NoOutput);
    states[84] = smart.CreateState("SamplePeriodInterval33", IntervalPulse * sample_bin, 1, SamplePeriodInterval33_Cond, 0, NoOutput);
    states[85] = smart.CreateState("SamplePeriodInterval34", IntervalPulse * sample_bin, 1, SamplePeriodInterval34_Cond, 0, NoOutput);
    states[86] = smart.CreateState("SamplePeriodInterval35", IntervalPulse * sample_bin, 1, SamplePeriodInterval35_Cond, 0, NoOutput);
    states[87] = smart.CreateState("SamplePeriodInterval36", IntervalPulse * sample_bin, 1, SamplePeriodInterval36_Cond, 0, NoOutput);
    states[88] = smart.CreateState("SamplePeriodInterval37", IntervalPulse * sample_bin, 1, SamplePeriodInterval37_Cond, 0, NoOutput);
    states[89] = smart.CreateState("SamplePeriodInterval38", IntervalPulse * sample_bin, 1, SamplePeriodInterval38_Cond, 0, NoOutput);
    states[90] = smart.CreateState("SamplePeriodInterval39", IntervalPulse * sample_bin, 1, SamplePeriodInterval39_Cond, 0, NoOutput);
    // pre-cue delay
    states[91] = smart.CreateState("PreCueDelayPeriod", S.PreCueDelayPeriod, 1, PreCueDelayPeriod_Cond, 1, PreCueDelayPeriod_Output);

    // Predefine State sequence.
    for (int i = 0; i < 92; i++)
    {
      smart.AddBlankState(states[i].Name);
    }
    // Add a state to state machine.
    for (int i = 0; i < 92; i++)
    {
      smart.AddState(&states[i]);
    }
    // Run the matrix
    smart.Run();
  }
  break;
  // Protocol 3: full protocol training mice to discrmination different sample 
  case 3: //  antiBias 
  case 4: //  antiBias
  case 5: //  antiBias
  case 6: //  antiBias
  case 7: //  antibias retention period
  case 8: 
  {       
    OutputAction SampleOutputL;
    OutputAction SampleOutputR;
    OutputAction AudSampleOutput;
    OutputAction AudSampleOutputBackground;
    OutputAction SampleOutputBackgroundL;
    OutputAction SampleOutputBackgroundR;
    OutputAction RewardOutput;

    SampleOutputL = LeftLightOutput;
    SampleOutputR = RightLightOutput;

    // contrast aud cue
    AudContrast = int(random(1 ,5));
    if(S.currProtocolIndex == 8){
      AudSampleOutput = AudContrastOutput[AudContrast - 1];
    }else{
      //TODO
      AudSampleOutput = HighSoundOutput;       // 10KHz freq clickes
    }

    AudSampleOutputBackground = {"Flag3", 0}; // no output
    SampleOutputBackgroundL = LeftLightOutputBackground;
    SampleOutputBackgroundR = RightLightOutputBackground;

    S.DelayPeriod = float(200); // 200-500 msec random delay
    S.PreCueDelayPeriod = float(random(100, 200));

    String LeftLickAction;
    String RightLickAction;
    String ActionAfterDelay;
    float reward_dur = S.reward_left;
    int SelectedContrast = 0;

    switch (TrialType)
    {
    case 1: // left low event rate
    {
      if(CP.CurrTrialContrast == 0){
        PoissonRate = 4;
      }else if(CP.CurrTrialContrast == 1){
        PoissonRate = 8;
      }

      LeftLickAction = "Reward";
      RightLickAction = "ErrorTrial"; // w/ error trial
      RewardOutput = LeftWaterOutput;
      reward_dur = S.reward_left;
    }
    break;
    case 2: // right high event rate
    {
      if(CP.CurrTrialContrast == 0){
        PoissonRate = 20;
      }else if(CP.CurrTrialContrast == 1){
        PoissonRate = 16;
      }

      LeftLickAction = "ErrorTrial"; // w/ error trial
      RightLickAction = "Reward";
      RewardOutput = RightWaterOutput;
      reward_dur = S.reward_right;
    }
    break;
    }

    int EventRate = 0; // using the real EventRate that presented to mice to judge the correct choice.
    while(EventRate != PoissonRate) // set the event rate to a constant
    {
    // sample in a binomial distribution to get the Contrast defined before
    TrialTypeContrast = 0;
    P_value = int(1000 * PoissonRate / PoissonTotal);
    for (int i = 0; i < 40; i ++)
    {
      if(i < NoOutputBinNum)
      {
        BinaryClick[i] = 0;
        BinaryFlash[i] = 0;
        PoissonEvent[i] = 2;
      }
      else if(i == NoOutputBinNum) // make sure the first event presents at the onset of sample period
      {
        BinaryClick[i] = 1;
        BinaryFlash[i] = 1;
        PoissonEvent[i] = 1;
        TrialTypeContrast++;
      }
      else
      {
        if (int(random(1000)) < P_value) // using 1000 as the cardinal number to increase resolution of P_value
        {
          BinaryClick[i] = 1;
          BinaryFlash[i] = 1;
          PoissonEvent[i] = 1;
          TrialTypeContrast++;
        }
        else
        {
          BinaryClick[i] = 0;
          BinaryFlash[i] = 0;
          PoissonEvent[i] = 0;
        } 
      }
    }
    // determine the real trialtype and trialContrast
    EventRate = int(round(float(TrialTypeContrast) / float(S.SamplePeriod) * 1000));
    if(S.currProtocolIndex >= 3)
    {
      if(TrialType == 1)
      {
        if( EventRate <= 12)
        {
          break;
        }
      }
      else if(TrialType == 2)
      {
        if(12 <= EventRate)
        {
          break;
        }
      }
    }
    }
    // mid event rate rewards to random port
    if(EventRate == 12)
    {
      if(int(random(2)) == 0)
      {
        LeftLickAction = "Reward";
        RightLickAction = "ErrorTrial"; // w/ error trial
        RewardOutput = LeftWaterOutput;
        reward_dur = S.reward_left;
      }
      else
      {
        LeftLickAction = "ErrorTrial"; // w/ error trial
        RightLickAction = "Reward";
        RewardOutput = RightWaterOutput;
        reward_dur = S.reward_right;
      }
    }

    // modality choice
    switch (modality)
    {
    case 0:
    {                         // visual only
     // TrialTypeContrast += 0; // encode the modality information into the TrialTypeContrast parameter
      for (int i = 0; i < 40; i++)
      {
        BinaryClick[i] = 0;
      }
      break;
    case 1:
    { // Auditory only
     // TrialTypeContrast += 20;
      for (int i = 0; i < 40; i++)
      {
        BinaryFlash[i] = 0;
      }
    }
    break;
    case 2:
    { // multi-modalities Synchronously
      //TrialTypeContrast += 40;
    }
    }
    break;
    }

    // free reward
    if ((TrialType == 1 && S.GaveFreeReward.flag_L_water == 1) || (TrialType == 2 && S.GaveFreeReward.flag_R_water == 1))
    {
      ActionAfterDelay = "GiveFreeDrop";
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
      ActionAfterDelay = "ResponseCue";
    }

    // flashes bin condition
    StateTransition SamplePeriod0_Cond[1] = {{"Tup", "SamplePeriodInterval0"}};
    StateTransition SamplePeriod1_Cond[1] = {{"Tup", "SamplePeriodInterval1"}};
    StateTransition SamplePeriod2_Cond[1] = {{"Tup", "SamplePeriodInterval2"}};
    StateTransition SamplePeriod3_Cond[1] = {{"Tup", "SamplePeriodInterval3"}};
    StateTransition SamplePeriod4_Cond[1] = {{"Tup", "SamplePeriodInterval4"}};
    StateTransition SamplePeriod5_Cond[1] = {{"Tup", "SamplePeriodInterval5"}};
    StateTransition SamplePeriod6_Cond[1] = {{"Tup", "SamplePeriodInterval6"}};
    StateTransition SamplePeriod7_Cond[1] = {{"Tup", "SamplePeriodInterval7"}};
    StateTransition SamplePeriod8_Cond[1] = {{"Tup", "SamplePeriodInterval8"}};
    StateTransition SamplePeriod9_Cond[1] = {{"Tup", "SamplePeriodInterval9"}};
    StateTransition SamplePeriod10_Cond[1] = {{"Tup", "SamplePeriodInterval10"}};
    StateTransition SamplePeriod11_Cond[1] = {{"Tup", "SamplePeriodInterval11"}};
    StateTransition SamplePeriod12_Cond[1] = {{"Tup", "SamplePeriodInterval12"}};
    StateTransition SamplePeriod13_Cond[1] = {{"Tup", "SamplePeriodInterval13"}};
    StateTransition SamplePeriod14_Cond[1] = {{"Tup", "SamplePeriodInterval14"}};
    StateTransition SamplePeriod15_Cond[1] = {{"Tup", "SamplePeriodInterval15"}};
    StateTransition SamplePeriod16_Cond[1] = {{"Tup", "SamplePeriodInterval16"}};
    StateTransition SamplePeriod17_Cond[1] = {{"Tup", "SamplePeriodInterval17"}};
    StateTransition SamplePeriod18_Cond[1] = {{"Tup", "SamplePeriodInterval18"}};
    StateTransition SamplePeriod19_Cond[1] = {{"Tup", "SamplePeriodInterval19"}};
    StateTransition SamplePeriod20_Cond[1] = {{"Tup", "SamplePeriodInterval20"}};
    StateTransition SamplePeriod21_Cond[1] = {{"Tup", "SamplePeriodInterval21"}};
    StateTransition SamplePeriod22_Cond[1] = {{"Tup", "SamplePeriodInterval22"}};
    StateTransition SamplePeriod23_Cond[1] = {{"Tup", "SamplePeriodInterval23"}};
    StateTransition SamplePeriod24_Cond[1] = {{"Tup", "SamplePeriodInterval24"}};
    StateTransition SamplePeriod25_Cond[1] = {{"Tup", "SamplePeriodInterval25"}};
    StateTransition SamplePeriod26_Cond[1] = {{"Tup", "SamplePeriodInterval26"}};
    StateTransition SamplePeriod27_Cond[1] = {{"Tup", "SamplePeriodInterval27"}};
    StateTransition SamplePeriod28_Cond[1] = {{"Tup", "SamplePeriodInterval28"}};
    StateTransition SamplePeriod29_Cond[1] = {{"Tup", "SamplePeriodInterval29"}};
    StateTransition SamplePeriod30_Cond[1] = {{"Tup", "SamplePeriodInterval30"}};
    StateTransition SamplePeriod31_Cond[1] = {{"Tup", "SamplePeriodInterval31"}};
    StateTransition SamplePeriod32_Cond[1] = {{"Tup", "SamplePeriodInterval32"}};
    StateTransition SamplePeriod33_Cond[1] = {{"Tup", "SamplePeriodInterval33"}};
    StateTransition SamplePeriod34_Cond[1] = {{"Tup", "SamplePeriodInterval34"}};
    StateTransition SamplePeriod35_Cond[1] = {{"Tup", "SamplePeriodInterval35"}};
    StateTransition SamplePeriod36_Cond[1] = {{"Tup", "SamplePeriodInterval36"}};
    StateTransition SamplePeriod37_Cond[1] = {{"Tup", "SamplePeriodInterval37"}};
    StateTransition SamplePeriod38_Cond[1] = {{"Tup", "SamplePeriodInterval38"}};
    StateTransition SamplePeriod39_Cond[1] = {{"Tup", "SamplePeriodInterval39"}};

    StateTransition SamplePeriodInterval0_Cond[1]      = {{"Tup" ,"SamplePeriod1"}};
		StateTransition SamplePeriodInterval1_Cond[1]      = {{"Tup" ,"SamplePeriod2"}};
		StateTransition SamplePeriodInterval2_Cond[1]      = {{"Tup" ,"SamplePeriod3"}};
		StateTransition SamplePeriodInterval3_Cond[1]      = {{"Tup" ,"SamplePeriod4"}};
		StateTransition SamplePeriodInterval4_Cond[1]      = {{"Tup" ,"SamplePeriod5"}};
		StateTransition SamplePeriodInterval5_Cond[1]      = {{"Tup" ,"SamplePeriod6"}};
		StateTransition SamplePeriodInterval6_Cond[1]      = {{"Tup" ,"SamplePeriod7"}};
		StateTransition SamplePeriodInterval7_Cond[1]      = {{"Tup" ,"SamplePeriod8"}};
		StateTransition SamplePeriodInterval8_Cond[1]      = {{"Tup" ,"SamplePeriod9"}};
		StateTransition SamplePeriodInterval9_Cond[1]      = {{"Tup" ,"SamplePeriod10"}};
		StateTransition SamplePeriodInterval10_Cond[1]      = {{"Tup" ,"SamplePeriod11"}};
		StateTransition SamplePeriodInterval11_Cond[1]      = {{"Tup" ,"SamplePeriod12"}};
		StateTransition SamplePeriodInterval12_Cond[1]      = {{"Tup" ,"SamplePeriod13"}};
		StateTransition SamplePeriodInterval13_Cond[1]      = {{"Tup" ,"SamplePeriod14"}};
		StateTransition SamplePeriodInterval14_Cond[1]      = {{"Tup" ,"SamplePeriod15"}};
		StateTransition SamplePeriodInterval15_Cond[1]      = {{"Tup" ,"SamplePeriod16"}};
		StateTransition SamplePeriodInterval16_Cond[1]      = {{"Tup" ,"SamplePeriod17"}};
		StateTransition SamplePeriodInterval17_Cond[1]      = {{"Tup" ,"SamplePeriod18"}};
		StateTransition SamplePeriodInterval18_Cond[1]      = {{"Tup" ,"SamplePeriod19"}};
		StateTransition SamplePeriodInterval19_Cond[1]      = {{"Tup" ,"SamplePeriod20"}};
		StateTransition SamplePeriodInterval20_Cond[1]      = {{"Tup" ,"SamplePeriod21"}};
		StateTransition SamplePeriodInterval21_Cond[1]      = {{"Tup" ,"SamplePeriod22"}};
		StateTransition SamplePeriodInterval22_Cond[1]      = {{"Tup" ,"SamplePeriod23"}};
		StateTransition SamplePeriodInterval23_Cond[1]      = {{"Tup" ,"SamplePeriod24"}};
		StateTransition SamplePeriodInterval24_Cond[1]      = {{"Tup" ,"SamplePeriod25"}};
		StateTransition SamplePeriodInterval25_Cond[1]      = {{"Tup" ,"SamplePeriod26"}};
		StateTransition SamplePeriodInterval26_Cond[1]      = {{"Tup" ,"SamplePeriod27"}};
		StateTransition SamplePeriodInterval27_Cond[1]      = {{"Tup" ,"SamplePeriod28"}};
		StateTransition SamplePeriodInterval28_Cond[1]      = {{"Tup" ,"SamplePeriod29"}};
		StateTransition SamplePeriodInterval29_Cond[1]      = {{"Tup" ,"SamplePeriod30"}};
		StateTransition SamplePeriodInterval30_Cond[1]      = {{"Tup" ,"SamplePeriod31"}};
		StateTransition SamplePeriodInterval31_Cond[1]      = {{"Tup" ,"SamplePeriod32"}};
		StateTransition SamplePeriodInterval32_Cond[1]      = {{"Tup" ,"SamplePeriod33"}};
		StateTransition SamplePeriodInterval33_Cond[1]      = {{"Tup" ,"SamplePeriod34"}};
		StateTransition SamplePeriodInterval34_Cond[1]      = {{"Tup" ,"SamplePeriod35"}};
		StateTransition SamplePeriodInterval35_Cond[1]      = {{"Tup" ,"SamplePeriod36"}};
		StateTransition SamplePeriodInterval36_Cond[1]      = {{"Tup" ,"SamplePeriod37"}};
		StateTransition SamplePeriodInterval37_Cond[1]      = {{"Tup" ,"SamplePeriod38"}};
		StateTransition SamplePeriodInterval38_Cond[1]      = {{"Tup" ,"SamplePeriod39"}};
		StateTransition SamplePeriodInterval39_Cond[1]      = {{"Tup" ,"DelayPeriod"}};

    StateTransition PreCueDelayPeriod_Cond[1] = {{"Tup", "SamplePeriod0"}};
    StateTransition TrialStart_Cond[1] = {{"Tup", "PreCueDelayPeriod"}}; //

    StateTransition EarlyLickDelay_Cond[1] = {{"Tup", "DelayPeriod"}}; 
    StateTransition DelayPeriod_Cond[3] = {{"Tup", ActionAfterDelay} ,{"Lick1In" ,"EarlyLickDelay"} ,{"Lick2In" ,"EarlyLickDelay"}};
    StateTransition ResponseCue_Cond[1] = {{"Tup", "AnswerPeriod"}};
    StateTransition GiveFreeDrop_Cond[1] = {{"Tup", "ResponseCue"}};
    StateTransition AnswerPeriod_Cond[3] = {{"Lick1In", LeftLickAction}, {"Lick2In", RightLickAction}, {"Tup", "NoResponse"}};
    StateTransition Reward_Cond[1] = {{"Tup", "RewardConsumption"}};
    StateTransition Tup_Exit_Cond[1] = {{"Tup", "exit"}};
    StateTransition NoResponse_Cond[3] = {{"Lick1In", "exit"}, {"Lick2In", "exit"}, {"Tup", "exit"}};
    StateTransition ErrorTrial_Cond[1] = {{"Tup", "TimeOut"}};
    StateTransition Tup_StopLicking_Cond[1] = {{"Tup", "StopLicking"}};
    StateTransition StopLicking_Cond[3] = {{"Lick1In", "StopLickingReturn"}, {"Lick2In", "StopLickingReturn"}, {"Tup", "exit"}}; //

    OutputAction FlashOutputL[3] = {SampleOutputBackgroundL, SampleOutputL, DelayNoOutput};
    OutputAction FlashOutputR[3] = {SampleOutputBackgroundR, SampleOutputR, DelayNoOutput};
    OutputAction AudOutput[3] = {AudSampleOutputBackground, AudSampleOutput, DelayNoOutput};
    // flash or click stimulus outputaction
    // set the distribution of flashes
    OutputAction SamplePeriod0_Output[3] = {FlashOutputL[BinaryFlash[0]], FlashOutputR[BinaryFlash[0]], AudOutput[BinaryClick[0]]};
    OutputAction SamplePeriod1_Output[3] = {FlashOutputL[BinaryFlash[1]], FlashOutputR[BinaryFlash[1]], AudOutput[BinaryClick[1]]};
    OutputAction SamplePeriod2_Output[3] = {FlashOutputL[BinaryFlash[2]], FlashOutputR[BinaryFlash[2]], AudOutput[BinaryClick[2]]};
    OutputAction SamplePeriod3_Output[3] = {FlashOutputL[BinaryFlash[3]], FlashOutputR[BinaryFlash[3]], AudOutput[BinaryClick[3]]};
    OutputAction SamplePeriod4_Output[3] = {FlashOutputL[BinaryFlash[4]], FlashOutputR[BinaryFlash[4]], AudOutput[BinaryClick[4]]};
    OutputAction SamplePeriod5_Output[3] = {FlashOutputL[BinaryFlash[5]], FlashOutputR[BinaryFlash[5]], AudOutput[BinaryClick[5]]};
    OutputAction SamplePeriod6_Output[3] = {FlashOutputL[BinaryFlash[6]], FlashOutputR[BinaryFlash[6]], AudOutput[BinaryClick[6]]};
    OutputAction SamplePeriod7_Output[3] = {FlashOutputL[BinaryFlash[7]], FlashOutputR[BinaryFlash[7]], AudOutput[BinaryClick[7]]};
    OutputAction SamplePeriod8_Output[3] = {FlashOutputL[BinaryFlash[8]], FlashOutputR[BinaryFlash[8]], AudOutput[BinaryClick[8]]};
    OutputAction SamplePeriod9_Output[3] = {FlashOutputL[BinaryFlash[9]], FlashOutputR[BinaryFlash[9]], AudOutput[BinaryClick[9]]};
    OutputAction SamplePeriod10_Output[3] = {FlashOutputL[BinaryFlash[10]], FlashOutputR[BinaryFlash[10]], AudOutput[BinaryClick[10]]};
    OutputAction SamplePeriod11_Output[3] = {FlashOutputL[BinaryFlash[11]], FlashOutputR[BinaryFlash[11]], AudOutput[BinaryClick[11]]};
    OutputAction SamplePeriod12_Output[3] = {FlashOutputL[BinaryFlash[12]], FlashOutputR[BinaryFlash[12]], AudOutput[BinaryClick[12]]};
    OutputAction SamplePeriod13_Output[3] = {FlashOutputL[BinaryFlash[13]], FlashOutputR[BinaryFlash[13]], AudOutput[BinaryClick[13]]};
    OutputAction SamplePeriod14_Output[3] = {FlashOutputL[BinaryFlash[14]], FlashOutputR[BinaryFlash[14]], AudOutput[BinaryClick[14]]};
    OutputAction SamplePeriod15_Output[3] = {FlashOutputL[BinaryFlash[15]], FlashOutputR[BinaryFlash[15]], AudOutput[BinaryClick[15]]};
    OutputAction SamplePeriod16_Output[3] = {FlashOutputL[BinaryFlash[16]], FlashOutputR[BinaryFlash[16]], AudOutput[BinaryClick[16]]};
    OutputAction SamplePeriod17_Output[3] = {FlashOutputL[BinaryFlash[17]], FlashOutputR[BinaryFlash[17]], AudOutput[BinaryClick[17]]};
    OutputAction SamplePeriod18_Output[3] = {FlashOutputL[BinaryFlash[18]], FlashOutputR[BinaryFlash[18]], AudOutput[BinaryClick[18]]};
    OutputAction SamplePeriod19_Output[3] = {FlashOutputL[BinaryFlash[19]], FlashOutputR[BinaryFlash[19]], AudOutput[BinaryClick[19]]};
    OutputAction SamplePeriod20_Output[3] = {FlashOutputL[BinaryFlash[20]], FlashOutputR[BinaryFlash[20]], AudOutput[BinaryClick[20]]};
    OutputAction SamplePeriod21_Output[3] = {FlashOutputL[BinaryFlash[21]], FlashOutputR[BinaryFlash[21]], AudOutput[BinaryClick[21]]};
    OutputAction SamplePeriod22_Output[3] = {FlashOutputL[BinaryFlash[22]], FlashOutputR[BinaryFlash[22]], AudOutput[BinaryClick[22]]};
    OutputAction SamplePeriod23_Output[3] = {FlashOutputL[BinaryFlash[23]], FlashOutputR[BinaryFlash[23]], AudOutput[BinaryClick[23]]};
    OutputAction SamplePeriod24_Output[3] = {FlashOutputL[BinaryFlash[24]], FlashOutputR[BinaryFlash[24]], AudOutput[BinaryClick[24]]};
    OutputAction SamplePeriod25_Output[3] = {FlashOutputL[BinaryFlash[25]], FlashOutputR[BinaryFlash[25]], AudOutput[BinaryClick[25]]};
    OutputAction SamplePeriod26_Output[3] = {FlashOutputL[BinaryFlash[26]], FlashOutputR[BinaryFlash[26]], AudOutput[BinaryClick[26]]};
    OutputAction SamplePeriod27_Output[3] = {FlashOutputL[BinaryFlash[27]], FlashOutputR[BinaryFlash[27]], AudOutput[BinaryClick[27]]};
    OutputAction SamplePeriod28_Output[3] = {FlashOutputL[BinaryFlash[28]], FlashOutputR[BinaryFlash[28]], AudOutput[BinaryClick[28]]};
    OutputAction SamplePeriod29_Output[3] = {FlashOutputL[BinaryFlash[29]], FlashOutputR[BinaryFlash[29]], AudOutput[BinaryClick[29]]};
    OutputAction SamplePeriod30_Output[3] = {FlashOutputL[BinaryFlash[30]], FlashOutputR[BinaryFlash[30]], AudOutput[BinaryClick[30]]};
    OutputAction SamplePeriod31_Output[3] = {FlashOutputL[BinaryFlash[31]], FlashOutputR[BinaryFlash[31]], AudOutput[BinaryClick[31]]};
    OutputAction SamplePeriod32_Output[3] = {FlashOutputL[BinaryFlash[32]], FlashOutputR[BinaryFlash[32]], AudOutput[BinaryClick[32]]};
    OutputAction SamplePeriod33_Output[3] = {FlashOutputL[BinaryFlash[33]], FlashOutputR[BinaryFlash[33]], AudOutput[BinaryClick[33]]};
    OutputAction SamplePeriod34_Output[3] = {FlashOutputL[BinaryFlash[34]], FlashOutputR[BinaryFlash[34]], AudOutput[BinaryClick[34]]};
    OutputAction SamplePeriod35_Output[3] = {FlashOutputL[BinaryFlash[35]], FlashOutputR[BinaryFlash[35]], AudOutput[BinaryClick[35]]};
    OutputAction SamplePeriod36_Output[3] = {FlashOutputL[BinaryFlash[36]], FlashOutputR[BinaryFlash[36]], AudOutput[BinaryClick[36]]};
    OutputAction SamplePeriod37_Output[3] = {FlashOutputL[BinaryFlash[37]], FlashOutputR[BinaryFlash[37]], AudOutput[BinaryClick[37]]};
    OutputAction SamplePeriod38_Output[3] = {FlashOutputL[BinaryFlash[38]], FlashOutputR[BinaryFlash[38]], AudOutput[BinaryClick[38]]};
    OutputAction SamplePeriod39_Output[3] = {FlashOutputL[BinaryFlash[39]], FlashOutputR[BinaryFlash[39]], AudOutput[BinaryClick[39]]};

    // OutputAction Sample_Output[1]     	   = {SampleOutput};
    OutputAction PreCueDelayPeriod_Output[1] = {GreenLightOutputBackground}; // low intensity green light as a pre cue
    OutputAction ResponseCue_Output[1] = {CueOutput};
    OutputAction Reward_Output[1] = {RewardOutput};
    OutputAction NoOutput[0] = {};
    OutputAction ErrorOutput[1] = {NoiseOutput};
    OutputAction DelayPeriod_Output[1] = {DelayNoOutput}; // red light

    gpSMART_State states[94] = {};
    states[0] = smart.CreateState("TrialStart", 5, 1, TrialStart_Cond, 0, NoOutput); // msec
    states[2] = smart.CreateState("DelayPeriod", float(S.DelayPeriod), 3, DelayPeriod_Cond, 0, NoOutput);
    states[3] = smart.CreateState("ResponseCue", 100, 1, ResponseCue_Cond, 1, ResponseCue_Output);
    states[4] = smart.CreateState("GiveFreeDrop", reward_dur, 1, GiveFreeDrop_Cond, 1, Reward_Output);
    states[5] = smart.CreateState("AnswerPeriod", S.AnswerPeriod, 3, AnswerPeriod_Cond, 0, NoOutput);
    states[6] = smart.CreateState("Reward", reward_dur, 1, Reward_Cond, 1, Reward_Output);
    states[7] = smart.CreateState("RewardConsumption", S.ConsumptionPeriod, 1, Tup_StopLicking_Cond, 0, NoOutput);
    states[8] = smart.CreateState("NoResponse", random(30, 60) * 60 * 1000, 3, NoResponse_Cond, 0, NoOutput); // 1-hr: 60*60*1000 msec
    states[9] = smart.CreateState("ErrorTrial", 500, 1, ErrorTrial_Cond, 1, ErrorOutput);                     //
    states[10] = smart.CreateState("TimeOut", S.TimeOut + S.extra_TimeOut, 1, Tup_Exit_Cond, 0, NoOutput);
    states[11] = smart.CreateState("EarlyLickDelay", S.EarlyLickPeriod, 1, EarlyLickDelay_Cond, 0, NoOutput); // not used
    states[12] = smart.CreateState("StopLicking", S.StopLickingPeriod, 3, StopLicking_Cond, 0, NoOutput);
    states[13] = smart.CreateState("StopLickingReturn", 8, 1, Tup_StopLicking_Cond, 0, NoOutput);
    // flashes bin
    states[1] = smart.CreateState("SamplePeriod0", PulseTime * sample_bin, 1, SamplePeriod0_Cond, 3, SamplePeriod0_Output);
    states[14] = smart.CreateState("SamplePeriod1", PulseTime * sample_bin, 1, SamplePeriod1_Cond, 3, SamplePeriod1_Output);
    states[15] = smart.CreateState("SamplePeriod2", PulseTime * sample_bin, 1, SamplePeriod2_Cond, 3, SamplePeriod2_Output);
    states[16] = smart.CreateState("SamplePeriod3", PulseTime * sample_bin, 1, SamplePeriod3_Cond, 3, SamplePeriod3_Output);
    states[17] = smart.CreateState("SamplePeriod4", PulseTime * sample_bin, 1, SamplePeriod4_Cond, 3, SamplePeriod4_Output);
    states[18] = smart.CreateState("SamplePeriod5", PulseTime * sample_bin, 1, SamplePeriod5_Cond, 3, SamplePeriod5_Output);
    states[19] = smart.CreateState("SamplePeriod6", PulseTime * sample_bin, 1, SamplePeriod6_Cond, 3, SamplePeriod6_Output);
    states[20] = smart.CreateState("SamplePeriod7", PulseTime * sample_bin, 1, SamplePeriod7_Cond, 3, SamplePeriod7_Output);
    states[21] = smart.CreateState("SamplePeriod8", PulseTime * sample_bin, 1, SamplePeriod8_Cond, 3, SamplePeriod8_Output);
    states[22] = smart.CreateState("SamplePeriod9", PulseTime * sample_bin, 1, SamplePeriod9_Cond, 3, SamplePeriod9_Output);
    states[23] = smart.CreateState("SamplePeriod10", PulseTime * sample_bin, 1, SamplePeriod10_Cond, 3, SamplePeriod10_Output);
    states[24] = smart.CreateState("SamplePeriod11", PulseTime * sample_bin, 1, SamplePeriod11_Cond, 3, SamplePeriod11_Output);
    states[25] = smart.CreateState("SamplePeriod12", PulseTime * sample_bin, 1, SamplePeriod12_Cond, 3, SamplePeriod12_Output);
    states[26] = smart.CreateState("SamplePeriod13", PulseTime * sample_bin, 1, SamplePeriod13_Cond, 3, SamplePeriod13_Output);
    states[27] = smart.CreateState("SamplePeriod14", PulseTime * sample_bin, 1, SamplePeriod14_Cond, 3, SamplePeriod14_Output);
    states[28] = smart.CreateState("SamplePeriod15", PulseTime * sample_bin, 1, SamplePeriod15_Cond, 3, SamplePeriod15_Output);
    states[29] = smart.CreateState("SamplePeriod16", PulseTime * sample_bin, 1, SamplePeriod16_Cond, 3, SamplePeriod16_Output);
    states[30] = smart.CreateState("SamplePeriod17", PulseTime * sample_bin, 1, SamplePeriod17_Cond, 3, SamplePeriod17_Output);
    states[31] = smart.CreateState("SamplePeriod18", PulseTime * sample_bin, 1, SamplePeriod18_Cond, 3, SamplePeriod18_Output);
    states[32] = smart.CreateState("SamplePeriod19", PulseTime * sample_bin, 1, SamplePeriod19_Cond, 3, SamplePeriod19_Output);
    states[33] = smart.CreateState("SamplePeriod20", PulseTime * sample_bin, 1, SamplePeriod20_Cond, 3, SamplePeriod20_Output);
    states[34] = smart.CreateState("SamplePeriod21", PulseTime * sample_bin, 1, SamplePeriod21_Cond, 3, SamplePeriod21_Output);
    states[35] = smart.CreateState("SamplePeriod22", PulseTime * sample_bin, 1, SamplePeriod22_Cond, 3, SamplePeriod22_Output);
    states[36] = smart.CreateState("SamplePeriod23", PulseTime * sample_bin, 1, SamplePeriod23_Cond, 3, SamplePeriod23_Output);
    states[37] = smart.CreateState("SamplePeriod24", PulseTime * sample_bin, 1, SamplePeriod24_Cond, 3, SamplePeriod24_Output);
    states[38] = smart.CreateState("SamplePeriod25", PulseTime * sample_bin, 1, SamplePeriod25_Cond, 3, SamplePeriod25_Output);
    states[39] = smart.CreateState("SamplePeriod26", PulseTime * sample_bin, 1, SamplePeriod26_Cond, 3, SamplePeriod26_Output);
    states[40] = smart.CreateState("SamplePeriod27", PulseTime * sample_bin, 1, SamplePeriod27_Cond, 3, SamplePeriod27_Output);
    states[41] = smart.CreateState("SamplePeriod28", PulseTime * sample_bin, 1, SamplePeriod28_Cond, 3, SamplePeriod28_Output);
    states[42] = smart.CreateState("SamplePeriod29", PulseTime * sample_bin, 1, SamplePeriod29_Cond, 3, SamplePeriod29_Output);
    states[43] = smart.CreateState("SamplePeriod30", PulseTime * sample_bin, 1, SamplePeriod30_Cond, 3, SamplePeriod30_Output);
    states[44] = smart.CreateState("SamplePeriod31", PulseTime * sample_bin, 1, SamplePeriod31_Cond, 3, SamplePeriod31_Output);
    states[45] = smart.CreateState("SamplePeriod32", PulseTime * sample_bin, 1, SamplePeriod32_Cond, 3, SamplePeriod32_Output);
    states[46] = smart.CreateState("SamplePeriod33", PulseTime * sample_bin, 1, SamplePeriod33_Cond, 3, SamplePeriod33_Output);
    states[47] = smart.CreateState("SamplePeriod34", PulseTime * sample_bin, 1, SamplePeriod34_Cond, 3, SamplePeriod34_Output);
    states[48] = smart.CreateState("SamplePeriod35", PulseTime * sample_bin, 1, SamplePeriod35_Cond, 3, SamplePeriod35_Output);
    states[49] = smart.CreateState("SamplePeriod36", PulseTime * sample_bin, 1, SamplePeriod36_Cond, 3, SamplePeriod36_Output);
    states[50] = smart.CreateState("SamplePeriod37", PulseTime * sample_bin, 1, SamplePeriod37_Cond, 3, SamplePeriod37_Output);
    states[51] = smart.CreateState("SamplePeriod38", PulseTime * sample_bin, 1, SamplePeriod38_Cond, 3, SamplePeriod38_Output);
    states[52] = smart.CreateState("SamplePeriod39", PulseTime * sample_bin, 1, SamplePeriod39_Cond, 3, SamplePeriod39_Output);

    states[53] = smart.CreateState("SamplePeriodInterval0", IntervalPulse * sample_bin, 1, SamplePeriodInterval0_Cond, 0,   NoOutput);
    states[54] = smart.CreateState("SamplePeriodInterval1", IntervalPulse * sample_bin, 1, SamplePeriodInterval1_Cond, 0,   NoOutput);
    states[55] = smart.CreateState("SamplePeriodInterval2", IntervalPulse * sample_bin, 1, SamplePeriodInterval2_Cond, 0,   NoOutput);
    states[56] = smart.CreateState("SamplePeriodInterval3", IntervalPulse * sample_bin, 1, SamplePeriodInterval3_Cond, 0,   NoOutput);
    states[57] = smart.CreateState("SamplePeriodInterval4", IntervalPulse * sample_bin, 1, SamplePeriodInterval4_Cond, 0,   NoOutput);
    states[58] = smart.CreateState("SamplePeriodInterval5", IntervalPulse * sample_bin, 1, SamplePeriodInterval5_Cond, 0,   NoOutput);
    states[59] = smart.CreateState("SamplePeriodInterval6", IntervalPulse * sample_bin, 1, SamplePeriodInterval6_Cond, 0,   NoOutput);
    states[60] = smart.CreateState("SamplePeriodInterval7", IntervalPulse * sample_bin, 1, SamplePeriodInterval7_Cond, 0,   NoOutput);
    states[61] = smart.CreateState("SamplePeriodInterval8", IntervalPulse * sample_bin, 1, SamplePeriodInterval8_Cond, 0,   NoOutput);
    states[62] = smart.CreateState("SamplePeriodInterval9", IntervalPulse * sample_bin, 1, SamplePeriodInterval9_Cond, 0,   NoOutput);
    states[63] = smart.CreateState("SamplePeriodInterval10", IntervalPulse * sample_bin, 1, SamplePeriodInterval10_Cond, 0, NoOutput);
    states[64] = smart.CreateState("SamplePeriodInterval11", IntervalPulse * sample_bin, 1, SamplePeriodInterval11_Cond, 0, NoOutput);
    states[65] = smart.CreateState("SamplePeriodInterval12", IntervalPulse * sample_bin, 1, SamplePeriodInterval12_Cond, 0, NoOutput);
    states[66] = smart.CreateState("SamplePeriodInterval13", IntervalPulse * sample_bin, 1, SamplePeriodInterval13_Cond, 0, NoOutput);
    states[67] = smart.CreateState("SamplePeriodInterval14", IntervalPulse * sample_bin, 1, SamplePeriodInterval14_Cond, 0, NoOutput);
    states[68] = smart.CreateState("SamplePeriodInterval15", IntervalPulse * sample_bin, 1, SamplePeriodInterval15_Cond, 0, NoOutput);
    states[69] = smart.CreateState("SamplePeriodInterval16", IntervalPulse * sample_bin, 1, SamplePeriodInterval16_Cond, 0, NoOutput);
    states[70] = smart.CreateState("SamplePeriodInterval17", IntervalPulse * sample_bin, 1, SamplePeriodInterval17_Cond, 0, NoOutput);
    states[71] = smart.CreateState("SamplePeriodInterval18", IntervalPulse * sample_bin, 1, SamplePeriodInterval18_Cond, 0, NoOutput);
    states[72] = smart.CreateState("SamplePeriodInterval19", IntervalPulse * sample_bin, 1, SamplePeriodInterval19_Cond, 0, NoOutput);
    states[73] = smart.CreateState("SamplePeriodInterval20", IntervalPulse * sample_bin, 1, SamplePeriodInterval20_Cond, 0, NoOutput);
    states[74] = smart.CreateState("SamplePeriodInterval21", IntervalPulse * sample_bin, 1, SamplePeriodInterval21_Cond, 0, NoOutput);
    states[75] = smart.CreateState("SamplePeriodInterval22", IntervalPulse * sample_bin, 1, SamplePeriodInterval22_Cond, 0, NoOutput);
    states[76] = smart.CreateState("SamplePeriodInterval23", IntervalPulse * sample_bin, 1, SamplePeriodInterval23_Cond, 0, NoOutput);
    states[77] = smart.CreateState("SamplePeriodInterval24", IntervalPulse * sample_bin, 1, SamplePeriodInterval24_Cond, 0, NoOutput);
    states[78] = smart.CreateState("SamplePeriodInterval25", IntervalPulse * sample_bin, 1, SamplePeriodInterval25_Cond, 0, NoOutput);
    states[79] = smart.CreateState("SamplePeriodInterval26", IntervalPulse * sample_bin, 1, SamplePeriodInterval26_Cond, 0, NoOutput);
    states[80] = smart.CreateState("SamplePeriodInterval27", IntervalPulse * sample_bin, 1, SamplePeriodInterval27_Cond, 0, NoOutput);
    states[81] = smart.CreateState("SamplePeriodInterval28", IntervalPulse * sample_bin, 1, SamplePeriodInterval28_Cond, 0, NoOutput);
    states[82] = smart.CreateState("SamplePeriodInterval29", IntervalPulse * sample_bin, 1, SamplePeriodInterval29_Cond, 0, NoOutput);
    states[83] = smart.CreateState("SamplePeriodInterval30", IntervalPulse * sample_bin, 1, SamplePeriodInterval30_Cond, 0, NoOutput);
    states[84] = smart.CreateState("SamplePeriodInterval31", IntervalPulse * sample_bin, 1, SamplePeriodInterval31_Cond, 0, NoOutput);
    states[85] = smart.CreateState("SamplePeriodInterval32", IntervalPulse * sample_bin, 1, SamplePeriodInterval32_Cond, 0, NoOutput);
    states[86] = smart.CreateState("SamplePeriodInterval33", IntervalPulse * sample_bin, 1, SamplePeriodInterval33_Cond, 0, NoOutput);
    states[87] = smart.CreateState("SamplePeriodInterval34", IntervalPulse * sample_bin, 1, SamplePeriodInterval34_Cond, 0, NoOutput);
    states[88] = smart.CreateState("SamplePeriodInterval35", IntervalPulse * sample_bin, 1, SamplePeriodInterval35_Cond, 0, NoOutput);
    states[89] = smart.CreateState("SamplePeriodInterval36", IntervalPulse * sample_bin, 1, SamplePeriodInterval36_Cond, 0, NoOutput);
    states[90] = smart.CreateState("SamplePeriodInterval37", IntervalPulse * sample_bin, 1, SamplePeriodInterval37_Cond, 0, NoOutput);
    states[91] = smart.CreateState("SamplePeriodInterval38", IntervalPulse * sample_bin, 1, SamplePeriodInterval38_Cond, 0, NoOutput);
    states[92] = smart.CreateState("SamplePeriodInterval39", IntervalPulse * sample_bin, 1, SamplePeriodInterval39_Cond, 0, NoOutput);
    // pre-cue delay
    states[93] = smart.CreateState("PreCueDelayPeriod", S.PreCueDelayPeriod, 1, PreCueDelayPeriod_Cond, 1, PreCueDelayPeriod_Output);

    // Predefine State sequence.
    for (int i = 0; i < 94; i++)
    {
      smart.AddBlankState(states[i].Name);
    }
    // Add a state to state machine.
    for (int i = 0; i < 94; i++)
    {
      smart.AddState(&states[i]);
    }
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
  if (S.currProtocolIndex >= 0)
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


void curriculumChangeProtocol()
{
  // 初始化超参数 ，从sd文件中读取现在的一些超参数
  // read_Curriculum_Param_info();
  // training loop
  if(!CP.evaFlag){
    if (TrialOutcome != 0 && TrialOutcome != 3){
      CP.currHistory_step++;
      Serial.print("Training step : ");
      Serial.print(TrialOutcome);
      Serial.print(" ");
      Serial.print(CP.CurrTrialModality);
      Serial.print(" ");
      Serial.print(CP.CurrTrialContrast);
      Serial.print(" ");
      Serial.println(CP.CurrTrialLightIntensity);
      // including the EarlyLick trials
    }
  }else{
    // eva loop
    if (TrialOutcome != 0 && TrialOutcome != 3 && is_earlylick == 0){
      Serial.print("Eva step : ");
      Serial.print(TrialOutcome);
      Serial.print(" ");
      Serial.print(CP.CurrTrialModality);
      Serial.print(" ");
      Serial.print(CP.CurrTrialContrast);
      Serial.print(" ");
      Serial.println(CP.CurrTrialLightIntensity);
      if(TrialOutcome == 1){
        CP.EvaPerf[CP.currEvaTaskIndex]++;
      }

      CP.currEvaNumIndex++;
      if(CP.currEvaNumIndex >= CP.EvaNum_per_task){
        CP.currEvaNumIndex = 0;
        CP.currEvaTaskIndex++;
      }

      if(CP.currEvaTaskIndex >= CP.subtaskNum){
        CP.currEvaTaskIndex = 0;
        CP.evaFlag = false;
        // select the trial structure with the greatest ALP value
        int maxQIndex = 0;
        for(int i=0;i<CP.subtaskNum;i++){
          CP.ALP[i] = abs(CP.EvaPerf[i] - CP.LastEvaPerf[i]);
          // update the lastEvaPerf
          CP.LastEvaPerf[i] = CP.EvaPerf[i];
          // find the max ALP index
          if(CP.ALP[i] > CP.ALP[maxQIndex]){
            maxQIndex = i;
          }
        }
        CP.CurrTrialModality = trialParamsSpace[maxQIndex][0];
        CP.CurrTrialContrast = trialParamsSpace[maxQIndex][1];
        CP.CurrTrialLightIntensity = trialParamsSpace[maxQIndex][2];
        // update the TargetTaskCounter
        for(int j=0;j<CP.subtaskNum;j++){
          if(CP.EvaPerf[j] >= int(CP.EvaNum_per_task * 0.8)){ // 80% performance
          CP.TargetTaskCounter[j]++;
        }
        }
        write_Curriculum_model_info(); // only recording the result of every evalution step.
        // t equal to 1 is the init evalution of total task
        CP.t++; // training step as the onset of a TS loop
        Serial.print("Eva End : ");
      Serial.print(TrialOutcome);
      Serial.print(" ");
      Serial.print(CP.CurrTrialModality);
      Serial.print(" ");
      Serial.print(CP.CurrTrialContrast);
      Serial.print(" ");
      Serial.println(CP.CurrTrialLightIntensity);
      }else{
        // get the trial structure
        CP.CurrTrialModality = trialParamsSpace[CP.currEvaTaskIndex][0];
        CP.CurrTrialContrast = trialParamsSpace[CP.currEvaTaskIndex][1];
        CP.CurrTrialLightIntensity = trialParamsSpace[CP.currEvaTaskIndex][2];
      }
    }
  }

  // determine if enter evaluation step
  if(CP.currHistory_step >= CP.updateALP_num){
    CP.currHistory_step = 0;
    CP.evaFlag = true;
    // tare the EvaPerf
    for(int i=0;i<CP.subtaskNum;i++){
      CP.EvaPerf[i] = 0;
    }
    // init the first eva trial
    CP.CurrTrialModality = trialParamsSpace[CP.currEvaTaskIndex][0];
    CP.CurrTrialContrast = trialParamsSpace[CP.currEvaTaskIndex][1];
    CP.CurrTrialLightIntensity = trialParamsSpace[CP.currEvaTaskIndex][2];
  }
  // update the params according to the CurrTrialProtocolParams and CurrTrialOtherParams
  // trial structure space
  // 第五步： 将选择的参数记录到txt文件中
  // write_Curriculum_Param_info();
}


void autoChangeProtocol()
{
  switch (S.currProtocolIndex)
  {
  case 0:
    if (S.currProtocolTrials > 1000 && Perf100 > 75)
    // if (S.currProtocolTrials > 2)
    {                          // 500 auditory + 500 visual + 500 muti-modalities
      S.currProtocolIndex = 1; // Advanced to Protocol 1: pattern left/right
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 1000;
      // S.DelayPeriod         = 500;
      S.TimeOut = 2000;
      S.AnswerPeriod = 5000;
      S.ConsumptionPeriod = 750;
      // S.StopLickingPeriod   = 1000;
      S.EarlyLickPeriod     = 100;
      S.TrialPresentMode = 0; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 1:
    if (S.currProtocolTrials > 100 && S.currProtocolPerf > 75)
    // if (S.currProtocolTrials > 2)
    {
      S.currProtocolIndex = 2; // Advanced to Protocol 2: antiBias left/right
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 1000;
      // S.DelayPeriod         = 500;
      S.TimeOut = 4000;
      S.AnswerPeriod = 3000;
      S.ConsumptionPeriod = 750;
      // S.StopLickingPeriod   = 1000;
      S.EarlyLickPeriod     = 100;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 2:
    if (S.currProtocolTrials > 100 && Perf100 > 75)
    // if (S.currProtocolTrials > 2)
    {
      S.currProtocolIndex = 3; // Advanced to Protocol 3: antiBias left/right w/ forced delay 0.5 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 1000;
      // S.DelayPeriod         = 500;
      S.TimeOut = 4000;
      S.AnswerPeriod = 3000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 500;
      S.EarlyLickPeriod     = 100;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 3:
    if (S.currProtocolTrials > 100 && S.currProtocolPerf > 75)
    // if (S.currProtocolTrials > 2)
    {
      S.currProtocolIndex = 4; // Advanced to Protocol 4: antiBias left/right w/ forced delay 0.8 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 1000;
      // S.DelayPeriod         = 800;
      S.TimeOut = 6000;
      S.AnswerPeriod = 3000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 800;
      S.EarlyLickPeriod     = 150;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 4:
    if (S.currProtocolTrials > 100 && S.currProtocolPerf > 75)
    // if (S.currProtocolTrials > 2)
    {
      S.currProtocolIndex = 5; // Advanced to Protocol 5: antiBias left/right w/ forced delay 1.0 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 1000;
      // S.DelayPeriod         = 1000;
      S.TimeOut = 6000;
      S.AnswerPeriod = 2000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 800;
      S.EarlyLickPeriod     = 200;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 5:
    if (S.currProtocolTrials > 100 && S.currProtocolPerf > 75)
    // if (S.currProtocolTrials > 2)
    {
      S.currProtocolIndex = 6; // Advanced to Protocol 6: antiBias left/right w/ forced delay 1.2 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 1000;
      // S.DelayPeriod         = 1200;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod     = 250;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 6:
    if (S.currProtocolTrials > 100 && Perf100 > 75)
    // if (S.currProtocolTrials > 2)
    {
      S.currProtocolIndex = 7; // Advanced to Protocol 7: antibias left/right w/ forced delay 1.2 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 1000;
      // S.DelayPeriod         = 1200;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod     = 300;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 7:  // 关注于 light flashes protocol
    for (int i = 0; i < 5; i++)
    {
      if(i == 0 || i==1 ){ // 0 ,1 target tasks , 优先级从0到9 减小
        if (CP.TargetTaskCounter[i] >= 2) // when the performance of target task is greater than 80/100 twice during the evalution.
        {
          CP.CurrTrialLightIntensity = trialParamsSpace[i][2];
          CP.CurrTrialContrast = trialParamsSpace[i][1];
          S.retention_counter = 0;
          S.currProtocolIndex = 8; // test protocol 如果light
          S.currProtocolTrials = 0;
          S.currProtocolPerf = 0;

          S.SamplePeriod = 1000;
          // S.DelayPeriod         = 1200;
          S.TimeOut = 8000;
          S.AnswerPeriod = 1000;
          S.ConsumptionPeriod = 750;
          S.StopLickingPeriod = 1000;
          S.EarlyLickPeriod     = 300;
          S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
          break;
        }
      }
    }
    break;
  default:
    break;
  }

  if(S.currProtocolIndex > 0 && S.currProtocolIndex < 8){
    curriculumChangeProtocol();
  }
  if(S.currProtocolIndex == 8){
    CP.CurrTrialModality = int(random(3));
  }
  if(S.currProtocolIndex == 0){
    int temp_random_task = int(random(5));
    CP.CurrTrialModality = trialParamsSpace[temp_random_task][0];
    CP.CurrTrialContrast = trialParamsSpace[temp_random_task][1];
    CP.CurrTrialLightIntensity = trialParamsSpace[temp_random_task][2];
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
    // bright_flag
    dataFile.print("bright_flag = ");
    dataFile.println(S.bright_flag);
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
    // bright_flag
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.bright_flag = string_tmp.toInt();
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

    dataFile.print(TrialTypeContrast); // alter the TrialType
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

    dataFile.print(S.bright_flag); 
    dataFile.print(" ");

    dataFile.print(AudContrast); 
    dataFile.print(" ");

    // poisson events
    for(int i = 0; i < SampleBinNum ;i++)
    {
      dataFile.print(PoissonEvent[i]);
      dataFile.print(" ");
    }

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


/************************************************************************************************************************/
int read_Curriculum_Param_info()
{
  // read file to identify the cage number
  File dataFile = SD.open("curriculumparam.txt", FILE_READ);
  if (dataFile)
  {
    dataFile.seek(0);
    // EvaPerf
    string_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < 5; i++)
    {
      string_tmp = dataFile.readStringUntil(';');
      CP.EvaPerf[i] = string_tmp.toInt();
    }
    string_tmp = dataFile.readStringUntil('\n');

    // LastEvaPerf
    string_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < 5; i++)
    {
      string_tmp = dataFile.readStringUntil(';');
      CP.LastEvaPerf[i] = string_tmp.toInt();
    }
    string_tmp = dataFile.readStringUntil('\n');

    // ALP
    string_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < 5; i++)
    {
      string_tmp = dataFile.readStringUntil(';');
      CP.ALP[i] = string_tmp.toInt();
    }
    string_tmp = dataFile.readStringUntil('\n');

    // TargetTaskCounter
    string_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < 5; i++)
    {
      string_tmp = dataFile.readStringUntil(';');
      CP.TargetTaskCounter[i] = string_tmp.toInt();
    }
    string_tmp = dataFile.readStringUntil('\n');

    // CurrTrialModality 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.CurrTrialModality  = string_tmp.toInt();
    
    // CurrTrialContrast 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.CurrTrialContrast  = string_tmp.toInt();

    // CurrTrialLightIntensity 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.CurrTrialLightIntensity = string_tmp.toInt();

    // EvaNum_per_task 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.EvaNum_per_task  = string_tmp.toInt();

    // subtaskNum 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.subtaskNum = string_tmp.toInt();

    // currEvaTaskIndex 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.currEvaTaskIndex  = string_tmp.toInt();

    // currEvaNumIndex 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.currEvaNumIndex  = string_tmp.toInt();

    // updateALP_num 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.updateALP_num  = string_tmp.toInt();

    // currHistory_step  
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.currHistory_step   = string_tmp.toInt();

    // t 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.t  = string_tmp.toInt();

    // evaFlag 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    CP.evaFlag = string_tmp.toInt();
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
    // EvaPerf
    dataFile.print("EvaPerf = ");
    for (int i = 0; i < 5; i++)
    {
      dataFile.print(CP.EvaPerf[i]);
      dataFile.print(";");
    }
    dataFile.println();

    // LastEvaPerf
    dataFile.print("LastEvaPerf = ");
    for (int i = 0; i < 5; i++)
    {
      dataFile.print(CP.LastEvaPerf[i]);
      dataFile.print(";");
    }
    dataFile.println();

    // ALP
    dataFile.print("ALP = ");
    for (int i = 0; i < 5; i++)
    {
      dataFile.print(CP.ALP[i]);
      dataFile.print(";");
    }
    dataFile.println();

    // TargetTaskCounter
    dataFile.print("TargetTaskCounter = ");
    for (int i = 0; i < 5; i++)
    {
      dataFile.print(CP.TargetTaskCounter[i]);
      dataFile.print(";");
    }
    dataFile.println();

    // CurrTrialModality 
    dataFile.print("CurrTrialModality = ");
    dataFile.println(CP.CurrTrialModality);

    // CurrTrialContrast 
    dataFile.print("CurrTrialContrast = ");
    dataFile.println(CP.CurrTrialContrast);

    // CurrTrialLightIntensity 
    dataFile.print("CurrTrialLightIntensity = ");
    dataFile.println(CP.CurrTrialLightIntensity);

    // EvaNum_per_task 
    dataFile.print("EvaNum_per_task = ");
    dataFile.println(CP.EvaNum_per_task);

    // subtaskNum 
    dataFile.print("subtaskNum = ");
    dataFile.println(CP.subtaskNum);

    // currEvaTaskIndex 
    dataFile.print("currEvaTaskIndex = ");
    dataFile.println(CP.currEvaTaskIndex);

    // currEvaNumIndex 
    dataFile.print("currEvaNumIndex = ");
    dataFile.println(CP.currEvaNumIndex);

    // updateALP_num  
    dataFile.print("updateALP_num = ");
    dataFile.println(CP.updateALP_num);

    // currHistory_step  
    dataFile.print("currHistory_step = ");
    dataFile.println(CP.currHistory_step);

    // t  
    dataFile.print("t = ");
    dataFile.println(CP.t);

    // evaFlag 
    dataFile.print("evaFlag = ");
    dataFile.println(CP.evaFlag);
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
    dataFile.print(" ");
    // iterations
    dataFile.print(CP.t);
    dataFile.print(" ");
    // hyper params
    dataFile.print(CP.CurrTrialModality);
    dataFile.print(" ");
    dataFile.print(CP.CurrTrialContrast);
    dataFile.print(" ");
    dataFile.print(CP.CurrTrialLightIntensity);

    // EvaPerf
    for (int i = 0; i < 5; i++)
    {
      dataFile.print(" ");
      dataFile.print(CP.EvaPerf[i]);
    }
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
/******************************************************************************************************************************/






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
