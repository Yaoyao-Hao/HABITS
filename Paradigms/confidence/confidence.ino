#include <gpSMART_Habits.h>

#include <string.h>
#include <SD.h>
#include <SPI.h> // SD card
// #include "MPR121.h"   // MPR121 Capacitive Touch Breakout
// #include <Wire.h>     // Already included in gpSMART_Habits
#include "HX711.h" // HX711 Weighting Amplifier
#include <WiFiEsp.h>
#include <WiFiEspUdp.h> // For WiFi

// #include "gpSMART_Habits.h" // For State Machine

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
char ssid[] = "HABITS";               // your network SSID (name) Brain&Brain-inspired Computing
char pass[] = "bcihabits";            // your network password BCIsLab514
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
  float currProtocolPerf = 0;          // performance: 0-100%
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
  int DelayReward = 600;
  int ModalityFlag = 0;
} Parameters_behavior;
Parameters_behavior S;

/********** Define OutputAction**********/
OutputAction LeftWaterOutput = {"DO1", 1};
OutputAction RightWaterOutput = {"DO2", 1};
OutputAction LowSoundOutput = {"tPWM2", 1}; // tPWM2 top sound
OutputAction HighSoundOutput = {"tPWM2", 2};
// Level is difficulty level, the greater level represents more difficult stimulus
OutputAction HighSoundOutputLevel1 = {"tPWM2", 9};
OutputAction HighSoundOutputLevel2 = {"tPWM2", 8};
OutputAction HighSoundOutputLevel3 = {"tPWM2", 7};

OutputAction LowSoundOutputLevel1 = {"tPWM2", 4};
OutputAction LowSoundOutputLevel2 = {"tPWM2", 5};
OutputAction LowSoundOutputLevel3 = {"tPWM2", 6};
OutputAction CueOutput = {"tPWM2", 3};

OutputAction NoiseOutput = {"Flag1", 1};
OutputAction LeftSoundOutput = {"tPWM1", 3};  // tPWM1 left sound ,10khz
OutputAction RightSoundOutput = {"tPWM3", 3}; // tPWM3 right sound , 10khz
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

byte TrialType = 1; // 0 undef; 1 left; 2 right; 3 middle;
byte TrialTypeContrastLeft = 0;
byte TrialTypeContrastRight = 0;
byte TrialOutcome = 3; // 0 no-response; 1 correct; 2 error; 3 others
byte is_earlylick = 2; // 0-no earlylick; 1-earlylick; 2-undef
byte MaxSame = 3;
byte MinCorrect = 1; // minimum correct trials in the last 5 trials

float Perf100 = 0;
float Alpha50 = 0.98;
float Alpha100 = 0.99;
float currProtocolPerf_corrected = 0;
float Perf100_corrected = 0;

byte EarlyLick100 = 0;
unsigned long last_reward_time = 0;
int timed_reward_count = 0;

int PoissonEvent[40] = {}; // (left right poisson_event) -> (0 0 0);(0 1 1);(1 0 2);(1 1 3);(2 2 4)
int NoOutputBinNum = 0;
int sample_bin = 1; // the basic duration of a single bin
int PulseTime = 10; // the duration of one pulse is 10ms
int IntervalPulse = 15;
int PoissonRate = 0; // 8-16
int PoissonRateTotal = 40;
int SampleBinNum = 40; // the duration of a bin is 25ms
int P_valueLeft = 0;   // P = lambda / N
int P_valueRight = 0;
int difficultLevel = 5; // 0~4
int modality = 2;       // 0~1 freq or clicks

int CatchTrialProb = 10; // use a small number of trials as catch trials probe trial(10s time Investment)
byte ProbeTrialFlag = 0;

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
  smart.init(3); // init smart with num_electrodes used
  byte PortEnabled[4] = {0, 0, 0, 0};
  smart.setDigitalInputsEnabled(PortEnabled);
  /// tPWM2:
  smart.setTruePWMFrequency(2, 1, 8000, 128);  // (low sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(2, 2, 32000, 128); // (high sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(2, 3, 16000, 128); // (go cue) byte tPWM_num, byte freq_num, uint32 frequency, byte duty

  smart.setTruePWMFrequency(2, 4, 10556, 128);
  smart.setTruePWMFrequency(2, 5, 13929, 128);
  smart.setTruePWMFrequency(2, 6, 14929, 128);

  smart.setTruePWMFrequency(2, 7, 17148, 128);
  smart.setTruePWMFrequency(2, 8, 18379, 128);
  smart.setTruePWMFrequency(2, 9, 24252, 128);
  FLASHLED(200);
  // tPWM1
  smart.setTruePWMFrequency(1, 1, 8000, 128);  // (low sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(1, 2, 32000, 128); // (high sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(1, 3, 10000, 128); // (go cue) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  FLASHLED(200);
  // tPWM3
  smart.setTruePWMFrequency(3, 1, 8000, 128);  // (low sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(3, 2, 32000, 128); // (high sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(3, 3, 10000, 128); // (go cue) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  FLASHLED(200);

  // read parameters from SD Card to override S
  read_SD_para_S();
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
            { // continuous 10 min weight data is below the tare threshold
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
  OutputAction LeftLightOutput = {"PWM1", S.low_light_intensity};  // left light, value 0-255,TODO just 1 ,the light intensity is unaffordable(anlogWrite()command)
  OutputAction RightLightOutput = {"PWM5", S.low_light_intensity}; // right light ,same as above
  // OutputAction LeftLightOutputBackground = {"Flag2", 1}; // left LED noise light
  // OutputAction RightLightOutputBackground = {"Flag3", 1};
  OutputAction LeftAudOutputBackground = {"Flag3", 0};
  OutputAction RightAudOutputBackground = {"Flag3", 0};

  OutputAction BlueLightOutput = {"PWM4", S.high_light_intensity};           // Blue light...
  OutputAction RedLightOutput = {"PWM2", S.high_light_intensity};            // red light...
  OutputAction GreenLightOutput = {"PWM3", S.high_light_intensity};          // Green light...
  OutputAction BlueLightOutputBackground = {"PWM4", S.low_light_intensity};  // Blue light Background...
  OutputAction RedLightOutputBackground = {"PWM2", S.low_light_intensity};   // red light Background...
  OutputAction GreenLightOutputBackground = {"PWM3", S.low_light_intensity}; // Green light Background...

  OutputAction DelayNoOutput = GreenLightOutputBackground; // no output
  // light intensity
  OutputAction LeftLightOutputH = {"PWM1", S.high_light_intensity};  // High left light intensity
  OutputAction RightLightOutputH = {"PWM5", S.high_light_intensity}; // High right light intensity

  // randomSeed(analogRead(39)); // anlog input as random seeds

  // catch trial (alter the samplePeriod)
  NoOutputBinNum = 0;
  P_valueLeft = 0; // P = lambda / N
  P_valueRight = 0;
  difficultLevel = 5;
  ProbeTrialFlag = 0;

  // clear PissonEvent
  for (int i = 0; i < 40; i++)
  {
    PoissonEvent[i] = 4;
  }                                                                                                                                                    // no def
                                                                                                                                                       // the interval of pulses is 15ms
  int BinaryClickRight[40] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2}; // 40 events maximum per 1000ms
  int BinaryClickLeft[40] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
  int BinaryClick[40] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // defaultly set NoOutput in the Sample period
  // random modality
  // int modality = int(random(3));
  // poisson click parameter

  /*******************************************************/

  smart.EmptyMatrix(); // clear matrix at the begining of each trial

  switch (S.currProtocolIndex)
  {
  case 0:
  {                             // PROTOCOL 0: teaching animal to lick both side
    OutputAction SampleOutputL; // low intensity
    OutputAction SampleOutputR;
    OutputAction SampleOutput; // freq output
    OutputAction AudSampleOutput;
    OutputAction SampleOutputBackgroundL;
    OutputAction SampleOutputBackgroundR;
    OutputAction AudSampleOutputBackground;
    OutputAction RewardOutput;

    SampleOutputL = LeftSoundOutput; // 10kHz freq pure tone click
    SampleOutputR = RightSoundOutput;

    AudSampleOutput = HighSoundOutput;        // 10KHz freq clickes top
    AudSampleOutputBackground = {"Flag3", 0}; // no output
    SampleOutputBackgroundL = LeftAudOutputBackground;
    SampleOutputBackgroundR = RightAudOutputBackground;

    // S.DelayPeriod = float(random(200, 500)); // 200-500 msec random delay
    S.PreCueDelayPeriod = float(random(200, 400));

    String SampleAfterPreDelay;

    String LeftLickAction;
    String RightLickAction;
    String CenterLickAction;

    String AnswerLeftLickAction;
    String AnswerRightLickAction;
    String AnswerCenterLickAction;

    String ActionAfterDelay;
    float reward_dur = S.reward_left;
    PoissonRate = 20;
    TrialTypeContrastLeft = 0;
    TrialTypeContrastRight = 0;

    modality = 0; 

    switch (TrialType)
    {
    case 1: // left low event rate
    {
      if (modality == 0)
      {
        SampleAfterPreDelay = "SampleDelayPeriod"; // Auditory frequency
        SampleOutput = LowSoundOutput;
      }
      else if (modality == 1)
      { // Auditory clicks
        SampleAfterPreDelay = "SamplePeriod0";
        for (int i = 0; i < 40; i++)
        {
          if (i < NoOutputBinNum)
          {
            BinaryClickLeft[i] = 0;
            BinaryClickRight[i] = 0;
            PoissonEvent[i] = 4;
          }
          else
          {
            if (i % 2 == 0 || i == NoOutputBinNum)
            {
              BinaryClickLeft[i] = 1;
              BinaryClickRight[i] = 0;
              PoissonEvent[i] = 2;
              TrialTypeContrastLeft++;
            }
            else
            {
              BinaryClickLeft[i] = 0;
              BinaryClickRight[i] = 0;
              PoissonEvent[i] = 0;
            }
          }
        }
      }

      LeftLickAction = "Reward";
      RightLickAction = "SamplePeriod"; // no error trial

      AnswerLeftLickAction = "Reward";
      AnswerRightLickAction = "AnswerPeriod";
      AnswerCenterLickAction = "TrialRestart";
      RewardOutput = LeftWaterOutput;
      reward_dur = S.reward_left;
    }
    break;
    case 2: // right high event rate
    {
      if (modality == 0)
      {
        SampleAfterPreDelay = "SampleDelayPeriod"; // Auditory frequency
        SampleOutput = HighSoundOutput;
      }
      else if (modality == 1)
      { // Auditory clicks
        SampleAfterPreDelay = "SamplePeriod0";
        for (int i = 0; i < 40; i++)
        {
          if (i < NoOutputBinNum)
          {
            BinaryClickLeft[i] = 0;
            BinaryClickRight[i] = 0;
            PoissonEvent[i] = 4;
          }
          else
          {
            if (i % 2 == 0 || i == NoOutputBinNum)
            {
              BinaryClickLeft[i] = 0;
              BinaryClickRight[i] = 1;
              PoissonEvent[i] = 1;
              TrialTypeContrastRight++;
            }
            else
            {
              BinaryClickLeft[i] = 0;
              BinaryClickRight[i] = 0;
              PoissonEvent[i] = 0;
            }
          }
          // Serial.print(BinaryClickRight[i]);
          // Serial.print("  ");
        }
      }
      LeftLickAction = "SamplePeriod";
      RightLickAction = "Reward"; // no error trial

      AnswerLeftLickAction = "AnswerPeriod";
      AnswerRightLickAction = "Reward";
      AnswerCenterLickAction = "TrialRestart";
      RewardOutput = RightWaterOutput;
      reward_dur = S.reward_right;
    }
    break;
    }

    // regular click train
    // int EventRate = 0; // using the real EventRate that presented to mice to judge the correct choice.

    if (random(100) < 60)
    { // free reward probability
      if (modality == 0)
      {
        ActionAfterDelay = "GiveFreeDrop";
      }
      else if (modality == 1)
      {
        ActionAfterDelay = "GiveFreeDropClick";
      }
    }
    else
    {
      if (modality == 0)
      {
        ActionAfterDelay = "SamplePeriod";
      }
      else if (modality == 1)
      {
        ActionAfterDelay = "AnswerPeriod";
      }
    }
    // delay period 250ms

    // flashes bin condition
    StateTransition SamplePeriod0_Cond[3] = {{"Tup", "SamplePeriodInterval0"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod1_Cond[3] = {{"Tup", "SamplePeriodInterval1"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod2_Cond[3] = {{"Tup", "SamplePeriodInterval2"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod3_Cond[3] = {{"Tup", "SamplePeriodInterval3"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod4_Cond[3] = {{"Tup", "SamplePeriodInterval4"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod5_Cond[3] = {{"Tup", "SamplePeriodInterval5"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod6_Cond[3] = {{"Tup", "SamplePeriodInterval6"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod7_Cond[3] = {{"Tup", "SamplePeriodInterval7"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod8_Cond[3] = {{"Tup", "SamplePeriodInterval8"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod9_Cond[3] = {{"Tup", "SamplePeriodInterval9"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};

    StateTransition SamplePeriodInterval0_Cond[3] = {{"Tup", "SamplePeriod1"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval1_Cond[3] = {{"Tup", "SamplePeriod2"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval2_Cond[3] = {{"Tup", "SamplePeriod3"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval3_Cond[3] = {{"Tup", "SamplePeriod4"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval4_Cond[3] = {{"Tup", "SamplePeriod5"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval5_Cond[3] = {{"Tup", "SamplePeriod6"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval6_Cond[3] = {{"Tup", "SamplePeriod7"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval7_Cond[3] = {{"Tup", "SamplePeriod8"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval8_Cond[3] = {{"Tup", "SamplePeriod9"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval9_Cond[3] = {{"Tup", ActionAfterDelay}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};

    StateTransition PreCueDelayPeriod_Cond[3] = {{"Tup", SampleAfterPreDelay}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition TrialStart_Cond[1] = {{"Tup", "PreCueDelayPeriod"}}; //
    // StateTransition SamplePeriod_Cond[1]   = {{"Tup", "DelayPeriod"}};
    StateTransition DelayPeriod_Cond[1] = {{"Tup", ActionAfterDelay}}; // not used
    StateTransition TrialEndCue_Cond[1] = {{"Tup", "TrialExit"}};
    StateTransition GiveFreeDrop_Cond[1] = {{"Tup", "SamplePeriod"}};
    StateTransition GiveFreeDropClick_Cond[1] = {{"Tup", "AnswerPeriod"}};
    StateTransition Reward_Cond[1] = {{"Tup", "RewardConsumption"}};
    StateTransition ErrorTrial_Cond[1] = {{"Tup", "exit"}};

    StateTransition SampleDelayPeriod_Cond[3] = {{"Tup", ActionAfterDelay}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod_Cond[3] = {{"Tup", "AnswerPeriod"}, {"Lick1In", LeftLickAction}, {"Lick2In", RightLickAction}};
    StateTransition AnswerPeriod_Cond[3] = {{"Lick1In", AnswerLeftLickAction}, {"Lick2In", AnswerRightLickAction} , {"Tup", "NoResponse"}};
    StateTransition Tup_Exit_Cond[1] = {{"Tup", "TrialEnd"}};
    StateTransition NoResponse_Cond[1] = {{"Tup", "TrialEndCue"}};
    StateTransition EarlyLickAborted_Cond[1] = {{"Tup", "TrialAborted"}};
    StateTransition TrialAborted_Cond[1] = {{"Tup", "TrialEnd"}};
    StateTransition TrialRestart_Cond[1] = {{"Tup", "TrialEnd"}};
    StateTransition TrialEnd_Cond[1] = {{"Tup", "TrialEndCue"}};

    OutputAction OutputL[3] = {SampleOutputBackgroundL, SampleOutputL, DelayNoOutput};
    OutputAction OutputR[3] = {SampleOutputBackgroundR, SampleOutputR, DelayNoOutput};
    OutputAction AudOutput[3] = {AudSampleOutputBackground, AudSampleOutput, DelayNoOutput}; // useless

    StateTransition TrialExit_Cond[2] = {{"Lick3In", "exit"}, {"Tup", "exit"}};
    // flash or click stimulus outputaction
    // set the distribution of flashes
    OutputAction SamplePeriod0_Output[3] = {OutputL[BinaryClickLeft[0]], OutputR[BinaryClickRight[0]], AudOutput[BinaryClick[0]]};
    OutputAction SamplePeriod1_Output[3] = {OutputL[BinaryClickLeft[1]], OutputR[BinaryClickRight[1]], AudOutput[BinaryClick[1]]};
    OutputAction SamplePeriod2_Output[3] = {OutputL[BinaryClickLeft[2]], OutputR[BinaryClickRight[2]], AudOutput[BinaryClick[2]]};
    OutputAction SamplePeriod3_Output[3] = {OutputL[BinaryClickLeft[3]], OutputR[BinaryClickRight[3]], AudOutput[BinaryClick[3]]};
    OutputAction SamplePeriod4_Output[3] = {OutputL[BinaryClickLeft[4]], OutputR[BinaryClickRight[4]], AudOutput[BinaryClick[4]]};
    OutputAction SamplePeriod5_Output[3] = {OutputL[BinaryClickLeft[5]], OutputR[BinaryClickRight[5]], AudOutput[BinaryClick[5]]};
    OutputAction SamplePeriod6_Output[3] = {OutputL[BinaryClickLeft[6]], OutputR[BinaryClickRight[6]], AudOutput[BinaryClick[6]]};
    OutputAction SamplePeriod7_Output[3] = {OutputL[BinaryClickLeft[7]], OutputR[BinaryClickRight[7]], AudOutput[BinaryClick[7]]};
    OutputAction SamplePeriod8_Output[3] = {OutputL[BinaryClickLeft[8]], OutputR[BinaryClickRight[8]], AudOutput[BinaryClick[8]]};
    OutputAction SamplePeriod9_Output[3] = {OutputL[BinaryClickLeft[9]], OutputR[BinaryClickRight[9]], AudOutput[BinaryClick[9]]};

    // OutputAction Sample_Output[1]     	   = {SampleOutput};
    OutputAction PreCueDelayPeriod_Output[2] = {LeftLightOutput ,RightLightOutput}; // low intensity green light as a pre cue
    OutputAction TrialEndCue_Output[1] = {CueOutput};
    OutputAction Reward_Output[1] = {RewardOutput};
    OutputAction NoOutput[0] = {};
    OutputAction ErrorOutput[1] = {NoiseOutput};
    OutputAction DelayPeriod_Output[1] = {DelayNoOutput};
    OutputAction SamplePeriod_Output[1] = {SampleOutput};
    OutputAction FreeReward_Output[2] = {RewardOutput, SampleOutput};
    OutputAction SampleDelayPeriod_Output[3] = {LeftLightOutput ,RightLightOutput ,SampleOutput};

    gpSMART_State states[38] = {};
    // visual flash states and conditions
    states[0] = smart.CreateState("TrialStart", 5, 1, TrialStart_Cond, 0, NoOutput);               // msec
    states[2] = smart.CreateState("DelayPeriod", S.DelayPeriod, 1, DelayPeriod_Cond, 0, NoOutput); // not used
    states[3] = smart.CreateState("TrialEndCue", 100, 1, TrialEndCue_Cond, 1, TrialEndCue_Output);
    states[4] = smart.CreateState("GiveFreeDrop", reward_dur, 1, GiveFreeDrop_Cond, 2, FreeReward_Output);
    states[5] = smart.CreateState("AnswerPeriod", S.AnswerPeriod, 3, AnswerPeriod_Cond, 0, NoOutput);
    states[6] = smart.CreateState("Reward", reward_dur, 1, Reward_Cond, 1, Reward_Output);
    states[7] = smart.CreateState("RewardConsumption", S.ConsumptionPeriod, 1, Tup_Exit_Cond, 0, NoOutput);
    states[8] = smart.CreateState("NoResponse", 10, 1, NoResponse_Cond, 0, NoOutput); // 1-hr: 60*60*1000 msec
    // error trial state (not used state)
    states[9] = smart.CreateState("ErrorTrial", 500, 1, ErrorTrial_Cond, 1, ErrorOutput); // not used
                                                                                          // flashes bin
    states[1] = smart.CreateState("SamplePeriod0", PulseTime * sample_bin, 3, SamplePeriod0_Cond, 3, SamplePeriod0_Output);
    states[10] = smart.CreateState("SamplePeriod1", PulseTime * sample_bin, 3, SamplePeriod1_Cond, 3, SamplePeriod1_Output);
    states[32] = smart.CreateState("SamplePeriod2", PulseTime * sample_bin, 3, SamplePeriod2_Cond, 3, SamplePeriod2_Output);
    states[12] = smart.CreateState("SamplePeriod3", PulseTime * sample_bin, 3, SamplePeriod3_Cond, 3, SamplePeriod3_Output);
    states[13] = smart.CreateState("SamplePeriod4", PulseTime * sample_bin, 3, SamplePeriod4_Cond, 3, SamplePeriod4_Output);
    states[14] = smart.CreateState("SamplePeriod5", PulseTime * sample_bin, 3, SamplePeriod5_Cond, 3, SamplePeriod5_Output);
    states[15] = smart.CreateState("SamplePeriod6", PulseTime * sample_bin, 3, SamplePeriod6_Cond, 3, SamplePeriod6_Output);
    states[16] = smart.CreateState("SamplePeriod7", PulseTime * sample_bin, 3, SamplePeriod7_Cond, 3, SamplePeriod7_Output);
    states[17] = smart.CreateState("SamplePeriod8", PulseTime * sample_bin, 3, SamplePeriod8_Cond, 3, SamplePeriod8_Output);
    states[18] = smart.CreateState("SamplePeriod9", PulseTime * sample_bin, 3, SamplePeriod9_Cond, 3, SamplePeriod9_Output);

    states[19] = smart.CreateState("SamplePeriodInterval0", IntervalPulse * sample_bin, 3, SamplePeriodInterval0_Cond, 0, NoOutput);
    states[20] = smart.CreateState("SamplePeriodInterval1", IntervalPulse * sample_bin, 3, SamplePeriodInterval1_Cond, 0, NoOutput);
    states[21] = smart.CreateState("SamplePeriodInterval2", IntervalPulse * sample_bin, 3, SamplePeriodInterval2_Cond, 0, NoOutput);
    states[22] = smart.CreateState("SamplePeriodInterval3", IntervalPulse * sample_bin, 3, SamplePeriodInterval3_Cond, 0, NoOutput);
    states[23] = smart.CreateState("SamplePeriodInterval4", IntervalPulse * sample_bin, 3, SamplePeriodInterval4_Cond, 0, NoOutput);
    states[24] = smart.CreateState("SamplePeriodInterval5", IntervalPulse * sample_bin, 3, SamplePeriodInterval5_Cond, 0, NoOutput);
    states[25] = smart.CreateState("SamplePeriodInterval6", IntervalPulse * sample_bin, 3, SamplePeriodInterval6_Cond, 0, NoOutput);
    states[26] = smart.CreateState("SamplePeriodInterval7", IntervalPulse * sample_bin, 3, SamplePeriodInterval7_Cond, 0, NoOutput);
    states[27] = smart.CreateState("SamplePeriodInterval8", IntervalPulse * sample_bin, 3, SamplePeriodInterval8_Cond, 0, NoOutput);
    states[28] = smart.CreateState("SamplePeriodInterval9", IntervalPulse * sample_bin, 3, SamplePeriodInterval9_Cond, 0, NoOutput);

    // pre-cue delay
    states[29] = smart.CreateState("PreCueDelayPeriod", S.PreCueDelayPeriod, 3, PreCueDelayPeriod_Cond, 2, PreCueDelayPeriod_Output);
    states[30] = smart.CreateState("SampleDelayPeriod", S.DelayPeriod, 3, SampleDelayPeriod_Cond, 3, SampleDelayPeriod_Output);
    states[31] = smart.CreateState("SamplePeriod", S.SamplePeriod - S.DelayPeriod, 3, SamplePeriod_Cond, 1, SamplePeriod_Output);
    states[11] = smart.CreateState("EarlyLickAborted", S.EarlyLickPeriod, 1, EarlyLickAborted_Cond, 1, ErrorOutput);
    states[33] = smart.CreateState("TrialAborted", random(3, 8) * 1000, 1, TrialAborted_Cond, 0, NoOutput); // Early lick punishment : randomly 3s ~ 7s
    states[34] = smart.CreateState("TrialRestart", 10, 1, TrialRestart_Cond, 0, NoOutput);
    states[35] = smart.CreateState("TrialEnd", 10, 1, TrialEnd_Cond, 0, NoOutput);

    states[36] = smart.CreateState("GiveFreeDropClick", reward_dur, 1, GiveFreeDropClick_Cond, 1, Reward_Output);
    states[37] = smart.CreateState("TrialExit", random(30, 60) * 60 * 1000, 2, TrialExit_Cond, 0, NoOutput);
    // Predefine State sequence.
    for (int i = 0; i < 38; i++)
    {
      smart.AddBlankState(states[i].Name);
    }

    // Add a state to state machine.
    for (int i = 0; i < 38; i++)
    {
      smart.AddState(&states[i]);
    }

    // smart.PrintMatrix(); // for debug

    // Run the matrix
    smart.Run();
  }
  break;
  // Protocol 1: add error trials to give a feedback which mice can use it to lick following cue
  case 1: // easiest stimuli (modality is 0 -> 1 or 1 -> 0)
  case 2: // difficulty level 1 introduced
  case 3: // difficulty level 2 introduced
  case 4: // difficulty level 3 introduced (perform > 1000 trials and go to the next protocol)
  {       // teach animal to lick according to cue
    modality = 0;
    OutputAction SampleOutputL;
    OutputAction SampleOutputR;
    OutputAction SampleOutput;
    OutputAction AudSampleOutput;
    OutputAction SampleOutputBackgroundL;
    OutputAction SampleOutputBackgroundR;
    OutputAction AudSampleOutputBackground;
    OutputAction RewardOutput;

    AudSampleOutput = HighSoundOutput;
    AudSampleOutputBackground = {"Flag3", 0}; // no output
    SampleOutputL = LeftSoundOutput;
    SampleOutputR = RightSoundOutput;
    SampleOutputBackgroundL = LeftAudOutputBackground;
    SampleOutputBackgroundR = RightAudOutputBackground;

    S.PreCueDelayPeriod = float(random(200, 400));

    String SampleAfterPreDelay;

    String LeftLickAction;
    String RightLickAction;
    String CenterLickAction;

    String AnswerLeftLickAction;
    String AnswerRightLickAction;
    String AnswerCenterLickAction;

    String ActionAfterDelay;

    float reward_dur = S.reward_left;
    switch (S.currProtocolIndex)
    {
    case 1:
      difficultLevel = 0;
      break;
    case 2:
      difficultLevel = random(0, 2);
      break;
    case 3:
      difficultLevel = random(0, 3);
      break;
    case 4:
      difficultLevel = random(0, 4);
      break;

    default:
      break;
    }
    switch (difficultLevel)
    {
    case 0:
    {
      PoissonRate = 39;
    }
    break;
    case 1:
    {
      PoissonRate = 37;
    }
    break;
    case 2:
    {
      PoissonRate = 31;
    }
    break;
    case 3:
    {
      PoissonRate = 26;
    }
    break;
    }

    switch (TrialType)
    {
    case 1: // left low event rate
    {
      if (modality == 0)
      {
        SampleAfterPreDelay = "SampleDelayPeriod"; // Auditory frequency
        switch (difficultLevel)
        {
        case 0:
        {
          SampleOutput = LowSoundOutput;
        }
        break;
        case 1:
        {
          SampleOutput = LowSoundOutputLevel1;
        }
        break;
        case 2:
        {
          SampleOutput = LowSoundOutputLevel2;
        }
        break;
        case 3:
        {
          SampleOutput = LowSoundOutputLevel3;
        }
        break;
        }
      }
      else if (modality == 1)
      { // Auditory clicks
        SampleAfterPreDelay = "SamplePeriod0";
        P_valueLeft = int(1000 * PoissonRate / SampleBinNum);
        P_valueRight = int(1000 * (PoissonRateTotal - PoissonRate) / SampleBinNum);
      }

      LeftLickAction = "Reward";
      RightLickAction = "ErrorTrial"; // no error trial

      AnswerLeftLickAction = "Reward";
      AnswerRightLickAction = "ErrorTrial";
      AnswerCenterLickAction = "TrialRestart";
      RewardOutput = LeftWaterOutput;
      reward_dur = S.reward_left;
    }
    break;
    case 2: // right high event rate
    {
      if (modality == 0)
      {
        SampleAfterPreDelay = "SampleDelayPeriod"; // Auditory frequency
        switch (difficultLevel)
        {
        case 0:
        {
          SampleOutput = HighSoundOutput;
        }
        break;
        case 1:
        {
          SampleOutput = HighSoundOutputLevel1;
        }
        break;
        case 2:
        {
          SampleOutput = HighSoundOutputLevel2;
        }
        break;
        case 3:
        {
          SampleOutput = HighSoundOutputLevel3;
        }
        break;
        }
      }
      else if (modality == 1)
      { // Auditory clicks
        SampleAfterPreDelay = "SamplePeriod0";
        P_valueLeft = int(1000 * (PoissonRateTotal - PoissonRate) / SampleBinNum);
        P_valueRight = int(1000 * PoissonRate / SampleBinNum);
      }

      LeftLickAction = "ErrorTrial";
      RightLickAction = "Reward"; // no error trial

      AnswerLeftLickAction = "ErrorTrial";
      AnswerRightLickAction = "Reward";
      AnswerCenterLickAction = "TrialRestart";
      RewardOutput = RightWaterOutput;
      reward_dur = S.reward_right;
    }
    break;
    }

    TrialTypeContrastLeft = 0;
    TrialTypeContrastRight = 0;

    if (modality == 1)
    {
      for (int i = 0; i < 40; i++)
      {
        if (i < NoOutputBinNum)
        {
          BinaryClickRight[i] = 0;
          BinaryClickLeft[i] = 0;
          PoissonEvent[i] = 4;
        }
        else
        {
          // left
          if (int(random(1000)) < P_valueLeft || i == NoOutputBinNum) // using 1000 as the cardinal number to increase resolution of P_value
          {
            BinaryClickLeft[i] = 1;
            TrialTypeContrastLeft++;
            PoissonEvent[i] = 1;
          }
          else
          {
            BinaryClickLeft[i] = 0;
            PoissonEvent[i] = 0;
          }
          // right
          if (int(random(1000)) < P_valueRight || i == NoOutputBinNum) // using 1000 as the cardinal number to increase resolution of P_value
          {
            BinaryClickRight[i] = 1;
            TrialTypeContrastRight++;
            if (PoissonEvent[i] == 0)
            {
              PoissonEvent[i] = 1;
            }
            else
            {
              PoissonEvent[i] = 3;
            }
          }
          else
          {
            BinaryClickRight[i] = 0;
            if (PoissonEvent[i] == 0)
            {
              PoissonEvent[i] = 0;
            }
            else
            {
              PoissonEvent[i] = 2;
            }
          }
        }
      }
      // determined the rewarded side for each trial based on the side where the greatest number of clicks was actually played
      int randomRewardFlag = 0;
      if (TrialTypeContrastLeft == TrialTypeContrastRight)
      {
        randomRewardFlag = int(random(1, 3)); // 1 is left ;2 is right
      }

      if (TrialTypeContrastLeft > TrialTypeContrastRight || randomRewardFlag == 1)
      {
        LeftLickAction = "Reward";
        RightLickAction = "ErrorTrial"; // no error trial

        AnswerLeftLickAction = "Reward";
        AnswerRightLickAction = "ErrorTrial";
        AnswerCenterLickAction = "TrialRestart";
        RewardOutput = LeftWaterOutput;
        reward_dur = S.reward_left;
        TrialType = 1;
      }
      else if (TrialTypeContrastLeft < TrialTypeContrastRight || randomRewardFlag == 2)
      {
        LeftLickAction = "ErrorTrial";
        RightLickAction = "Reward"; // no error trial

        AnswerLeftLickAction = "ErrorTrial";
        AnswerRightLickAction = "Reward";
        AnswerCenterLickAction = "TrialRestart";
        RewardOutput = RightWaterOutput;
        reward_dur = S.reward_right;
        TrialType = 2;
      }
    }
    else
    {
    }

    // free reward
    if ((TrialType == 1 && S.GaveFreeReward.flag_L_water == 1) || (TrialType == 2 && S.GaveFreeReward.flag_R_water == 1))
    { // free reward probability
      if (modality == 0)
      {
        ActionAfterDelay = "GiveFreeDrop";
      }
      else if (modality == 1)
      {
        ActionAfterDelay = "GiveFreeDropClick";
      }
    }
    else
    {
      if (modality == 0)
      {
        ActionAfterDelay = "SamplePeriod";
      }
      else if (modality == 1)
      {
        ActionAfterDelay = "AnswerPeriod";
      }
    }

    // flashes bin condition
    StateTransition SamplePeriod0_Cond[3] = {{"Tup", "SamplePeriodInterval0"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod1_Cond[3] = {{"Tup", "SamplePeriodInterval1"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod2_Cond[3] = {{"Tup", "SamplePeriodInterval2"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod3_Cond[3] = {{"Tup", "SamplePeriodInterval3"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod4_Cond[3] = {{"Tup", "SamplePeriodInterval4"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod5_Cond[3] = {{"Tup", "SamplePeriodInterval5"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod6_Cond[3] = {{"Tup", "SamplePeriodInterval6"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod7_Cond[3] = {{"Tup", "SamplePeriodInterval7"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod8_Cond[3] = {{"Tup", "SamplePeriodInterval8"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod9_Cond[3] = {{"Tup", "SamplePeriodInterval9"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};

    StateTransition SamplePeriodInterval0_Cond[3] = {{"Tup", "SamplePeriod1"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval1_Cond[3] = {{"Tup", "SamplePeriod2"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval2_Cond[3] = {{"Tup", "SamplePeriod3"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval3_Cond[3] = {{"Tup", "SamplePeriod4"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval4_Cond[3] = {{"Tup", "SamplePeriod5"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval5_Cond[3] = {{"Tup", "SamplePeriod6"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval6_Cond[3] = {{"Tup", "SamplePeriod7"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval7_Cond[3] = {{"Tup", "SamplePeriod8"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval8_Cond[3] = {{"Tup", "SamplePeriod9"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval9_Cond[3] = {{"Tup", ActionAfterDelay}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};

    StateTransition PreCueDelayPeriod_Cond[3] = {{"Tup", SampleAfterPreDelay}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition TrialStart_Cond[1] = {{"Tup", "PreCueDelayPeriod"}}; //
    // StateTransition SamplePeriod_Cond[1]   = {{"Tup", "DelayPeriod"}};
    StateTransition DelayPeriod_Cond[1] = {{"Tup", ActionAfterDelay}}; // not used
    StateTransition TrialEndCue_Cond[1] = {{"Tup", "TrialExit"}};
    StateTransition GiveFreeDrop_Cond[1] = {{"Tup", "SamplePeriod"}};
    StateTransition GiveFreeDropClick_Cond[1] = {{"Tup", "AnswerPeriod"}};
    StateTransition Reward_Cond[1] = {{"Tup", "RewardConsumption"}};
    StateTransition ErrorTrial_Cond[1] = {{"Tup", "TimeOut"}};

    StateTransition SampleDelayPeriod_Cond[3] = {{"Tup", ActionAfterDelay}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod_Cond[3] = {{"Tup", "AnswerPeriod"}, {"Lick1In", LeftLickAction}, {"Lick2In", RightLickAction}};
    StateTransition AnswerPeriod_Cond[3] = {{"Lick1In", AnswerLeftLickAction}, {"Lick2In", AnswerRightLickAction}, {"Tup", "NoResponse"}};
    StateTransition Tup_Exit_Cond[1] = {{"Tup", "TrialEnd"}};
    StateTransition NoResponse_Cond[1] = {{"Tup", "TrialEndCue"}};
    StateTransition EarlyLickAborted_Cond[1] = {{"Tup", "TrialAborted"}};
    StateTransition TrialAborted_Cond[1] = {{"Tup", "TrialEnd"}};
    StateTransition TrialRestart_Cond[1] = {{"Tup", "TrialEnd"}};
    StateTransition TrialEnd_Cond[1] = {{"Tup", "TrialEndCue"}};
    StateTransition Tup_StopLicking_Cond[1] = {{"Tup", "StopLicking"}};
    StateTransition StopLicking_Cond[4] = {{"Lick1In", "StopLickingReturn"}, {"Lick2In", "StopLickingReturn"}, {"Lick3In", "StopLickingReturn"}, {"Tup", "TrialEnd"}}; //

    OutputAction OutputL[3] = {SampleOutputBackgroundL, SampleOutputL, DelayNoOutput};
    OutputAction OutputR[3] = {SampleOutputBackgroundR, SampleOutputR, DelayNoOutput};
    OutputAction AudOutput[3] = {AudSampleOutputBackground, AudSampleOutput, DelayNoOutput}; // useless

    StateTransition TrialExit_Cond[2] = {{"Lick3In", "exit"}, {"Tup", "exit"}};
    // flash or click stimulus outputaction
    // set the distribution of flashes
    OutputAction SamplePeriod0_Output[3] = {OutputL[BinaryClickLeft[0]], OutputR[BinaryClickRight[0]], AudOutput[BinaryClick[0]]};
    OutputAction SamplePeriod1_Output[3] = {OutputL[BinaryClickLeft[1]], OutputR[BinaryClickRight[1]], AudOutput[BinaryClick[1]]};
    OutputAction SamplePeriod2_Output[3] = {OutputL[BinaryClickLeft[2]], OutputR[BinaryClickRight[2]], AudOutput[BinaryClick[2]]};
    OutputAction SamplePeriod3_Output[3] = {OutputL[BinaryClickLeft[3]], OutputR[BinaryClickRight[3]], AudOutput[BinaryClick[3]]};
    OutputAction SamplePeriod4_Output[3] = {OutputL[BinaryClickLeft[4]], OutputR[BinaryClickRight[4]], AudOutput[BinaryClick[4]]};
    OutputAction SamplePeriod5_Output[3] = {OutputL[BinaryClickLeft[5]], OutputR[BinaryClickRight[5]], AudOutput[BinaryClick[5]]};
    OutputAction SamplePeriod6_Output[3] = {OutputL[BinaryClickLeft[6]], OutputR[BinaryClickRight[6]], AudOutput[BinaryClick[6]]};
    OutputAction SamplePeriod7_Output[3] = {OutputL[BinaryClickLeft[7]], OutputR[BinaryClickRight[7]], AudOutput[BinaryClick[7]]};
    OutputAction SamplePeriod8_Output[3] = {OutputL[BinaryClickLeft[8]], OutputR[BinaryClickRight[8]], AudOutput[BinaryClick[8]]};
    OutputAction SamplePeriod9_Output[3] = {OutputL[BinaryClickLeft[9]], OutputR[BinaryClickRight[9]], AudOutput[BinaryClick[9]]};

    // OutputAction Sample_Output[1]     	   = {SampleOutput};
    OutputAction PreCueDelayPeriod_Output[2] = {LeftLightOutput ,RightLightOutput}; // low intensity green light as a pre cue
    OutputAction TrialEndCue_Output[1] = {CueOutput};
    OutputAction Reward_Output[1] = {RewardOutput};
    OutputAction NoOutput[0] = {};
    OutputAction ErrorOutput[1] = {NoiseOutput};
    OutputAction DelayPeriod_Output[1] = {DelayNoOutput};
    OutputAction SamplePeriod_Output[1] = {SampleOutput};
    OutputAction FreeReward_Output[2] = {RewardOutput, SampleOutput};
    OutputAction SampleDelayPeriod_Output[3] = {LeftLightOutput ,RightLightOutput ,SampleOutput};

    gpSMART_State states[41] = {};
    // visual flash states and conditions
    states[0] = smart.CreateState("TrialStart", 5, 1, TrialStart_Cond, 0, NoOutput);               // msec
    states[2] = smart.CreateState("DelayPeriod", S.DelayPeriod, 1, DelayPeriod_Cond, 0, NoOutput); // not used
    states[3] = smart.CreateState("TrialEndCue", 100, 1, TrialEndCue_Cond, 1, TrialEndCue_Output);
    states[4] = smart.CreateState("GiveFreeDrop", reward_dur, 1, GiveFreeDrop_Cond, 2, FreeReward_Output);
    states[5] = smart.CreateState("AnswerPeriod", S.AnswerPeriod, 3, AnswerPeriod_Cond, 0, NoOutput);
    states[6] = smart.CreateState("Reward", reward_dur, 1, Reward_Cond, 1, Reward_Output);
    states[7] = smart.CreateState("RewardConsumption", S.ConsumptionPeriod, 1, Tup_StopLicking_Cond, 0, NoOutput);
    states[8] = smart.CreateState("NoResponse", 10, 1, NoResponse_Cond, 0, NoOutput); // 1-hr: 60*60*1000 msec
    // error trial state (not used state)
    states[9] = smart.CreateState("ErrorTrial", 500, 1, ErrorTrial_Cond, 1, ErrorOutput);
    // flashes bin
    states[1] = smart.CreateState("SamplePeriod0", PulseTime * sample_bin, 3, SamplePeriod0_Cond, 3, SamplePeriod0_Output);
    states[10] = smart.CreateState("SamplePeriod1", PulseTime * sample_bin, 3, SamplePeriod1_Cond, 3, SamplePeriod1_Output);
    states[32] = smart.CreateState("SamplePeriod2", PulseTime * sample_bin, 3, SamplePeriod2_Cond, 3, SamplePeriod2_Output);
    states[12] = smart.CreateState("SamplePeriod3", PulseTime * sample_bin, 3, SamplePeriod3_Cond, 3, SamplePeriod3_Output);
    states[13] = smart.CreateState("SamplePeriod4", PulseTime * sample_bin, 3, SamplePeriod4_Cond, 3, SamplePeriod4_Output);
    states[14] = smart.CreateState("SamplePeriod5", PulseTime * sample_bin, 3, SamplePeriod5_Cond, 3, SamplePeriod5_Output);
    states[15] = smart.CreateState("SamplePeriod6", PulseTime * sample_bin, 3, SamplePeriod6_Cond, 3, SamplePeriod6_Output);
    states[16] = smart.CreateState("SamplePeriod7", PulseTime * sample_bin, 3, SamplePeriod7_Cond, 3, SamplePeriod7_Output);
    states[17] = smart.CreateState("SamplePeriod8", PulseTime * sample_bin, 3, SamplePeriod8_Cond, 3, SamplePeriod8_Output);
    states[18] = smart.CreateState("SamplePeriod9", PulseTime * sample_bin, 3, SamplePeriod9_Cond, 3, SamplePeriod9_Output);

    states[19] = smart.CreateState("SamplePeriodInterval0", IntervalPulse * sample_bin, 3, SamplePeriodInterval0_Cond, 0, NoOutput);
    states[20] = smart.CreateState("SamplePeriodInterval1", IntervalPulse * sample_bin, 3, SamplePeriodInterval1_Cond, 0, NoOutput);
    states[21] = smart.CreateState("SamplePeriodInterval2", IntervalPulse * sample_bin, 3, SamplePeriodInterval2_Cond, 0, NoOutput);
    states[22] = smart.CreateState("SamplePeriodInterval3", IntervalPulse * sample_bin, 3, SamplePeriodInterval3_Cond, 0, NoOutput);
    states[23] = smart.CreateState("SamplePeriodInterval4", IntervalPulse * sample_bin, 3, SamplePeriodInterval4_Cond, 0, NoOutput);
    states[24] = smart.CreateState("SamplePeriodInterval5", IntervalPulse * sample_bin, 3, SamplePeriodInterval5_Cond, 0, NoOutput);
    states[25] = smart.CreateState("SamplePeriodInterval6", IntervalPulse * sample_bin, 3, SamplePeriodInterval6_Cond, 0, NoOutput);
    states[26] = smart.CreateState("SamplePeriodInterval7", IntervalPulse * sample_bin, 3, SamplePeriodInterval7_Cond, 0, NoOutput);
    states[27] = smart.CreateState("SamplePeriodInterval8", IntervalPulse * sample_bin, 3, SamplePeriodInterval8_Cond, 0, NoOutput);
    states[28] = smart.CreateState("SamplePeriodInterval9", IntervalPulse * sample_bin, 3, SamplePeriodInterval9_Cond, 0, NoOutput);

    // pre-cue delay
    states[29] = smart.CreateState("PreCueDelayPeriod", S.PreCueDelayPeriod, 3, PreCueDelayPeriod_Cond, 2, PreCueDelayPeriod_Output);
    states[30] = smart.CreateState("SampleDelayPeriod", S.DelayPeriod, 3, SampleDelayPeriod_Cond, 3, SampleDelayPeriod_Output);
    states[31] = smart.CreateState("SamplePeriod", S.SamplePeriod - S.DelayPeriod, 3, SamplePeriod_Cond, 1, SamplePeriod_Output);
    states[11] = smart.CreateState("EarlyLickAborted", S.EarlyLickPeriod, 1, EarlyLickAborted_Cond, 1, ErrorOutput);
    states[33] = smart.CreateState("TrialAborted", random(3, 8) * 1000, 1, TrialAborted_Cond, 0, NoOutput); // Early lick punishment : randomly 3s ~ 7s
    states[34] = smart.CreateState("TrialRestart", 10, 1, TrialRestart_Cond, 0, NoOutput);
    states[35] = smart.CreateState("TrialEnd", 10, 1, TrialEnd_Cond, 0, NoOutput);

    states[36] = smart.CreateState("GiveFreeDropClick", reward_dur, 1, GiveFreeDropClick_Cond, 1, Reward_Output);
    //
    states[37] = smart.CreateState("TimeOut", S.TimeOut + S.extra_TimeOut, 1, Tup_Exit_Cond, 0, NoOutput);
    states[38] = smart.CreateState("StopLicking", S.StopLickingPeriod, 4, StopLicking_Cond, 0, NoOutput);
    states[39] = smart.CreateState("StopLickingReturn", 8, 1, Tup_StopLicking_Cond, 0, NoOutput);
    states[40] = smart.CreateState("TrialExit", random(30, 60) * 60 * 1000, 2, TrialExit_Cond, 0, NoOutput);
    // Predefine State sequence.
    for (int i = 0; i < 41; i++)
    {
      smart.AddBlankState(states[i].Name);
    }

    // Add a state to state machine.
    for (int i = 0; i < 41; i++)
    {
      smart.AddState(&states[i]);
    }
    // smart.PrintMatrix(); // for debug

    // Run the matrix
    smart.Run();
  }
  break;
  // Protocol 3: add delayed reward period ;remove error trial punishment
  case 5: //  delay 0.6s ~ 2s perform 1000 trials and then drop to next protocol
  case 6: //  delay 0.6s ~ 4s perform 1000 trials and then drop to next protocol antiBias
  case 7: //  delay 0.6s ~ 6s perform 1000 trials and then drop to next protocol antiBias
  case 8: //  delay 0.6s ~ 8s perform 3000 trials and then drop to next protocol antiBias
  case 9: //  test protocol 10% probe trials
  {
    modality = 0;
    difficultLevel = int(random(0, 4)); // randomly select the difficult level
    if(S.currProtocolIndex == 9)
    {
      // difficulty level
      difficultLevel = int(random(0, 5)); // 4 is the random reward trials
      // catch trials
      if (int(random(100)) < CatchTrialProb)
      {
        ProbeTrialFlag = 1;
      }
      else
      {
        ProbeTrialFlag = 0;
      }
    }
    // delay reward period
    switch (S.currProtocolIndex)
    {
    case 5:
      S.DelayReward = exponent(1.0, 0.5, 2.0);
      break;
    case 6:
      S.DelayReward = exponent(1.0, 0.5, 3.0);
      break;
    case 7:
      S.DelayReward = exponent(1.0, 0.5, 4.0);
      break;
    case 8:
    case 9:
      S.DelayReward = exponent(1.0, 0.5, 5.0);
      break;
    default:
      break;
    }

    // set global counter
    smart.SetGlobalCounter(1, "Lick1In", 2);
    smart.SetGlobalCounter(2, "Lick2In", 2);
    smart.SetGlobalCounter(3, "Lick3In", 2);

    OutputAction GlobalCounterLick1In = {"GlobalCounterReset", 1};
    OutputAction GlobalCounterLick2In = {"GlobalCounterReset", 2};
    OutputAction GlobalCounterLick3In = {"GlobalCounterReset", 3};

    OutputAction SampleOutputL;
    OutputAction SampleOutputR;
    OutputAction SampleOutput;
    OutputAction AudSampleOutput;
    OutputAction SampleOutputBackgroundL;
    OutputAction SampleOutputBackgroundR;
    OutputAction AudSampleOutputBackground;
    OutputAction RewardOutput;

    AudSampleOutput = HighSoundOutput;
    AudSampleOutputBackground = {"Flag3", 0}; // no output
    SampleOutputL = LeftSoundOutput;
    SampleOutputR = RightSoundOutput;
    SampleOutputBackgroundL = LeftAudOutputBackground;
    SampleOutputBackgroundR = RightAudOutputBackground;

    S.PreCueDelayPeriod = float(random(200, 400));

    String SampleAfterPreDelay;

    String LeftLickAction;
    String RightLickAction;
    String CenterLickAction;

    String AnswerLeftLickAction;
    String AnswerRightLickAction;
    String AnswerCenterLickAction;

    String ActionAfterDelay;

    String DelayRewardLick1;
    String DelayRewardLick2;

    String ErrorDelayLick1;
    String ErrorDelayLick2;

    String ProbeDelayLick1;
    String ProbeDelayLick2;

    String ReAnswerLick1;
    String ReAnswerLick2;

    float reward_dur = S.reward_left;
   

    switch (difficultLevel)
    {
    case 0:
    {
      PoissonRate = 39;
    }
    break;
    case 1:
    {
      PoissonRate = 37;
    }
    break;
    case 2:
    {
      PoissonRate = 31;
    }
    break;
    case 3:
    {
      PoissonRate = 26;
    }
    break;
    }

    switch (TrialType)
    {
    case 1: // left low event rate
    {
      if (modality == 0)
      {
        SampleAfterPreDelay = "SampleDelayPeriod"; // Auditory frequency
        switch (difficultLevel)
        {
        case 0:
        {
          SampleOutput = LowSoundOutput;
        }
        break;
        case 1:
        {
          SampleOutput = LowSoundOutputLevel1;
        }
        break;
        case 2:
        {
          SampleOutput = LowSoundOutputLevel2;
        }
        break;
        case 3:
        {
          SampleOutput = LowSoundOutputLevel3;
        }
        break;
        case 4:
        {
          SampleOutput = CueOutput;
        }
        break;
        }
      }
      else if (modality == 1)
      { // Auditory clicks
        SampleAfterPreDelay = "SamplePeriod0";
        P_valueLeft = int(1000 * PoissonRate / SampleBinNum);
        P_valueRight = int(1000 * (PoissonRateTotal - PoissonRate) / SampleBinNum);
        // Serial.println(P_valueLeft);
      }

      LeftLickAction = "DelayReward";
      RightLickAction = "ErrorTrial";

      AnswerLeftLickAction = "DelayReward";
      AnswerRightLickAction = "ErrorTrial";
      AnswerCenterLickAction = "TrialRestart";

      DelayRewardLick1 = "DelayReward"; // left confirmed lick ,and the choice of this trial is left
      DelayRewardLick2 = "StopTimeInvestment";

      ErrorDelayLick1 = "StopTimeInvestment"; // the choice of this trial is right
      ErrorDelayLick2 = "ErrorTrial";

      ReAnswerLick1 = "Reward";
      ReAnswerLick2 = "StopTimeInvestment";
      RewardOutput = LeftWaterOutput;
      reward_dur = S.reward_left;
    }
    break;
    case 2: // right high event rate
    {
      if (modality == 0)
      {
        SampleAfterPreDelay = "SampleDelayPeriod"; // Auditory frequency
        switch (difficultLevel)
        {
        case 0:
        {
          SampleOutput = HighSoundOutput;
        }
        break;
        case 1:
        {
          SampleOutput = HighSoundOutputLevel1;
        }
        break;
        case 2:
        {
          SampleOutput = HighSoundOutputLevel2;
        }
        break;
        case 3:
        {
          SampleOutput = HighSoundOutputLevel3;
        }
        break;
        case 4:
        {
          SampleOutput = CueOutput;
        }
        break;
        }
      }
      else if (modality == 1)
      { // Auditory clicks
        SampleAfterPreDelay = "SamplePeriod0";
        P_valueLeft = int(1000 * (PoissonRateTotal - PoissonRate) / SampleBinNum);
        P_valueRight = int(1000 * PoissonRate / SampleBinNum);
      }

      LeftLickAction = "ErrorTrial";
      RightLickAction = "DelayReward"; // no error trial

      AnswerLeftLickAction = "ErrorTrial";
      AnswerRightLickAction = "DelayReward";
      AnswerCenterLickAction = "TrialRestart";

      DelayRewardLick1 = "StopTimeInvestment"; // left confirmed lick ,and the choice of this trial is right
      DelayRewardLick2 = "DelayReward";

      ErrorDelayLick1 = "ErrorTrial"; // the choice of this trial is left
      ErrorDelayLick2 = "StopTimeInvestment";

      ReAnswerLick1 = "StopTimeInvestment";
      ReAnswerLick2 = "Reward";

      RewardOutput = RightWaterOutput;
      reward_dur = S.reward_right;
    }
    break;
    }

    TrialTypeContrastLeft = 0;
    TrialTypeContrastRight = 0;

    if (modality == 1)
    {
      for (int i = 0; i < 40; i++)
      {
        if (i < NoOutputBinNum)
        {
          BinaryClickRight[i] = 0;
          BinaryClickLeft[i] = 0;
          PoissonEvent[i] = 4;
        }
        else
        {
          // left
          if (int(random(1000)) < P_valueLeft || i == NoOutputBinNum) // using 1000 as the cardinal number to increase resolution of P_value
          {
            BinaryClickLeft[i] = 1;
            TrialTypeContrastLeft++;
            PoissonEvent[i] = 1;
          }
          else
          {
            BinaryClickLeft[i] = 0;
            PoissonEvent[i] = 0;
          }
          // right
          if (int(random(1000)) < P_valueRight || i == NoOutputBinNum) // using 1000 as the cardinal number to increase resolution of P_value
          {
            BinaryClickRight[i] = 1;
            TrialTypeContrastRight++;
            if (PoissonEvent[i] == 0)
            {
              PoissonEvent[i] = 1;
            }
            else
            {
              PoissonEvent[i] = 3;
            }
          }
          else
          {
            BinaryClickRight[i] = 0;
            if (PoissonEvent[i] == 0)
            {
              PoissonEvent[i] = 0;
            }
            else
            {
              PoissonEvent[i] = 2;
            }
          }
        }
      }
      // determined the rewarded side for each trial based on the side where the greatest number of clicks was actually played
      int randomRewardFlag = 0;
      if (TrialTypeContrastLeft == TrialTypeContrastRight)
      {
        randomRewardFlag = int(random(1, 3)); // 1 is left ;2 is right
      }

      if (TrialTypeContrastLeft > TrialTypeContrastRight || randomRewardFlag == 1)
      {
        LeftLickAction = "DelayReward";
        RightLickAction = "ErrorTrial"; // no error trial

        AnswerLeftLickAction = "DelayReward";
        AnswerRightLickAction = "ErrorTrial";
        AnswerCenterLickAction = "TrialRestart";

        DelayRewardLick1 = "DelayReward"; // left confirmed lick ,and the choice of this trial is left
        DelayRewardLick2 = "StopTimeInvestment";

        ErrorDelayLick1 = "StopTimeInvestment"; // the choice of this trial is right
        ErrorDelayLick2 = "ErrorTrial";

        ReAnswerLick1 = "Reward";
        ReAnswerLick2 = "StopTimeInvestment";
        RewardOutput = LeftWaterOutput;
        reward_dur = S.reward_left;
        TrialType = 1;
      }
      else if (TrialTypeContrastLeft < TrialTypeContrastRight || randomRewardFlag == 2)
      {
        LeftLickAction = "ErrorTrial";
        RightLickAction = "DelayReward"; // no error trial

        AnswerLeftLickAction = "ErrorTrial";
        AnswerRightLickAction = "DelayReward";
        AnswerCenterLickAction = "TrialRestart";

        DelayRewardLick1 = "StopTimeInvestment"; // left confirmed lick ,and the choice of this trial is right
        DelayRewardLick2 = "DelayReward";

        ErrorDelayLick1 = "ErrorTrial"; // the choice of this trial is left
        ErrorDelayLick2 = "StopTimeInvestment";

        ReAnswerLick1 = "StopTimeInvestment";
        ReAnswerLick2 = "Reward";
        reward_dur = S.reward_right;
        RewardOutput = RightWaterOutput;
        TrialType = 2;
      }
    }
    else
    {
    }
    // probe trial
    if (ProbeTrialFlag == 1)
    {
      if (TrialType == 1)
      {
        LeftLickAction = "ProbeTrial";
        AnswerLeftLickAction = "ProbeTrial";
        DelayRewardLick1 = "ProbeTrial";
      }
      else if (TrialType == 2)
      {
        RightLickAction = "ProbeTrial";
        AnswerRightLickAction = "ProbeTrial";
        DelayRewardLick2 = "ProbeTrial";
      }
    }
    
    // free reward dont have free drop
    if ((TrialType == 1 && S.GaveFreeReward.flag_L_water == 1) || (TrialType == 2 && S.GaveFreeReward.flag_R_water == 1))
    { // free reward probability
      if (modality == 0)
      {
        ActionAfterDelay = "SamplePeriod";
      }
      else if (modality == 1)
      {
        ActionAfterDelay = "AnswerPeriod";
      }
    }
    else
    {
      if (modality == 0)
      {
        ActionAfterDelay = "SamplePeriod";
      }
      else if (modality == 1)
      {
        ActionAfterDelay = "AnswerPeriod";
      }
    }

    // flashes bin condition
    StateTransition SamplePeriod0_Cond[3] = {{"Tup", "SamplePeriodInterval0"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod1_Cond[3] = {{"Tup", "SamplePeriodInterval1"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod2_Cond[3] = {{"Tup", "SamplePeriodInterval2"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod3_Cond[3] = {{"Tup", "SamplePeriodInterval3"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod4_Cond[3] = {{"Tup", "SamplePeriodInterval4"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod5_Cond[3] = {{"Tup", "SamplePeriodInterval5"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod6_Cond[3] = {{"Tup", "SamplePeriodInterval6"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod7_Cond[3] = {{"Tup", "SamplePeriodInterval7"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod8_Cond[3] = {{"Tup", "SamplePeriodInterval8"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod9_Cond[3] = {{"Tup", "SamplePeriodInterval9"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};

    StateTransition SamplePeriodInterval0_Cond[3] = {{"Tup", "SamplePeriod1"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval1_Cond[3] = {{"Tup", "SamplePeriod2"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval2_Cond[3] = {{"Tup", "SamplePeriod3"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval3_Cond[3] = {{"Tup", "SamplePeriod4"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval4_Cond[3] = {{"Tup", "SamplePeriod5"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval5_Cond[3] = {{"Tup", "SamplePeriod6"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval6_Cond[3] = {{"Tup", "SamplePeriod7"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval7_Cond[3] = {{"Tup", "SamplePeriod8"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval8_Cond[3] = {{"Tup", "SamplePeriod9"}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriodInterval9_Cond[3] = {{"Tup", ActionAfterDelay}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};

    StateTransition PreCueDelayPeriod_Cond[3] = {{"Tup", SampleAfterPreDelay}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition TrialStart_Cond[1] = {{"Tup", "PreCueDelayPeriod"}}; //
    // StateTransition SamplePeriod_Cond[1]   = {{"Tup", "DelayPeriod"}};
    StateTransition DelayPeriod_Cond[1] = {{"Tup", ActionAfterDelay}}; // not used
    StateTransition TrialEndCue_Cond[1] = {{"Tup", "TrialExit"}};
    StateTransition GiveFreeDrop_Cond[1] = {{"Tup", "SamplePeriod"}};
    StateTransition GiveFreeDropClick_Cond[1] = {{"Tup", "AnswerPeriod"}};
    StateTransition Reward_Cond[1] = {{"Tup", "RewardConsumption"}};
    StateTransition ErrorTrial_Cond[4] = {{"Tup", "NoResponse"}, {"GlobalCounter1_End", ErrorDelayLick1}, {"GlobalCounter2_End", ErrorDelayLick2}, {"GlobalCounter3_End", "StopTimeInvestment"}}; // probe trial condition
    StateTransition StopTimeInvestment_Cond[1] = {{"Tup", "StopTimeInvestmentCue"}};
    StateTransition StopTimeInvestmentCue_Cond[1] = {{"Tup", "TrialEnd"}};

    StateTransition SampleDelayPeriod_Cond[3] = {{"Tup", ActionAfterDelay}, {"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SamplePeriod_Cond[3] = {{"Tup", "AnswerPeriod"}, {"Lick1In", LeftLickAction}, {"Lick2In", RightLickAction}};
    StateTransition AnswerPeriod_Cond[3] = {{"Lick1In", AnswerLeftLickAction}, {"Lick2In", AnswerRightLickAction}, {"Tup", "NoResponse"}};
    StateTransition Tup_Exit_Cond[1] = {{"Tup", "TrialEnd"}};
    StateTransition NoResponse_Cond[1] = {{"Tup", "TrialEnd"}};
    StateTransition EarlyLickAborted_Cond[1] = {{"Tup", "TrialAborted"}};
    StateTransition TrialAborted_Cond[1] = {{"Tup", "TrialEnd"}};
    StateTransition TrialRestart_Cond[1] = {{"Tup", "TrialEnd"}};
    StateTransition TrialEnd_Cond[1] = {{"Tup", "TrialEndCue"}};
    StateTransition Tup_StopLicking_Cond[1] = {{"Tup", "StopLicking"}};
    StateTransition StopLicking_Cond[4] = {{"Lick1In", "StopLickingReturn"}, {"Lick2In", "StopLickingReturn"}, {"Lick3In", "StopLickingReturn"}, {"Tup", "TrialEnd"}}; //
    StateTransition DelayReward_Cond[4] = {{"GlobalCounter1_End", DelayRewardLick1}, {"GlobalCounter2_End", DelayRewardLick2}, {"GlobalCounter3_End", "StopTimeInvestment"}, {"Tup", "RewardCue"}};
    StateTransition ReAnswerPeriod_Cond[4] = {{"Lick1In", ReAnswerLick1}, {"Lick2In", ReAnswerLick2}, {"Lick3In", "StopTimeInvestment"}, {"Tup", "NoResponse"}};
    StateTransition ProbeTrial_Cond[4] = {{"GlobalCounter1_End", DelayRewardLick1}, {"GlobalCounter2_End", DelayRewardLick2}, {"GlobalCounter3_End", "StopTimeInvestment"}, {"Tup", "NoResponse"}};
    StateTransition TrialExit_Cond[2] = {{"Lick3In", "exit"}, {"Tup", "exit"}};
    StateTransition RewardCue_Cond[1] = {{"Tup", "ReAnswerPeriod"}};

    OutputAction OutputL[3] = {SampleOutputBackgroundL, SampleOutputL, DelayNoOutput};
    OutputAction OutputR[3] = {SampleOutputBackgroundR, SampleOutputR, DelayNoOutput};
    OutputAction AudOutput[3] = {AudSampleOutputBackground, AudSampleOutput, DelayNoOutput}; // useless
    // flash or click stimulus outputaction
    // set the distribution of flashes
    OutputAction SamplePeriod0_Output[3] = {OutputL[BinaryClickLeft[0]], OutputR[BinaryClickRight[0]], AudOutput[BinaryClick[0]]};
    OutputAction SamplePeriod1_Output[3] = {OutputL[BinaryClickLeft[1]], OutputR[BinaryClickRight[1]], AudOutput[BinaryClick[1]]};
    OutputAction SamplePeriod2_Output[3] = {OutputL[BinaryClickLeft[2]], OutputR[BinaryClickRight[2]], AudOutput[BinaryClick[2]]};
    OutputAction SamplePeriod3_Output[3] = {OutputL[BinaryClickLeft[3]], OutputR[BinaryClickRight[3]], AudOutput[BinaryClick[3]]};
    OutputAction SamplePeriod4_Output[3] = {OutputL[BinaryClickLeft[4]], OutputR[BinaryClickRight[4]], AudOutput[BinaryClick[4]]};
    OutputAction SamplePeriod5_Output[3] = {OutputL[BinaryClickLeft[5]], OutputR[BinaryClickRight[5]], AudOutput[BinaryClick[5]]};
    OutputAction SamplePeriod6_Output[3] = {OutputL[BinaryClickLeft[6]], OutputR[BinaryClickRight[6]], AudOutput[BinaryClick[6]]};
    OutputAction SamplePeriod7_Output[3] = {OutputL[BinaryClickLeft[7]], OutputR[BinaryClickRight[7]], AudOutput[BinaryClick[7]]};
    OutputAction SamplePeriod8_Output[3] = {OutputL[BinaryClickLeft[8]], OutputR[BinaryClickRight[8]], AudOutput[BinaryClick[8]]};
    OutputAction SamplePeriod9_Output[3] = {OutputL[BinaryClickLeft[9]], OutputR[BinaryClickRight[9]], AudOutput[BinaryClick[9]]};

    // OutputAction Sample_Output[1]     	   = {SampleOutput};
    OutputAction PreCueDelayPeriod_Output[2] = {LeftLightOutput ,RightLightOutput}; // low intensity green light as a pre cue
    OutputAction TrialEndCue_Output[1] = {CueOutput};
    OutputAction Reward_Output[1] = {RewardOutput};
    OutputAction NoOutput[0] = {};
    OutputAction ErrorOutput[1] = {NoiseOutput};
    OutputAction DelayPeriod_Output[1] = {DelayNoOutput}; // not used
    OutputAction SamplePeriod_Output[1] = {SampleOutput};
    OutputAction FreeReward_Output[2] = {RewardOutput, SampleOutput};
    OutputAction SampleDelayPeriod_Output[3] = {LeftLightOutput ,RightLightOutput ,SampleOutput};

    OutputAction ResetGlobalCounter_Output[3] = {GlobalCounterLick1In, GlobalCounterLick2In, GlobalCounterLick3In};

    gpSMART_State states[47] = {};
    // visual flash states and conditions
    states[0] = smart.CreateState("TrialStart", 5, 1, TrialStart_Cond, 0, NoOutput); // msec
    states[1] = smart.CreateState("SamplePeriod0", PulseTime * sample_bin, 3, SamplePeriod0_Cond, 3, SamplePeriod0_Output);
    states[2] = smart.CreateState("DelayPeriod", S.DelayPeriod, 1, DelayPeriod_Cond, 0, NoOutput); // not used
    states[3] = smart.CreateState("TrialEndCue", 100, 1, TrialEndCue_Cond, 1, TrialEndCue_Output);
    states[4] = smart.CreateState("GiveFreeDrop", reward_dur, 1, GiveFreeDrop_Cond, 2, FreeReward_Output); // not used
    states[5] = smart.CreateState("AnswerPeriod", S.AnswerPeriod, 3, AnswerPeriod_Cond, 0, NoOutput);
    states[6] = smart.CreateState("Reward", reward_dur, 1, Reward_Cond, 1, Reward_Output);
    states[7] = smart.CreateState("RewardConsumption", S.ConsumptionPeriod, 1, Tup_StopLicking_Cond, 0, NoOutput);
    states[8] = smart.CreateState("NoResponse", 10, 1, NoResponse_Cond, 0, NoOutput); // 1-hr: 60*60*1000 msec
    // error trial state (not used state)
    states[9] = smart.CreateState("ErrorTrial", 1000 * 20, 4, ErrorTrial_Cond, 3, ResetGlobalCounter_Output); // no feedback
                                                                                                              // flashes bin
    states[10] = smart.CreateState("SamplePeriod1", PulseTime * sample_bin, 3, SamplePeriod1_Cond, 3, SamplePeriod1_Output);
    states[11] = smart.CreateState("EarlyLickAborted", S.EarlyLickPeriod, 1, EarlyLickAborted_Cond, 1, ErrorOutput);
    states[12] = smart.CreateState("SamplePeriod3", PulseTime * sample_bin, 3, SamplePeriod3_Cond, 3, SamplePeriod3_Output);

    states[13] = smart.CreateState("SamplePeriod4", PulseTime * sample_bin, 3, SamplePeriod4_Cond, 3, SamplePeriod4_Output);
    states[14] = smart.CreateState("SamplePeriod5", PulseTime * sample_bin, 3, SamplePeriod5_Cond, 3, SamplePeriod5_Output);
    states[15] = smart.CreateState("SamplePeriod6", PulseTime * sample_bin, 3, SamplePeriod6_Cond, 3, SamplePeriod6_Output);
    states[16] = smart.CreateState("SamplePeriod7", PulseTime * sample_bin, 3, SamplePeriod7_Cond, 3, SamplePeriod7_Output);
    states[17] = smart.CreateState("SamplePeriod8", PulseTime * sample_bin, 3, SamplePeriod8_Cond, 3, SamplePeriod8_Output);
    states[18] = smart.CreateState("SamplePeriod9", PulseTime * sample_bin, 3, SamplePeriod9_Cond, 3, SamplePeriod9_Output);

    states[19] = smart.CreateState("SamplePeriodInterval0", IntervalPulse * sample_bin, 3, SamplePeriodInterval0_Cond, 0, NoOutput);
    states[20] = smart.CreateState("SamplePeriodInterval1", IntervalPulse * sample_bin, 3, SamplePeriodInterval1_Cond, 0, NoOutput);
    states[21] = smart.CreateState("SamplePeriodInterval2", IntervalPulse * sample_bin, 3, SamplePeriodInterval2_Cond, 0, NoOutput);
    states[22] = smart.CreateState("SamplePeriodInterval3", IntervalPulse * sample_bin, 3, SamplePeriodInterval3_Cond, 0, NoOutput);
    states[23] = smart.CreateState("SamplePeriodInterval4", IntervalPulse * sample_bin, 3, SamplePeriodInterval4_Cond, 0, NoOutput);
    states[24] = smart.CreateState("SamplePeriodInterval5", IntervalPulse * sample_bin, 3, SamplePeriodInterval5_Cond, 0, NoOutput);
    states[25] = smart.CreateState("SamplePeriodInterval6", IntervalPulse * sample_bin, 3, SamplePeriodInterval6_Cond, 0, NoOutput);
    states[26] = smart.CreateState("SamplePeriodInterval7", IntervalPulse * sample_bin, 3, SamplePeriodInterval7_Cond, 0, NoOutput);
    states[27] = smart.CreateState("SamplePeriodInterval8", IntervalPulse * sample_bin, 3, SamplePeriodInterval8_Cond, 0, NoOutput);
    states[28] = smart.CreateState("SamplePeriodInterval9", IntervalPulse * sample_bin, 3, SamplePeriodInterval9_Cond, 0, NoOutput);

    // pre-cue delay
    states[29] = smart.CreateState("PreCueDelayPeriod", S.PreCueDelayPeriod, 3, PreCueDelayPeriod_Cond, 2, PreCueDelayPeriod_Output);
    states[30] = smart.CreateState("SampleDelayPeriod", S.DelayPeriod, 3, SampleDelayPeriod_Cond, 3, SampleDelayPeriod_Output);
    states[31] = smart.CreateState("SamplePeriod", S.SamplePeriod - S.DelayPeriod, 3, SamplePeriod_Cond, 1, SamplePeriod_Output);
    states[32] = smart.CreateState("SamplePeriod2", PulseTime * sample_bin, 3, SamplePeriod2_Cond, 3, SamplePeriod2_Output);
    states[33] = smart.CreateState("TrialAborted", random(3, 8) * 1000, 1, TrialAborted_Cond, 0, NoOutput); // Early lick punishment : randomly 3s ~ 7s
    states[34] = smart.CreateState("TrialRestart", 10, 1, TrialRestart_Cond, 0, NoOutput);
    states[35] = smart.CreateState("TrialEnd", 10, 1, TrialEnd_Cond, 0, NoOutput);

    states[36] = smart.CreateState("GiveFreeDropClick", reward_dur, 1, GiveFreeDropClick_Cond, 1, Reward_Output);
    //
    states[37] = smart.CreateState("TimeOut", S.TimeOut + S.extra_TimeOut, 1, Tup_Exit_Cond, 0, NoOutput); // not used
    states[38] = smart.CreateState("StopLicking", S.StopLickingPeriod, 4, StopLicking_Cond, 0, NoOutput);
    states[39] = smart.CreateState("StopLickingReturn", 8, 1, Tup_StopLicking_Cond, 0, NoOutput);
    states[40] = smart.CreateState("DelayReward", S.DelayReward, 4, DelayReward_Cond, 3, ResetGlobalCounter_Output);
    states[41] = smart.CreateState("StopTimeInvestment", 20, 1, StopTimeInvestment_Cond, 0, NoOutput);
    states[42] = smart.CreateState("ProbeTrial", 1000 * 20, 4, ProbeTrial_Cond, 3, ResetGlobalCounter_Output); // correct choice happend
    states[43] = smart.CreateState("TrialExit", random(30, 60) * 60 * 1000, 2, TrialExit_Cond, 0, NoOutput);
    states[44] = smart.CreateState("RewardCue", reward_dur, 1, RewardCue_Cond, 1, Reward_Output); // reward cue 30ms water volume
    states[45] = smart.CreateState("ReAnswerPeriod", S.AnswerPeriod, 4, ReAnswerPeriod_Cond, 0, NoOutput);
    states[46] = smart.CreateState("StopTimeInvestmentCue", 20, 1, StopTimeInvestmentCue_Cond, 0, NoOutput);
    // Predefine State sequence.
    for (int i = 0; i < 47; i++)
    {
      smart.AddBlankState(states[i].Name);
    }

    // Add a state to state machine.
    for (int i = 0; i < 47; i++)
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

  TrialOutcome = 3; // 0 no-response; 1 correct; 2 error; 3 earlylick
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

  // byte Outcomes_sum = 0;
  // for (int i = RECORD_TRIALS - recent_trials; i < RECORD_TRIALS; i++)
  // {
  //   if (S.OutcomeHistory[i] == 1)
  //   {
  //     Outcomes_sum++;
  //   }
  // }
  // S.currProtocolPerf = round((float)Outcomes_sum / recent_trials * 100);

  // EMA S.currProtocolPerf ; alpha is 0.02 ; 50 trials average
  float CurrOutcome = 0;
  // recording the last valid trial
  if (S.OutcomeHistory[RECORD_TRIALS - 1] != 3 && S.OutcomeHistory[RECORD_TRIALS - 1] != 0)
  {
    S.currProtocolTrials++;
    if (S.OutcomeHistory[RECORD_TRIALS - 1] == 1)
    {
      CurrOutcome = 1.0;
    }
    else if (S.OutcomeHistory[RECORD_TRIALS - 1] == 2)
    {
      CurrOutcome = 0.0;
    }
    S.currProtocolPerf = S.currProtocolPerf * Alpha50 + (1 - Alpha50) * CurrOutcome;
    Perf100 = Perf100 * Alpha100 + (1 - Alpha100) * CurrOutcome;
  }
  currProtocolPerf_corrected = S.currProtocolPerf / (1 - pow(Alpha50 ,S.currProtocolTrials));
  Perf100_corrected = Perf100 / (1 - pow(Alpha100 ,S.currTrialNum));

  EarlyLick100 = 0;
  for (int i = 0; i < RECORD_TRIALS; i++)
  {
    // if (S.OutcomeHistory[i] == 1)
    // {
    //   Perf100++;
    // }
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

  TrialInfo_str += String(int(Perf100 * 100));
  TrialInfo_str += ",";

  TrialInfo_str += String(S.currProtocolTrials);
  TrialInfo_str += ",";

  TrialInfo_str += String(int(currProtocolPerf_corrected * 100));
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
    if (S.currProtocolTrials > 1000)
    {
      S.currProtocolIndex = 1; // Advanced to Protocol 1: pattern left/right
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod = 250;
      S.TimeOut = 2000;
      S.AnswerPeriod = 5000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 500;
      S.EarlyLickPeriod = 500;
      S.TrialPresentMode = 0; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 1:
    if (S.currProtocolTrials > 100 && int(currProtocolPerf_corrected * 100) > 75)
    {
      S.currProtocolIndex = 2; // Advanced to Protocol 2: antiBias left/right
      S.currProtocolTrials = 0; 
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod = 250;
      S.TimeOut = 4000;
      S.AnswerPeriod = 3000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 800;
      S.EarlyLickPeriod = 500;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 2:
    if (S.currProtocolTrials > 100 && int(currProtocolPerf_corrected * 100) > 75)
    {
      S.currProtocolIndex = 3; // Advanced to Protocol 3: antiBias left/right w/ forced delay 0.5 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod = 250;
      S.TimeOut = 6000;
      S.AnswerPeriod = 2000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 800;
      S.EarlyLickPeriod = 500;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 3:
    if (S.currProtocolTrials > 100 && int(currProtocolPerf_corrected * 100) > 75)
    {
      S.currProtocolIndex = 4; // Advanced to Protocol 4: antiBias left/right w/ forced delay 0.8 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod = 250;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod = 500;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 4:
    if (S.currProtocolTrials > 100 && int(Perf100_corrected * 100) > 70)
    {
      S.currProtocolIndex = 5; // Advanced to Protocol 5: antiBias left/right w/ forced delay 1.0 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod = 250;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod = 500;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;
  // delayed reward
  case 5:
    if (S.currProtocolTrials > 1000)
    {
      S.currProtocolIndex = 6; // Advanced to Protocol 6: antiBias left/right w/ forced delay 1.2 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod = 250;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod = 500;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;

  case 6:
    if (S.currProtocolTrials > 1000)
    {
      S.currProtocolIndex = 7; // Advanced to Protocol 7: antibias left/right w/ forced delay 1.2 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod = 250;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod = 500;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;
  case 7:
    if (S.currProtocolTrials > 1000)
    {
      S.currProtocolIndex = 8; // Advanced to Protocol 7: antibias left/right w/ forced delay 1.2 sec
      S.currProtocolTrials = 0;
      S.currProtocolPerf = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod = 250;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod = 500;
      S.TrialPresentMode = 2; // 0"pattern",1"random",2"antiBias",3"fixed"
    }
    break;
  case 8:
    if (S.currProtocolTrials > 1000)
    {
        S.currProtocolIndex = 9;

        S.currProtocolTrials = 0;
        S.currProtocolPerf = 0;

        S.SamplePeriod = 500;
        S.DelayPeriod = 250;
        S.TimeOut = 8000;
        S.AnswerPeriod = 1000;
        S.ConsumptionPeriod = 750;
        S.StopLickingPeriod = 1000;
        S.EarlyLickPeriod = 500;
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
    dataFile.println(S.currProtocolPerf ,4);
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
    // DelayReward
    dataFile.print("DelayReward = ");
    dataFile.println(S.DelayReward);
    // ModalityFlag
    dataFile.print("ModalityFlag = ");
    dataFile.println(S.ModalityFlag);
    // Perf100
    dataFile.print("Perf100 = ");
    dataFile.println(Perf100 ,4);
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
    S.currProtocolPerf = string_tmp.toFloat();
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
    // DelayReward
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.DelayReward = string_tmp.toInt();
    // ModalityFlag
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.ModalityFlag = string_tmp.toInt();
    // Perf100
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    Perf100 = string_tmp.toFloat();
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

    dataFile.print(TrialTypeContrastLeft); // alter the TrialType
    dataFile.print(" ");

    dataFile.print(TrialTypeContrastRight);
    dataFile.print(" ");

    dataFile.print(difficultLevel);
    dataFile.print(" ");

    dataFile.print(S.DelayReward);
    dataFile.print(" ");

    dataFile.print(modality);
    dataFile.print(" ");

    dataFile.print(ProbeTrialFlag);
    dataFile.print(" ");

    dataFile.print(S.PreCueDelayPeriod);
    dataFile.print(" ");

    // poisson events
    for (int i = 0; i < SampleBinNum; i++)
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
