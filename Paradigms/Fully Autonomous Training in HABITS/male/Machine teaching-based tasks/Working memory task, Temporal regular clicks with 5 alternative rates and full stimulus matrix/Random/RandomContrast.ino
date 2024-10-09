/*
* ud freq + reverse + aud orientation + reverse + light orientation + reverse + transparent
*	其中在aud freq 和 aud orientation的变化过程中，可以加入一个手动给的bias来看看模型是否能够处
*  理这个bias；给的方法就是在aud orientation任务中给予的声音频率 为aud freq 任务中的一个，这样
*  在刚进入到aud orientation任务的时候可能会引入一个明显的bias，使用trial block的方式来循环的
*  更新bias 的方向（right bias -》 left bias -》no bias）
*/

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
  float currProtocolPerf = 0;           // performance: 0-100%
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
  int SamplePeriod = 500;
  int DelayPeriod = 500;
  int OnsetDelayPeriod = 500;
  int TimeOut = 1000;
  int AnswerPeriod = 3000;
  int ConsumptionPeriod = 750;
  int StopLickingPeriod = 1000;
  int EarlyLickPeriod = 100;
  int extra_TimeOut = 0;

  int PreCueDelayPeriod = 200;
  int EventsDuration = 5;
  int EventsInterval[5] = {120 ,58 ,26 ,11 ,3};
  int PsychometricInterval[7] = {58 ,45 ,35 ,26 ,20 ,15 ,11};
  int NoResponseTimeOut = 15000;

  float Perf100 = 0;
  float TrialContrastRate = 0; // the rate of most difficult trial type in the last 100 trials
  unsigned int hardestTrials = 0;
} Parameters_behavior;
Parameters_behavior S;

/********** Define OutputAction**********/
OutputAction LeftWaterOutput = {"DO1", 1};
OutputAction RightWaterOutput = {"DO2", 1};
OutputAction CenterWaterOutput = {"DO3", 1};

OutputAction LowSoundOutput = {"tPWM2", 1}; // 3000
OutputAction HighSoundOutput = {"tPWM2", 2}; // 12000
OutputAction CueOutput = {"tPWM2", 3}; // 6000

OutputAction NoiseOutput = {"Flag1", 1};

OutputAction LeftSoundOutput = {"tPWM1", 3};  // tPWM1 left sound ,freq is 6KHz
OutputAction LeftSoundOutputH = {"tPWM1", 2}; // high freq 12khz 
OutputAction LeftSoundOutputL = {"tPWM1", 1}; // Low freq 3khz

OutputAction RightSoundOutput = {"tPWM3", 3}; // tPWM3 right sound ,freq is 6KHz
OutputAction RightSoundOutputH = {"tPWM3", 2}; // tPWM3 right sound ,freq is 12KHz
OutputAction RightSoundOutputL = {"tPWM3", 1}; // tPWM3 right sound ,freq is 3KHz

// Note: the freq setting is working for all regular PWM below
OutputAction LeftLightOutput = {"PWM1", S.low_light_intensity};  // left light, value 0-255
OutputAction RightLightOutput = {"PWM5", S.low_light_intensity}; // right light ,same as above

OutputAction BlueLightOutput = {"PWM4", S.high_light_intensity};  // Blue light...
OutputAction RedLightOutput = {"PWM2", S.high_light_intensity};   // red light...
OutputAction GreenLightOutput = {"PWM3", S.high_light_intensity}; // Green light...

// light intensity
OutputAction LeftLightOutputH = {"PWM1", S.high_light_intensity};  // High left light intensity
OutputAction RightLightOutputH = {"PWM5", S.high_light_intensity}; // High right light intensity

OutputAction SampleTimerTrigOutput = {"GlobalTimerTrig" ,1}; // active the global timer 1
OutputAction SampleTimerCancelOutput = {"GlobalTimerCancel" ,1}; // inactive the global timer 1

// sound frequency ;sound orients ;light orients ;reversal trials ;wavelength ;light intensity;

byte TrialType = 1;    // 0 udef; 1 left; 2 right; 3 middle;
byte TrialOutcome = 3; // 0 no-response; 1 correct; 2 error; 3 others
byte is_earlylick = 2; // 0-no earlylick; 1-earlylick; 2-undef
byte MaxSame = 3;
byte MinCorrect = 1; // minimum correct trials in the last 5 trials

byte TrialTypeContrast[2] = {0 ,0}; // X1 & X2 ; range from 0 ~ 4
byte TrialTypeContrastPC[2] = {3 ,3}; // 0~6 ; {16 ,20 ,25 ,32 ,40 ,50 ,64}

byte ELInhibitaionStage[4] = {0} ; // 3 stage : 1 -> onset delay ; 2-> Stimulus 1 ; 3 -> Delay 1 ; 4 -> Stimulus 2 ; value :0 or 2 

float Alpha50 = 0.98;
float Alpha100 = 0.99;
float currProtocolPerf_corrected = 0;
float Perf100_corrected = 0;
float TrialContrastRate_corrected = 0;


byte EarlyLick100 = 0;
unsigned long last_reward_time = 0;
int timed_reward_count = 0;

int modality = 0; // sample stimulus : 0 is no PC ;1 is PC

int CatchTrialProb = 2; // use a small number of trials as catch trials

byte pause_signal_PC = 0;
byte tare_flag = 1;
byte tare_length = 0;
bool paused = 1;
byte ledState = LOW;


/********** AlignMax Machine Teaching Logistic Regression based **********/

// 定义数据结构
struct DATAModel{
    float x[7] = {}; // 特征变量 bias S0{X1 ,X2} WSLS S1 A1 R1
    int y = 0; // 目标变量 Left or Right 
};
struct DATAModel MiceChoiceData;
struct DATAModel TrialTypeData;

// 定义逻辑回归模型结构
typedef struct {
    float theta[7] = {0}; // 参数向量，包括截距项
    float alpha = 0.01; // 学习率 Correct Sample
    float L1        = 0;        // L1 regularization value
    float Gamma    = 0.03;    // argMax param: learning rate of Mouse model 用来调整自适应训练算法的策略
    float Eta = 1.0; // argMax中小鼠模型对梯度的学习率
    float beta      = 0;  // Momentum param
    int    t         = 0;    // iterative number
    float m[7]     = {0};    // moment value
    float S0[5] = {-1, 0.5, 0, 0.5, 1}; // S0 params 
} LOGISTIC;
//LOGISTIC *model; //  注意；这里 只是定义了一个结构体的指针，并没有实例化这个结构体；需要实例化后再让这个指针指向这个实例结构体变量,这样可以节约空间，结构体实例可以随时free掉
LOGISTIC model; // 使用off-policy
LOGISTIC actionmodel;

// float SGM[5][5][2] = {}; // stimuli generation matrix 注意去掉 X1 == X2 的情况
// float PsychometricSGM[7][2] = {}; // test2 : calc the psychometric curve

float W_star[7] = {0 ,10 ,-10 ,0 ,0 ,0 ,0}; // 目标参数 X1>X2 turn right else turn left
float loss = 0; // teacher model loss

int batchsize = 2; 
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
  // tPWM2:
  smart.setTruePWMFrequency(2, 1, 3000, 128);  // (low sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(2, 2, 12000, 128); // (high sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(2, 3, 6000, 128);  // (go cue) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  FLASHLED(200);
  // tPWM1
  smart.setTruePWMFrequency(1, 1, 3000, 128);  // (low sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(1, 2, 12000, 128); // (high sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(1, 3, 6000, 128);  // (go cue) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  FLASHLED(200);
  // tPWM3
  smart.setTruePWMFrequency(3, 1, 3000, 128);  // (low sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(3, 2, 12000, 128); // (high sound) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  smart.setTruePWMFrequency(3, 3, 6000, 128);  // (go cue) byte tPWM_num, byte freq_num, uint32 frequency, byte duty
  FLASHLED(200);

  // read parameters from SD Card to override S
  read_SD_para_S();
  read_Model_Param_info();
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
      read_Model_Param_info();
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
  OutputAction LeftLightOutput = {"PWM1", S.low_light_intensity};  // left light, value 0-255
  OutputAction RightLightOutput = {"PWM5", S.low_light_intensity}; // right light ,same as above
  OutputAction NoSampleOutput = {"Flag4" , 0}; // empty output

  smart.EmptyMatrix(); // clear matrix at the begining of each trial
  
  modality = 0;

  // test
  Serial.print(TrialTypeContrast[0]);
  Serial.print(" ");
  Serial.println(TrialTypeContrast[1]);
  Serial.print(TrialTypeContrastPC[0]);
  Serial.print(" ");
  Serial.println(TrialTypeContrastPC[1]);
  switch (S.currProtocolIndex)
  {
  case 0: // habitation 1 : answerPeriod : 5s ; earlylick inhibitation : onset delay
  case 1: // habitation 2 : earlylick inhibitation period :{onset delay -> Stimuli 1 -> Delay 1 -> Stimuli 2 } per 500 valid trials ; AnswerPeriod:{5s->3s->1s} 
  {                             
    // Earlylick inhibitation  answerPeriod
    int freeRewardProb = 0;
    int ELTimeOutPeriod = 3000;
    switch(S.currProtocolIndex){
      case 0:
      { 
        ELInhibitaionStage[0] = 2;
        ELInhibitaionStage[1] = 0;
        ELInhibitaionStage[2] = 0;
        ELInhibitaionStage[3] = 0;
        S.AnswerPeriod = 5000;
        freeRewardProb = 60;
        ELTimeOutPeriod = exponent(3.0, 3.0, 8.0);
      }
      break;
      case 1:
      {
        freeRewardProb = 60; // remove free reward
        switch ((int)(S.currProtocolTrials / 500))
        {
        case 0:
          {
            ELInhibitaionStage[0] = 2;
            ELInhibitaionStage[1] = 0;
            ELInhibitaionStage[2] = 0;
            ELInhibitaionStage[3] = 0;
            S.AnswerPeriod = 5000;
            ELTimeOutPeriod = exponent(3.0, 3.0, 8.0);
          }
            break;
        case 1:
          {
            ELInhibitaionStage[0] = 2;
            ELInhibitaionStage[1] = 2;
            ELInhibitaionStage[2] = 0;
            ELInhibitaionStage[3] = 0;
            S.AnswerPeriod = 3000;
            ELTimeOutPeriod = exponent(4.0, 3.0, 8.0);
          }
          break;
        case 2:
          {
            ELInhibitaionStage[0] = 2;
            ELInhibitaionStage[1] = 2;
            ELInhibitaionStage[2] = 2;
            ELInhibitaionStage[3] = 0;
            S.AnswerPeriod = 2000;
            ELTimeOutPeriod = exponent(5.0, 3.0, 8.0);
          }
          break;
        case 3:
          {
            ELInhibitaionStage[0] = 2;
            ELInhibitaionStage[1] = 2;
            ELInhibitaionStage[2] = 2;
            ELInhibitaionStage[3] = 2;
            S.AnswerPeriod = 1000;
            ELTimeOutPeriod = exponent(6.0, 3.0, 8.0);
          }
          break;
        default:
        {
          ELInhibitaionStage[0] = 2;
          ELInhibitaionStage[1] = 2;
          ELInhibitaionStage[2] = 2;
          ELInhibitaionStage[3] = 2;
          S.AnswerPeriod = 1000;
          ELTimeOutPeriod = exponent(6.0, 3.0, 8.0);
        }
          break;
        }
      }
      default:
      break;
    } 

    OutputAction SampleOutput; 
    OutputAction RewardOutput;

    S.OnsetDelayPeriod = float(random(200, 500));

    String LeftLickAction;
    String RightLickAction;
    String CenterLickAction;
    String AnswerLeftLickAction;
    String AnswerRightLickAction;
    String AnswerCenterLickAction;
    String ActionAfterDelay;
    float reward_dur = S.reward_left;

    SampleOutput = HighSoundOutput;

    //TrialTypeContrast
  
    switch (TrialType) // determine left or right
    {
    case 1: //  left
    {
     
      LeftLickAction = "Reward";
      RightLickAction = "SamplePeriod"; // no error trial
      
      AnswerLeftLickAction = "Reward";
      AnswerRightLickAction = "AnswerPeriod";
      RewardOutput = LeftWaterOutput;
      reward_dur = S.reward_left;
    }
    break;
    case 2: // right
    {

      LeftLickAction = "SamplePeriod";
      RightLickAction = "Reward"; // no error trial
      
      AnswerLeftLickAction = "AnswerPeriod";
      AnswerRightLickAction = "Reward";
      RewardOutput = RightWaterOutput;
      reward_dur = S.reward_right;
    }
    break;
    }

    if (int(random(100)) < freeRewardProb)
    { // free reward probability
      ActionAfterDelay = "GiveFreeDrop";
    }
    else
    {
      ActionAfterDelay = "AnswerPeriod";
    }

    // global timer 
    smart.SetGlobalTimer(1,  S.SamplePeriod + 10);

    
    StateTransition OnsetDelayPeriod_Cond[3] = {{"Tup", "SampleActive"},{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition TrialStart_Cond[1] = {{"Tup", "OnsetDelayPeriod"}}; 
    StateTransition SampleActive_Cond[3] = {{"Tup" ,"SampleEventPeriod"} ,{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SampleEventPeriod_Cond[4]   = {{"Tup", "SampleIntervalPeriod"} ,{"GlobalTimer1_End" ,"DelayPeriod"} ,{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SampleIntervalPeriod_Cond[4] = {{"Tup", "SampleEventPeriod"} ,{"GlobalTimer1_End" ,"DelayPeriod"},{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};

    StateTransition DelayPeriod_Cond[3] = {{"Tup" ,"TestActive"},{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};

    StateTransition TestActive_Cond[3] = {{"Tup" ,"TestEventPeriod"},{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition TestEventPeriod_Cond[4]   = {{"Tup", "TestIntervalPeriod"} ,{"GlobalTimer1_End" ,"PreCueDelayPeriod"},{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition TestIntervalPeriod_Cond[4] = {{"Tup", "TestEventPeriod"} ,{"GlobalTimer1_End" ,"PreCueDelayPeriod"},{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};

    StateTransition PreCueDelayPeriod_Cond[1] = {{"Tup" ,"ResponseCue"}};

    StateTransition ResponseCue_Cond[1] = {{"Tup", ActionAfterDelay}}; 
    StateTransition GiveFreeDrop_Cond[1] = {{"Tup", "AnswerPeriod"}};
    StateTransition AnswerPeriod_Cond[3] = {{"Lick1In", AnswerLeftLickAction}, {"Lick2In", AnswerRightLickAction} ,{"Tup", "NoResponse"}};
    StateTransition Reward_Cond[1] = {{"Tup", "RewardConsumption"}};
    StateTransition Tup_Exit_Cond[1] = {{"Tup", "TrialEndCue"}};
    // StateTransition NoResponse_Cond[1] = {{"Tup", "TrialEnd"}};
    StateTransition ErrorTrial_Cond[1] = {{"Tup", "exit"}};
    StateTransition EarlyLickAborted_Cond[1]     = {{"Tup" ,"TrialAborted"}};
    StateTransition TrialAborted_Cond[1]   = {{"Tup", "TrialEndCue"}};
    StateTransition TrialEndCue_Cond[1]   = {{"Tup", "TrialEnd"}};
    StateTransition TrialEnd_Cond[2] = {{"Lick3In", "exit"} ,{"Tup", "exit"}};

    OutputAction SamplePeriod_Output[3]     	   = {SampleOutput ,LeftLightOutput ,RightLightOutput};
    OutputAction OnsetDelayPeriod_Output[3] = {GreenLightOutputBackground ,LeftLightOutput ,RightLightOutput}; // low intensity green light as a trial start signal
    OutputAction ResponseCue_Output[1] = {CueOutput}; 
    OutputAction FreeReward_Output[1] = {RewardOutput};
    OutputAction Reward_Output[1] = {RewardOutput};
    OutputAction NoOutput[0] = {};
    OutputAction EL_Output[2] = {LeftLightOutput ,RightLightOutput};
    OutputAction ErrorOutput[2] = {NoiseOutput ,SampleTimerCancelOutput}; 
    OutputAction TrialEndCue_Output[1] = {LowSoundOutput}; // 3000hz cue

    OutputAction SampleActive_Output[3] = {SampleTimerTrigOutput ,LeftLightOutput ,RightLightOutput};
    OutputAction DelayPeriod_Ouput[3] = {SampleTimerCancelOutput ,LeftLightOutput ,RightLightOutput};
    OutputAction TestActive_Output[3] = {SampleTimerTrigOutput ,LeftLightOutput ,RightLightOutput};
    OutputAction PreCueDelayPeriod_Ouput[1] = {SampleTimerCancelOutput};
    
    

    gpSMART_State states[21] = {};
    // visual flash states and conditions
    states[0] = smart.CreateState("TrialStart", 5, 1, TrialStart_Cond, 0, NoOutput); // msec
    states[10] = smart.CreateState("OnsetDelayPeriod", S.OnsetDelayPeriod, 1 + ELInhibitaionStage[0], OnsetDelayPeriod_Cond, 1 + ELInhibitaionStage[0], OnsetDelayPeriod_Output);

    states[15] = smart.CreateState("SampleActive",    10,    1 + ELInhibitaionStage[1], SampleActive_Cond,  1 + ELInhibitaionStage[1], SampleActive_Output);
    states[1] = smart.CreateState("SampleEventPeriod", S.EventsDuration, 2 + ELInhibitaionStage[1], SampleEventPeriod_Cond, 1 + ELInhibitaionStage[1], SamplePeriod_Output);
    states[2] = smart.CreateState("SampleIntervalPeriod", S.EventsInterval[TrialTypeContrast[0]], 2 + ELInhibitaionStage[1], SampleIntervalPeriod_Cond, 0 + ELInhibitaionStage[1], EL_Output);

    states[16] = smart.CreateState("DelayPeriod",    S.DelayPeriod,    1 + ELInhibitaionStage[2], DelayPeriod_Cond,  1 + ELInhibitaionStage[2], DelayPeriod_Ouput);

    states[17] = smart.CreateState("TestActive",    10,    1 + ELInhibitaionStage[3], TestActive_Cond,  1 + ELInhibitaionStage[3], TestActive_Output);
    states[18] = smart.CreateState("TestEventPeriod", S.EventsDuration, 2 + ELInhibitaionStage[3], TestEventPeriod_Cond, 1 + ELInhibitaionStage[3], SamplePeriod_Output);
    states[19] = smart.CreateState("TestIntervalPeriod", S.EventsInterval[TrialTypeContrast[1]], 2 + ELInhibitaionStage[3], TestIntervalPeriod_Cond, 0 + ELInhibitaionStage[3], EL_Output);

    states[20] = smart.CreateState("PreCueDelayPeriod",    S.PreCueDelayPeriod,    1, PreCueDelayPeriod_Cond,  1, PreCueDelayPeriod_Ouput);

    states[3] = smart.CreateState("ResponseCue", 100, 1, ResponseCue_Cond, 1, ResponseCue_Output); 
    states[4] = smart.CreateState("GiveFreeDrop", reward_dur, 1, GiveFreeDrop_Cond, 1, FreeReward_Output);
    states[5] = smart.CreateState("AnswerPeriod", S.AnswerPeriod, 3, AnswerPeriod_Cond, 0, NoOutput);
    states[6] = smart.CreateState("Reward", reward_dur, 1, Reward_Cond, 1, Reward_Output);
    states[7] = smart.CreateState("RewardConsumption", S.ConsumptionPeriod, 1, Tup_Exit_Cond, 0, NoOutput);
    states[8] = smart.CreateState("NoResponse", S.NoResponseTimeOut, 1, Tup_Exit_Cond, 0, NoOutput); // NoResponse timeout punishment
    // error trial state (not used state)
    states[9] = smart.CreateState("ErrorTrial", 500, 1, ErrorTrial_Cond, 1, ErrorOutput); // not used
    // pre-cue delay
    states[11] = smart.CreateState("EarlyLickAborted",    S.EarlyLickPeriod,    1, EarlyLickAborted_Cond,  2, ErrorOutput);
    states[12] = smart.CreateState("TrialAborted",   ELTimeOutPeriod ,    1, TrialAborted_Cond,  0, NoOutput); // Early lick punishment : randomly 3s ~ 8s
    states[13] = smart.CreateState("TrialEndCue",    100,    1, TrialEndCue_Cond,  1, TrialEndCue_Output);
    states[14] = smart.CreateState("TrialEnd",    random(30, 60) * 60 * 1000,    2, TrialEnd_Cond,  0, NoOutput);
    
    
    // Predefine State sequence.
    for (int i = 0; i < 21; i++)
    {
      smart.AddBlankState(states[i].Name);
    }

    // Add a state to state machine.
    for (int i = 0; i < 21; i++)
    {
      smart.AddState(&states[i]);
    }

    // smart.PrintMatrix(); // for debug

    // Run the matrix
    smart.Run();
  }
  break;

  case 2: // Test 0
  case 3: // Early training
  case 4: // Retention protocol
  case 5: // Test 1
  case 6: // Test 2
  {                  
    int Sample_EventsInterval =  S.EventsInterval[TrialTypeContrast[0]];
    int Test_EventsInterval =  S.EventsInterval[TrialTypeContrast[1]];

    int ELTimeOutPeriod = exponent(6.0, 3.0, 8.0);

    ELInhibitaionStage[0] = 2;
    ELInhibitaionStage[1] = 2;
    ELInhibitaionStage[2] = 2;
    ELInhibitaionStage[3] = 2;

    // currProtocolIndex 
    switch (S.currProtocolIndex)
    {
    case 2:
      switch ((int)(S.currProtocolTrials / 500))
      {
        case 0:
         S.TimeOut = 2000;
         S.StopLickingPeriod = 400;
        break;
        case 1:
          S.TimeOut = 4000;
         S.StopLickingPeriod = 600;
        break;
        case 2:
          S.TimeOut = 6000;
         S.StopLickingPeriod = 800;
        break;
        case 3:
          S.TimeOut = 8000;
         S.StopLickingPeriod = 1000;
        break;
        default:
          S.TimeOut = 8000;
         S.StopLickingPeriod = 1000;
        break;
      }
      break;
    case 5: // Test 1
      {
        S.DelayPeriod = int(random(500 ,2000));
      }
      break;
    case 6: // Test 2
    {
      S.DelayPeriod = int(random(500 ,2000));
       if(int(random(2)) == 0){
        modality = 1;
        // psychometric sample
        Sample_EventsInterval = S.PsychometricInterval[TrialTypeContrastPC[0]];
        Test_EventsInterval = S.PsychometricInterval[TrialTypeContrastPC[1]];
        // Trial type
        if(TrialTypeContrastPC[0] < TrialTypeContrastPC[1]){
          TrialType = 1;
        }else if(TrialTypeContrastPC[0] > TrialTypeContrastPC[1]){
          TrialType = 2;
        }else{ // Stimuli 1 == Stimuli 2 ; randomly reward
          TrialType = int(random(0 ,2)) + 1;
        }
      }else{
        modality = 0;
      }
    }
    break;
    default:
      break;
    }

    OutputAction SampleOutput; 
    OutputAction RewardOutput;

    S.OnsetDelayPeriod = float(random(200, 500));

    String LeftLickAction;
    String RightLickAction;
    String CenterLickAction;
    String AnswerLeftLickAction;
    String AnswerRightLickAction;
    String AnswerCenterLickAction;
    String ActionAfterDelay;
    float reward_dur = S.reward_left;

    SampleOutput = HighSoundOutput;
  
    switch (TrialType) // determine left or right
    {
    case 1: //  left
    {
      LeftLickAction = "Reward";
      RightLickAction = "ErrorTrial"; // no error trial
      
      AnswerLeftLickAction = "Reward";
      AnswerRightLickAction = "ErrorTrial";
      AnswerCenterLickAction = "TrialRestart";
      RewardOutput = LeftWaterOutput;
      reward_dur = S.reward_left;
    }
    break;
    case 2: // right
    {
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

    // free reward
    if ((TrialType == 1 && S.GaveFreeReward.flag_L_water == 1) || (TrialType == 2 && S.GaveFreeReward.flag_R_water == 1))
    {
      ActionAfterDelay = "GiveFreeDrop";
      if (TrialType == 1)
      {
        S.GaveFreeReward.flag_L_water = 0;
      }
      else if(TrialType == 2)
      {
        S.GaveFreeReward.flag_R_water = 0;
      }
    }
    else
    {
      ActionAfterDelay = "AnswerPeriod";
    }

    // global timer 
    smart.SetGlobalTimer(1,  S.SamplePeriod + 10);

    StateTransition OnsetDelayPeriod_Cond[3] = {{"Tup", "SampleActive"},{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition TrialStart_Cond[1] = {{"Tup", "OnsetDelayPeriod"}}; 
    StateTransition SampleActive_Cond[3] = {{"Tup" ,"SampleEventPeriod"} ,{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SampleEventPeriod_Cond[4]   = {{"Tup", "SampleIntervalPeriod"} ,{"GlobalTimer1_End" ,"DelayPeriod"} ,{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition SampleIntervalPeriod_Cond[4] = {{"Tup", "SampleEventPeriod"} ,{"GlobalTimer1_End" ,"DelayPeriod"},{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};

    StateTransition DelayPeriod_Cond[3] = {{"Tup" ,"TestActive"},{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};

    StateTransition TestActive_Cond[3] = {{"Tup" ,"TestEventPeriod"},{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition TestEventPeriod_Cond[4]   = {{"Tup", "TestIntervalPeriod"} ,{"GlobalTimer1_End" ,"PreCueDelayPeriod"},{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};
    StateTransition TestIntervalPeriod_Cond[4] = {{"Tup", "TestEventPeriod"} ,{"GlobalTimer1_End" ,"PreCueDelayPeriod"},{"Lick1In", "EarlyLickAborted"}, {"Lick2In", "EarlyLickAborted"}};

    StateTransition PreCueDelayPeriod_Cond[1] = {{"Tup" ,"ResponseCue"}};

    StateTransition ResponseCue_Cond[1] = {{"Tup", ActionAfterDelay}}; 
    StateTransition GiveFreeDrop_Cond[1] = {{"Tup", "AnswerPeriod"}};
    StateTransition AnswerPeriod_Cond[3] = {{"Lick1In", AnswerLeftLickAction}, {"Lick2In", AnswerRightLickAction} ,{"Tup", "NoResponse"}};
    StateTransition Reward_Cond[1] = {{"Tup", "RewardConsumption"}};
    StateTransition Tup_Exit_Cond[1] = {{"Tup", "TrialEndCue"}};
    StateTransition Tup_StopLicking_Cond[1] = {{"Tup", "StopLicking"}};
    StateTransition StopLicking_Cond[4] = {{"Lick1In", "StopLickingReturn"}, {"Lick2In", "StopLickingReturn"}, {"Lick3In", "StopLickingReturn"}, {"Tup", "TrialEndCue"}}; 
    // StateTransition NoResponse_Cond[1] = {{"Tup", "TrialEnd"}};
    StateTransition ErrorTrial_Cond[1] = {{"Tup", "TimeOut"}};
    StateTransition EarlyLickAborted_Cond[1]     = {{"Tup" ,"TrialAborted"}};
    StateTransition TrialAborted_Cond[1]   = {{"Tup", "TrialEndCue"}};
    StateTransition TrialEndCue_Cond[1]   = {{"Tup", "TrialEnd"}};
    StateTransition TrialEnd_Cond[2] = {{"Lick3In", "exit"} ,{"Tup", "exit"}};

    OutputAction SamplePeriod_Output[3]     	   = {SampleOutput,LeftLightOutput ,RightLightOutput};
    OutputAction OnsetDelayPeriod_Output[3] = {GreenLightOutputBackground ,LeftLightOutput ,RightLightOutput}; // low intensity green light as a trial start signal
    OutputAction ResponseCue_Output[1] = {CueOutput}; 
    OutputAction FreeReward_Output[1] = {RewardOutput};
    OutputAction Reward_Output[1] = {RewardOutput};
    OutputAction NoOutput[0] = {};
    OutputAction EL_Output[2] = {LeftLightOutput ,RightLightOutput};
    OutputAction ErrorOutput[2] = {NoiseOutput ,SampleTimerCancelOutput}; 
    OutputAction TrialEndCue_Output[1] = {LowSoundOutput}; // 3000hz cue

    OutputAction SampleActive_Output[3] = {SampleTimerTrigOutput ,LeftLightOutput ,RightLightOutput};
    OutputAction DelayPeriod_Ouput[3] = {SampleTimerCancelOutput ,LeftLightOutput ,RightLightOutput};
    OutputAction TestActive_Output[3] = {SampleTimerTrigOutput ,LeftLightOutput ,RightLightOutput};
    OutputAction PreCueDelayPeriod_Ouput[1] = {SampleTimerCancelOutput};
    
    

    gpSMART_State states[24] = {};
    // visual flash states and conditions
    states[0] = smart.CreateState("TrialStart", 5, 1, TrialStart_Cond, 0, NoOutput); // msec
    states[10] = smart.CreateState("OnsetDelayPeriod", S.OnsetDelayPeriod, 1 + ELInhibitaionStage[0], OnsetDelayPeriod_Cond, 3, OnsetDelayPeriod_Output);

    states[15] = smart.CreateState("SampleActive",    10,    1 + ELInhibitaionStage[1], SampleActive_Cond,  3, SampleActive_Output);
    states[1] = smart.CreateState("SampleEventPeriod", S.EventsDuration, 2 + ELInhibitaionStage[1], SampleEventPeriod_Cond, 3, SamplePeriod_Output);
    states[2] = smart.CreateState("SampleIntervalPeriod", Sample_EventsInterval, 2 + ELInhibitaionStage[1], SampleIntervalPeriod_Cond, 2, EL_Output);

    states[16] = smart.CreateState("DelayPeriod",    S.DelayPeriod,    1 + ELInhibitaionStage[2], DelayPeriod_Cond,  3, DelayPeriod_Ouput);

    states[17] = smart.CreateState("TestActive",    10,    1 + ELInhibitaionStage[3], TestActive_Cond,  3, TestActive_Output);
    states[18] = smart.CreateState("TestEventPeriod", S.EventsDuration, 2 + ELInhibitaionStage[3], TestEventPeriod_Cond, 3, SamplePeriod_Output);
    states[19] = smart.CreateState("TestIntervalPeriod", Test_EventsInterval, 2 + ELInhibitaionStage[3], TestIntervalPeriod_Cond, 2, EL_Output);

    states[20] = smart.CreateState("PreCueDelayPeriod",    S.PreCueDelayPeriod,    1, PreCueDelayPeriod_Cond,  1, PreCueDelayPeriod_Ouput);

    states[3] = smart.CreateState("ResponseCue", 100, 1, ResponseCue_Cond, 1, ResponseCue_Output); 
    states[4] = smart.CreateState("GiveFreeDrop", reward_dur, 1, GiveFreeDrop_Cond, 1, FreeReward_Output);
    states[5] = smart.CreateState("AnswerPeriod", S.AnswerPeriod, 3, AnswerPeriod_Cond, 0, NoOutput);
    states[6] = smart.CreateState("Reward", reward_dur, 1, Reward_Cond, 1, Reward_Output);
    states[7] = smart.CreateState("RewardConsumption", S.ConsumptionPeriod, 1, Tup_StopLicking_Cond, 0, NoOutput);
    states[8] = smart.CreateState("NoResponse", S.NoResponseTimeOut, 1, Tup_Exit_Cond, 0, NoOutput); // NoResponse timeout punishment

    states[22] = smart.CreateState("StopLicking",       S.StopLickingPeriod,  4, StopLicking_Cond,     0, NoOutput);
    states[23] = smart.CreateState("StopLickingReturn", 100,                   1, Tup_StopLicking_Cond, 0, NoOutput); //TODO 100 ?
    // error trial state 
    states[9] = smart.CreateState("ErrorTrial", 500, 1, ErrorTrial_Cond, 1, ErrorOutput); 
    states[21]  = smart.CreateState("TimeOut",   S.TimeOut + S.extra_TimeOut,   1, Tup_Exit_Cond,        0, NoOutput);
    
    states[11] = smart.CreateState("EarlyLickAborted",    S.EarlyLickPeriod,    1, EarlyLickAborted_Cond,  2, ErrorOutput);
    states[12] = smart.CreateState("TrialAborted",   ELTimeOutPeriod ,    1, TrialAborted_Cond,  0, NoOutput); // Early lick punishment : randomly 3s ~ 8s
    states[13] = smart.CreateState("TrialEndCue",    100,    1, TrialEndCue_Cond,  1, TrialEndCue_Output);
    states[14] = smart.CreateState("TrialEnd",    random(30, 60) * 60 * 1000,    2, TrialEnd_Cond,  0, NoOutput);
    
    
    // Predefine State sequence.
    for (int i = 0; i < 24; i++)
    {
      smart.AddBlankState(states[i].Name);
    }

    // Add a state to state machine.
    for (int i = 0; i < 24; i++)
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


  // EMA S.currProtocolPerf ; alpha is 0.02 ; 50 trials average
  float CurrOutcome = 0;
  float CurrTrialTypeContrast = 0; // if this trial is the most difficult trial type
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
    S.currProtocolPerf = S.currProtocolPerf * Alpha50 + (1.0 - Alpha50) * CurrOutcome;
    S.Perf100 = S.Perf100 * Alpha100 + (1.0 - Alpha100) * CurrOutcome;
    // trial type contrast rate (performance)
    int difff;
    difff = TrialTypeContrast[0] - TrialTypeContrast[1];
    if(abs(difff) == 1){
      CurrTrialTypeContrast = 1.0;
      S.TrialContrastRate = S.TrialContrastRate * Alpha100 + (1.0 - Alpha100) * CurrOutcome;
      S.hardestTrials++;
    }
  }
  currProtocolPerf_corrected = S.currProtocolPerf / (1.0 - pow(Alpha50 ,S.currProtocolTrials));
  Perf100_corrected = S.Perf100 / (1.0 - pow(Alpha100 ,S.currProtocolTrials));
  TrialContrastRate_corrected = S.TrialContrastRate / (1.0 - pow(Alpha100 ,S.hardestTrials));

  // earlylick rate
  EarlyLick100 = 0;
  for (int i = 0; i < RECORD_TRIALS; i++)
  {
    // EL
    if (S.EarlyLickHistory[i] == 1)
    {
      EarlyLick100++;
    }
  }
}
//TODO perf100 的correct 要更改
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

  TrialInfo_str += String(int(Perf100_corrected * 100));
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
    if ((S.currProtocolTrials > 1000 && EarlyLick100 <= 20) || (S.currProtocolTrials > 2000)) // Habitation 1
    {                          
      S.currProtocolIndex = 1; // Habitation 2
      S.currProtocolTrials = 0;
      S.hardestTrials = 0;
      S.currProtocolPerf = 0;
      S.Perf100 = 0;
      S.TrialContrastRate = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 500;
      S.PreCueDelayPeriod = 200;
      S.TimeOut = 2000;
      S.AnswerPeriod = 5000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod   = 500; 
      S.EarlyLickPeriod     = 100;// Earlylick noise cue
      S.TrialPresentMode = 1; // 0 "random of most difficult trials" ; 1 "random" ;2 "MT" 
    }
    break;

  case 1: // Habitation 2
    if (S.currProtocolTrials > 2000)
    {
      S.currProtocolIndex = 2; // Test 0
      S.currProtocolTrials = 0;
      S.hardestTrials = 0;
      S.currProtocolPerf = 0;
      S.Perf100 = 0;
      S.TrialContrastRate = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 500;
      S.PreCueDelayPeriod = 200;
      S.TimeOut = 2000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod   = 400;
      S.EarlyLickPeriod     = 100;
      S.TrialPresentMode = 1; 
    }
    break;

  case 2: // Test 0
    if (S.currProtocolTrials > 2000)
    {
      S.currProtocolIndex = 3; // early training
      S.currProtocolTrials = 0;
      S.hardestTrials = 0;
      S.currProtocolPerf = 0;
      S.Perf100 = 0;
      S.TrialContrastRate = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 500;
      S.PreCueDelayPeriod = 200;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod   = 100;
      S.TrialPresentMode = 1;  // random
    }
    break;

  case 3: // Early training
    if (S.currProtocolTrials > 100 && int(Perf100_corrected * 100) > 75 && int(TrialContrastRate_corrected * 100) > 75 ) // TrialContrastRate_corrected 记录最难contrast 的performance
    {
      S.currProtocolIndex = 4; // retention 
      S.currProtocolTrials = 0;
      S.hardestTrials = 0;
      S.currProtocolPerf = 0;
      S.retention_counter = 0;
      S.Perf100 = 0;
      S.TrialContrastRate = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 500;
      S.PreCueDelayPeriod = 200;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod   = 100;
      S.TrialPresentMode = 4; 

    }
    break;

  case 4: // retention 
    if (S.currProtocolTrials  % 100 == 0 && int(Perf100_corrected * 100) > 75 && int(TrialContrastRate_corrected * 100) > 75)
    {
      S.retention_counter++;
    }
    if(S.retention_counter >= 10){
      S.currProtocolIndex = 5;  // Test 1
      S.currProtocolTrials = 0;
      S.hardestTrials = 0;
      S.currProtocolPerf = 0;
      S.retention_counter = 0;
      S.Perf100 = 0;
      S.TrialContrastRate = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 500; // random
      S.PreCueDelayPeriod = 200;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod   = 100;
      S.TrialPresentMode = 4; 

    }
    break;

  case 5: // test 1
    if (S.currProtocolTrials > 5000)
    {
      S.currProtocolIndex = 6; // Test 2
      S.currProtocolTrials = 0;
      S.hardestTrials = 0;
      S.currProtocolPerf = 0;
      S.retention_counter = 0;
      S.Perf100 = 0;
      S.TrialContrastRate = 0;

      S.SamplePeriod = 500;
      S.DelayPeriod  = 500; // random
      S.PreCueDelayPeriod = 200;
      S.TimeOut = 8000;
      S.AnswerPeriod = 1000;
      S.ConsumptionPeriod = 750;
      S.StopLickingPeriod = 1000;
      S.EarlyLickPeriod   = 100;
      S.TrialPresentMode = 4; 
    }
    break;

 
  default:
    break;
  }
}

void autoReward() {
  S.GaveFreeReward.past_trials ++;
  byte error_trials = 3; // consecutive 3 errors in a paticular trial type => free reward in next trial
  if (S.GaveFreeReward.past_trials >= error_trials) {
    byte n_RSideErrors = 0;
    byte n_LSideErrors = 0;
    for (int i = 0; i < error_trials; i++) {
      if (S.TrialTypeHistory[RECORD_TRIALS - error_trials + i] == 2 && S.OutcomeHistory[RECORD_TRIALS - error_trials + i] != 1) {
        n_RSideErrors++;
      } else if (S.TrialTypeHistory[RECORD_TRIALS - error_trials + i] == 1 && S.OutcomeHistory[RECORD_TRIALS - error_trials + i] != 1) {
        n_LSideErrors++;
      }
    }
    if (n_RSideErrors == error_trials) {
      S.GaveFreeReward.flag_R_water = 1;
      S.GaveFreeReward.flag_L_water = 0;
      S.GaveFreeReward.past_trials = 0;
    } else if (n_LSideErrors == error_trials) {
      S.GaveFreeReward.flag_R_water = 0;
      S.GaveFreeReward.flag_L_water = 1;
      S.GaveFreeReward.past_trials = 0;
    }
  }
}

void trialSelection() { // trial type: 1 left 2 right ; trialtype contrast : (X1 ,X2) X ->range(0 ,5)
  int trialtypeSelectedFlag = 3; // random ;WSLS antibias  ; MT ;; 0 ,1 ,2
  int trialcontrastSelectedFlag = 3; //random ; MT ;limited ;; 0 ,1 ,2
  int modelfitFlag = 2; // 0 is not fit ;1 is fit
  if(S.currProtocolIndex >= 2){
    modelfitFlag = 1;
  }else{
    modelfitFlag = 0;
  }
  // 在habitation 之后都使用MT 来建模
  // S.trialPresentMode
  // TrialContrast ,trialtype:
  // 1: random + WSLS antibias ; 2: MT + MT 
  // 0: limited + random ; 3: limited + MT ;4 :limited + WSLS antibias
  // 5: random + random ; 
  switch (S.TrialPresentMode) {
    case 0: // limited + random 
    {
      trialcontrastSelectedFlag = 2;
      trialtypeSelectedFlag = 0;
      
      // if curr trial is not correct ,then repeat it
      TrialTypeContrast[0] = random(5);
      TrialTypeContrast[1] = random(5);
      int difff;
      difff = TrialTypeContrast[0] - TrialTypeContrast[1];
      while(abs(difff) != 1){
        TrialTypeContrast[0] = random(5);
        TrialTypeContrast[1] = random(5);
        difff = TrialTypeContrast[0] - TrialTypeContrast[1];
      }
      if(TrialTypeContrast[0] > TrialTypeContrast[1]){
        TrialType = 2; // right
      }else{
        TrialType = 1; // left
      }
      TrialTypeContrastPC[1] = 3; // fix the stimulus 2 to 3
      TrialTypeContrastPC[0] = random(0 ,7);
      
    }
    break;
    case 1: // random + WSLS antibias 
    {
      trialcontrastSelectedFlag = 0;
      trialtypeSelectedFlag = 1;

      // random
      // antibias
      if(S.OutcomeHistory[RECORD_TRIALS - 1] == 1){
        TrialTypeContrast[0] = random(5);
        TrialTypeContrast[1] = random(5);
        while(TrialTypeContrast[0] == TrialTypeContrast[1]){
          TrialTypeContrast[0] = random(5);
          TrialTypeContrast[1] = random(5);
        }
        if(TrialTypeContrast[0] > TrialTypeContrast[1]){
          TrialType = 2; // right
        }else{
          TrialType = 1; // left
        }
      }
    }
    break;
    case 2: // MT + MT 
    {
      trialcontrastSelectedFlag = 1;
      trialtypeSelectedFlag = 2;
    }
    break;
    case 3: // limited + MT
    {
      trialcontrastSelectedFlag = 2;
      trialtypeSelectedFlag = 2;

      TrialTypeContrastPC[1] = 3; // fix the stimulus 2 to 3
      TrialTypeContrastPC[0] = random(0 ,7);
    }
    break;
    case 4: // limited + WSLS antibias
    {
      trialcontrastSelectedFlag = 2;
      trialtypeSelectedFlag = 1;
      if(S.OutcomeHistory[RECORD_TRIALS - 1] == 1){
        // if curr trial is not correct ,then repeat it
        TrialTypeContrast[0] = random(5);
        TrialTypeContrast[1] = random(5);
        int difff;
        difff = TrialTypeContrast[0] - TrialTypeContrast[1];
        while(abs(difff) != 1){
          TrialTypeContrast[0] = random(5);
          TrialTypeContrast[1] = random(5);
          difff = TrialTypeContrast[0] - TrialTypeContrast[1];
        }
        if(TrialTypeContrast[0] > TrialTypeContrast[1]){
          TrialType = 2; // right
        }else{
          TrialType = 1; // left
        }
      }
      TrialTypeContrastPC[1] = 3; // fix the stimulus 2 to 3
      TrialTypeContrastPC[0] = random(0 ,7);
    }
    break;
    case 5: // random + random
    {
      trialcontrastSelectedFlag = 0;
      trialtypeSelectedFlag = 0;

      // random
      TrialTypeContrast[0] = random(5);
      TrialTypeContrast[1] = random(5);
      while(TrialTypeContrast[0] == TrialTypeContrast[1]){
        TrialTypeContrast[0] = random(5);
        TrialTypeContrast[1] = random(5);
      }
      if(TrialTypeContrast[0] > TrialTypeContrast[1]){
        TrialType = 2; // right
      }else{
        TrialType = 1; // left
      } 
    }
    break;
    default:
    break;
  }

  if(modelfitFlag == 1){
    if(S.OutcomeHistory[RECORD_TRIALS - 1] == 0 || S.OutcomeHistory[RECORD_TRIALS - 1] == 3 || modality == 1) // outcome == 3 : earlylick  ;outcome == 0 : NoResponse trials ; PC probe trials
    {
      TrialType = S.TrialTypeHistory[RECORD_TRIALS - 1]; // repeat last trialtype
      write_SD_Model_info(); 
    }
    else{
      // S ,A ,R back_n data S1
      float S_n_Back = 0;
      float A_n_Back = 0;
      float R_n_Back = 0;
      float y;
      float CurrReward;
      // generate dataset
      for (int i = RECORD_TRIALS - 2; i >= 0; i--){
        if(S.OutcomeHistory[i] != 3){ // remove Earlylick trials
        if(S.OutcomeHistory[i] != 0){
          // S history
          if(S.TrialTypeHistory[i] == 1){ // left
            S_n_Back = -1.0;
          }else if(S.TrialTypeHistory[i] == 2){ // right
            S_n_Back = 1.0;
          }
          // Action history
          if(S.OutcomeHistory[i] == 1){ // reward
            R_n_Back = 1.0;
          }else if(S.OutcomeHistory[i] == 2){ // error
            R_n_Back = -1.0;
          }
          A_n_Back = S_n_Back * R_n_Back;
        }
        else{ // No Response trials
          S_n_Back = 0;
          A_n_Back = 0;
          R_n_Back = 0;
        }
        break;
        }
      }
      // bias
      MiceChoiceData.x[0] = 1; // bias
      // S0 generation
      MiceChoiceData.x[1] = model.S0[TrialTypeContrast[0]]; // Stimuli 1 Sample
      MiceChoiceData.x[2] = model.S0[TrialTypeContrast[1]]; // Stimuli 2 Test
      // history regressors
      MiceChoiceData.x[3] = S_n_Back; // S1
      MiceChoiceData.x[4] = A_n_Back; // A1
      MiceChoiceData.x[5] = R_n_Back; // R1
      // WSLS
      MiceChoiceData.x[6]    = A_n_Back * R_n_Back; // WSLS
      // label
      if(S.OutcomeHistory[RECORD_TRIALS - 1] == 1){ // reward
          CurrReward = 1.0;
      }else if(S.OutcomeHistory[RECORD_TRIALS - 1] == 2){ // error
          CurrReward = -1.0;
      }else{ // not happen
          CurrReward = 1.0;
      }
      y = CurrReward * float(2 * TrialType - 3);
      // using label y {0 ,1} 0 is left ; 1 is right
      if(y == 1){
        MiceChoiceData.y = (float)1.0;
      }else if(y == -1){
        MiceChoiceData.y = (float)0.0;
      }
      loss = UpdateMiceParams(&MiceChoiceData ,1); //  使用小鼠的实际选择数据来更新小鼠的model
      write_Model_Param_info();
      write_Dataset_info(&MiceChoiceData  ,S.OutcomeHistory[RECORD_TRIALS - 1]);
      // TrialTypeData
      TrialTypeData.x[0] = 1; // bias
      // S1
      TrialTypeData.x[3] = float(2 * TrialType - 3); // S1
      // A1
      TrialTypeData.x[4] = y; // A1
      // R1
      TrialTypeData.x[5]  = CurrReward; // R1
      // WSLS
      TrialTypeData.x[6] = y * CurrReward;

      // batch size
      if(model.t % batchsize == 0){
        deepcopymodel();
      }
      // trial selection
      // 根据两种算法来最优选择 trialtype ,并把一些需要记录的预测值更新 到FP 中 ,kernel is 0(alignMax) ,1(Iternative MT) ;limited; 1 use limited contrast
      if(S.TrialPresentMode == 2){ // MT + MT
        SelectTrialType(&TrialTypeData ,1 ,0);
      }else if(S.TrialPresentMode == 3){ // limited + MT
        SelectTrialType(&TrialTypeData ,1 ,1);
      }
      write_SD_Model_info(); // 将当前的model的参数，trialnumber， 对下一个trial不同trialtype预测小鼠选择的左选概率 ，迭代的次数 loss 记录到 model.txt 
    }
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
    // EventsDuration 
    dataFile.print("EventsDuration = ");
    dataFile.println(S.EventsDuration);

    // EventsInterval 
    dataFile.print("EventsInterval = ");
    for (int i = 0; i < 5; i++)
    {
      dataFile.print(S.EventsInterval[i]);
      dataFile.print("; ");
    }
    dataFile.println();

    // PsychometricInterval  
    dataFile.print("PsychometricInterval = ");
    for (int i = 0; i < 7; i++)
    {
      dataFile.print(S.PsychometricInterval[i]);
      dataFile.print("; ");
    }
    dataFile.println();

    // NoResponseTimeOut 
    dataFile.print("NoResponseTimeOut = ");
    dataFile.println(S.NoResponseTimeOut);
    // S.Perf100
    dataFile.print("Perf100 = ");
    dataFile.println(S.Perf100 ,4);
    // S.TrialContrastRate
    dataFile.print("TrialContrastRate = ");
    dataFile.println(S.TrialContrastRate ,4);
    // hardestTrials
    dataFile.print("hardestTrials = ");
    dataFile.println(S.hardestTrials);
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
    // EventsDuration 
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.EventsDuration = string_tmp.toInt();
    // EventsInterval 
    string_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < 5; i++)
    {
      string_tmp = dataFile.readStringUntil(';');
      S.EventsInterval[i] = string_tmp.toInt();
    }
    string_tmp = dataFile.readStringUntil('\n');
    // PsychometricInterval 
    string_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < 7; i++)
    {
      string_tmp = dataFile.readStringUntil(';');
      S.PsychometricInterval[i] = string_tmp.toInt();
    }
    string_tmp = dataFile.readStringUntil('\n');

    // NoResponseTimeOut  
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.NoResponseTimeOut = string_tmp.toInt();
    // S.Perf100
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.Perf100 = string_tmp.toFloat();
    // S.TrialContrastRate
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.TrialContrastRate = string_tmp.toFloat();
    // S.hardestTrials
    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n');
    S.hardestTrials = string_tmp.toInt();
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
    dataFile.print("task = ");
    dataFile.println(task_name);
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

// read ModelParam.txt
int read_Model_Param_info()
{
  // read file to identify the cage number
  File dataFile = SD.open("modelparam.txt", FILE_READ);
  if (dataFile)
  {
    dataFile.seek(0);
    string_tmp = dataFile.readStringUntil('=');
    for(int i = 0;i<7;i++)
    {
      string_tmp = dataFile.readStringUntil(';'); 
      model.theta[i] = string_tmp.toFloat(); 
	  // if(i == 1){
	  // Serial.print("theta S0: ");
	  // Serial.println(model.theta[i] ,4);
	  // }else if(i == 0){
	  // Serial.print("theta bias: ");
	  // Serial.println(model.theta[i] ,4);
	  // }else if(i == 2){
	  // Serial.print("theta S1: ");
	  // Serial.println(model.theta[i] ,4);
	  // }else if(i == 5){
	  //  Serial.print("theta A1: ");
	  //  Serial.print(MiceChoiceData.x[5]);
	  //  Serial.print("  ");
	  // Serial.println(model.theta[i] ,4);
	  // }else if(i == 8){
	  //  Serial.print("theta R1: ");
	  // Serial.println(model.theta[i] ,4);
	  // }else if(i == 11){
	  //  Serial.print("theta WSLS: ");
	  // Serial.println(model.theta[i] ,4);
	  // }
    }
    string_tmp = dataFile.readStringUntil('\n'); 

    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n'); 
    model.alpha = string_tmp.toFloat(); 

    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n'); 
    model.L1 = string_tmp.toFloat(); 

    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n'); 
    model.Gamma = string_tmp.toFloat(); 

    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n'); 
    model.Eta = string_tmp.toFloat(); 

    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n'); 
    model.beta = string_tmp.toFloat(); 

    string_tmp = dataFile.readStringUntil('=');
    string_tmp = dataFile.readStringUntil('\n'); 
    model.t = string_tmp.toInt(); 

    string_tmp = dataFile.readStringUntil('=');
    for(int i = 0;i<7;i++)
    {
      string_tmp = dataFile.readStringUntil(';'); 
      model.m[i] = string_tmp.toFloat(); // 这里的double 精读只有 小数点后两位
    }
    string_tmp = dataFile.readStringUntil('\n'); 

    string_tmp = dataFile.readStringUntil('=');
    for(int i = 0;i<5;i++)
    {
      string_tmp = dataFile.readStringUntil(';'); 
      model.S0[i] = string_tmp.toFloat(); // 这里的double 精读只有 小数点后两位
    }
  }
  else
  {
    Serial.println("Can not open file: 'modelparam.txt'.");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 1;
}

// write modelparam.txt
int write_Model_Param_info()
{
  File dataFile = SD.open("modelparam.txt", FILE_WRITE_BEGIN);
  if (dataFile)
  {
    dataFile.print("theta = ");
    for(int i = 0;i<7;i++)
    {
      dataFile.print(model.theta[i] ,4);
      dataFile.print(";");
    }
    dataFile.println();
    dataFile.print("alpha = ");
    dataFile.println(model.alpha);
    dataFile.print("L1 = ");
    dataFile.println(model.L1);
    dataFile.print("Gamma = ");
    dataFile.println(model.Gamma);
    dataFile.print("Eta = ");
    dataFile.println(model.Eta);
    dataFile.print("beta = ");
    dataFile.println(model.beta);
    dataFile.print("t = ");
    dataFile.println(model.t);

    dataFile.print("m = ");
    for(int i = 0;i<7;i++)
    {
      dataFile.print(model.m[i] ,4);
      dataFile.print(";");
    }
    dataFile.println();

    dataFile.print("S0 = ");
    for(int i = 0;i<5;i++)
    {
      dataFile.print(model.S0[i] ,4);
      dataFile.print(";");
    }
    dataFile.println();
  }
  else
  {
    Serial.println("Can not open file: 'modelparam.txt'.");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 1;
}

// write_SD_model_info ; / 将当前的model的参数，trialnumber， 对下一个trial不同trialtype预测小鼠选择的右选概率 ，迭代的次数 loss 记录到 model.txt 
int write_SD_Model_info()
{
  File dataFile = SD.open("model.txt", FILE_WRITE);
  if (dataFile)
  {
    // trial#
    dataFile.print(S.currTrialNum);
    dataFile.print(" ");
    // iterations
    dataFile.print(model.t);
    dataFile.print(" ");
    // loss of teacher model
    dataFile.print(loss ,4);
    dataFile.print(" ");
    // hyper params
    dataFile.print(model.alpha);
    dataFile.print(" ");
    dataFile.print(model.L1);
    dataFile.print(" ");
    dataFile.print(model.Gamma);
    dataFile.print(" ");
    dataFile.print(model.Eta);
    dataFile.print(" ");
    dataFile.print(model.beta);
    // params of teacher model
    for(int i=0;i<7;i++){
      dataFile.print(" ");
      dataFile.print(model.theta[i] ,4);
    }
    dataFile.println();
  }
  else
  {
    Serial.println("E: error opening model.txt'.");
    udpPrint("E: error opening model.txt");
    dataFile.close();
    return -1;
  }
  dataFile.close();
  return 1;
}

// write_Dataset_info ; // 将所有用于更新 model 的dataset 记录到一个新的文件之中 
int write_Dataset_info(struct DATAModel *data , byte outcome)
{
  File dataFile = SD.open("dataset.txt", FILE_WRITE);
  if (dataFile)
  {
    // trial#
    dataFile.print(S.currTrialNum);
    dataFile.print(" ");
    // iterations
    dataFile.print(model.t);
    dataFile.print(" ");
    // outcome
    dataFile.print(outcome);

    for(int i=0;i<7;i++){
      dataFile.print(" ");
      dataFile.print(data->x[i] ,4);
    }
    dataFile.print(" ");
    dataFile.print(data->y ,4);

    dataFile.println();
  }
  else
  {
    Serial.println("E: error opening dataset.txt'.");
    udpPrint("E: error opening dataset.txt");
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

    dataFile.print(TrialTypeContrast[0]);
    dataFile.print(" ");

    dataFile.print(TrialTypeContrast[1]);
    dataFile.print(" ");

    dataFile.print(TrialTypeContrastPC[0]);
    dataFile.print(" ");

    dataFile.print(TrialTypeContrastPC[1]);
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

/*****Logistic Regression*****/
// 定义sigmoid函数
float sigmoid(float z) {
    return 1.0 / (1.0 + exp(-z));
}

// 定义预测函数，返回样本属于第k类的概率
float predictFunc(struct DATAModel *data, int k) {
    float z = 0;
    for (int i = 0; i < 7; i++) { 
        z += model.theta[i] * data->x[i];
    }
    if (k == 0) { // 第0类的概率为1-sigmoid(z) // left 0
        return 1.0 - sigmoid(z);
    } 
    else if (k == 1) { // 第1类的概率为sigmoid(z) // right 1
        return sigmoid(z);
    } else {
      Serial.print("Invalid class label");
      return -1.0;
    }
}

// 定义损失函数，返回对数似然损失值
float lossFunc(struct DATAModel *data ,int n) {  //  n == 1
    float l = 0.0;
    
    l += log(predictFunc(data, 1)) * data->y; // 计算每个样本的对数似然值，并累加到损失值中
    l += (1.0 - data->y) * log(1.0 - predictFunc(data, 1));
    
    return -l / n; // 返回负对数似然损失值的平均值（最小化目标）
}

// 定义随机梯度下降法，更新参数向量，并返回更新后的损失值 ，使用的数据为小鼠的实际选择数据
float UpdateMiceParams(struct DATAModel *data, int n) { // batch size is 1
    model.t = model.t + 1;
    float yhat = predictFunc(data, 1); // 计算预测值
    for (int j = 0; j < 7; j++) { // 遍历所有参数
      model.m[j] = ((yhat - data->y) * data->x[j]) * (1.0 - model.beta) + model.beta * model.m[j];
      model.theta[j] -= model.alpha * (model.m[j]/(1.0 - pow(model.beta ,model.t)) + model.L1 * sgn(model.theta[j])) ; // 初始化偏差校正
    }
    return lossFunc(data, n); // 返回更新后的损失值
}


void SelectTrialType(struct DATAModel *data ,byte Kernel ,byte limitedSelect) { 
    // limitedSelect 参数将 算法的trial选择能力限制在trialtype 参数的选择上，同时使用random的方法来生成 trialcontrast 参数
    float ArgMinValue[5][5]; 
    float min[3]; // min[0] -> min value ; min[1] -> row index ; min[2] -> col index
    // 计算 不同trialtype 下的梯度
    float delta_W[5][5][7];
    float delta_W_Star[7];
    float yhat;

    for(int i=0;i<7;i++){
      delta_W_Star[i] = actionmodel.theta[i] - W_star[i]; // 注意，在计算deltaW和deltaW_Star的时候，主要通过内积来判断相关性和方向一致性；但要注意这两个向量的方向是正确的；
    }
    // iterative calc all the dataset
    // i < j -> left ; i > j -> right
    for(int i=0;i<5;i++){ // Sitmuli 1 -- Sample
      for(int j=0;j<5;j++){ // Stimuli 2 -- Test
        // S0 
        if(i != j){
          data->x[1] = actionmodel.S0[i]; 
          data->x[2] = actionmodel.S0[j];
          // label
          if(i > j){
            data->y    = 1.0; // right 
          }else{
            data->y    = 0.0; // left
          }
          // predicted mouse choice
          yhat = predictFunc(data, 1); // 计算预测值, 预测小鼠的选择
          for(int w=0;w<7;w++){
            delta_W[i][j][w] = (yhat - data->y) * data->x[w];
            delta_W[i][j][w] = delta_W[i][j][w] * float(actionmodel.Eta);
          }
          // calc the argMin value
          float T1 = 0; // difficulty item
          float T2 = 0; // usefulness item
          for(int w = 0;w<7;w++){
            T1 += pow(delta_W[i][j][w] ,2);
            T2 += delta_W[i][j][w] * delta_W_Star[w];
          }
          ArgMinValue[i][j] = pow(actionmodel.Gamma ,2) * T1 * float(Kernel) - 2.0 * actionmodel.Gamma * T2;
        }
      }  
    }

    // argMin  : Min T1 and Max T2 ; 选择更简单的并且更能够align 设定参数的 trialtype
    // TrialType ; TrialTypeContrast
    int limitedContrast = 0;
    float limitedArgMinValue[2];
    if(limitedSelect){
      limitedContrast = int(random(1 ,5)); // 1-4:对应四个最难得contrast stimulus
      limitedArgMinValue[0] = ArgMinValue[limitedContrast][limitedContrast - 1]; // trial type right 2
      limitedArgMinValue[1] = ArgMinValue[limitedContrast - 1][limitedContrast]; // trial type left 1
     if(limitedArgMinValue[0] < limitedArgMinValue[1]){
        TrialType = 2;
        TrialTypeContrast[0] = limitedContrast;
        TrialTypeContrast[1] = limitedContrast - 1;
      }else{
        TrialType = 1;
        TrialTypeContrast[0] = limitedContrast - 1;
        TrialTypeContrast[1] = limitedContrast;
      }
    }else{
      min[0] = ArgMinValue[0][1];
      min[1] = 0;
      min[2] = 1;
      for(int i = 0; i < 5; i++)
        for(int j = 0; j < 5; j++)
          if(i != j)
            if(min[0] > ArgMinValue[i][j]){
              min[0] = ArgMinValue[i][j]; // min
              min[1] = i;  // Stimulus 1
              min[2] = j;  // Stimulus 2
          }
      // output
      if(min[1] < min[2]){
        TrialType = 1; // left
      }else{
        TrialType = 2; // right
      }
      TrialTypeContrast[0] = min[1];
      TrialTypeContrast[1] = min[2];
    }
}


float sgn(float w){
  if(w > 0)
  {
    return 1;
  }
  else if(w < 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

void deepcopymodel(void){
  for(int i=0;i<7;i++){
    actionmodel.theta[i] = model.theta[i];
    actionmodel.m[i] = model.m[i];
  }
  actionmodel.alpha = model.alpha;
  actionmodel.L1 = model.L1;
  actionmodel.Gamma = model.Gamma;
  actionmodel.Eta = model.Eta;
  actionmodel.beta = model.beta;
  actionmodel.t = model.t;
  for(int i=0;i<5;i++){
    actionmodel.S0[i] = model.S0[i];
  }
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
