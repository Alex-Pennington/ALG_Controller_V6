#include <Arduino.h>
#include "HX711.h"
#include <PID_v1.h>
#include <EEPROM.h>
#include "controller.h"
#include <OneWire.h>
#include <ezButton.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Enable and select radio type attached
#define MY_NODE_ID 201
#define MY_RADIO_RF24
#define MY_RF24_PA_LEVEL RF24_PA_HIGH
#define MY_RF24_CE_PIN 49
#define MY_RF24_CS_PIN 53
//#define MY_DEBUG
#include <MySensors.h>

// MySensors Child IDs
enum CHILD_ID {
  T0 = 0,
  T1 = 1,
  T2 = 2,
  T3 = 3,
  T4 = 4,

  dC_1 = 5,
  SSR_Armed = 6,
  PIDMODE_1 = 7,
  PIDSETPOINT_1 = 8,
  AdaptiveSP_1 = 9,
  PIDkP0_1 = 10,
  PIDkI0_1 = 11,
  PIDkD0_1 = 12,
  PIDkP1_1 = 13,
  PIDkI1_1 = 14,
  PIDkD1_1 = 15,
  AdaptiveMode_1 = 16,
  //EDC_1 = 17,
  PIDThreshold_1 = 18,
  
  dC_2 = 19,
  PIDMODE_2 = 20,
  PIDSETPOINT_2 = 21,
  AdaptiveSP_2 = 22,
  PIDkP0_2 = 23,
  PIDkI0_2 = 24,
  PIDkD0_2 = 25,
  PIDkP1_2 = 26,
  PIDkI1_2 = 27,
  PIDkD1_2 = 28,
  AdaptiveMode_2 = 29,
  //EDC_2 = 31,
  PIDThreshold_2 = 32,
  
  dC_3 = 33,
  PIDMODE_3 = 34,
  PIDSETPOINT_3 = 35,
  Duty3 = 36,
  PIDkP0_3 = 37,
  PIDkI0_3 = 38,
  PIDkD0_3 = 39,
  //EDC_3 = 40,
  PIDThreshold_3 = 41,

  
  Scale = 42,
  ScaleTare = 43,
  ScaleCal = 44,
  ScaleOffset = 45,
    
  MainsCurrent = 46,
  MainsCurrentMultiplier = 47,
  SSRFail_Threshhold = 48,
  SSRFail_Alarm = 49,
  MainsCurrentOffset = 50,

  P1 = 51,
  P1Cal = 52,
  Press1Offset = 53,    
  P2 = 54,
  P2Cal = 55,
  Press2Offset = 56,
  P3 = 57,
  P3Cal = 58,
  Press3Offset = 59,  
  P4 = 60,
  P4Cal = 61,
  Press4Offset = 62,  
  
  VccCurrent = 63,
  VccVoltage = 64,
  
  THMS1 = 65,
  THMS2 = 66,

  s_debug = 67,
  Info = 68,
  LOAD_MEMORY = 69,

  dTscale = 70,
  T5 = 71,
  runTime = 72,
  switch1 = 73,
  switch2 = 74,
  switch3 = 75,
  switch4 = 76,
  switch5 = 77,
  P1Offset = 78,
  P2Offset = 79,
  P3Offset = 80,
  P4Offset = 81,
  relay1 = 82,
  relay2 = 83,
  relay3 = 84,
  relay4 = 85,
  relay5 = 86,
  relay6 = 87,
  relay7 = 88,
  relay8 = 89,
  RefrigerantSwitch = 90,
  FlowSwitch = 91,
  ScaleCalibrateKnownValue = 92

};

// MySensors Message Definitions

MyMessage msgSSRArmed(CHILD_ID::SSR_Armed, V_STATUS);
MyMessage msgLoadEEPROM(CHILD_ID::LOAD_MEMORY, V_STATUS);
MyMessage msgINFO(CHILD_ID::Info, V_TEXT);

MyMessage msgPIDMODE_1(CHILD_ID::PIDMODE_1, V_STATUS);
//MyMessage msgPIDSETPOINT(CHILD_ID::PIDSETPOINT_1, V_TEMP);
//MyMessage msgKp(CHILD_ID::PIDkP0_1, V_LEVEL);
//MyMessage msgKi(CHILD_ID::PIDkI0_1, V_LEVEL);
//MyMessage msgKd(CHILD_ID::PIDkD0_1, V_LEVEL);
//MyMessage msgEDC(CHILD_ID::EDC_1, V_PERCENTAGE);


MyMessage msgPIDMODE_2(CHILD_ID::PIDMODE_2, V_STATUS);
//MyMessage msgPIDSETPOINT_2(CHILD_ID::PIDSETPOINT_2, V_TEMP);
//MyMessage msgKp_2(CHILD_ID::PIDkP0_2, V_LEVEL);
//MyMessage msgKi_2(CHILD_ID::PIDkI0_2, V_LEVEL);
//MyMessage msgKd_2(CHILD_ID::PIDkD0_2, V_LEVEL);
//MyMessage msgEDC_2(CHILD_ID::EDC_2, V_PERCENTAGE);

MyMessage msgPIDMODE_3(CHILD_ID::PIDMODE_3, V_STATUS);
//MyMessage msgPIDSETPOINT_3(CHILD_ID::PIDSETPOINT_3, V_TEMP);
//MyMessage msgKp_3(CHILD_ID::PIDkP0_3, V_LEVEL);
//MyMessage msgKi_3(CHILD_ID::PIDkI0_3, V_LEVEL);
//MyMessage msgKd_3(CHILD_ID::PIDkD0_3, V_LEVEL);
//MyMessage msgEDC_3(CHILD_ID::EDC_3, V_PERCENTAGE);

MyMessage msgTemp0(CHILD_ID::T0, V_TEMP);
MyMessage msgTemp1(CHILD_ID::T1, V_TEMP);
MyMessage msgTemp2(CHILD_ID::T2, V_TEMP);
MyMessage msgTemp3(CHILD_ID::T3, V_TEMP);
MyMessage msgTemp4(CHILD_ID::T4, V_TEMP);
MyMessage msgTemp5(CHILD_ID::T5, V_TEMP);
MyMessage msgTHMS1(CHILD_ID::THMS1, V_TEMP);
MyMessage msgTHMS2(CHILD_ID::THMS2, V_TEMP);

MyMessage msgScale(CHILD_ID::Scale, V_WEIGHT);
MyMessage msgScaleTare(CHILD_ID::ScaleTare, V_STATUS);
MyMessage msgScaleOffset(CHILD_ID::ScaleOffset, V_LEVEL);
MyMessage msgScaleRate(CHILD_ID::dTscale, V_LEVEL);

MyMessage msgPressure1(CHILD_ID::P1, V_PRESSURE);
//MyMessage msgPress1Offset(CHILD_ID::Press1Offset, V_LEVEL);

MyMessage msgPressure2(CHILD_ID::P2, V_PRESSURE);
MyMessage msgPressure3(CHILD_ID::P3, V_PRESSURE);
MyMessage msgPressure4(CHILD_ID::P4, V_PRESSURE);

MyMessage msgMainsCurrent(CHILD_ID::MainsCurrent, V_CURRENT);
//MyMessage msgMainsCurrentMultiplier(CHILD_ID::MainsCurrentMultiplier, V_LEVEL);
MyMessage msgSSRFailAlarm(CHILD_ID::SSRFail_Alarm, V_STATUS);
MyMessage msgVccVoltage(CHILD_ID::VccVoltage, V_VOLTAGE);
MyMessage msgVccCurrent(CHILD_ID::VccCurrent, V_CURRENT);
MyMessage msgRunTime(CHILD_ID::runTime, V_VAR1);

MyMessage msgSwitch1(CHILD_ID::switch1, V_VAR1);
MyMessage msgSwitch2(CHILD_ID::switch2, V_VAR1);
MyMessage msgSwitch3(CHILD_ID::switch3, V_VAR1);
MyMessage msgSwitch4(CHILD_ID::switch4, V_VAR1);
MyMessage msgSwitch5(CHILD_ID::switch5, V_VAR1);
MyMessage msgRefrigerantSwitch(CHILD_ID::RefrigerantSwitch, V_STATUS);
MyMessage msgFlowSwitch(CHILD_ID::FlowSwitch, V_STATUS);
MyMessage msgScaleCalibrateKnownValue(CHILD_ID::ScaleCalibrateKnownValue, V_LEVEL);

#define NUM_RELAYS 8

bool relayStates[NUM_RELAYS] = {false};

// MySensors Message Definitions for Relays
MyMessage msgRelay[NUM_RELAYS] = {
  MyMessage(CHILD_ID::relay1, V_STATUS),
  MyMessage(CHILD_ID::relay2, V_STATUS),
  MyMessage(CHILD_ID::relay3, V_STATUS),
  MyMessage(CHILD_ID::relay4, V_STATUS),
  MyMessage(CHILD_ID::relay5, V_STATUS),
  MyMessage(CHILD_ID::relay6, V_STATUS),
  MyMessage(CHILD_ID::relay7, V_STATUS),
  MyMessage(CHILD_ID::relay8, V_STATUS)
};

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define VERSION "0.6.0.1"
#define SENDDELAY 50
#define CELSIUS_TO_FAHRENHEIT_FACTOR 1.8
#define CELSIUS_TO_FAHRENHEIT_OFFSET 32.0

//Structs

struct DS18B20Values {
  byte addr[8];
  float C;
  float F;
};
DS18B20Values ds18b20Values[5];

struct ConfigurationValues {
  int SENSORLOOPTIME = 23000;
  bool sDebug = true;
  bool toACK = false;
  int pidLoopTime = 10000;
};
ConfigurationValues configValues;

struct PIDConfig {
    bool mode;
    double setpoint;
    double kp, ki, kd;
    double aggKp, aggKi, aggKd;
    byte aggSP;
    bool adaptiveMode;
    int alarmThreshold;
    double input;
    double output;
};
PIDConfig pid1 = {false, 150, 0.5, 0.005, 0, 1, 0, 0, 190, false, 0, 0, 0};
PIDConfig pid2 = {false, 150, 0.35, 0, 0, 5, 0, 0, 190, false, 0, 0, 0};
PIDConfig pid3 = {false, 120, 1.2, 0.01, 0, 0, 0, 190, false, 0, 0, 0};

struct CalibrationValues {
  float pressure1Cal = 20.77;
  int pressure1Offset = 425;
  float pressure2Cal = 20.77;
  int pressure2Offset = 425;
  float pressure3Cal = 20.77;
  int pressure3Offset = 425;
  float pressure4Cal = 20.77;
  int pressure4Offset = 425;
  float emonCal = 0.128;
  byte ssrFailThreshold = 2;
  double currOffset = 5.28;
  float zeroOffsetScale = 403361;
  float scaleCal = 53.200;
  byte aScale =  255; // times is a parameter of type byte that specifies the number of times the raw value should be read and averaged.
  int mScale  = 1; //scale multiplier
  int rScale =  1000; //rate scalar coefficient
  int VccCurrentOffset = 566;
  float VccCurrentMultiplier = 0.004887;
};
CalibrationValues calValues;

struct SteinhartConfig {
    const long seriesResistor = 98600;
    const long nominalResistance = 100000;
    const int nominalTemperature = 25;
    const int bCoefficient = 3950;
};
SteinhartConfig thermistorConfig;

struct EmonVars {
    //unsigned long sampleTime = 0;
    bool ssrFail = false;
    int ssrFailCount = 0;
    float rms = 0.0;
};
EmonVars emonVars;

struct DutyCycle {
    bool element = false;
    int loopTime = 10; // Duty cycle total loop time in seconds.
    unsigned long onTime = 0;
    unsigned long offTime = 0;
};
DutyCycle dutyCycle[3];

struct SteinhartValues {
  float resistance = 0.0;
  float adcValue = 0;
  float steinhart = 0;
};
SteinhartValues steinhartValues;

struct buttons {
  bool up = false;
  bool down = false;
};
buttons button[7];
//buttons 1-5 are for the switches
//button 6 is the coolant flow switch
//button 7 is the refrigerant tank capacity switch

//Working Variables
unsigned long dutyCycle_timer = 0;
unsigned long SensorLoop_timer = 0;
unsigned long pid_compute_loop_time = 0;
unsigned long switchesLoop_timer = 0;
float dC = 0.0;
float dC2 = 0.0;
float dC3 = 0.0;
float valueScale = 0;
float value_oldScale = 0;
unsigned long oldtimeScale = 0;
float gramsPerSecondScale = 0; //rate of change of scale in kilograms per second
long AREF_V = 0;
bool ssrArmed = false;
float pressure1Var = 0;
float pressure2Var = 0;
float pressure3Var = 0;
float pressure4Var = 0;
float VccCurrentVar = 0;
extern unsigned int __heap_start;
extern void *__brkval;
float THMS1var = 0.0;
float THMS2var = 0.0;

// Array to hold all temperature sensor values
float temperatureValues[8] = {0.0}; // 5 DS18B20 sensors, 3 thermistors

//Pin Definitions
#define HX711_dout 9 
#define HX711_sck 10 
#define SSRArmed_PIN 29
// #define speakerPin 30 //defined in controller.h
#define ElementPowerPin 25
#define ElementPowerPin2 26
#define ElementPowerPin3 27
#define DS18B20_PIN 44 
#define Pressure1PIN A15 
#define Pressure2PIN A14 
#define Pressure3PIN A13 
#define Pressure4PIN A12 
#define emon_Input_PIN A8
#define VccCurrentSensor A0 
#define SteinhartPin A3 
#define Serial3_RX 15 
#define Serial3_TX 14 

#define Switch1_UP_Pin 43
#define Switch1_DOWN_Pin 42
#define Switch2_UP_Pin 41
#define Switch2_DOWN_Pin 40
#define Switch3_UP_Pin 39
#define Switch3_DOWN_Pin 38
#define Switch4_UP_Pin 37
#define Switch4_DOWN_Pin 36
#define Switch5_UP_Pin 35
#define Switch5_DOWN_Pin 34
#define FlowSwitchPin 33
#define RefrigerantSwitchPin 32



/*Connector Pinouts
Pressure(3)  1 5V, 2 GND, 3 SIG
Thermistor(2) 1 SIG, 2 SIG
Steinhart(2) 1 SIG, 2 SIG
HX711(4) 1 E-, 2 A+, 3 A-, 4 E+
Serial(4) 1 5V, 2 RX, 3 TX, 4 GND
R1(8) GND , <NC> , ElementPowerPin , <NC> , +5v , ElementPowerPin2 , SSRArmed_PIN , (Center) emon_Input_PIN
R2(8) GND , +5v , <NC>, ElementPowerPin3, <NC>, <NC>, <NC>, (Center) SteinhartPin
*/

//Switches
#define NUM_SWITCHES 12

ezButton switches[NUM_SWITCHES] = {
  ezButton(Switch1_UP_Pin, INTERNAL_PULLUP),
  ezButton(Switch1_DOWN_Pin, INTERNAL_PULLUP),
  ezButton(Switch2_UP_Pin, INTERNAL_PULLUP),
  ezButton(Switch2_DOWN_Pin, INTERNAL_PULLUP),
  ezButton(Switch3_UP_Pin, INTERNAL_PULLUP),
  ezButton(Switch3_DOWN_Pin, INTERNAL_PULLUP),
  ezButton(Switch4_UP_Pin, INTERNAL_PULLUP),
  ezButton(Switch4_DOWN_Pin, INTERNAL_PULLUP),
  ezButton(Switch5_UP_Pin, INTERNAL_PULLUP),
  ezButton(Switch5_DOWN_Pin, INTERNAL_PULLUP),
  ezButton(FlowSwitchPin, INTERNAL_PULLUP),
  ezButton(RefrigerantSwitchPin, INTERNAL_PULLUP)
};


//Scale
HX711 LoadCell;

//DS18B20
OneWire  ds(DS18B20_PIN);  // on pin DS18B20_PIN (a 4.7K resistor is necessary)

//PIDs
PID myPID1(&pid1.input, &pid1.output, &pid1.setpoint, pid1.kp, pid1.ki, pid1.kd, DIRECT);
int gap1 = 0;

PID myPID2(&pid2.input, &pid2.output, &pid2.setpoint, pid2.kp, pid2.ki, pid2.kd, DIRECT);
int gap2 = 0;

PID myPID3(&pid3.input, &pid3.output, &pid3.setpoint, pid3.kp, pid3.ki, pid3.kd, DIRECT);
int gap3 = 0;

// Function Declarations
void DutyCycleLoop();
float Steinhart();
void AllStop();
void StoreEEPROM();
void getEEPROM();
void FactoryResetEEPROM();
void printConfig();
long getBandgap(void);
void sendInfo(String);
void DS18B20();
//float getThermistor(int);
float readPressure(int pin, int offset, float cal);
void displayLine(const char* line);
int freeMemory();
void emon();
void switchesLoop();
void getSwitches();
void TempAlarm();
void getScale();
void getVccCurrent();
void queryRelayStates();
void sendRelayStates();
void sendAllStates();
void setScaleCalibration(float knownWeight);
float voltageDivider(int pin, float dividerResistor);
void getMainsCurrent();

enum EEPROMAddresses {
  ZERO_OFFSET_SCALE = 0,       // float, 4 bytes
  PRESSURE1_OFFSET = 4,        // int, 4 bytes
  SCALE_CAL = 6,               // float, 4 bytes
  PRESSURE1_CAL = 10,          // float, 4 bytes
  PID1_SETPOINT = 14,          // double, 4 bytes
  PID1_KP = 18,                // double, 4 bytes
  PID1_KI = 22,                // double, 4 bytes
  PID1_KD = 26,                // double, 4 bytes
  PID1_AGG_KP = 30,            // double, 4 bytes
  PID1_AGG_KI = 34,            // double, 4 bytes
  PID1_AGG_KD = 38,            // double, 4 bytes
  PID2_SETPOINT = 42,          // double, 4 bytes
  PID2_KP = 46,                // double, 4 bytes
  PID2_KI = 50,                // double, 4 bytes
  PID2_KD = 54,                // double, 4 bytes
  PID2_AGG_KP = 58,            // double, 4 bytes
  PID2_AGG_KI = 62,            // double, 4 bytes
  PID2_AGG_KD = 66,            // double, 4 bytes
  PID1_AGG_SP = 70,            // byte, 1 byte
  PID2_AGG_SP = 71,            // byte, 1 byte
  PID3_SETPOINT = 72,          // double, 4 bytes
  PID3_KP = 76,                // double, 4 bytes
  PID3_KI = 80,                // double, 4 bytes
  PID3_KD = 84,                // double, 4 bytes
  EMON_CAL = 88,               // float, 4 bytes
  SSR_FAIL_THRESHOLD = 92,     // byte, 1 byte
  CURR_OFFSET = 96,            // double, 4 bytes
  SSR_ARMED = 100,             // bool, 1 byte
  PID1_MODE = 101,             // bool, 1 byte
  PID2_MODE = 102,             // bool, 1 byte
  PID3_MODE = 103,             // bool, 1 byte
  PID2_ADAPTIVE_MODE = 104,    // bool, 1 byte
  PID1_ADAPTIVE_MODE = 105,    // bool, 1 byte
  S_DEBUG = 106,               // bool, 1 byte
  PID1_ALARM_THRESHOLD = 107,  // int, 2 bytes
  PID2_ALARM_THRESHOLD = 109,  // int, 2 bytes
  PID3_ALARM_THRESHOLD = 111,  // int, 2 bytes
  PRESSURE2_OFFSET = 114,      // int, 4 bytes
  PRESSURE2_CAL = 118,         // float, 4 bytes
  PRESSURE3_CAL = 120,         // float, 4 bytes
  PRESSURE4_CAL = 122,          // float, 4 bytes
  PRESSURE3_OFFSET = 126,      // int, 4 bytes
  PRESSURE4_OFFSET = 130,      // int, 4 bytes
  VCC_CURRENT_OFFSET = 134,    // int, 4 bytes
  VCC_CURRENT_MULTIPLIER = 138 // float, 4 bytes
};

void presentation() {
  //Send the sensor node sketch version information to the gateway
  sendSketchInfo("Controller", VERSION);

  present(CHILD_ID::T0, S_TEMP, "D1"); wait(SENDDELAY);
  present(CHILD_ID::T1, S_TEMP, "D2"); wait(SENDDELAY);
  present(CHILD_ID::T2, S_TEMP, "D3"); wait(SENDDELAY);
  present(CHILD_ID::T3, S_TEMP, "D4"); wait(SENDDELAY);
  present(CHILD_ID::T4, S_TEMP, "D5"); wait(SENDDELAY);
  present(CHILD_ID::T5, S_TEMP, "S1"); wait(SENDDELAY);
  present(CHILD_ID::THMS1, S_TEMP, "T1"); wait(SENDDELAY);
  present(CHILD_ID::THMS2, S_TEMP, "T2"); wait(SENDDELAY);  
  present(CHILD_ID::Scale, S_WEIGHT, "Scl"); wait(SENDDELAY);
  present(CHILD_ID::ScaleTare, S_BINARY, "Scl T"); wait(SENDDELAY);
  present(CHILD_ID::ScaleOffset, S_LIGHT_LEVEL, "Scl O"); wait(SENDDELAY);
  present(CHILD_ID::ScaleCal, S_LIGHT_LEVEL, "Scl C"); wait(SENDDELAY);
  present(CHILD_ID::P1, S_BARO, "P1"); wait(SENDDELAY);
  present(CHILD_ID::P1Cal, S_LIGHT_LEVEL, "P1C"); wait(SENDDELAY);
  present(CHILD_ID::P2, S_BARO, "P2"); wait(SENDDELAY);
  present(CHILD_ID::P2Cal, S_LIGHT_LEVEL, "P2C"); wait(SENDDELAY);
  present(CHILD_ID::P3, S_BARO, "P3"); wait(SENDDELAY);
  present(CHILD_ID::P3Cal, S_LIGHT_LEVEL, "P3C"); wait(SENDDELAY);
  present(CHILD_ID::P4, S_BARO, "P4"); wait(SENDDELAY);
  present(CHILD_ID::P4Cal, S_LIGHT_LEVEL, "P4C"); wait(SENDDELAY);
  present(CHILD_ID::MainsCurrent, S_MULTIMETER, "Mc"); wait(SENDDELAY);
  present(CHILD_ID::MainsCurrentMultiplier, S_LIGHT_LEVEL, "McM"); wait(SENDDELAY);
  present(CHILD_ID::MainsCurrentOffset, S_LIGHT_LEVEL, "McO"); wait(SENDDELAY);
  present(CHILD_ID::SSRFail_Threshhold, S_LIGHT_LEVEL, "SFT"); wait(SENDDELAY);   
  present(CHILD_ID::VccCurrent, S_MULTIMETER, "VccC"); wait(SENDDELAY);   
  present(CHILD_ID::VccVoltage, S_LIGHT_LEVEL, "VccV"); wait(SENDDELAY);   
  present(CHILD_ID::dC_1, S_DIMMER, "dC1"); wait(SENDDELAY);   
  present(CHILD_ID::PIDMODE_1, S_BINARY, "PM1"); wait(SENDDELAY);   
  present(CHILD_ID::PIDSETPOINT_1, S_TEMP, "PS1"); wait(SENDDELAY);   
  present(CHILD_ID::PIDkP0_1, S_LIGHT_LEVEL, "kP1"); wait(SENDDELAY);   
  present(CHILD_ID::PIDkI0_1, S_LIGHT_LEVEL, "kI1"); wait(SENDDELAY);   
  present(CHILD_ID::PIDkD0_1, S_LIGHT_LEVEL, "kD1"); wait(SENDDELAY);   
  present(CHILD_ID::dC_2, S_DIMMER, "dC2"); wait(SENDDELAY);   
  present(CHILD_ID::PIDMODE_2, S_BINARY, "PM2"); wait(SENDDELAY);   
  present(CHILD_ID::PIDSETPOINT_2, S_TEMP, "PS2"); wait(SENDDELAY);   
  present(CHILD_ID::PIDkP0_2, S_LIGHT_LEVEL, "kP2"); wait(SENDDELAY);   
  present(CHILD_ID::PIDkI0_2, S_LIGHT_LEVEL, "kI2"); wait(SENDDELAY);   
  present(CHILD_ID::PIDkD0_2, S_LIGHT_LEVEL, "kD2"); wait(SENDDELAY);   
  present(CHILD_ID::dC_3, S_DIMMER, "dC3"); wait(SENDDELAY);   
  present(CHILD_ID::PIDMODE_3, S_BINARY, "PM3"); wait(SENDDELAY);   
  present(CHILD_ID::PIDSETPOINT_3, S_TEMP, "PS3"); wait(SENDDELAY);   
  present(CHILD_ID::PIDkP0_3, S_LIGHT_LEVEL, "kP3"); wait(SENDDELAY);   
  present(CHILD_ID::PIDkI0_3, S_LIGHT_LEVEL, "kI3"); wait(SENDDELAY);   
  present(CHILD_ID::PIDkD0_3, S_LIGHT_LEVEL, "kD3"); wait(SENDDELAY);   
  present(CHILD_ID::dTscale, S_LIGHT_LEVEL, "Scl R"); wait(SENDDELAY);   
  present(CHILD_ID::SSR_Armed, S_BINARY, "Armed"); wait(SENDDELAY);   
  present(CHILD_ID::SSRFail_Alarm, S_BINARY, "Fail"); wait(SENDDELAY);   
  present(CHILD_ID::Info, S_INFO, "Info"); wait(SENDDELAY);   
  present(CHILD_ID::LOAD_MEMORY, S_BINARY, "fRst"); wait(SENDDELAY);   
  present(CHILD_ID::runTime, S_CUSTOM, "rT"); wait(SENDDELAY);   

  // Present Switch Sensors
  for (int i = 0; i < 5; i++) {
    present(CHILD_ID::switch1 + i, S_CUSTOM, "Sw"); wait(SENDDELAY);   
  }

  present(CHILD_ID::FlowSwitch, S_BINARY, "FSw"); wait(SENDDELAY);   
  present(CHILD_ID::RefrigerantSwitch, S_BINARY, "RefSw"); wait(SENDDELAY);   

  present(CHILD_ID::P1Offset, S_LIGHT_LEVEL, "P1o"); wait(SENDDELAY);   
  present(CHILD_ID::P2Offset, S_LIGHT_LEVEL, "P2o"); wait(SENDDELAY);   
  present(CHILD_ID::P3Offset, S_LIGHT_LEVEL, "P3o"); wait(SENDDELAY);   
  present(CHILD_ID::P4Offset, S_LIGHT_LEVEL, "P4o"); wait(SENDDELAY);   

  // Present Relay Sensors
  for (int i = 0; i < NUM_RELAYS; i++) {
    present(CHILD_ID::relay1 + i, S_BINARY, "Rly"); wait(SENDDELAY);   
  }

  present(CHILD_ID::ScaleCalibrateKnownValue, S_LIGHT_LEVEL, "SCalKV"); wait(SENDDELAY);   

}
#define SSRARMED_OFF LOW
#define SSRARMED_ON LOW
#define ELEMENT_ON LOW

void setup() {
  Serial.begin(115200);
  Serial3.begin(9600);
  getEEPROM();
  LoadCell.begin(HX711_dout, HX711_sck);
  LoadCell.set_scale(calValues.scaleCal);
  LoadCell.set_offset(calValues.zeroOffsetScale);
  LoadCell.set_gain();
  //pinMode(Thermistor1PIN, INPUT);
  //pinMode(Thermistor2PIN, INPUT);
  pinMode(SSRArmed_PIN, OUTPUT);
  digitalWrite(SSRArmed_PIN, SSRARMED_OFF);
  pinMode(ElementPowerPin, OUTPUT);
  digitalWrite(ElementPowerPin, !ELEMENT_ON);
  pinMode(ElementPowerPin2, OUTPUT);
  digitalWrite(ElementPowerPin2, !ELEMENT_ON);
  pinMode(ElementPowerPin3, OUTPUT);
  digitalWrite(ElementPowerPin3, !ELEMENT_ON);
  pinMode(speakerPin, OUTPUT);
  pinMode(SteinhartPin, INPUT);
  pinMode(Pressure1PIN, INPUT);
  pinMode(Pressure2PIN, INPUT);
  pinMode(emon_Input_PIN, INPUT);
  pinMode(VccCurrentSensor, INPUT);

  //OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    sendInfo("E_OLED");
    while (true); // Halt execution if display initialization fails
  }

  pid1.mode = false;
  pid2.mode = false;
  pid3.mode = false;

  myPID1.SetSampleTime(configValues.pidLoopTime);
  myPID1.SetOutputLimits(0, 100);
  myPID2.SetSampleTime(configValues.pidLoopTime);
  myPID2.SetOutputLimits(0, 100);
  myPID3.SetSampleTime(configValues.pidLoopTime);
  myPID3.SetOutputLimits(0, 100);
  myPID1.SetMode(MANUAL);
  myPID2.SetMode(MANUAL);
  myPID3.SetMode(MANUAL);

  for (int i = 0; i < NUM_SWITCHES; i++) {
    switches[i].setDebounceTime(50);
  }

  displayLine("Booting...");
  sendInfo("Operational");
  //sendAllStates();
  play_one_up();
  wait(1000);
}

void loop() {
  DutyCycleLoop();
  _process();
  switchesLoop();

  pid1.input = temperatureValues[0];
  pid2.input = temperatureValues[1];
  pid3.input = temperatureValues[5]; //Steinhart

  if ( (millis() - switchesLoop_timer) > (unsigned long)configValues.SENSORLOOPTIME) {
    getSwitches();
    switchesLoop_timer = millis();
  }

  if ( (millis() - SensorLoop_timer) > (unsigned long)configValues.SENSORLOOPTIME)  {
    unsigned long sensorLoopTime = millis();
    AREF_V = getBandgap();

    getVccCurrent();
    msgVccCurrent.set(VccCurrentVar, 2); send(msgVccCurrent);
    msgVccVoltage.set(AREF_V, 2); send(msgVccVoltage);
    _process();

    
    getMainsCurrent();
    msgMainsCurrent.set(emonVars.rms, 2); send(msgMainsCurrent);
    _process();

    DS18B20();
    for (int i = 0; i < 5; i++) {
      temperatureValues[i] = ds18b20Values[i].F;
      send(MyMessage(CHILD_ID::T0 + i, V_TEMP).set(temperatureValues[i], 2));
    }
    _process();
    
    float steinhartVar = Steinhart();
    temperatureValues[5] = steinhartVar;
    msgTemp5.set(temperatureValues[5], 2); send(msgTemp5);
    _process();

    getScale();
    msgScale.set(valueScale, 2); send(msgScale);
    msgScaleRate.set(gramsPerSecondScale, 2); send(msgScaleRate);
    _process();
    DutyCycleLoop();
    
    char buffer[16];
    dtostrf(valueScale, 6, 2, buffer);
    displayLine(buffer);
    _process();

    pressure1Var = readPressure(Pressure1PIN, calValues.pressure1Offset, calValues.pressure1Cal);
    msgPressure1.set(pressure1Var, 2); send(msgPressure1);
    pressure2Var = readPressure(Pressure2PIN, calValues.pressure2Offset, calValues.pressure2Cal);
    msgPressure2.set(pressure2Var, 2); send(msgPressure2);
    pressure3Var = readPressure(Pressure3PIN, calValues.pressure3Offset, calValues.pressure3Cal);
    msgPressure3.set(pressure3Var, 2); send(msgPressure3);
    pressure4Var = readPressure(Pressure4PIN, calValues.pressure4Offset, calValues.pressure4Cal);
    msgPressure4.set(pressure4Var, 2); send(msgPressure4);
    _process();
    
    //temperatureValues[6] = getThermistor(Thermistor1PIN);
    //temperatureValues[7] = getThermistor(Thermistor2PIN);
    //msgTHMS1.set(temperatureValues[6], 2); send(msgTHMS1);
    //msgTHMS2.set(temperatureValues[7], 2); send(msgTHMS2);
    //_process();
    
    msgRunTime.set((float)(millis()/1000.0/60.0/60.0),2); send(msgRunTime);
    _process();
            
    TempAlarm();
    _process();
    queryRelayStates();
    sendRelayStates();
    SensorLoop_timer = millis();
    Serial.println(millis()-sensorLoopTime);
  }
  
  if ((millis() - pid_compute_loop_time) > (unsigned long)configValues.pidLoopTime)  {
    int gap = abs(pid1.setpoint - pid1.input); //distance away from setpoint
    if ((gap < pid1.aggSP && pid1.adaptiveMode == true) || pid1.adaptiveMode == false) {
      myPID1.SetTunings(pid1.kp, pid1.ki, pid1.kp);
    } else if (gap > pid1.aggSP && pid1.adaptiveMode == true) {
      myPID1.SetTunings(pid1.aggKp, pid1.aggKi, pid1.aggKd);
    }
    int gap_2 = abs(pid2.setpoint - pid2.input); //distance away from setpoint
    if ((gap_2 < pid2.aggSP && pid2.adaptiveMode == true) || pid2.adaptiveMode == false) {
      myPID2.SetTunings(pid2.kp, pid2.ki, pid2.kp);
    } else if (gap_2 > pid2.aggSP && pid2.adaptiveMode == true) {
      myPID2.SetTunings(pid2.aggKp, pid2.aggKi, pid2.aggKd);
    }
    myPID1.Compute();
    myPID2.Compute();
    myPID3.Compute();
    dC = pid1.output / 100.0;
    dC2 = pid2.output / 100.0;
    dC3 = pid3.output / 100.0;
    send(MyMessage(CHILD_ID::dC_1, V_PERCENTAGE).set(pid1.output,2), configValues.toACK);
    send(MyMessage(CHILD_ID::dC_2, V_PERCENTAGE).set(pid2.output,2), configValues.toACK);
    send(MyMessage(CHILD_ID::dC_3, V_PERCENTAGE).set(pid3.output,2), configValues.toACK);
    pid_compute_loop_time = millis();
  }
}

void switchesLoop() {
  for (int i = 0; i < NUM_SWITCHES; i++) {
    switches[i].loop();
  }
}

void getSwitches() {
  for (int i = 0; i < 5; i++) {
    int btnState_UP = switches[i * 2].getState();
    int btnState_DOWN = switches[i * 2 + 1].getState();
    if (btnState_UP == 1 && btnState_DOWN == 0) {
      send(MyMessage(CHILD_ID::switch1 + i, V_VAR1).set(1), configValues.toACK);
    } else if (btnState_UP == 0 && btnState_DOWN == 1) {
      send(MyMessage(CHILD_ID::switch1 + i, V_VAR1).set(0), configValues.toACK);
    } else if (btnState_UP == 1 && btnState_DOWN == 1) {
      send(MyMessage(CHILD_ID::switch1 + i, V_VAR1).set(2), configValues.toACK);
    }
  }

  int btnState_switchFlow = switches[10].getState();
  int btnState_switchRefrigerant = switches[11].getState();

  send(msgFlowSwitch.set(btnState_switchFlow == 1), configValues.toACK);
  send(msgRefrigerantSwitch.set(btnState_switchRefrigerant == 1), configValues.toACK);

  return;
}

void getVccCurrent()
{
  for (int i = 0; i < 10; i++)
  {
    VccCurrentVar += analogRead(VccCurrentSensor);
    wait(10); // Small delay for better averaging
  }
  VccCurrentVar /= 10;                                                                        // Average the readings
  VccCurrentVar = (VccCurrentVar - calValues.VccCurrentOffset) * calValues.VccCurrentMultiplier; //
  return;
  }
void TempAlarm()
{
  // Temp Alarm
  if (pid1.alarmThreshold < pid1.input)
  {
    sendInfo("PID1!");
    red_alert();
    AllStop();
  }
  if (pid2.alarmThreshold < pid2.input)
  {
    sendInfo("PID2!");
    red_alert();
    AllStop();
  }
  if (pid3.alarmThreshold < pid3.input)
  {
    sendInfo("PID3!");
    red_alert();
    AllStop();
  }
}
/**
 * @brief Measures the root mean square (RMS) current using an energy monitor.
 *
 * This function reads the current from an analog input pin, calculates the RMS value,
 * and checks if it exceeds a predefined threshold. If the threshold is exceeded, it triggers
 * a red alert, stops all operations, and sends a failure message. Otherwise, it sets the 
 * power pins according to the duty cycle values and sends a status message indicating no failure.
 *
 * The function performs the following steps:
 * 1. Sets the power pins to HIGH. (OFF)
 * 2. Initializes a sum variable to accumulate the squared current values.
 * 3. Samples the current 1000 times, calculates the squared current, and accumulates the sum.
 * 4. Calculates the RMS current and adjusts it by subtracting the current offset.
 * 5. Checks if the RMS current exceeds the SSR fail threshold.
 *    - If true, triggers a red alert, stops all operations, and sends a failure message.
 *    - If false, sets the power pins according to the duty cycle values and sends a status message.
 * 6. Repeats the current sampling and RMS calculation.
 *
 */
void emon()
{
  float sum = 0;

  for (int i = 0; i < 1000; i++)  {
    float current = calValues.emonCal * (analogRead(emon_Input_PIN) - 512); // in amps I presume
    sum += current * current;                                               // sum squares
    wait(10);
  }
  emonVars.rms = sqrt(sum / 1000) - calValues.currOffset;
  if ((int(emonVars.rms) > int(calValues.ssrFailThreshold)))  {
    red_alert();
    AllStop();
    sendInfo("emon SSR Fail");
    send(MyMessage(CHILD_ID::SSRFail_Alarm, V_STATUS).set(1), configValues.toACK);
  }
}
void getMainsCurrent() {
  float sum = 0;
  for (int i = 0; i < 1000; i++)  {
    float current = calValues.emonCal * (analogRead(emon_Input_PIN) - 512); // in amps I presume
    sum += current * current;                                               // sum squares
    wait(10);
  }
  emonVars.rms = sqrt(sum / 1000) - calValues.currOffset;
  return;
}
/**
 * @brief Reads the temperature from a DS18B20 sensor.
 *
 * This function reads the temperature from a DS18B20 sensor connected to the specified pin.
 * It reads the sensor multiple times to increase resolution, calculates the temperature in Fahrenheit,
 * and sends the temperature value to the controller.
 *
 * @param pinVar The pin number where the DS18B20 sensor is connected.
 */
void getScale() {
  value_oldScale = valueScale;
  if (LoadCell.is_ready())
  {
    valueScale = LoadCell.get_units(10) * calValues.mScale;
  }
  else
  {
    sendInfo("E2");
    valueScale = value_oldScale; // or handle the error as needed
  }
  if (valueScale < 0)
  {
    valueScale = 0;
  }

  gramsPerSecondScale = ((valueScale - value_oldScale) / ((millis() - oldtimeScale)));
  oldtimeScale = millis();
  return;
}
float readPressure(int pin, int offset, float cal) {
  int RawADCavg = 0;
  int i = 0;
  for (i = 0; i < 10; i++) {
    wait(10);
    RawADCavg += analogRead(Pressure1PIN);
  }
  float avgADC = (float)RawADCavg / 10.0;
  float offsetCorrected = avgADC - (float)offset;
  return offsetCorrected * (1.0 / cal);
}
/**
 * @brief Calculates the temperature from the thermistor using the Steinhart-Hart equation.
 *
 * This function reads the analog value from the thermistor pin multiple times to increase resolution,
 * calculates the resistance of the thermistor, and then applies the Steinhart-Hart equation to convert
 * the resistance to temperature in Fahrenheit.
 *
 * @return float The temperature in Fahrenheit.
 */
float Steinhart() {
  double adcValue = 0;
  for(int i = 0; i < 10 ; i++ ) {
    wait(10); //this increased resolution signifigantly
    adcValue += analogRead(SteinhartPin);
  }
  steinhartValues.adcValue = (float)adcValue/10.0;
  float Resistance = ((thermistorConfig.seriesResistor + thermistorConfig.nominalResistance) * (1 / (1023 / steinhartValues.adcValue)));
  float steinhart = Resistance / (float)thermistorConfig.nominalResistance; // (R/Ro)
  steinhart = log(steinhart); // ln(R/Ro)
  steinhart /= (float)thermistorConfig.bCoefficient; // 1/B * ln(R/Ro)
  steinhart += 1.0 / ((float)thermistorConfig.nominalTemperature + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart; // Invert
  steinhart -= 273.15; // convert to C
  steinhart = (((steinhart * 9 ) / 5 ) + 32);
  steinhartValues.steinhart = steinhart;
  return steinhart;
}
/**
 * @brief Reads the analog value from a thermistor and calculates the temperature in Fahrenheit.
 *
 * This function reads the analog value from a specified pin connected to a thermistor,
 * calculates the resistance of the thermistor, and then uses the Steinhart-Hart equation
 * to convert the resistance to a temperature in Kelvin. Finally, it converts the temperature
 * from Kelvin to Fahrenheit.
 *
 * @param pinVar The analog pin number where the thermistor is connected.
 * @return The temperature in Fahrenheit.
 */
//}
/**
 * @brief Manages the duty cycle for three PID-controlled elements.
 *
 * This function controls the duty cycle of three elements based on their respective PID outputs.
 * It ensures that each element is turned on and off according to its calculated duty cycle,
 * while also considering the SSR (Solid State Relay) armed state and PID mode.
 *
 * Variables:
 * - pid1, pid2, pid3: Structures containing PID configurations and states for three different elements.
 * - dC, dC2, dC3: Duty cycle values for the three elements, calculated as a percentage (0.0 to 1.0).
 * - ssrArmed: Boolean indicating whether the SSR is armed (true) or not (false).
 * - dutyCycle: Array of structures containing duty cycle loop configurations and states for the three elements.
 * - ElementPowerPin, ElementPowerPin2, ElementPowerPin3: Pin numbers for controlling the power to the three elements.
 *
 * DutyCycle structure:
 * - element: Boolean indicating whether the element is currently on (true) or off (false).
 * - loopTime: Total loop time for the duty cycle in seconds.
 * - onTime: Timestamp of when the element was last turned on.
 * - offTime: Timestamp of when the element was last turned off.
 *
 * The function iterates over the three elements, checks their respective PID modes and duty cycle values,
 * and turns the elements on or off based on the calculated on and off durations.
 */
void DutyCycleLoop() {
if ((millis() - dutyCycle_timer) > (unsigned long)(dutyCycle[0].loopTime * 1000)) {
    digitalWrite(ElementPowerPin, !ELEMENT_ON);
    digitalWrite(ElementPowerPin2, !ELEMENT_ON);
    digitalWrite(ElementPowerPin3, !ELEMENT_ON);
    dutyCycle_timer = millis();
    dutyCycle[0].onTime = 0;
    dutyCycle[0].offTime = 0;
    dutyCycle[1].onTime = 0;
    dutyCycle[1].offTime = 0;
    dutyCycle[2].onTime = 0;
    dutyCycle[2].offTime = 0;
    dutyCycle[0].element = false;
    dutyCycle[1].element = false;
    dutyCycle[2].element = false;
    emon();
  }

  for (int i = 0; i < 3; i++) {
    // Determine the current PID mode and duty cycle
    bool pidMode;
    float dutyCycleValue;
    int elementPin;

    if (i == 0) {
      pidMode = pid1.mode;
      dutyCycleValue = dC;
      elementPin = ElementPowerPin;
    } else if (i == 1) {
      pidMode = pid2.mode;
      dutyCycleValue = dC2;
      elementPin = ElementPowerPin2;
    } else {
      pidMode = pid3.mode;
      dutyCycleValue = dC3;
      elementPin = ElementPowerPin3;
    }

    // Check if the duty cycle loop is active and SSR is armed
    if (dutyCycleValue > 0.0 && ssrArmed && pidMode) {
      unsigned long currentTime = millis();
      unsigned long onDuration = dutyCycle[i].loopTime * 1000 * dutyCycleValue; // Convert duty cycle to on milliseconds
      unsigned long offDuration = dutyCycle[i].loopTime * 1000 - onDuration; // Convert duty cycle to off milliseconds

      // If the element is off, check if it's time to turn it on
      if (!dutyCycle[i].element && (currentTime - dutyCycle[i].offTime) > offDuration) {
        dutyCycle[i].onTime = currentTime;
        digitalWrite(elementPin, ELEMENT_ON);
        dutyCycle[i].element = true;
      }
      // If the element is on, check if it's time to turn it off
      else if (dutyCycle[i].element && (currentTime - dutyCycle[i].onTime) > onDuration) {
        dutyCycle[i].offTime = currentTime;
        digitalWrite(elementPin, !ELEMENT_ON);
        dutyCycle[i].element = false;
      }
    } else { // If the duty cycle loop is not active or SSR is not armed, turn off the element
      digitalWrite(elementPin, !ELEMENT_ON);
      dutyCycle[i].element = false;
    }
  }
}
void AllStop() {
  digitalWrite(SSRArmed_PIN, LOW);
  pid1.mode = false;
  pid2.mode = false;
  pid3.mode = false;
  ssrArmed = false;
  myPID1.SetMode(MANUAL);
  myPID2.SetMode(MANUAL);
  myPID3.SetMode(MANUAL);
  msgPIDMODE_1.set(pid1.mode); send(msgPIDMODE_1);
  msgPIDMODE_2.set(pid2.mode); send(msgPIDMODE_2);
  msgPIDMODE_3.set(pid3.mode); send(msgPIDMODE_3);
  }
void StoreEEPROM() {
  EEPROM.put(EEPROMAddresses::PRESSURE3_CAL, calValues.pressure3Cal);
  EEPROM.put(EEPROMAddresses::PRESSURE4_CAL, calValues.pressure4Cal);
  EEPROM.put(EEPROMAddresses::PRESSURE3_OFFSET, calValues.pressure3Offset);
  EEPROM.put(EEPROMAddresses::PRESSURE4_OFFSET, calValues.pressure4Offset);
  EEPROM.put(EEPROMAddresses::VCC_CURRENT_OFFSET, calValues.VccCurrentOffset);
  EEPROM.put(EEPROMAddresses::VCC_CURRENT_MULTIPLIER, calValues.VccCurrentMultiplier);
  EEPROM.put(EEPROMAddresses::ZERO_OFFSET_SCALE, calValues.zeroOffsetScale);
  EEPROM.put(EEPROMAddresses::PRESSURE1_OFFSET, calValues.pressure1Offset);
  EEPROM.put(EEPROMAddresses::SCALE_CAL, calValues.scaleCal);
  EEPROM.put(EEPROMAddresses::PRESSURE1_CAL, calValues.pressure1Cal);
  EEPROM.put(EEPROMAddresses::PID1_SETPOINT, pid1.setpoint);
  EEPROM.put(EEPROMAddresses::PID1_KP, pid1.kp);
  EEPROM.put(EEPROMAddresses::PID1_KI, pid1.ki);
  EEPROM.put(EEPROMAddresses::PID1_KD, pid1.kd);
  EEPROM.put(EEPROMAddresses::PID1_AGG_KP, pid1.aggKp);
  EEPROM.put(EEPROMAddresses::PID1_AGG_KI, pid1.aggKi);
  EEPROM.put(EEPROMAddresses::PID1_AGG_KD, pid1.aggKd);
  EEPROM.put(EEPROMAddresses::PID2_SETPOINT, pid2.setpoint);
  EEPROM.put(EEPROMAddresses::PID2_KP, pid2.kp);
  EEPROM.put(EEPROMAddresses::PID2_KI, pid2.ki);
  EEPROM.put(EEPROMAddresses::PID2_KD, pid2.kd);
  EEPROM.put(EEPROMAddresses::PID2_AGG_KP, pid2.aggKp);
  EEPROM.put(EEPROMAddresses::PID2_AGG_KI, pid2.aggKi);
  EEPROM.put(EEPROMAddresses::PID2_AGG_KD, pid2.aggKd);
  EEPROM.put(EEPROMAddresses::PID1_AGG_SP, pid1.aggSP);
  EEPROM.put(EEPROMAddresses::PID2_AGG_SP, pid2.aggSP);
  EEPROM.put(EEPROMAddresses::PID3_SETPOINT, pid3.setpoint);
  EEPROM.put(EEPROMAddresses::PID3_KP, pid3.kp);
  EEPROM.put(EEPROMAddresses::PID3_KI, pid3.ki);
  EEPROM.put(EEPROMAddresses::PID3_KD, pid3.kd);
  EEPROM.put(EEPROMAddresses::EMON_CAL, calValues.emonCal);
  EEPROM.put(EEPROMAddresses::SSR_FAIL_THRESHOLD, calValues.ssrFailThreshold);
  EEPROM.put(EEPROMAddresses::CURR_OFFSET, calValues.currOffset);
  EEPROM.put(EEPROMAddresses::SSR_ARMED, ssrArmed);
  EEPROM.put(EEPROMAddresses::PID1_MODE, pid1.mode);
  EEPROM.put(EEPROMAddresses::PID2_MODE, pid2.mode);
  EEPROM.put(EEPROMAddresses::PID3_MODE, pid3.mode);
  EEPROM.put(EEPROMAddresses::PID2_ADAPTIVE_MODE, pid2.adaptiveMode);
  EEPROM.put(EEPROMAddresses::PID1_ADAPTIVE_MODE, pid1.adaptiveMode);
  EEPROM.put(EEPROMAddresses::S_DEBUG, configValues.sDebug);
  EEPROM.put(EEPROMAddresses::PID1_ALARM_THRESHOLD, pid1.alarmThreshold);
  EEPROM.put(EEPROMAddresses::PID2_ALARM_THRESHOLD, pid2.alarmThreshold);
  EEPROM.put(EEPROMAddresses::PID3_ALARM_THRESHOLD, pid3.alarmThreshold);
  EEPROM.put(EEPROMAddresses::PRESSURE2_OFFSET, calValues.pressure2Offset);
  EEPROM.put(EEPROMAddresses::PRESSURE2_CAL, calValues.pressure2Cal);

  printConfig();
}
void getEEPROM() {
  EEPROM.get(EEPROMAddresses::PRESSURE3_CAL, calValues.pressure3Cal);
  EEPROM.get(EEPROMAddresses::PRESSURE4_CAL, calValues.pressure4Cal);
  EEPROM.get(EEPROMAddresses::PRESSURE3_OFFSET, calValues.pressure3Offset);
  EEPROM.get(EEPROMAddresses::PRESSURE4_OFFSET, calValues.pressure4Offset);
  EEPROM.get(EEPROMAddresses::VCC_CURRENT_OFFSET, calValues.VccCurrentOffset);
  EEPROM.get(EEPROMAddresses::VCC_CURRENT_MULTIPLIER, calValues.VccCurrentMultiplier);
  EEPROM.get(EEPROMAddresses::ZERO_OFFSET_SCALE, calValues.zeroOffsetScale);
  EEPROM.get(EEPROMAddresses::PRESSURE1_OFFSET, calValues.pressure1Offset);
  EEPROM.get(EEPROMAddresses::SCALE_CAL, calValues.scaleCal);
  EEPROM.get(EEPROMAddresses::PRESSURE1_CAL, calValues.pressure1Cal);
  EEPROM.get(EEPROMAddresses::PID1_SETPOINT, pid1.setpoint);
  EEPROM.get(EEPROMAddresses::PID1_KP, pid1.kp);
  EEPROM.get(EEPROMAddresses::PID1_KI, pid1.ki);
  EEPROM.get(EEPROMAddresses::PID1_KD, pid1.kd);
  EEPROM.get(EEPROMAddresses::PID1_AGG_KP, pid1.aggKp);
  EEPROM.get(EEPROMAddresses::PID1_AGG_KI, pid1.aggKi);
  EEPROM.get(EEPROMAddresses::PID1_AGG_KD, pid1.aggKd);
  EEPROM.get(EEPROMAddresses::PID2_SETPOINT, pid2.setpoint);
  EEPROM.get(EEPROMAddresses::PID2_KP, pid2.kp);
  EEPROM.get(EEPROMAddresses::PID2_KI, pid2.ki);
  EEPROM.get(EEPROMAddresses::PID2_KD, pid2.kd);
  EEPROM.get(EEPROMAddresses::PID2_AGG_KP, pid2.aggKp);
  EEPROM.get(EEPROMAddresses::PID2_AGG_KI, pid2.aggKi);
  EEPROM.get(EEPROMAddresses::PID2_AGG_KD, pid2.aggKd);
  EEPROM.get(EEPROMAddresses::PID1_AGG_SP, pid1.aggSP);
  EEPROM.get(EEPROMAddresses::PID2_AGG_SP, pid2.aggSP);
  EEPROM.get(EEPROMAddresses::PID3_SETPOINT, pid3.setpoint);
  EEPROM.get(EEPROMAddresses::PID3_KP, pid3.kp);
  EEPROM.get(EEPROMAddresses::PID3_KI, pid3.ki);
  EEPROM.get(EEPROMAddresses::PID3_KD, pid3.kd);
  EEPROM.get(EEPROMAddresses::EMON_CAL, calValues.emonCal);
  EEPROM.get(EEPROMAddresses::SSR_FAIL_THRESHOLD, calValues.ssrFailThreshold);
  EEPROM.get(EEPROMAddresses::CURR_OFFSET, calValues.currOffset);
  //EEPROM.get(EEPROMAddresses::SSR_ARMED, ssrArmed);
  //EEPROM.get(EEPROMAddresses::PID1_MODE, pid1.mode);
  //EEPROM.get(EEPROMAddresses::PID2_MODE, pid2.mode);
  //EEPROM.get(EEPROMAddresses::PID3_MODE, pid3.mode);
  EEPROM.get(EEPROMAddresses::PID2_ADAPTIVE_MODE, pid2.adaptiveMode);
  EEPROM.get(EEPROMAddresses::PID1_ADAPTIVE_MODE, pid1.adaptiveMode);
  EEPROM.get(EEPROMAddresses::S_DEBUG, configValues.sDebug);
  EEPROM.get(EEPROMAddresses::PID1_ALARM_THRESHOLD, pid1.alarmThreshold);
  EEPROM.get(EEPROMAddresses::PID2_ALARM_THRESHOLD, pid2.alarmThreshold);
  EEPROM.get(EEPROMAddresses::PID3_ALARM_THRESHOLD, pid3.alarmThreshold);
  EEPROM.get(EEPROMAddresses::PRESSURE2_OFFSET, calValues.pressure2Offset);
  EEPROM.get(EEPROMAddresses::PRESSURE2_CAL, calValues.pressure2Cal);
  printConfig();
}
void FactoryResetEEPROM() {
  // Reset Calibration Values
  calValues.pressure1Cal = 1;
  calValues.pressure1Offset = 0;
  calValues.pressure2Cal = 1;
  calValues.pressure2Offset = 0;
  calValues.pressure3Cal = 1;
  calValues.pressure3Offset = 0;
  calValues.pressure4Cal = 1;
  calValues.pressure4Offset = 0;
  calValues.emonCal = 0.128;
  calValues.ssrFailThreshold = 2;
  calValues.currOffset = 5.28;
  calValues.zeroOffsetScale = 403361;
  calValues.scaleCal = 53.200;
  calValues.aScale = 255;
  calValues.mScale = 1;
  calValues.rScale = 1;
  calValues.VccCurrentOffset = 566;
  calValues.VccCurrentMultiplier = 0.004887;

  // Reset PID Configurations
  pid1.mode = false;
  pid1.setpoint = 150;
  pid1.kp = 0.5;
  pid1.ki = 0.005;
  pid1.kd = 0;
  pid1.aggKp = 1;
  pid1.aggKi = 0;
  pid1.aggKd = 0;
  pid1.aggSP = 10;
  pid1.adaptiveMode = false;
  pid1.alarmThreshold = 190;
  pid1.input = 0;
  pid1.output = 0;

  pid2.mode = false;
  pid2.setpoint = 150;
  pid2.kp = 0.35;
  pid2.ki = 0;
  pid2.kd = 0;
  pid2.aggKp = 5;
  pid2.aggKi = 0;
  pid2.aggKd = 0;
  pid2.aggSP = 10;
  pid2.adaptiveMode = false;
  pid2.alarmThreshold = 190;
  pid2.input = 0;
  pid2.output = 0;

  pid3.mode = false;
  pid3.setpoint = 120;
  pid3.kp = 1.2;
  pid3.ki = 0.01;
  pid3.kd = 0;
  pid3.aggKp = 0;
  pid3.aggKi = 0;
  pid3.aggKd = 0;
  pid3.aggSP = 10;
  pid3.adaptiveMode = false;
  pid3.alarmThreshold = 190;
  pid3.input = 0;
  pid3.output = 0;

  // Reset other configurations
  ssrArmed = false;
  configValues.SENSORLOOPTIME = 23000;
  configValues.sDebug = true;
  configValues.toACK = false;

  StoreEEPROM();
}
void printConfig() {
//   Serial.print("zeroOffsetScale: ");
//   Serial.println(calValues.zeroOffsetScale);
//   Serial.print("pressure1Offset: ");
//   Serial.println(calValues.pressure1Offset);
//   Serial.print("pressure1Cal: ");
//   Serial.println(calValues.pressure1Cal);
//   Serial.print("pressure2Offset: ");
//   Serial.println(calValues.pressure2Offset);
//   Serial.print("pressure2Cal: ");
//   Serial.println(calValues.pressure2Cal);
//   Serial.print("pressure3Offset: ");
//   Serial.println(calValues.pressure3Offset);
//   Serial.print("pressure3Cal: ");
//   Serial.println(calValues.pressure3Cal);
//   Serial.print("pressure4Offset: ");
//   Serial.println(calValues.pressure4Offset);
//   Serial.print("pressure4Cal: ");
//   Serial.println(calValues.pressure4Cal);
//   Serial.print("scaleCal: ");
//   Serial.println(calValues.scaleCal);
//   Serial.print("aScale: ");
//   Serial.println(calValues.aScale);
//   Serial.print("mScale: ");
//   Serial.println(calValues.mScale);
//   Serial.print("rScale: ");
//   Serial.println(calValues.rScale);
//   Serial.print("VccCurrentOffset: ");
//   Serial.println(calValues.VccCurrentOffset);
//   Serial.print("VccCurrentMultiplier: ");
//   Serial.println(calValues.VccCurrentMultiplier,6);
//   Serial.print("PID1 Setpoint: ");
//   Serial.println(pid1.setpoint);
//   Serial.print("PID1 kp: ");
//   Serial.println(pid1.kp);
//   Serial.print("PID1 ki: ");
//   Serial.println(pid1.ki);
//   Serial.print("PID1 kd: ");
//   Serial.println(pid1.kd);
//   Serial.print("PID1 aggKp: ");
//   Serial.println(pid1.aggKp);
//   Serial.print("PID1 aggKi: ");
//   Serial.println(pid1.aggKi);
//   Serial.print("PID1 aggKd: ");
//   Serial.println(pid1.aggKd);
//   Serial.print("PID2 Setpoint: ");
//   Serial.println(pid2.setpoint);
//   Serial.print("PID2 kp: ");
//   Serial.println(pid2.kp);
//   Serial.print("PID2 ki: ");
//   Serial.println(pid2.ki);
//   Serial.print("PID2 kd: ");
//   Serial.println(pid2.kd);
//   Serial.print("PID2 aggKp: ");
//   Serial.println(pid2.aggKp);
//   Serial.print("PID2 aggKi: ");
//   Serial.println(pid2.aggKi);
//   Serial.print("PID2 aggKd: ");
//   Serial.println(pid2.aggKd);
//   Serial.print("PID1 aggSP: ");
//   Serial.println(pid1.aggSP);
//   Serial.print("PID2 aggSP: ");
//   Serial.println(pid2.aggSP);
//   Serial.print("PID3 Setpoint: ");
//   Serial.println(pid3.setpoint);
//   Serial.print("PID3 kp: ");
//   Serial.println(pid3.kp);
//   Serial.print("PID3 ki: ");
//   Serial.println(pid3.ki);
//   Serial.print("PID3 kd: ");
//   Serial.println(pid3.kd);
//   Serial.print("PID3 aggKp: ");
//   Serial.println(pid3.aggKp);
//   Serial.print("PID3 aggKi: ");
//   Serial.println(pid3.aggKi);
//   Serial.print("PID3 aggKd: ");
//   Serial.println(pid3.aggKd);
//   Serial.print("PID3 aggSP: ");
//   Serial.println(pid3.aggSP);
//   Serial.print("emonCal: ");
//   Serial.println(calValues.emonCal);
//   Serial.print("ssrFailThreshold: ");
//   Serial.println(calValues.ssrFailThreshold);
//   Serial.print("currOffset: ");
//   Serial.println(calValues.currOffset);
//   Serial.print("ssrArmed: ");
//   Serial.println(ssrArmed);
//   Serial.print("PID1 mode: ");
//   Serial.println(pid1.mode);
//   Serial.print("PID2 mode: ");
//   Serial.println(pid2.mode);
//   Serial.print("PID3 mode: ");
//   Serial.println(pid3.mode);
//   Serial.print("PID2 adaptiveMode: ");
//   Serial.println(pid2.adaptiveMode);
//   Serial.print("PID1 adaptiveMode: ");
//   Serial.println(pid1.adaptiveMode);
//   Serial.print("PID3 adaptiveMode: ");
//   Serial.println(pid3.adaptiveMode);
//   Serial.print("sDebug: ");
//   Serial.println(configValues.sDebug);
//   Serial.print("PID1 alarmThreshold: ");
//   Serial.println(pid1.alarmThreshold);
//   Serial.print("PID2 alarmThreshold: ");
//   Serial.println(pid2.alarmThreshold);
//   Serial.print("PID3 alarmThreshold: ");
//   Serial.println(pid3.alarmThreshold);
//   Serial.print("SENSORLOOPTIME: ");
//   Serial.println(configValues.SENSORLOOPTIME);
//   Serial.print("toACK: ");
//   Serial.println(configValues.toACK);

}
/**
 * @brief Reads the internal 1.1V reference against AVcc to calculate the supply voltage (Vcc).
 *
 * This function sets the ADC (Analog-to-Digital Converter) to measure the internal 1.1V reference
 * against the supply voltage (Vcc). It then performs an ADC conversion and calculates the Vcc
 * based on the result.
 *
 * The function is useful for determining the actual supply voltage of the microcontroller,
 * which can be used for various purposes such as battery monitoring.
 *
 * @return The calculated Vcc in millivolts.
 *
 * Steps:
 * 1. Configure the ADC to measure the internal 1.1V reference.
 * 2. Wait for the reference voltage to settle.
 * 3. Start an ADC conversion.
 * 4. Wait for the conversion to complete.
 * 5. Read the ADC result.
 * 6. Calculate the Vcc based on the ADC result.
 *
 * Variables:
 * - ADMUX: ADC Multiplexer Selection Register, used to select the reference voltage and input channel.
 * - ADCSRA: ADC Control and Status Register A, used to control the ADC and check the conversion status.
 * - ADCL, ADCH: ADC Data Registers, used to read the result of the ADC conversion.
 *
 * The calculation formula is:
 * Vcc = 1125300L / ADC_result
 * where 1125300 = 1.1 * 1023 * 1000 (1.1V reference, 10-bit ADC resolution, scaling factor).
 */
long getBandgap(void) {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRB &= ~_BV(MUX5); //
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  wait(10); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
void sendInfo(String payload) {
  msgINFO.set(payload.c_str());
  send(msgINFO);
}
void DS18B20() {
  byte addr[8];
  int count = 0;
   
  while (ds.search(addr)) {
    byte i;
    byte type_s;
    byte data[9];
    
    switch (addr[0]) {
      case 0x10:
        type_s = 1;
        break;
      case 0x28:
        type_s = 0;
        break;
      case 0x22:
        type_s = 0;
        break;
      default:
        return;
    } 

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 0);
    wait(10);
    ds.reset();
    ds.select(addr);    
    ds.write(0xBE);
    for ( i = 0; i < 9; i++) { data[i] = ds.read(); }
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    }
    ds18b20Values[count].C = (float)raw / 16.0;
    ds18b20Values[count].F = ds18b20Values[count].C * 1.8 + 32.0;
    count = count + 1;
  }
  ds.reset_search();
  return;
}
void displayLine(const char* line) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(3);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(line); 
  display.display();
}
int freeMemory() {
  int free_memory;
  if ((int)__brkval == 0) {
    free_memory = ((int)&free_memory) - ((int)&__heap_start);
  } else {
    free_memory = ((int)&free_memory) - ((int)__brkval);
  }
  return free_memory;
}
void receive(const MyMessage & message)  {
  int msgcmd = mGetCommand(message);
  if (configValues.sDebug) {
    if (message.isAck()) {
      Serial.print("Ack : ");
      Serial.print("Cmd = ");
      Serial.print(msgcmd);
      Serial.print(" : Set sensor ");
      Serial.print(message.sender);
      Serial.print("/");
      Serial.println(message.sensor);
      return;
    }
    Serial.print("Cmd = ");
    Serial.print(msgcmd);
    Serial.print(" : Set sensor ");
    Serial.print(message.sender);
    Serial.print("/");
    Serial.println(message.sensor);
  }

  switch (message.sensor) {
    case CHILD_ID::ScaleTare:
      if (message.getBool()) {
        if (configValues.sDebug) Serial.print("Tare ... ");
        calValues.zeroOffsetScale = LoadCell.read_average(100);
        if (configValues.sDebug) Serial.println(" done.");
        EEPROM.put(EEPROMAddresses::ZERO_OFFSET_SCALE, calValues.zeroOffsetScale);
        LoadCell.set_offset(calValues.zeroOffsetScale);
        send(msgScaleTare.set(false), configValues.toACK);
        wait(SENDDELAY);
        send(msgScaleOffset.set(calValues.zeroOffsetScale, 1));
        wait(SENDDELAY);
        sendInfo("Scale Tare");
        play_fireball();
      }
      break;
    case CHILD_ID::ScaleOffset:
      calValues.zeroOffsetScale = message.getFloat();
      EEPROM.put(EEPROMAddresses::ZERO_OFFSET_SCALE, calValues.zeroOffsetScale);
      LoadCell.set_offset(calValues.zeroOffsetScale);
      break;
    case CHILD_ID::dC_1:
      dC = message.getFloat() / 100.0;
      break;
    case CHILD_ID::dC_2:
      dC2 = message.getFloat() / 100.0;
      break;
    case CHILD_ID::dC_3:
      dC3 = message.getFloat() / 100.0;
      break;
    case CHILD_ID::SSR_Armed:
      ssrArmed = message.getBool();
      EEPROM.put(EEPROMAddresses::SSR_ARMED, ssrArmed);
      digitalWrite(SSRArmed_PIN, ssrArmed);
      play_coin();
      break;
    case CHILD_ID::Press1Offset:
      calValues.pressure1Offset = message.getInt();
      EEPROM.put(EEPROMAddresses::PRESSURE1_OFFSET, calValues.pressure1Offset);
      break;
    case CHILD_ID::ScaleCal:
      calValues.scaleCal = message.getFloat();
      LoadCell.set_scale(calValues.scaleCal);
      EEPROM.put(EEPROMAddresses::SCALE_CAL, calValues.scaleCal);
      break;
        case CHILD_ID::P1Cal:
      calValues.pressure1Cal = message.getFloat();
      EEPROM.put(EEPROMAddresses::PRESSURE1_CAL, calValues.pressure1Cal);
      break;
        case CHILD_ID::P2Cal:
      calValues.pressure2Cal = message.getFloat();
      EEPROM.put(EEPROMAddresses::PRESSURE2_CAL, calValues.pressure2Cal);
      break;
        case CHILD_ID::P3Cal:
      calValues.pressure3Cal = message.getFloat();
      EEPROM.put(EEPROMAddresses::PRESSURE3_CAL, calValues.pressure3Cal);
      break;
        case CHILD_ID::P4Cal:
      calValues.pressure4Cal = message.getFloat();
      EEPROM.put(EEPROMAddresses::PRESSURE4_CAL, calValues.pressure4Cal);
      break;
        case CHILD_ID::PIDMODE_1:
      pid1.mode = message.getBool();
      myPID1.SetMode(message.getBool());
      EEPROM.put(EEPROMAddresses::PID1_MODE, message.getBool());
      play_coin();
      break;
    case CHILD_ID::PIDSETPOINT_1:
      pid1.setpoint = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID1_SETPOINT, pid1.setpoint);
      break;
    case CHILD_ID::PIDkP0_1:
      pid1.kp = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID1_KP, pid1.kp);
      break;
    case CHILD_ID::PIDkI0_1:
      pid1.ki = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID1_KI, pid1.ki);
      break;
    case CHILD_ID::PIDkD0_1:
      pid1.kd = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID1_KD, pid1.kd);
      break;
    case CHILD_ID::PIDkP1_1:
      pid1.aggKp = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID1_AGG_KP, pid1.aggKp);
      break;
    case CHILD_ID::PIDkI1_1:
      pid1.aggKi = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID1_AGG_KI, pid1.aggKi);
      break;
    case CHILD_ID::PIDkD1_1:
      pid1.aggKd = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID1_AGG_KD, pid1.aggKd);
      break;
    case CHILD_ID::AdaptiveSP_1:
      pid1.aggSP = message.getByte();
      EEPROM.put(EEPROMAddresses::PID1_AGG_SP, pid1.aggSP);
      break;
    case CHILD_ID::AdaptiveMode_1:
      pid1.adaptiveMode = message.getBool();
      EEPROM.put(EEPROMAddresses::PID1_ADAPTIVE_MODE, pid1.adaptiveMode);
      break;
    case CHILD_ID::PIDMODE_2:
      pid2.mode = message.getBool();
      myPID2.SetMode(message.getBool());
      EEPROM.put(EEPROMAddresses::PID2_MODE, message.getBool());
      play_coin();
      break;
    case CHILD_ID::PIDSETPOINT_2:
      pid2.setpoint = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID2_SETPOINT, pid2.setpoint);
      break;
    case CHILD_ID::PIDkP0_2:
      pid2.kp = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID2_KP, pid2.kp);
      break;
    case CHILD_ID::PIDkI0_2:
      pid2.ki = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID2_KI, pid2.ki);
      break;
    case CHILD_ID::PIDkD0_2:
      pid2.kd = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID2_KD, pid2.kd);
      break;
    case CHILD_ID::PIDkP1_2:
      pid2.aggKp = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID2_AGG_KP, pid2.aggKp);
      break;
    case CHILD_ID::PIDkI1_2:
      pid2.aggKi = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID2_AGG_KI, pid2.aggKi);
      break;
    case CHILD_ID::PIDkD1_2:
      pid2.aggKd = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID2_AGG_KD, pid2.aggKd);
      break;
    case CHILD_ID::AdaptiveSP_2:
      pid2.aggSP = message.getByte();
      EEPROM.put(EEPROMAddresses::PID2_AGG_SP, pid2.aggSP);
      break;
    case CHILD_ID::AdaptiveMode_2:
      pid2.adaptiveMode = message.getBool();
      EEPROM.put(EEPROMAddresses::PID2_ADAPTIVE_MODE, pid2.adaptiveMode);
      break;
    case CHILD_ID::PIDMODE_3:
      pid3.mode = message.getBool();
      myPID3.SetMode(message.getBool());
      EEPROM.put(EEPROMAddresses::PID3_MODE, message.getBool());
      play_coin();
      break;
    case CHILD_ID::PIDSETPOINT_3:
      pid3.setpoint = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID3_SETPOINT, pid3.setpoint);
      break;
    case CHILD_ID::PIDkP0_3:
      pid3.kp = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID3_KP, pid3.kp);
      break;
    case CHILD_ID::PIDkI0_3:
      pid3.ki = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID3_KI, pid3.ki);
      break;
    case CHILD_ID::PIDkD0_3:
      pid3.kd = message.getFloat();
      EEPROM.put(EEPROMAddresses::PID3_KD, pid3.kd);
      break;
    case CHILD_ID::MainsCurrentMultiplier:
      calValues.emonCal = message.getFloat();
      EEPROM.put(EEPROMAddresses::EMON_CAL, calValues.emonCal);
      break;
    case CHILD_ID::SSRFail_Threshhold:
      calValues.ssrFailThreshold = message.getByte();
      EEPROM.put(EEPROMAddresses::SSR_FAIL_THRESHOLD, calValues.ssrFailThreshold);
      break;
    case CHILD_ID::MainsCurrentOffset:
      calValues.currOffset = message.getFloat();
      EEPROM.put(EEPROMAddresses::CURR_OFFSET, calValues.currOffset);
      break;
    case CHILD_ID::LOAD_MEMORY:
      if (message.getBool()) {
        FactoryResetEEPROM();
        sendInfo("FR");
      }
      break;
    case CHILD_ID::s_debug:
      configValues.sDebug = message.getBool();
      EEPROM.put(EEPROMAddresses::S_DEBUG, configValues.sDebug);
      break;
    case CHILD_ID::PIDThreshold_1:
      pid1.alarmThreshold = message.getInt();
      EEPROM.put(EEPROMAddresses::PID1_ALARM_THRESHOLD, pid1.alarmThreshold);
      break;
    case CHILD_ID::PIDThreshold_2:
      pid2.alarmThreshold = message.getInt();
      EEPROM.put(EEPROMAddresses::PID2_ALARM_THRESHOLD, pid2.alarmThreshold);
      break;
    case CHILD_ID::PIDThreshold_3:
      pid3.alarmThreshold = message.getInt();
      EEPROM.put(EEPROMAddresses::PID3_ALARM_THRESHOLD, pid3.alarmThreshold);
      break;
    case CHILD_ID::P1Offset:
      calValues.pressure1Offset = message.getFloat();
      EEPROM.put(EEPROMAddresses::PRESSURE1_OFFSET, calValues.pressure1Offset);
      break;
    case CHILD_ID::P2Offset:
      calValues.pressure2Offset = message.getFloat();
      EEPROM.put(EEPROMAddresses::PRESSURE2_OFFSET, calValues.pressure2Offset);
      break;
    case CHILD_ID::P3Offset:
      calValues.pressure3Offset = message.getFloat();
      EEPROM.put(EEPROMAddresses::PRESSURE3_OFFSET, calValues.pressure3Offset);
      break;
    case CHILD_ID::P4Offset:
      calValues.pressure4Offset = message.getFloat();
      EEPROM.put(EEPROMAddresses::PRESSURE4_OFFSET, calValues.pressure4Offset);
      break;
    case CHILD_ID::ScaleCalibrateKnownValue:
      setScaleCalibration(message.getFloat());
      break;
  }

  // Handle Relay States
  for (int i = 0; i < NUM_RELAYS; i++) {
    if (message.sensor == CHILD_ID::relay1 + i) {
      relayStates[i] = message.getBool();
      sendRelayStates();
    }
  }
}
void queryRelayStates() {
  if (Serial3.available()) {
    byte relayByte = Serial3.read();
    for (int i = 0; i < NUM_RELAYS; i++) {
      relayStates[i] = relayByte & (1 << i);
    }
  }
}
void sendRelayStates() {
  byte relayByte = 0;
  for (int i = 0; i < NUM_RELAYS; i++) {
    if (relayStates[i]) {
      relayByte |= (1 << i);
    }
    send(msgRelay[i].set(relayStates[i]), configValues.toACK);
  }
  Serial3.write(relayByte);
}

void sendAllStates() {
  // Send calibration values
  send(MyMessage(CHILD_ID::P1Cal, V_LEVEL).set(calValues.pressure1Cal, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::P2Cal, V_LEVEL).set(calValues.pressure2Cal, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::P3Cal, V_LEVEL).set(calValues.pressure3Cal, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::P4Cal, V_LEVEL).set(calValues.pressure4Cal, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::ScaleCal, V_LEVEL).set(calValues.scaleCal, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::MainsCurrentMultiplier, V_LEVEL).set(calValues.emonCal, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::SSRFail_Threshhold, V_LEVEL).set(calValues.ssrFailThreshold));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::MainsCurrentOffset, V_LEVEL).set(calValues.currOffset, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::P1Offset, V_LEVEL).set(calValues.pressure1Offset));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::P2Offset, V_LEVEL).set(calValues.pressure2Offset));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::P3Offset, V_LEVEL).set(calValues.pressure3Offset));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::P4Offset, V_LEVEL).set(calValues.pressure4Offset));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::ScaleOffset, V_LEVEL).set(calValues.zeroOffsetScale, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::s_debug, V_STATUS).set(configValues.sDebug));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::SSR_Armed, V_STATUS).set(ssrArmed));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::ScaleTare, V_STATUS).set(false));

  // Send PID values
  send(MyMessage(CHILD_ID::PIDSETPOINT_1, V_TEMP).set(pid1.setpoint, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkP0_1, V_LEVEL).set(pid1.kp, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkI0_1, V_LEVEL).set(pid1.ki, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkD0_1, V_LEVEL).set(pid1.kd, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkP1_1, V_LEVEL).set(pid1.aggKp, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkI1_1, V_LEVEL).set(pid1.aggKi, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkD1_1, V_LEVEL).set(pid1.aggKd, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::AdaptiveSP_1, V_LEVEL).set(pid1.aggSP));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::AdaptiveMode_1, V_STATUS).set(pid1.adaptiveMode));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDThreshold_1, V_LEVEL).set(pid1.alarmThreshold));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDMODE_1, V_STATUS).set(pid1.mode));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDSETPOINT_2, V_TEMP).set(pid2.setpoint, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkP0_2, V_LEVEL).set(pid2.kp, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkI0_2, V_LEVEL).set(pid2.ki, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkD0_2, V_LEVEL).set(pid2.kd, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkP1_2, V_LEVEL).set(pid2.aggKp, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkI1_2, V_LEVEL).set(pid2.aggKi, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkD1_2, V_LEVEL).set(pid2.aggKd, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::AdaptiveSP_2, V_LEVEL).set(pid2.aggSP));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::AdaptiveMode_2, V_STATUS).set(pid2.adaptiveMode));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDThreshold_2, V_LEVEL).set(pid2.alarmThreshold));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDMODE_2, V_STATUS).set(pid2.mode));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDSETPOINT_3, V_TEMP).set(pid3.setpoint, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkP0_3, V_LEVEL).set(pid3.kp, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkI0_3, V_LEVEL).set(pid3.ki, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDkD0_3, V_LEVEL).set(pid3.kd, 2));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDThreshold_3, V_LEVEL).set(pid3.alarmThreshold));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::PIDMODE_3, V_STATUS).set(pid3.mode));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::ScaleCalibrateKnownValue, V_LEVEL).set(0));
  wait(SENDDELAY);
  send(MyMessage(CHILD_ID::LOAD_MEMORY, V_STATUS).set(false));
  wait(SENDDELAY);


}

/**
 * @brief Sets the scale calibration value based on a known weight.
 *
 * This function takes a known weight as input, measures the raw value from the load cell,
 * and calculates the calibration factor. It then updates the calibration value in the
 * calibration structure and stores it in EEPROM.
 *
 * @param knownWeight The known weight in the same units as the scale is set to measure.
 */
void setScaleCalibration(float knownWeight) {
  if (LoadCell.is_ready()) {
    float rawValue = LoadCell.get_value(10);
    calValues.scaleCal = rawValue / knownWeight;
    LoadCell.set_scale(calValues.scaleCal);
    EEPROM.put(EEPROMAddresses::SCALE_CAL, calValues.scaleCal);
    send(MyMessage(CHILD_ID::ScaleCal, V_LEVEL).set(calValues.scaleCal, 2));
    wait(SENDDELAY);
    sendInfo("Scale calibrated");
  } else {
    sendInfo("Scale not ready");
  }
}
/**
 * @brief Calculates the resistance using a voltage divider.
 *
 * This function takes an analog pin and a known divider resistor value as input,
 * reads the analog value from the pin, and calculates the resistance of the unknown resistor
 * using the voltage divider formula.
 *
 * @param pin The analog pin number where the voltage divider is connected.
 * @param dividerResistor The known resistance value of the divider resistor in ohms.
 * @return The calculated resistance of the unknown resistor in ohms.
 */
float voltageDivider(int pin, float dividerResistor) {
  float ADCvalue = 0;
  float unknownResistance = 0.0;
  for (int n = 0; n < 10; n++) {
    wait(10);
    ADCvalue += analogRead(pin);
  }
  ADCvalue /= 10;
  float Vin = (AREF_V/1000.0);
  if(ADCvalue){
    float Vadc = ADCvalue * (Vin / 1023.0);
    //Serial.print("Voltage Pin ");
    //Serial.print(pin);
    //Serial.print(" : ");
    //Serial.print(Vadc,2);
    //Serial.print(" : ");
    //Serial.print(Vin,2);
    if (( Vadc / Vin ) > 0.5 ) {
      unknownResistance = (Vadc/ Vin) * dividerResistor;
    } else {
      unknownResistance = (Vin / Vadc) * dividerResistor;
    }  
    //Serial.print(" : ");
    //Serial.println(unknownResistance,2);
  }
  return unknownResistance;
}