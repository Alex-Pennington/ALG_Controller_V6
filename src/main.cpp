/*TODO:
  1) add function to check if pid inputs are out of bounds, hi or low, and allStop()
  2) update default values for PID to be stored in EEPROM
  3) update all calValues to be stored in EEPROM
  4) add function to check if pressure sensor readings are out of bounds, hi, and allStop()
  5) add function to check if temperature sensor readings are out of bounds, hi, and alertOperator()
  6) check out HX711_ADC by Olav Kallhovd

*/
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
#include "MegunoLink.h"
#include "Filter.h"
#include <CRC32.h>

// Enable and select radio type attached
#define MY_NODE_ID 201
#define MY_RADIO_RF24
#define MY_RF24_PA_LEVEL RF24_PA_HIGH
#define MY_RF24_CE_PIN 49
#define MY_RF24_CS_PIN 53
// #define MY_DEBUG
#include <MySensors.h>

// MySensors Child IDs
enum CHILD_ID
{
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
  // 17,
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
  // 30,
  // 31,
  PIDThreshold_2 = 32,

  dC_3 = 33,
  PIDMODE_3 = 34,
  PIDSETPOINT_3 = 35,
  Duty3 = 36,
  PIDkP0_3 = 37,
  PIDkI0_3 = 38,
  PIDkD0_3 = 39,
  // 40,
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
  Steinhart_SensorID = 71,
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
  RefrigerantPumpHighPressureSwitch = 90,
  FlowSwitch = 91,
  ScaleCalibrateKnownValue = 92,
  ScaleTempCalibrationMultiplier = 93,
  OLED_line1 = 94,
  OLED_line2 = 95,
  PID1_SENSORID = 96,
  PID2_SENSORID = 97,
  PID3_SENSORID = 98,
  T5 = 99,
  T6 = 100,
  T7 = 101,
  T8 = 102,
  T9 = 103,
  T10 = 104,
  OLED_line3 = 105,
  OLED_line4 = 106,
  OLED_line5 = 107,
  OLED_line6 = 108,

};

// MySensors Message Definitions

MyMessage msgSSRArmed(CHILD_ID::SSR_Armed, V_STATUS);
MyMessage msgLoadEEPROM(CHILD_ID::LOAD_MEMORY, V_STATUS);
MyMessage msgINFO(CHILD_ID::Info, V_TEXT);

MyMessage msgPIDMODE_1(CHILD_ID::PIDMODE_1, V_STATUS);
// MyMessage msgPIDSETPOINT(CHILD_ID::PIDSETPOINT_1, V_TEMP);
// MyMessage msgKp(CHILD_ID::PIDkP0_1, V_LEVEL);
// MyMessage msgKi(CHILD_ID::PIDkI0_1, V_LEVEL);
// MyMessage msgKd(CHILD_ID::PIDkD0_1, V_LEVEL);
// MyMessage msgEDC(CHILD_ID::EDC_1, V_PERCENTAGE);

MyMessage msgPIDMODE_2(CHILD_ID::PIDMODE_2, V_STATUS);
// MyMessage msgPIDSETPOINT_2(CHILD_ID::PIDSETPOINT_2, V_TEMP);
// MyMessage msgKp_2(CHILD_ID::PIDkP0_2, V_LEVEL);
// MyMessage msgKi_2(CHILD_ID::PIDkI0_2, V_LEVEL);
// MyMessage msgKd_2(CHILD_ID::PIDkD0_2, V_LEVEL);
// MyMessage msgEDC_2(CHILD_ID::EDC_2, V_PERCENTAGE);

MyMessage msgPIDMODE_3(CHILD_ID::PIDMODE_3, V_STATUS);
// MyMessage msgPIDSETPOINT_3(CHILD_ID::PIDSETPOINT_3, V_TEMP);
// MyMessage msgKp_3(CHILD_ID::PIDkP0_3, V_LEVEL);
// MyMessage msgKi_3(CHILD_ID::PIDkI0_3, V_LEVEL);
// MyMessage msgKd_3(CHILD_ID::PIDkD0_3, V_LEVEL);
// MyMessage msgEDC_3(CHILD_ID::EDC_3, V_PERCENTAGE);

MyMessage msgTemp0(CHILD_ID::T0, V_TEMP);
MyMessage msgTemp1(CHILD_ID::T1, V_TEMP);
MyMessage msgTemp2(CHILD_ID::T2, V_TEMP);
MyMessage msgTemp3(CHILD_ID::T3, V_TEMP);
MyMessage msgTemp4(CHILD_ID::T4, V_TEMP);
MyMessage msgTemp5(CHILD_ID::T5, V_TEMP);
MyMessage msgTemp6(CHILD_ID::T6, V_TEMP);
MyMessage msgTemp7(CHILD_ID::T7, V_TEMP);
MyMessage msgTemp8(CHILD_ID::T8, V_TEMP);
MyMessage msgTemp9(CHILD_ID::T9, V_TEMP);
MyMessage msgTemp10(CHILD_ID::T10, V_TEMP);
MyMessage msgSteinhart(CHILD_ID::Steinhart_SensorID, V_TEMP);
MyMessage msgTHMS1(CHILD_ID::THMS1, V_TEMP);
MyMessage msgTHMS2(CHILD_ID::THMS2, V_TEMP);

MyMessage msgScale(CHILD_ID::Scale, V_WEIGHT);
MyMessage msgScaleTare(CHILD_ID::ScaleTare, V_STATUS);
MyMessage msgScaleOffset(CHILD_ID::ScaleOffset, V_LEVEL);
MyMessage msgScaleRate(CHILD_ID::dTscale, V_LEVEL);

MyMessage msgPressure1(CHILD_ID::P1, V_PRESSURE);
// MyMessage msgPress1Offset(CHILD_ID::Press1Offset, V_LEVEL);

MyMessage msgPressure2(CHILD_ID::P2, V_PRESSURE);
MyMessage msgPressure3(CHILD_ID::P3, V_PRESSURE);
MyMessage msgPressure4(CHILD_ID::P4, V_PRESSURE);

MyMessage msgMainsCurrent(CHILD_ID::MainsCurrent, V_CURRENT);
// MyMessage msgMainsCurrentMultiplier(CHILD_ID::MainsCurrentMultiplier, V_LEVEL);
MyMessage msgSSRFailAlarm(CHILD_ID::SSRFail_Alarm, V_STATUS);
MyMessage msgVccVoltage(CHILD_ID::VccVoltage, V_VOLTAGE);
MyMessage msgVccCurrent(CHILD_ID::VccCurrent, V_CURRENT);
MyMessage msgRunTime(CHILD_ID::runTime, V_VAR1);

MyMessage msgSwitch1(CHILD_ID::switch1, V_VAR1);
MyMessage msgSwitch2(CHILD_ID::switch2, V_VAR1);
MyMessage msgSwitch3(CHILD_ID::switch3, V_VAR1);
MyMessage msgSwitch4(CHILD_ID::switch4, V_VAR1);
MyMessage msgSwitch5(CHILD_ID::switch5, V_VAR1);
MyMessage msgRefrigerantSwitch(CHILD_ID::RefrigerantPumpHighPressureSwitch, V_STATUS);
MyMessage msgFlowSwitch(CHILD_ID::FlowSwitch, V_STATUS);

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
    MyMessage(CHILD_ID::relay8, V_STATUS)};

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define VERSION "0.6.1.0"
#define SENDDELAY 50
#define CELSIUS_TO_FAHRENHEIT_FACTOR 1.8
#define CELSIUS_TO_FAHRENHEIT_OFFSET 32.0

#define EEPROM_SIZE 4096 // Adjust this size according to your EEPROM size *MEGA 4096*

// Structs

struct DS18B20Values
{
  byte addr[8];
  float F;
};
DS18B20Values ds18b20Values[10];

struct ConfigurationValues
{
  int sensorLoopTime = 23000;
  int scaleLoopTime = 5000;
  bool sDebug = true;
  bool toACK = false;
  int pidLoopTime = 10000;
  int pressureFilterWeight = 90;
  int scaleFilterWeight = 90;
};
ConfigurationValues configValues;

struct PIDConfig
{
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

struct CalibrationValues
{
  float pressure1Cal = 1;
  int pressure1Offset = 0;
  float pressure2Cal = 1;
  int pressure2Offset = 0;
  float pressure3Cal = 1;
  int pressure3Offset = 0;
  float pressure4Cal = 1;
  int pressure4Offset = 0;
  float emonCurrCal = 1;
  byte ssrFailThreshold = 2;
  double emonCurrOffset = 0;
  float zeroOffsetScale = 0;
  float scaleCal = 1;
  int VccCurrentOffset = 566;
  float VccCurrentMultiplier = 0.004887;
  float scaleTempCalibrationMultiplier = 7.77;
};
CalibrationValues calValues;

struct SteinhartConfig
{
  const long seriesResistor = 98600;
  const long nominalResistance = 100000;
  const int nominalTemperature = 25;
  const int bCoefficient = 3950;
};
SteinhartConfig thermistorConfig;

struct EmonVars
{
  // unsigned long sampleTime = 0;
  bool ssrFail = false;
  int ssrFailCount = 0;
};
EmonVars emonVars;

struct DutyCycle
{
  bool element = false;
  int loopTime = 10; // Duty cycle total loop time in seconds.
  unsigned long onTime = 0;
  unsigned long offTime = 0;
};
DutyCycle dutyCycle[3];

struct SteinhartValues
{
  float resistance = 0.0;
  float adcValue = 0;
  float steinhart = 0;
};
SteinhartValues steinhartValues;

struct buttons
{
  bool up = false;
  bool down = false;
};
buttons button[7];
// buttons 1-5 are for the switches
// button 6 is the coolant flow switch
// button 7 is the refrigerant tank capacity switch

// Working Variables
unsigned long scaleLoop_timer = 0;
unsigned long dutyCycle_timer = 0;
unsigned long SensorLoop_timer = 0;
unsigned long pid_compute_loop_time = 0;
unsigned long switchesLoop_timer = 0;
float value_oldScale = 0;
unsigned long oldtimeScale = 0;
extern unsigned int __heap_start;
extern void *__brkval;
bool firstrunSensorLoop = false;
#define SSRARMED_ON HIGH // Note to Copilot: Do Not Modify this value
#define ELEMENT_ON LOW   // Note to Copilot: Do Not Modify this value

// Array to hold all temperature sensor values
struct SensorValues
{
  float T0 = 0.0;
  float T1 = 0.0;
  float T2 = 0.0;
  float T3 = 0.0;
  float T4 = 0.0;
  float T5 = 0.0;
  float T6 = 0.0;
  float T7 = 0.0;
  float T8 = 0.0;
  float T9 = 0.0;
  float Steinhart = 0.0;
  float Scale = 0.0;
  float ScaleRate = 0.0;
  float Pressure1 = 0.0;
  float Pressure2 = 0.0;
  float Pressure3 = 0.0;
  float Pressure4 = 0.0;
  float VccVoltage = 0.0;
  float VccCurrent = 0.0;
  float MainsCurrent = 0.0;
  bool SSRFail_Alarm = false;
  bool relay1 = false;
  bool relay2 = false;
  bool relay3 = false;
  bool relay4 = false;
  bool relay5 = false;
  bool relay6 = false;
  bool relay7 = false;
  bool relay8 = false;
  bool RefrigerantPumpHighPressureSwitch = false;
  bool FlowSwitch = false;
  float dC1 = 0.0;
  float dC2 = 0.0;
  float dC3 = 0.0;
  bool ssrArmed = false;
  int OLED_line1_SENSORID = 0;
  int OLED_line2_SENSORID = 0;
  int PID1_SENSORID_VAR = 0;
  int PID2_SENSORID_VAR = 1;
  int PID3_SENSORID_VAR = 71;
  int OLED_line3_SENSORID = 0;
  int OLED_line4_SENSORID = 0;
  int OLED_line5_SENSORID = 0;
  int OLED_line6_SENSORID = 0;
  float THMS1 = 0.0;
  float THMS2 = 0.0;
};
SensorValues sensorValues;

ExponentialFilter<float> scaleWeightFiltered(10, 0);

// Pin Definitions
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
#define Thermistor1_PIN A1
#define Thermistor2_PIN A2

/*Connector Pinouts
Pressure(3)  1 5V, 2 GND, 3 SIG
Thermistor(2) 1 SIG, 2 SIG
Steinhart(2) 1 SIG, 2 SIG
HX711(4) 1 E-, 2 A+, 3 A-, 4 E+
Serial(4) 1 5V, 2 RX, 3 TX, 4 GND
R1(8) GND , <NC> , ElementPowerPin , <NC> , +5v , ElementPowerPin2 , SSRArmed_PIN , (Center) emon_Input_PIN
R2(8) GND , +5v , <NC>, ElementPowerPin3, <NC>, <NC>, <NC>, (Center) SteinhartPin
*/

// Switches
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
    ezButton(RefrigerantSwitchPin, INTERNAL_PULLUP)};

// Scale
HX711 LoadCell;

// DS18B20
OneWire ds(DS18B20_PIN); // on pin DS18B20_PIN (a 4.7K resistor is necessary)

// PIDs
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
// float getThermistor(int);
float readPressure(int pin, int offset, float cal, float lastValue);
void displayLine(const char *line, int row);
int freeMemory();
void emon();
void switchesLoop();
void getSwitches();
void TempAlarm();
void getScale();
void getVccCurrent();
void queryRelayStates();
void sendRelayStates();
void setScaleCalibration(float knownWeight);
float voltageDivider(int pin, float dividerResistor);
void getMainsCurrent();
float getSensorFloat(int sensorID);
char *getSensorString(int sensorID);
bool checkEEPROMCRC();
void updateEEPROMCRC();
void sanityCheckEEPROM();
float getThermistor(int pin);
void displayKnightRider();

enum EEPROMAddresses
{
  // 0,
  PRESSURE1_OFFSET = 4,           // int, 4 bytes
  SCALE_CAL = 8,                  // float, 4 bytes
  PRESSURE1_CAL = 12,             // float, 4 bytes
  PID1_SETPOINT = 16,             // double, 4 bytes
  PID1_KP = 20,                   // double, 4 bytes
  PID1_KI = 24,                   // double, 4 bytes
  PID1_KD = 28,                   // double, 4 bytes
  PID1_AGG_KP = 32,               // double, 4 bytes
  PID1_AGG_KI = 36,               // double, 4 bytes
  PID1_AGG_KD = 40,               // double, 4 bytes
  PID2_SETPOINT = 44,             // double, 4 bytes
  PID2_KP = 48,                   // double, 4 bytes
  PID2_KI = 52,                   // double, 4 bytes
  PID2_KD = 56,                   // double, 4 bytes
  PID2_AGG_KP = 60,               // double, 4 bytes
  PID2_AGG_KI = 64,               // double, 4 bytes
  PID2_AGG_KD = 68,               // double, 4 bytes
  PID1_AGG_SP = 72,               // byte, 1 byte
  PID2_AGG_SP = 73,               // byte, 1 byte
  PID3_SETPOINT = 74,             // double, 4 bytes
  PID3_KP = 78,                   // double, 4 bytes
  PID3_KI = 82,                   // double, 4 bytes
  PID3_KD = 86,                   // double, 4 bytes
  EMON_CAL = 90,                  // float, 4 bytes
  SSR_FAIL_THRESHOLD = 94,        // byte, 1 byte
  CURR_OFFSET = 95,               // double, 4 bytes
  SSR_ARMED = 99,                 // bool, 1 byte
  PID1_MODE = 100,                // bool, 1 byte
  PID2_MODE = 101,                // bool, 1 byte
  PID3_MODE = 102,                // bool, 1 byte
  PID2_ADAPTIVE_MODE = 103,       // bool, 1 byte
  PID1_ADAPTIVE_MODE = 104,       // bool, 1 byte
  S_DEBUG = 105,                  // bool, 1 byte
  PID1_ALARM_THRESHOLD = 106,     // int, 2 bytes
  PID2_ALARM_THRESHOLD = 108,     // int, 2 bytes
  PID3_ALARM_THRESHOLD = 110,     // int, 2 bytes
  PRESSURE2_OFFSET = 112,         // int, 4 bytes
  PRESSURE2_CAL = 116,            // float, 4 bytes
  PRESSURE3_CAL = 120,            // float, 4 bytes
  PRESSURE4_CAL = 124,            // float, 4 bytes
  PRESSURE3_OFFSET = 128,         // int, 4 bytes
  PRESSURE4_OFFSET = 132,         // int, 4 bytes
  VCC_CURRENT_OFFSET = 136,       // int, 4 bytes
  VCC_CURRENT_MULTIPLIER = 140,   // float, 4 bytes
  OLED_line1_SENSORID_ADDR = 144, // int, 2 bytes
  OLED_line2_SENSORID_ADDR = 146, // int, 2 bytes
  PID1_SENSORID_ADDR = 148,       // int, 2 bytes
  PID2_SENSORID_ADDR = 150,       // int, 2 bytes
  PID3_SENSORID_ADDR = 152,       // int, 2 bytes
  OLED_line3_SENSORID_ADDR = 154, // int, 2 bytes
  OLED_line4_SENSORID_ADDR = 156, // int, 2 bytes
  OLED_line5_SENSORID_ADDR = 158, // int, 2 bytes
  OLED_line6_SENSORID_ADDR = 160, // int, 2 bytes
  ZERO_OFFSET_SCALE = 162,          // float, 4 bytes
};

void presentation()
{
  // Send the sensor node sketch version information to the gateway
  sendSketchInfo("Controller", VERSION);

  present(CHILD_ID::T0, S_TEMP, "D0");
  wait(SENDDELAY);
  present(CHILD_ID::T1, S_TEMP, "D1");
  wait(SENDDELAY);
  present(CHILD_ID::T2, S_TEMP, "D2");
  wait(SENDDELAY);
  present(CHILD_ID::T3, S_TEMP, "D3");
  wait(SENDDELAY);
  present(CHILD_ID::T4, S_TEMP, "D4");
  wait(SENDDELAY);
  present(CHILD_ID::T5, S_TEMP, "D5");
  wait(SENDDELAY);
  present(CHILD_ID::T6, S_TEMP, "D6");
  wait(SENDDELAY);
  present(CHILD_ID::T7, S_TEMP, "D7");
  wait(SENDDELAY);
  present(CHILD_ID::T8, S_TEMP, "D8");
  wait(SENDDELAY);
  present(CHILD_ID::T9, S_TEMP, "D9");
  wait(SENDDELAY);
  present(CHILD_ID::Steinhart_SensorID, S_TEMP, "St1");
  wait(SENDDELAY);
  present(CHILD_ID::THMS1, S_TEMP, "Th1");
  wait(SENDDELAY);
  present(CHILD_ID::THMS2, S_TEMP, "Th2");
  wait(SENDDELAY);
  present(CHILD_ID::Scale, S_WEIGHT, "Scl");
  wait(SENDDELAY);
  present(CHILD_ID::ScaleTare, S_BINARY, "Scl T");
  wait(SENDDELAY);
  present(CHILD_ID::ScaleOffset, S_LIGHT_LEVEL, "Scl O");
  wait(SENDDELAY);
  present(CHILD_ID::ScaleCal, S_LIGHT_LEVEL, "Scl C");
  wait(SENDDELAY);
  present(CHILD_ID::P1, S_BARO, "P1");
  wait(SENDDELAY);
  present(CHILD_ID::P1Cal, S_LIGHT_LEVEL, "P1C");
  wait(SENDDELAY);
  present(CHILD_ID::P2, S_BARO, "P2");
  wait(SENDDELAY);
  present(CHILD_ID::P2Cal, S_LIGHT_LEVEL, "P2C");
  wait(SENDDELAY);
  present(CHILD_ID::P3, S_BARO, "P3");
  wait(SENDDELAY);
  present(CHILD_ID::P3Cal, S_LIGHT_LEVEL, "P3C");
  wait(SENDDELAY);
  present(CHILD_ID::P4, S_BARO, "P4");
  wait(SENDDELAY);
  present(CHILD_ID::P4Cal, S_LIGHT_LEVEL, "P4C");
  wait(SENDDELAY);
  present(CHILD_ID::MainsCurrent, S_MULTIMETER, "Mc");
  wait(SENDDELAY);
  present(CHILD_ID::MainsCurrentMultiplier, S_LIGHT_LEVEL, "McM");
  wait(SENDDELAY);
  present(CHILD_ID::MainsCurrentOffset, S_LIGHT_LEVEL, "McO");
  wait(SENDDELAY);
  present(CHILD_ID::SSRFail_Threshhold, S_LIGHT_LEVEL, "SFT");
  wait(SENDDELAY);
  present(CHILD_ID::VccCurrent, S_MULTIMETER, "VccC");
  wait(SENDDELAY);
  present(CHILD_ID::VccVoltage, S_LIGHT_LEVEL, "VccV");
  wait(SENDDELAY);
  present(CHILD_ID::dC_1, S_DIMMER, "dC1");
  wait(SENDDELAY);
  present(CHILD_ID::PIDMODE_1, S_BINARY, "PM1");
  wait(SENDDELAY);
  present(CHILD_ID::PIDSETPOINT_1, S_TEMP, "PS1");
  wait(SENDDELAY);
  present(CHILD_ID::PIDkP0_1, S_LIGHT_LEVEL, "kP1");
  wait(SENDDELAY);
  present(CHILD_ID::PIDkI0_1, S_LIGHT_LEVEL, "kI1");
  wait(SENDDELAY);
  present(CHILD_ID::PIDkD0_1, S_LIGHT_LEVEL, "kD1");
  wait(SENDDELAY);
  present(CHILD_ID::PIDThreshold_1, S_LIGHT_LEVEL, "P1T");
  wait(SENDDELAY);
  present(CHILD_ID::PIDThreshold_2, S_LIGHT_LEVEL, "P2T");
  wait(SENDDELAY);
  present(CHILD_ID::PIDThreshold_3, S_LIGHT_LEVEL, "P3T");
  wait(SENDDELAY);
  present(CHILD_ID::dC_2, S_DIMMER, "dC2");
  wait(SENDDELAY);
  present(CHILD_ID::PIDMODE_2, S_BINARY, "PM2");
  wait(SENDDELAY);
  present(CHILD_ID::PIDSETPOINT_2, S_TEMP, "PS2");
  wait(SENDDELAY);
  present(CHILD_ID::PIDkP0_2, S_LIGHT_LEVEL, "kP2");
  wait(SENDDELAY);
  present(CHILD_ID::PIDkI0_2, S_LIGHT_LEVEL, "kI2");
  wait(SENDDELAY);
  present(CHILD_ID::PIDkD0_2, S_LIGHT_LEVEL, "kD2");
  wait(SENDDELAY);
  present(CHILD_ID::dC_3, S_DIMMER, "dC3");
  wait(SENDDELAY);
  present(CHILD_ID::PIDMODE_3, S_BINARY, "PM3");
  wait(SENDDELAY);
  present(CHILD_ID::PIDSETPOINT_3, S_TEMP, "PS3");
  wait(SENDDELAY);
  present(CHILD_ID::PIDkP0_3, S_LIGHT_LEVEL, "kP3");
  wait(SENDDELAY);
  present(CHILD_ID::PIDkI0_3, S_LIGHT_LEVEL, "kI3");
  wait(SENDDELAY);
  present(CHILD_ID::PIDkD0_3, S_LIGHT_LEVEL, "kD3");
  wait(SENDDELAY);
  present(CHILD_ID::dTscale, S_LIGHT_LEVEL, "Scl R");
  wait(SENDDELAY);
  present(CHILD_ID::SSR_Armed, S_BINARY, "Armed");
  wait(SENDDELAY);
  present(CHILD_ID::SSRFail_Alarm, S_BINARY, "Fail");
  wait(SENDDELAY);
  present(CHILD_ID::Info, S_INFO, "Info");
  wait(SENDDELAY);
  present(CHILD_ID::LOAD_MEMORY, S_BINARY, "fRst");
  wait(SENDDELAY);
  present(CHILD_ID::runTime, S_CUSTOM, "rT");
  wait(SENDDELAY);

  // Present Switch Sensors
  for (int i = 0; i < 5; i++)
  {
    present(CHILD_ID::switch1 + i, S_CUSTOM, "Sw");
    wait(SENDDELAY);
  }

  present(CHILD_ID::FlowSwitch, S_BINARY, "FSw");
  wait(SENDDELAY);
  present(CHILD_ID::RefrigerantPumpHighPressureSwitch, S_BINARY, "RefSw");
  wait(SENDDELAY);

  present(CHILD_ID::P1Offset, S_LIGHT_LEVEL, "P1o");
  wait(SENDDELAY);
  present(CHILD_ID::P2Offset, S_LIGHT_LEVEL, "P2o");
  wait(SENDDELAY);
  present(CHILD_ID::P3Offset, S_LIGHT_LEVEL, "P3o");
  wait(SENDDELAY);
  present(CHILD_ID::P4Offset, S_LIGHT_LEVEL, "P4o");
  wait(SENDDELAY);

  // Present Relay Sensors
  for (int i = 0; i < NUM_RELAYS; i++)
  {
    present(CHILD_ID::relay1 + i, S_BINARY, "Rly");
    wait(SENDDELAY);
  }

  present(CHILD_ID::ScaleCalibrateKnownValue, S_LIGHT_LEVEL, "SCalKV");
  wait(SENDDELAY);
  present(CHILD_ID::ScaleTempCalibrationMultiplier, S_LIGHT_LEVEL, "STCM");
  wait(SENDDELAY);
  present(CHILD_ID::OLED_line1, S_LIGHT_LEVEL, "OL1");
  wait(SENDDELAY);
  present(CHILD_ID::OLED_line2, S_LIGHT_LEVEL, "OL2");
  wait(SENDDELAY);
  present(CHILD_ID::PID1_SENSORID, S_LIGHT_LEVEL, "P1ISv");
  wait(SENDDELAY);
  present(CHILD_ID::PID2_SENSORID, S_LIGHT_LEVEL, "P2ISv");
  wait(SENDDELAY);
  present(CHILD_ID::PID3_SENSORID, S_LIGHT_LEVEL, "P3ISv");
  wait(SENDDELAY);
  present(CHILD_ID::OLED_line3, S_LIGHT_LEVEL, "OL3");
  wait(SENDDELAY);
  present(CHILD_ID::OLED_line4, S_LIGHT_LEVEL, "OL4");
  wait(SENDDELAY);
  present(CHILD_ID::OLED_line5, S_LIGHT_LEVEL, "OL5");
  wait(SENDDELAY);
  present(CHILD_ID::OLED_line6, S_LIGHT_LEVEL, "OL6");
  wait(SENDDELAY);
}

void setup()
{
  Serial.begin(115200);
  Serial3.begin(9600);
  _process();

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    sendInfo("E_OLED");
    while (true)
      ; // Halt execution if display initialization fails
  }
  if (!checkEEPROMCRC())
  {
    sendInfo("EPROM ERROR");
    display.clearDisplay();
    displayLine("EPROM", 0);
    displayLine("ERROR", 1);
    display.display();
    //wait(30000);
    FactoryResetEEPROM();
  }
  getEEPROM();
  LoadCell.begin(HX711_dout, HX711_sck);
  LoadCell.set_scale(calValues.scaleCal);
  LoadCell.set_offset(calValues.zeroOffsetScale);
  LoadCell.set_gain();
  scaleWeightFiltered.SetWeight(configValues.scaleFilterWeight);
  pinMode(Thermistor1_PIN, INPUT);
  pinMode(Thermistor2_PIN, INPUT);
  pinMode(SSRArmed_PIN, OUTPUT);
  digitalWrite(SSRArmed_PIN, !SSRARMED_ON);
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

  for (int i = 0; i < NUM_SWITCHES; i++)
  {
    switches[i].setDebounceTime(50);
  }

  display.clearDisplay();
  displayLine("Booting...", 1);
  display.display();
  sendInfo("Operational");
  play_one_up();
  wait(1000);
}

void loop()
{
  DutyCycleLoop();
  _process();
  switchesLoop();

  if ((millis() - scaleLoop_timer) > (unsigned long)configValues.scaleLoopTime)
  {
    getScale();

    msgScale.set(sensorValues.Scale, 2);
    send(msgScale);
    wait(SENDDELAY);
    msgScaleRate.set(sensorValues.ScaleRate, 2);
    send(msgScaleRate);
    _process();
    scaleLoop_timer = millis();
  }

  if ((millis() - switchesLoop_timer) > (unsigned long)configValues.sensorLoopTime)
  {
    getSwitches();
    switchesLoop_timer = millis();
  }

  if ((millis() - SensorLoop_timer) > (unsigned long)configValues.sensorLoopTime)
  {
    if (!firstrunSensorLoop)
    {
      firstrunSensorLoop = true;
    }
    unsigned long sensorLoopTime = millis();
    sensorValues.VccVoltage = getBandgap();

    getVccCurrent();
    msgVccCurrent.set(sensorValues.VccCurrent, 2);
    send(msgVccCurrent);
    msgVccVoltage.set(sensorValues.VccVoltage, 2);
    send(msgVccVoltage);
    _process();

    getMainsCurrent();
    msgMainsCurrent.set(sensorValues.MainsCurrent, 2);
    send(msgMainsCurrent);
    _process();

    DS18B20();
    sensorValues.T0 = ds18b20Values[0].F;
    sensorValues.T1 = ds18b20Values[1].F;
    sensorValues.T2 = ds18b20Values[2].F;
    sensorValues.T3 = ds18b20Values[3].F;
    sensorValues.T4 = ds18b20Values[4].F;
    sensorValues.T5 = ds18b20Values[5].F;
    sensorValues.T6 = ds18b20Values[6].F;
    sensorValues.T7 = ds18b20Values[7].F;
    sensorValues.T8 = ds18b20Values[8].F;
    sensorValues.T9 = ds18b20Values[9].F;

    send(MyMessage(CHILD_ID::T0, V_TEMP).set(sensorValues.T0, 2));
    _process();
    send(MyMessage(CHILD_ID::T1, V_TEMP).set(sensorValues.T1, 2));
    _process();
    send(MyMessage(CHILD_ID::T2, V_TEMP).set(sensorValues.T2, 2));
    _process();
    send(MyMessage(CHILD_ID::T3, V_TEMP).set(sensorValues.T3, 2));
    _process();
    send(MyMessage(CHILD_ID::T4, V_TEMP).set(sensorValues.T4, 2));
    _process();
    send(MyMessage(CHILD_ID::T5, V_TEMP).set(sensorValues.T5, 2));
    _process();
    send(MyMessage(CHILD_ID::T6, V_TEMP).set(sensorValues.T6, 2));
    _process();
    send(MyMessage(CHILD_ID::T7, V_TEMP).set(sensorValues.T7, 2));
    _process();
    send(MyMessage(CHILD_ID::T8, V_TEMP).set(sensorValues.T8, 2));
    _process();
    send(MyMessage(CHILD_ID::T9, V_TEMP).set(sensorValues.T9, 2));
    _process();
    sensorValues.Steinhart = Steinhart();
    send(msgSteinhart.set(sensorValues.Steinhart, 2));
    _process();
    sensorValues.THMS1 = getThermistor(Thermistor1_PIN);
    send(msgTHMS1.set(sensorValues.THMS1, 2));
    _process();
    sensorValues.THMS2 = getThermistor(Thermistor2_PIN);
    send(msgTHMS2.set(sensorValues.THMS2, 2));
    _process();

    DutyCycleLoop();

    sensorValues.Pressure1 = readPressure(Pressure1PIN, calValues.pressure1Offset, calValues.pressure1Cal, sensorValues.Pressure1);
    msgPressure1.set(sensorValues.Pressure1, 2);
    send(msgPressure1);
    wait(SENDDELAY);
    _process();
    sensorValues.Pressure2 = readPressure(Pressure2PIN, calValues.pressure2Offset, calValues.pressure2Cal, sensorValues.Pressure2);
    msgPressure2.set(sensorValues.Pressure2, 2);
    send(msgPressure2);
    wait(SENDDELAY);
    _process();
    sensorValues.Pressure3 = readPressure(Pressure3PIN, calValues.pressure3Offset, calValues.pressure3Cal, sensorValues.Pressure3);
    msgPressure3.set(sensorValues.Pressure3, 2);
    send(msgPressure3);
    wait(SENDDELAY);
    _process();
    sensorValues.Pressure4 = readPressure(Pressure4PIN, calValues.pressure4Offset, calValues.pressure4Cal, sensorValues.Pressure4);
    msgPressure4.set(sensorValues.Pressure4, 2);
    send(msgPressure4);
    wait(SENDDELAY);
    _process();

    msgRunTime.set((float)(millis() / 1000.0 / 60.0 / 60.0), 2);
    send(msgRunTime);
    _process();

    queryRelayStates();
    wait(SENDDELAY);
    _process();
    sendRelayStates();
    wait(SENDDELAY);
    _process();

    display.clearDisplay();
    static char buffer[20];
    strncpy(buffer, getSensorString(sensorValues.OLED_line1_SENSORID), sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
    displayLine(buffer, 0);
    strncpy(buffer, getSensorString(sensorValues.OLED_line2_SENSORID), sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
    displayLine(buffer, 1);
    strncpy(buffer, getSensorString(sensorValues.OLED_line3_SENSORID), sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
    displayLine(buffer, 2);
    strncpy(buffer, getSensorString(sensorValues.OLED_line4_SENSORID), sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
    displayLine(buffer, 3);
    strncpy(buffer, getSensorString(sensorValues.OLED_line5_SENSORID), sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
    displayLine(buffer, 4);
    strncpy(buffer, getSensorString(sensorValues.OLED_line6_SENSORID), sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0'; // Ensure null-termination
    displayLine(buffer, 5);
    displayKnightRider();
    display.display();
    SensorLoop_timer = millis();
    Serial.print(millis() - sensorLoopTime);
    Serial.println(" ms");
  }

  if (((millis() - pid_compute_loop_time) > (unsigned long)configValues.pidLoopTime) & firstrunSensorLoop)
  {
    pid1.input = (double)getSensorFloat(sensorValues.PID1_SENSORID_VAR);
    pid2.input = (double)getSensorFloat(sensorValues.PID2_SENSORID_VAR);
    pid3.input = (double)getSensorFloat(sensorValues.PID3_SENSORID_VAR);
    TempAlarm();

    int gap = abs(pid1.setpoint - pid1.input); // distance away from setpoint
    if ((gap < pid1.aggSP && pid1.adaptiveMode == true) || pid1.adaptiveMode == false)
    {
      myPID1.SetTunings(pid1.kp, pid1.ki, pid1.kp);
    }
    else if (gap > pid1.aggSP && pid1.adaptiveMode == true)
    {
      myPID1.SetTunings(pid1.aggKp, pid1.aggKi, pid1.aggKd);
    }
    int gap_2 = abs(pid2.setpoint - pid2.input); // distance away from setpoint
    if ((gap_2 < pid2.aggSP && pid2.adaptiveMode == true) || pid2.adaptiveMode == false)
    {
      myPID2.SetTunings(pid2.kp, pid2.ki, pid2.kp);
    }
    else if (gap_2 > pid2.aggSP && pid2.adaptiveMode == true)
    {
      myPID2.SetTunings(pid2.aggKp, pid2.aggKi, pid2.aggKd);
    }
    myPID1.Compute();
    myPID2.Compute();
    myPID3.Compute();
    sensorValues.dC1 = pid1.output / 100.0;
    sensorValues.dC2 = pid2.output / 100.0;
    sensorValues.dC3 = pid3.output / 100.0;
    send(MyMessage(CHILD_ID::dC_1, V_PERCENTAGE).set(pid1.output, 2), configValues.toACK);
    send(MyMessage(CHILD_ID::dC_2, V_PERCENTAGE).set(pid2.output, 2), configValues.toACK);
    send(MyMessage(CHILD_ID::dC_3, V_PERCENTAGE).set(pid3.output, 2), configValues.toACK);
    pid_compute_loop_time = millis();
  }
}

void switchesLoop()
{
  for (int i = 0; i < NUM_SWITCHES; i++)
  {
    switches[i].loop();
  }
}
void getSwitches()
{
  for (int i = 0; i < 5; i++)
  {
    int btnState_UP = switches[i * 2].getState();
    int btnState_DOWN = switches[i * 2 + 1].getState();
    if (btnState_UP == 1 && btnState_DOWN == 0)
    {
      send(MyMessage(CHILD_ID::switch1 + i, V_VAR1).set(1), configValues.toACK);
    }
    else if (btnState_UP == 0 && btnState_DOWN == 1)
    {
      send(MyMessage(CHILD_ID::switch1 + i, V_VAR1).set(0), configValues.toACK);
    }
    else if (btnState_UP == 1 && btnState_DOWN == 1)
    {
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
  ExponentialFilter<long> adcFilter(10, sensorValues.VccCurrent);
  for (int i = 0; i < 10; i++)
  {
    adcFilter.Filter(analogRead(VccCurrentSensor));
    wait(10); // Small delay for better averaging
  }
  sensorValues.VccCurrent = adcFilter.Current();
  sensorValues.VccCurrent = -1.0 * (sensorValues.VccCurrent - (float)calValues.VccCurrentOffset) * calValues.VccCurrentMultiplier; //
  return;
}
void TempAlarm()
{
  // Temp Alarm
  if ((pid1.alarmThreshold < pid1.input) || (pid1.input < 1.0))
  {
    sendInfo("PID1!");
    // Serial.println("PID1!");
    red_alert();
    AllStop();
  }
  if ((pid2.alarmThreshold < pid2.input) || (pid2.input < 1.0))
  {
    sendInfo("PID2!");
    // Serial.println("PID2!");
    red_alert();
    AllStop();
  }
  if ((pid3.alarmThreshold < pid3.input) || (pid3.input < 1.0))
  {
    sendInfo("PID3!");
    // Serial.println("PID3!");
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

  for (int i = 0; i < 1000; i++)
  {
    float current = calValues.emonCurrCal * (analogRead(emon_Input_PIN) - 512); // in amps I presume
    sum += current * current;                                                   // sum squares
    wait(10);
  }
  sensorValues.MainsCurrent = sqrt(sum / 1000) - calValues.emonCurrOffset;
  if ((int(sensorValues.MainsCurrent) > int(calValues.ssrFailThreshold)))
  {
    red_alert();
    AllStop();
    sendInfo("emon SSR Fail");
    // Serial.println("emon SSR Fail");
    send(MyMessage(CHILD_ID::SSRFail_Alarm, V_STATUS).set(1), configValues.toACK);
  }
}
void getMainsCurrent()
{
  float sum = 0;
  for (int i = 0; i < 1000; i++)
  {
    float current = calValues.emonCurrCal * (analogRead(emon_Input_PIN) - 512); // in amps I presume
    sum += current * current;                                                   // sum squares
    wait(10);
  }
  sensorValues.MainsCurrent = sqrt(sum / 1000) - calValues.emonCurrOffset;
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
void getScale()
{
  value_oldScale = sensorValues.Scale;
  if (LoadCell.is_ready())
  {
    float tempOffset = (72 - sensorValues.T2) * calValues.scaleTempCalibrationMultiplier; // 72 is calibration temp in degrees Farhenheit
    scaleWeightFiltered.Filter((LoadCell.get_units(10)) + tempOffset);
  }
  else
  {
    sendInfo("E2");
    sensorValues.Scale = value_oldScale; // or handle the error as needed
  }
  if (scaleWeightFiltered.Current() < 0)
  {
    sensorValues.Scale = 0;
  }
  else
  {
    sensorValues.Scale = scaleWeightFiltered.Current();
  }

  sensorValues.ScaleRate = ((sensorValues.Scale - value_oldScale) / ((millis() - oldtimeScale)));
  oldtimeScale = millis();
  return;
}
float readPressure(int pin, int offset, float cal, float lastValue)
{
  ExponentialFilter<long> adcFilter(10, lastValue);
  adcFilter.SetWeight(configValues.pressureFilterWeight);
  int i = 0;
  for (i = 0; i < 10; i++)
  {
    wait(10);
    adcFilter.Filter(analogRead(Pressure1PIN));
  }
  float offsetCorrected = adcFilter.Current() - (float)offset;
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
float Steinhart()
{
  double adcValue = 0;
  for (int i = 0; i < 10; i++)
  {
    wait(10); // this increased resolution signifigantly
    adcValue += analogRead(SteinhartPin);
  }
  steinhartValues.adcValue = (float)adcValue / 10.0;
  float Resistance = ((thermistorConfig.seriesResistor + thermistorConfig.nominalResistance) * (1 / (1023 / steinhartValues.adcValue)));
  float steinhart = Resistance / (float)thermistorConfig.nominalResistance; // (R/Ro)
  steinhart = log(steinhart);                                               // ln(R/Ro)
  steinhart /= (float)thermistorConfig.bCoefficient;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / ((float)thermistorConfig.nominalTemperature + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                                              // Invert
  steinhart -= 273.15;                                                      // convert to C
  steinhart = (((steinhart * 9) / 5) + 32);
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
void DutyCycleLoop()
{
  if ((millis() - dutyCycle_timer) > (unsigned long)(dutyCycle[0].loopTime * 1000))
  {
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

  for (int i = 0; i < 3; i++)
  {
    // Determine the current PID mode and duty cycle
    bool pidMode;
    float dutyCycleValue;
    int elementPin;

    if (i == 0)
    {
      pidMode = pid1.mode;
      dutyCycleValue = sensorValues.dC1;
      elementPin = ElementPowerPin;
    }
    else if (i == 1)
    {
      pidMode = pid2.mode;
      dutyCycleValue = sensorValues.dC2;
      elementPin = ElementPowerPin2;
    }
    else
    {
      pidMode = pid3.mode;
      dutyCycleValue = sensorValues.dC3;
      elementPin = ElementPowerPin3;
    }

    // Check if the duty cycle loop is active and SSR is armed
    if (dutyCycleValue > 0.0 && sensorValues.ssrArmed && pidMode)
    {
      unsigned long currentTime = millis();
      unsigned long onDuration = dutyCycle[i].loopTime * 1000 * dutyCycleValue; // Convert duty cycle to on milliseconds
      unsigned long offDuration = dutyCycle[i].loopTime * 1000 - onDuration;    // Convert duty cycle to off milliseconds

      // If the element is off, check if it's time to turn it on
      if (!dutyCycle[i].element && (currentTime - dutyCycle[i].offTime) > offDuration)
      {
        dutyCycle[i].onTime = currentTime;
        digitalWrite(elementPin, ELEMENT_ON);
        dutyCycle[i].element = true;
      }
      // If the element is on, check if it's time to turn it off
      else if (dutyCycle[i].element && (currentTime - dutyCycle[i].onTime) > onDuration)
      {
        dutyCycle[i].offTime = currentTime;
        digitalWrite(elementPin, !ELEMENT_ON);
        dutyCycle[i].element = false;
      }
    }
    else
    { // If the duty cycle loop is not active or SSR is not armed, turn off the element
      digitalWrite(elementPin, !ELEMENT_ON);
      dutyCycle[i].element = false;
    }
  }
}
void AllStop()
{
  digitalWrite(SSRArmed_PIN, LOW);
  pid1.mode = false;
  pid2.mode = false;
  pid3.mode = false;
  sensorValues.ssrArmed = false;
  myPID1.SetMode(MANUAL);
  myPID2.SetMode(MANUAL);
  myPID3.SetMode(MANUAL);
  msgPIDMODE_1.set(pid1.mode);
  send(msgPIDMODE_1);
  msgPIDMODE_2.set(pid2.mode);
  send(msgPIDMODE_2);
  msgPIDMODE_3.set(pid3.mode);
  send(msgPIDMODE_3);
}
void StoreEEPROM()
{
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
  EEPROM.put(EEPROMAddresses::EMON_CAL, calValues.emonCurrCal);
  EEPROM.put(EEPROMAddresses::SSR_FAIL_THRESHOLD, calValues.ssrFailThreshold);
  EEPROM.put(EEPROMAddresses::CURR_OFFSET, calValues.emonCurrOffset);
  EEPROM.put(EEPROMAddresses::SSR_ARMED, sensorValues.ssrArmed);
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
  EEPROM.put(EEPROMAddresses::OLED_line1_SENSORID_ADDR, sensorValues.OLED_line1_SENSORID);
  EEPROM.put(EEPROMAddresses::OLED_line2_SENSORID_ADDR, sensorValues.OLED_line2_SENSORID);
  EEPROM.put(EEPROMAddresses::OLED_line3_SENSORID_ADDR, sensorValues.OLED_line3_SENSORID);
  EEPROM.put(EEPROMAddresses::OLED_line4_SENSORID_ADDR, sensorValues.OLED_line4_SENSORID);
  EEPROM.put(EEPROMAddresses::PID1_SENSORID_ADDR, sensorValues.PID1_SENSORID_VAR);
  EEPROM.put(EEPROMAddresses::PID2_SENSORID_ADDR, sensorValues.PID2_SENSORID_VAR);
  EEPROM.put(EEPROMAddresses::PID3_SENSORID_ADDR, sensorValues.PID3_SENSORID_VAR);
  EEPROM.put(EEPROMAddresses::OLED_line5_SENSORID_ADDR, sensorValues.OLED_line5_SENSORID);
  EEPROM.put(EEPROMAddresses::OLED_line6_SENSORID_ADDR, sensorValues.OLED_line6_SENSORID);

  printConfig();
  updateEEPROMCRC();
}
void getEEPROM()
{
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
  EEPROM.get(EEPROMAddresses::EMON_CAL, calValues.emonCurrCal);
  EEPROM.get(EEPROMAddresses::SSR_FAIL_THRESHOLD, calValues.ssrFailThreshold);
  EEPROM.get(EEPROMAddresses::CURR_OFFSET, calValues.emonCurrOffset);
  // EEPROM.get(EEPROMAddresses::SSR_ARMED, sensorValues.ssrArmed);
  // EEPROM.get(EEPROMAddresses::PID1_MODE, pid1.mode);
  // EEPROM.get(EEPROMAddresses::PID2_MODE, pid2.mode);
  // EEPROM.get(EEPROMAddresses::PID3_MODE, pid3.mode);
  EEPROM.get(EEPROMAddresses::PID2_ADAPTIVE_MODE, pid2.adaptiveMode);
  EEPROM.get(EEPROMAddresses::PID1_ADAPTIVE_MODE, pid1.adaptiveMode);
  EEPROM.get(EEPROMAddresses::S_DEBUG, configValues.sDebug);
  EEPROM.get(EEPROMAddresses::PID1_ALARM_THRESHOLD, pid1.alarmThreshold);
  EEPROM.get(EEPROMAddresses::PID2_ALARM_THRESHOLD, pid2.alarmThreshold);
  EEPROM.get(EEPROMAddresses::PID3_ALARM_THRESHOLD, pid3.alarmThreshold);
  EEPROM.get(EEPROMAddresses::PRESSURE2_OFFSET, calValues.pressure2Offset);
  EEPROM.get(EEPROMAddresses::PRESSURE2_CAL, calValues.pressure2Cal);
  EEPROM.get(EEPROMAddresses::PRESSURE3_OFFSET, calValues.pressure3Offset);
  EEPROM.get(EEPROMAddresses::PRESSURE3_CAL, calValues.pressure3Cal);
  EEPROM.get(EEPROMAddresses::PRESSURE4_OFFSET, calValues.pressure4Offset);
  EEPROM.get(EEPROMAddresses::PRESSURE4_CAL, calValues.pressure4Cal);
  EEPROM.get(EEPROMAddresses::VCC_CURRENT_OFFSET, calValues.VccCurrentOffset);
  EEPROM.get(EEPROMAddresses::VCC_CURRENT_MULTIPLIER, calValues.VccCurrentMultiplier);
  EEPROM.get(EEPROMAddresses::OLED_line1_SENSORID_ADDR, sensorValues.OLED_line1_SENSORID);
  EEPROM.get(EEPROMAddresses::OLED_line2_SENSORID_ADDR, sensorValues.OLED_line2_SENSORID);
  EEPROM.get(EEPROMAddresses::OLED_line3_SENSORID_ADDR, sensorValues.OLED_line3_SENSORID);
  EEPROM.get(EEPROMAddresses::OLED_line4_SENSORID_ADDR, sensorValues.OLED_line4_SENSORID);
  EEPROM.get(EEPROMAddresses::PID1_SENSORID_ADDR, sensorValues.PID1_SENSORID_VAR);
  EEPROM.get(EEPROMAddresses::PID2_SENSORID_ADDR, sensorValues.PID2_SENSORID_VAR);
  EEPROM.get(EEPROMAddresses::PID3_SENSORID_ADDR, sensorValues.PID3_SENSORID_VAR);
  EEPROM.get(EEPROMAddresses::OLED_line5_SENSORID_ADDR, sensorValues.OLED_line5_SENSORID);
  EEPROM.get(EEPROMAddresses::OLED_line6_SENSORID_ADDR, sensorValues.OLED_line6_SENSORID);
  printConfig();
}
void initDefaultValues() {
  
  /*Note to current future mainainers:
  It is my intention that this be the source function for the configurations and sensor definitions.
  When a change is made to the functionality of the apparatus make the changes here first.
  Then, use this as a reference to maintain consistancy accros all the other functions.
  Note to Copilot:  Follow the above directive.*/
  // Reset Calibration Values
  calValues.pressure1Cal = 1;
  calValues.pressure1Offset = 0;
  calValues.pressure2Cal = 1;
  calValues.pressure2Offset = 0;
  calValues.pressure3Cal = 1;
  calValues.pressure3Offset = 0;
  calValues.pressure4Cal = 1;
  calValues.pressure4Offset = 0;
  calValues.emonCurrCal = 1;
  calValues.ssrFailThreshold = 2;
  calValues.emonCurrOffset = 5.28;
  calValues.zeroOffsetScale = 0;
  calValues.scaleCal = 1;
  calValues.VccCurrentOffset = 566;
  calValues.VccCurrentMultiplier = 0.004887;
  calValues.scaleTempCalibrationMultiplier = 7.77;

  // Reset PID Configurations
  // 1500W 100V Water Heater Element /w circulation pump
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

  // Lower Hotplate various wattages, default 750 (hotplate set to full on).
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
  pid2.alarmThreshold = 150;
  pid2.input = 0;
  pid2.output = 0;

  // Bulk Purge Platter Hotplate, 400W 120V - Way to big!
  pid3.mode = false;
  pid3.setpoint = 90;
  pid3.kp = 1.2;
  pid3.ki = 0.01;
  pid3.kd = 0;
  pid3.aggKp = 0;
  pid3.aggKi = 0;
  pid3.aggKd = 0;
  pid3.aggSP = 10;
  pid3.adaptiveMode = false;
  pid3.alarmThreshold = 150;
  pid3.input = 0;
  pid3.output = 0;

  // Reset other configurations
  sensorValues.ssrArmed = false;
  configValues.sensorLoopTime = 23000;
  configValues.sDebug = false;
  configValues.toACK = false;
  configValues.pidLoopTime = 1000;
  configValues.pressureFilterWeight = 0.1;
  configValues.scaleFilterWeight = 0.1;
  sensorValues.OLED_line1_SENSORID = 0;
  sensorValues.OLED_line2_SENSORID = 1;
  sensorValues.OLED_line3_SENSORID = 0;
  sensorValues.OLED_line4_SENSORID = 1;
  sensorValues.OLED_line5_SENSORID = 1;
  sensorValues.OLED_line6_SENSORID = 1;
  sensorValues.PID1_SENSORID_VAR = 0;
  sensorValues.PID2_SENSORID_VAR = 1;
  sensorValues.PID3_SENSORID_VAR = 71;
}
void FactoryResetEEPROM() {
  initDefaultValues();
  StoreEEPROM();
}
void printConfig()
{
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
long getBandgap(void)
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRB &= ~_BV(MUX5); //
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  wait(10);            // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC))
    ; // measuring

  uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result;              // Vcc in millivolts
}
void sendInfo(String payload)
{
  _process();
  msgINFO.set(payload.c_str());
  send(msgINFO);
  _process();
  wait(SENDDELAY);
  Serial.println(payload);
}
void DS18B20()
{
  byte addr[8];
  int count = 0;

  while (ds.search(addr))
  {
    byte i;
    byte type_s;
    byte data[9];

    switch (addr[0])
    {
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
    for (i = 0; i < 9; i++)
    {
      data[i] = ds.read();
    }
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s)
    {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10)
      {
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    }
    else
    {
      byte cfg = (data[4] & 0x60);
      if (cfg == 0x00)
        raw = raw & ~7; // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20)
        raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40)
        raw = raw & ~1; // 11 bit res, 375 ms
    }
    float C = (float)raw / 16.0;
    ds18b20Values[count].F = C * 1.8 + 32.0;
    count = count + 1;
  }
  ds.reset_search();
  return;
}
#define LINE_HEIGHT 10
void displayLine(const char *line, int row)
{
  display.setCursor(0, LINE_HEIGHT * row);
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(line);
}
int freeMemory()
{
  int free_memory;
  if ((int)__brkval == 0)
  {
    free_memory = ((int)&free_memory) - ((int)&__heap_start);
  }
  else
  {
    free_memory = ((int)&free_memory) - ((int)__brkval);
  }
  return free_memory;
}
void receive(const MyMessage &message)
{
  int msgcmd = mGetCommand(message);
  if (configValues.sDebug || 1 == 1)
  {
    if (message.isAck())
    {
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
    Serial.print(" = ");
    Serial.println(message.data);
  }

  switch (message.sensor)
  {
  case CHILD_ID::ScaleTare:
    if (message.getBool())
    {
      if (configValues.sDebug)
        Serial.print("Tare ... ");
      calValues.zeroOffsetScale = LoadCell.read_average(100);
      if (configValues.sDebug)
        Serial.println(" done.");
      EEPROM.put(EEPROMAddresses::ZERO_OFFSET_SCALE, calValues.zeroOffsetScale);
      updateEEPROMCRC();
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
    updateEEPROMCRC();
    LoadCell.set_offset(calValues.zeroOffsetScale);
    break;
  case CHILD_ID::dC_1:
    sensorValues.dC1 = message.getFloat() / 100.0;
    break;
  case CHILD_ID::dC_2:
    sensorValues.dC2 = message.getFloat() / 100.0;
    break;
  case CHILD_ID::dC_3:
    sensorValues.dC3 = message.getFloat() / 100.0;
    break;
  case CHILD_ID::SSR_Armed:
    sensorValues.ssrArmed = message.getBool();
    EEPROM.put(EEPROMAddresses::SSR_ARMED, sensorValues.ssrArmed);
    updateEEPROMCRC();
    digitalWrite(SSRArmed_PIN, sensorValues.ssrArmed);
    play_coin();
    break;
  case CHILD_ID::Press1Offset:
    calValues.pressure1Offset = message.getInt();
    EEPROM.put(EEPROMAddresses::PRESSURE1_OFFSET, calValues.pressure1Offset);
    updateEEPROMCRC();
    break;
  case CHILD_ID::ScaleCal:
    calValues.scaleCal = message.getFloat();
    LoadCell.set_scale(calValues.scaleCal);
    EEPROM.put(EEPROMAddresses::SCALE_CAL, calValues.scaleCal);
    updateEEPROMCRC();
    break;
  case CHILD_ID::P1Cal:
    calValues.pressure1Cal = message.getFloat();
    EEPROM.put(EEPROMAddresses::PRESSURE1_CAL, calValues.pressure1Cal);
    updateEEPROMCRC();
    break;
  case CHILD_ID::P2Cal:
    calValues.pressure2Cal = message.getFloat();
    EEPROM.put(EEPROMAddresses::PRESSURE2_CAL, calValues.pressure2Cal);
    updateEEPROMCRC();
    break;
  case CHILD_ID::P3Cal:
    calValues.pressure3Cal = message.getFloat();
    EEPROM.put(EEPROMAddresses::PRESSURE3_CAL, calValues.pressure3Cal);
    updateEEPROMCRC();
    break;
  case CHILD_ID::P4Cal:
    calValues.pressure4Cal = message.getFloat();
    EEPROM.put(EEPROMAddresses::PRESSURE4_CAL, calValues.pressure4Cal);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDMODE_1:
    pid1.mode = message.getBool();
    myPID1.SetMode(message.getBool());
    EEPROM.put(EEPROMAddresses::PID1_MODE, message.getBool());
    updateEEPROMCRC();
    play_coin();
    break;
  case CHILD_ID::PIDSETPOINT_1:
    pid1.setpoint = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID1_SETPOINT, pid1.setpoint);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkP0_1:
    pid1.kp = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID1_KP, pid1.kp);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkI0_1:
    pid1.ki = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID1_KI, pid1.ki);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkD0_1:
    pid1.kd = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID1_KD, pid1.kd);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkP1_1:
    pid1.aggKp = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID1_AGG_KP, pid1.aggKp);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkI1_1:
    pid1.aggKi = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID1_AGG_KI, pid1.aggKi);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkD1_1:
    pid1.aggKd = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID1_AGG_KD, pid1.aggKd);
    updateEEPROMCRC();
    break;
  case CHILD_ID::AdaptiveSP_1:
    pid1.aggSP = message.getByte();
    EEPROM.put(EEPROMAddresses::PID1_AGG_SP, pid1.aggSP);
    updateEEPROMCRC();
    break;
  case CHILD_ID::AdaptiveMode_1:
    pid1.adaptiveMode = message.getBool();
    EEPROM.put(EEPROMAddresses::PID1_ADAPTIVE_MODE, pid1.adaptiveMode);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDMODE_2:
    pid2.mode = message.getBool();
    myPID2.SetMode(message.getBool());
    EEPROM.put(EEPROMAddresses::PID2_MODE, message.getBool());
    updateEEPROMCRC();
    play_coin();
    break;
  case CHILD_ID::PIDSETPOINT_2:
    pid2.setpoint = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID2_SETPOINT, pid2.setpoint);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkP0_2:
    pid2.kp = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID2_KP, pid2.kp);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkI0_2:
    pid2.ki = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID2_KI, pid2.ki);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkD0_2:
    pid2.kd = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID2_KD, pid2.kd);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkP1_2:
    pid2.aggKp = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID2_AGG_KP, pid2.aggKp);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkI1_2:
    pid2.aggKi = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID2_AGG_KI, pid2.aggKi);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkD1_2:
    pid2.aggKd = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID2_AGG_KD, pid2.aggKd);
    updateEEPROMCRC();
    break;
  case CHILD_ID::AdaptiveSP_2:
    pid2.aggSP = message.getByte();
    EEPROM.put(EEPROMAddresses::PID2_AGG_SP, pid2.aggSP);
    updateEEPROMCRC();
    break;
  case CHILD_ID::AdaptiveMode_2:
    pid2.adaptiveMode = message.getBool();
    EEPROM.put(EEPROMAddresses::PID2_ADAPTIVE_MODE, pid2.adaptiveMode);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDMODE_3:
    pid3.mode = message.getBool();
    myPID3.SetMode(message.getBool());
    EEPROM.put(EEPROMAddresses::PID3_MODE, message.getBool());
    updateEEPROMCRC();
    play_coin();
    break;
  case CHILD_ID::PIDSETPOINT_3:
    pid3.setpoint = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID3_SETPOINT, pid3.setpoint);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkP0_3:
    pid3.kp = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID3_KP, pid3.kp);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkI0_3:
    pid3.ki = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID3_KI, pid3.ki);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDkD0_3:
    pid3.kd = message.getFloat();
    EEPROM.put(EEPROMAddresses::PID3_KD, pid3.kd);
    updateEEPROMCRC();
    break;
  case CHILD_ID::MainsCurrentMultiplier:
    calValues.emonCurrCal = message.getFloat();
    EEPROM.put(EEPROMAddresses::EMON_CAL, calValues.emonCurrCal);
    updateEEPROMCRC();
    break;
  case CHILD_ID::SSRFail_Threshhold:
    calValues.ssrFailThreshold = message.getByte();
    EEPROM.put(EEPROMAddresses::SSR_FAIL_THRESHOLD, calValues.ssrFailThreshold);
    updateEEPROMCRC();
    break;
  case CHILD_ID::MainsCurrentOffset:
    calValues.emonCurrOffset = message.getFloat();
    EEPROM.put(EEPROMAddresses::CURR_OFFSET, calValues.emonCurrOffset);
    updateEEPROMCRC();
    break;
  case CHILD_ID::LOAD_MEMORY:
    if (message.getBool())
    {
      FactoryResetEEPROM();
      sendInfo("FR");
    }
    break;
  case CHILD_ID::s_debug:
    configValues.sDebug = message.getBool();
    EEPROM.put(EEPROMAddresses::S_DEBUG, configValues.sDebug);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDThreshold_1:
    pid1.alarmThreshold = message.getInt();
    EEPROM.put(EEPROMAddresses::PID1_ALARM_THRESHOLD, pid1.alarmThreshold);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDThreshold_2:
    pid2.alarmThreshold = message.getInt();
    EEPROM.put(EEPROMAddresses::PID2_ALARM_THRESHOLD, pid2.alarmThreshold);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PIDThreshold_3:
    pid3.alarmThreshold = message.getInt();
    EEPROM.put(EEPROMAddresses::PID3_ALARM_THRESHOLD, pid3.alarmThreshold);
    updateEEPROMCRC();
    break;
  case CHILD_ID::P1Offset:
    calValues.pressure1Offset = message.getFloat();
    EEPROM.put(EEPROMAddresses::PRESSURE1_OFFSET, calValues.pressure1Offset);
    updateEEPROMCRC();
    break;
  case CHILD_ID::P2Offset:
    calValues.pressure2Offset = message.getFloat();
    EEPROM.put(EEPROMAddresses::PRESSURE2_OFFSET, calValues.pressure2Offset);
    updateEEPROMCRC();
    break;
  case CHILD_ID::P3Offset:
    calValues.pressure3Offset = message.getFloat();
    EEPROM.put(EEPROMAddresses::PRESSURE3_OFFSET, calValues.pressure3Offset);
    updateEEPROMCRC();
    break;
  case CHILD_ID::P4Offset:
    calValues.pressure4Offset = message.getFloat();
    EEPROM.put(EEPROMAddresses::PRESSURE4_OFFSET, calValues.pressure4Offset);
    updateEEPROMCRC();
    break;
  case CHILD_ID::ScaleCalibrateKnownValue:
    setScaleCalibration(message.getFloat());
    updateEEPROMCRC();
    break;
  case CHILD_ID::OLED_line1:
    sensorValues.OLED_line1_SENSORID = message.getInt();
    EEPROM.put(EEPROMAddresses::OLED_line1_SENSORID_ADDR, sensorValues.OLED_line1_SENSORID);
    updateEEPROMCRC();
    break;
  case CHILD_ID::OLED_line2:
    sensorValues.OLED_line2_SENSORID = message.getInt();
    EEPROM.put(EEPROMAddresses::OLED_line2_SENSORID_ADDR, sensorValues.OLED_line2_SENSORID);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PID1_SENSORID:
    sensorValues.PID1_SENSORID_VAR = message.getInt();
    EEPROM.put(EEPROMAddresses::PID1_SENSORID_ADDR, PID1_SENSORID_ADDR);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PID2_SENSORID:
    sensorValues.PID2_SENSORID_VAR = message.getInt();
    EEPROM.put(EEPROMAddresses::PID2_SENSORID_ADDR, PID2_SENSORID_ADDR);
    updateEEPROMCRC();
    break;
  case CHILD_ID::PID3_SENSORID:
    sensorValues.PID3_SENSORID_VAR = message.getInt();
    EEPROM.put(EEPROMAddresses::PID3_SENSORID_ADDR, PID3_SENSORID_ADDR);
    updateEEPROMCRC();
    break;
  case CHILD_ID::OLED_line3:
    sensorValues.OLED_line3_SENSORID = message.getInt();
    EEPROM.put(EEPROMAddresses::OLED_line3_SENSORID_ADDR, sensorValues.OLED_line3_SENSORID);
    updateEEPROMCRC();
    break;
  case CHILD_ID::OLED_line4:
    sensorValues.OLED_line4_SENSORID = message.getInt();
    EEPROM.put(EEPROMAddresses::OLED_line4_SENSORID_ADDR, sensorValues.OLED_line4_SENSORID);
    updateEEPROMCRC();
    break;
  case CHILD_ID::OLED_line5:
    sensorValues.OLED_line5_SENSORID = message.getInt();
    EEPROM.put(EEPROMAddresses::OLED_line5_SENSORID_ADDR, sensorValues.OLED_line5_SENSORID);
    updateEEPROMCRC();
    break;
  case CHILD_ID::OLED_line6:
    sensorValues.OLED_line6_SENSORID = message.getInt();
    EEPROM.put(EEPROMAddresses::OLED_line6_SENSORID_ADDR, sensorValues.OLED_line6_SENSORID);
    updateEEPROMCRC();
    break;
  }

  // Handle Relay States
  for (int i = 0; i < NUM_RELAYS; i++)
  {
    if (message.sensor == CHILD_ID::relay1 + i)
    {
      relayStates[i] = message.getBool();
      sendRelayStates();
    }
  }
}
void queryRelayStates()
{
  if (Serial3.available())
  {
    byte relayByte = Serial3.read();
    for (int i = 0; i < NUM_RELAYS; i++)
    {
      relayStates[i] = relayByte & (1 << i);
    }
  }
}
void sendRelayStates()
{
  byte relayByte = 0;
  for (int i = 0; i < NUM_RELAYS; i++)
  {
    if (relayStates[i])
    {
      relayByte |= (1 << i);
    }
    send(msgRelay[i].set(relayStates[i]), configValues.toACK);
  }
  Serial3.write(relayByte);
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
void setScaleCalibration(float knownWeight)
{
  if (LoadCell.is_ready())
  {
    float rawValue = LoadCell.get_value(10);
    calValues.scaleCal = rawValue / knownWeight;
    LoadCell.set_scale(calValues.scaleCal);
    EEPROM.put(EEPROMAddresses::SCALE_CAL, calValues.scaleCal);
    updateEEPROMCRC();
    send(MyMessage(CHILD_ID::ScaleCal, V_LEVEL).set(calValues.scaleCal, 2));
    wait(SENDDELAY);
    sendInfo("Scale calibrated");
  }
  else
  {
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
float voltageDivider(int pin, float dividerResistor)
{
  float ADCvalue = 0;
  float unknownResistance = 0.0;
  for (int n = 0; n < 10; n++)
  {
    wait(10);
    ADCvalue += analogRead(pin);
  }
  ADCvalue /= 10;
  float Vin = (sensorValues.VccVoltage / 1000.0);
  if (ADCvalue)
  {
    float Vadc = ADCvalue * (Vin / 1023.0);
    // Serial.print("Voltage Pin ");
    // Serial.print(pin);
    // Serial.print(" : ");
    // Serial.print(Vadc,2);
    // Serial.print(" : ");
    // Serial.print(Vin,2);
    if ((Vadc / Vin) > 0.5)
    {
      unknownResistance = (Vadc / Vin) * dividerResistor;
    }
    else
    {
      unknownResistance = (Vin / Vadc) * dividerResistor;
    }
    // Serial.print(" : ");
    // Serial.println(unknownResistance,2);
  }
  return unknownResistance;
}
char *getSensorString(int sensorID)
{
  static char tempString[20] = "";

  char vBuffer[15];
  switch (sensorID)
  {
  case CHILD_ID::T0:
    dtostrf(sensorValues.T0, 4, 1, vBuffer);
    sprintf(tempString, "T0: %s", vBuffer);
    break;
  case CHILD_ID::T1:
    dtostrf(sensorValues.T1, 4, 1, vBuffer);
    sprintf(tempString, "T1: %s", vBuffer);
    break;
  case CHILD_ID::T2:
    dtostrf(sensorValues.T2, 4, 1, vBuffer);
    sprintf(tempString, "T2: %s", vBuffer);
    break;
  case CHILD_ID::T3:
    dtostrf(sensorValues.T3, 4, 1, vBuffer);
    sprintf(tempString, "T3: %s", vBuffer);
    break;
  case CHILD_ID::T4:
    dtostrf(sensorValues.T4, 4, 1, vBuffer);
    sprintf(tempString, "T4: %s", vBuffer);
    break;
  case CHILD_ID::T5:
    dtostrf(sensorValues.T5, 4, 1, vBuffer);
    sprintf(tempString, "T5: %s", vBuffer);
    break;
  case CHILD_ID::Steinhart_SensorID:
    dtostrf(sensorValues.Steinhart, 4, 1, vBuffer);
    sprintf(tempString, "Steinhart: %s", vBuffer);
    break;
  case CHILD_ID::Scale:
    dtostrf(sensorValues.Scale, 4, 1, vBuffer);
    sprintf(tempString, "Scale: %s", vBuffer);
    break;
  case CHILD_ID::dTscale:
    dtostrf(sensorValues.ScaleRate, 4, 2, vBuffer);
    sprintf(tempString, "dTs: %s", vBuffer);
    break;
  case CHILD_ID::P1:
    dtostrf(sensorValues.Pressure1, 4, 1, vBuffer);
    sprintf(tempString, "P1: %s", vBuffer);
    break;
  case CHILD_ID::P2:
    dtostrf(sensorValues.Pressure2, 4, 1, vBuffer);
    sprintf(tempString, "P2: %s", vBuffer);
    break;
  case CHILD_ID::P3:
    dtostrf(sensorValues.Pressure3, 4, 1, vBuffer);
    sprintf(tempString, "P3: %s", vBuffer);
    break;
  case CHILD_ID::P4:
    dtostrf(sensorValues.Pressure4, 4, 1, vBuffer);
    sprintf(tempString, "P4: %s", vBuffer);
    break;
  case CHILD_ID::VccVoltage:
    dtostrf(sensorValues.VccVoltage, 4, 1, vBuffer);
    sprintf(tempString, "VV: %s", vBuffer);
    break;
  case CHILD_ID::VccCurrent:
    dtostrf(sensorValues.VccCurrent, 4, 1, vBuffer);
    sprintf(tempString, "VC: %s", vBuffer);
    break;
  case CHILD_ID::MainsCurrent:
    dtostrf(sensorValues.MainsCurrent, 4, 1, vBuffer);
    sprintf(tempString, "MC: %s", vBuffer);
    break;
  case CHILD_ID::relay1:
    sprintf(tempString, "r1: %d", (int)sensorValues.relay1);
    break;
  case CHILD_ID::relay2:
    sprintf(tempString, "r2: %d", (int)sensorValues.relay2);
    break;
  case CHILD_ID::relay3:
    sprintf(tempString, "r3: %d", (int)sensorValues.relay3);
    break;
  case CHILD_ID::relay4:
    sprintf(tempString, "r4: %d", (int)sensorValues.relay4);
    break;
  case CHILD_ID::relay5:
    sprintf(tempString, "r5: %d", (int)sensorValues.relay5);
    break;
  case CHILD_ID::relay6:
    sprintf(tempString, "r6: %d", (int)sensorValues.relay6);
    break;
  case CHILD_ID::relay7:
    sprintf(tempString, "r7: %d", (int)sensorValues.relay7);
    break;
  case CHILD_ID::relay8:
    sprintf(tempString, "r8: %d", (int)sensorValues.relay8);
    break;
  case CHILD_ID::RefrigerantPumpHighPressureSwitch:
    sprintf(tempString, "RSw: %d", sensorValues.RefrigerantPumpHighPressureSwitch);
    break;
  case CHILD_ID::FlowSwitch:
    sprintf(tempString, "FlSw: %d", (int)sensorValues.FlowSwitch);
    break;
  case CHILD_ID::dC_1:
    dtostrf(sensorValues.dC1, 4, 2, vBuffer);
    sprintf(tempString, "dC1: %s", vBuffer);
    break;
  case CHILD_ID::dC_2:
    dtostrf(sensorValues.dC2, 4, 2, vBuffer);
    sprintf(tempString, "dC2: %s", vBuffer);
    break;
  case CHILD_ID::dC_3:
    dtostrf(sensorValues.dC3, 4, 2, vBuffer);
    sprintf(tempString, "dC3: %s", vBuffer);
    break;
  // case CHILD_ID::SSR_Armed:
  //   sprintf(tempString, "ssrArmed: %d", (int)sensorValues.ssrArmed);
  //   break;
  // case CHILD_ID::FreeMemory:
  //   sprintf(tempString, "FreeMem: %d", freeMemory());
  //   break;
  case CHILD_ID::T6:
    dtostrf(ds18b20Values[0].F, 4, 1, vBuffer);
    sprintf(tempString, "T6: %s", vBuffer);
    break;
  case CHILD_ID::T7:
    dtostrf(ds18b20Values[1].F, 4, 1, vBuffer);
    sprintf(tempString, "T7: %s", vBuffer);
    break;
  case CHILD_ID::T8:
    dtostrf(ds18b20Values[2].F, 4, 1, vBuffer);
    sprintf(tempString, "T8: %s", vBuffer);
    break;
  case CHILD_ID::T9:
    dtostrf(ds18b20Values[3].F, 4, 1, vBuffer);
    sprintf(tempString, "T9: %s", vBuffer);
    break;
  case CHILD_ID::T10:
    dtostrf(ds18b20Values[4].F, 4, 1, vBuffer);
    sprintf(tempString, "T10: %s", vBuffer);
    break;
  case CHILD_ID::THMS1:
    dtostrf(sensorValues.THMS1, 4, 1, vBuffer);
    sprintf(tempString, "THMS1: %s", vBuffer);
    break;
  case CHILD_ID::THMS2:
    dtostrf(sensorValues.THMS2, 4, 1, vBuffer);
    sprintf(tempString, "THMS2: %s", vBuffer);
    break;
  default:
    sprintf(tempString, "Unk");
    break;
  }

  return tempString;
}
float getSensorFloat(int sensorID)
{
  float tempFloat = 0.0;

  switch (sensorID)
  {
  case CHILD_ID::T0:
    tempFloat = sensorValues.T0;
    break;
  case CHILD_ID::T1:
    tempFloat = sensorValues.T1;
    break;
  case CHILD_ID::T2:
    tempFloat = sensorValues.T2;
    break;
  case CHILD_ID::T3:
    tempFloat = sensorValues.T3;
    break;
  case CHILD_ID::T4:
    tempFloat = sensorValues.T4;
    break;
  case CHILD_ID::T5:
    tempFloat = sensorValues.T5;
    break;
  case CHILD_ID::T6:
    tempFloat = sensorValues.T6;
    break;
  case CHILD_ID::T7:
    tempFloat = sensorValues.T7;
    break;
  case CHILD_ID::T8:
    tempFloat = sensorValues.T8;
    break;
  case CHILD_ID::T9:
    tempFloat = sensorValues.T9;
    break;
  case CHILD_ID::Steinhart_SensorID:
    tempFloat = sensorValues.Steinhart;
    break;
  case CHILD_ID::Scale:
    tempFloat = sensorValues.Scale;
    break;
  case CHILD_ID::dTscale:
    tempFloat = sensorValues.ScaleRate;
    break;
  case CHILD_ID::P1:
    tempFloat = sensorValues.Pressure1;
    break;
  case CHILD_ID::P2:
    tempFloat = sensorValues.Pressure2;
    break;
  case CHILD_ID::P3:
    tempFloat = sensorValues.Pressure3;
    break;
  case CHILD_ID::P4:
    tempFloat = sensorValues.Pressure4;
    break;
  case CHILD_ID::VccVoltage:
    tempFloat = sensorValues.VccVoltage;
    break;
  case CHILD_ID::VccCurrent:
    tempFloat = sensorValues.VccCurrent;
    break;
  case CHILD_ID::MainsCurrent:
    tempFloat = sensorValues.MainsCurrent;
    break;
  case CHILD_ID::SSRFail_Alarm:
    tempFloat = (float)sensorValues.SSRFail_Alarm;
    break;
  case CHILD_ID::relay1:
    tempFloat = (float)sensorValues.relay1;
    break;
  case CHILD_ID::relay2:
    tempFloat = (float)sensorValues.relay2;
    break;
  case CHILD_ID::relay3:
    tempFloat = (float)sensorValues.relay3;
    break;
  case CHILD_ID::relay4:
    tempFloat = (float)sensorValues.relay4;
    break;
  case CHILD_ID::relay5:
    tempFloat = (float)sensorValues.relay5;
    break;
  case CHILD_ID::relay6:
    tempFloat = (float)sensorValues.relay6;
    break;
  case CHILD_ID::relay7:
    tempFloat = (float)sensorValues.relay7;
    break;
  case CHILD_ID::relay8:
    tempFloat = (float)sensorValues.relay8;
    break;
  case CHILD_ID::RefrigerantPumpHighPressureSwitch:
    tempFloat = (float)(sensorValues.RefrigerantPumpHighPressureSwitch ? 1 : 0);
    break;
  case CHILD_ID::FlowSwitch:
    tempFloat = (float)sensorValues.FlowSwitch;
    break;
  case CHILD_ID::dC_1:
    tempFloat = sensorValues.dC1;
    break;
  case CHILD_ID::dC_2:
    tempFloat = sensorValues.dC2;
    break;
  case CHILD_ID::dC_3:
    tempFloat = sensorValues.dC3;
    break;
  case CHILD_ID::SSR_Armed:
    tempFloat = (float)sensorValues.ssrArmed;
    break;
  case CHILD_ID::THMS1:
    tempFloat = sensorValues.THMS1;
    break;
  case CHILD_ID::THMS2:
    tempFloat = sensorValues.THMS2;
    break;
  default:
    tempFloat = -999.010101;
    break;
  }

  return tempFloat;
}
uint32_t getEEPROMCRC() {
  CRC32 crc;
  for (int i = 0; i < EEPROM_SIZE - 4; i++)
  {
    crc.update(EEPROM.read(i));
  }
  return crc.finalize();
}
bool checkEEPROMCRC()
{
  uint32_t newCRC = getEEPROMCRC();
  uint32_t storedCRC;
  EEPROM.get(EEPROM_SIZE - 4, storedCRC);
  Serial.print("Check: ");
  Serial.print("stored CRC: ");
  Serial.print(storedCRC);
  Serial.print("  finalized() CRC: ");
  Serial.println(newCRC);
  return storedCRC == newCRC;
}
void updateEEPROMCRC()
{
  uint32_t newCRC = getEEPROMCRC();
  Serial.print("Update: ");
  Serial.print("New CRC: ");
  Serial.print(newCRC);
  EEPROM.put(EEPROM_SIZE - 4, newCRC);
  uint32_t storedCRC;
  EEPROM.get(EEPROM_SIZE - 4, storedCRC);
  Serial.print("  Stored CRC: ");
  Serial.println(storedCRC);
}
void sanityCheckEEPROM() {
  int tempINT;
  float tempFLOAT;
  double tempDOUBLE;
  byte tempBYTE;
  bool tempBOOL;

  Serial.println("Sanity Check EEPROM");

  if (EEPROM.get(EEPROMAddresses::ZERO_OFFSET_SCALE, tempFLOAT) != calValues.zeroOffsetScale)
  {
    Serial.print("Zero Offset Scale: ");
    Serial.println(tempFLOAT);
  }
  if (EEPROM.get(EEPROMAddresses::PRESSURE1_OFFSET, tempINT) != calValues.pressure1Offset)
  {
    Serial.print("Pressure1 Offset: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::SCALE_CAL, tempFLOAT) != calValues.scaleCal)
  {
    Serial.print("Scale Cal: ");
    Serial.println(tempFLOAT);
  }
  if (EEPROM.get(EEPROMAddresses::PRESSURE1_CAL, tempFLOAT) != calValues.pressure1Cal)
  {
    Serial.print("Pressure1 Cal: ");
    Serial.println(tempFLOAT);
  }
  if (EEPROM.get(EEPROMAddresses::PID1_SETPOINT, tempDOUBLE) != pid1.setpoint)
  {
    Serial.print("PID1 Setpoint: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID1_KP, tempDOUBLE) != pid1.kp)
  {
    Serial.print("PID1 KP: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID1_KI, tempDOUBLE) != pid1.ki)
  {
    Serial.print("PID1 KI: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID1_KD, tempDOUBLE) != pid1.kd)
  {
    Serial.print("PID1 KD: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID1_AGG_KP, tempDOUBLE) != pid1.aggKp)
  {
    Serial.print("PID1 Agg KP: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID1_AGG_KI, tempDOUBLE) != pid1.aggKi)
  {
    Serial.print("PID1 Agg KI: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID1_AGG_KD, tempDOUBLE) != pid1.aggKd)
  {
    Serial.print("PID1 Agg KD: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID2_SETPOINT, tempDOUBLE) != pid2.setpoint)
  {
    Serial.print("PID2 Setpoint: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID2_KP, tempDOUBLE) != pid2.kp)
  {
    Serial.print("PID2 KP: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID2_KI, tempDOUBLE) != pid2.ki)
  {
    Serial.print("PID2 KI: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID2_KD, tempDOUBLE) != pid2.kd)
  {
    Serial.print("PID2 KD: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID2_AGG_KP, tempDOUBLE) != pid2.aggKp)
  {
    Serial.print("PID2 Agg KP: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID2_AGG_KI, tempDOUBLE) != pid2.aggKi)
  {
    Serial.print("PID2 Agg KI: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID2_AGG_KD, tempDOUBLE) != pid2.aggKd) {
    Serial.print("PID2 Agg KD: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID1_AGG_SP, tempBYTE) != pid1.aggSP) {
    Serial.print("PID1 Agg SP: ");
    Serial.println(tempBYTE);
  }
  if (EEPROM.get(EEPROMAddresses::PID2_AGG_SP, tempBYTE) != pid2.aggSP)
  {
    Serial.print("PID2 Agg SP: ");
    Serial.println(tempBYTE);
  }
  if (EEPROM.get(EEPROMAddresses::PID3_SETPOINT, tempDOUBLE) != pid3.setpoint)
  {
    Serial.print("PID3 Setpoint: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID3_KP, tempDOUBLE) != pid3.kp)
  {
    Serial.print("PID3 KP: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID3_KI, tempDOUBLE) != pid3.ki)
  {
    Serial.print("PID3 KI: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::PID3_KD, tempDOUBLE) != pid3.kd)
  {
    Serial.print("PID3 KD: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::EMON_CAL, tempFLOAT) != calValues.emonCurrCal)
  {
    Serial.print("Emon Cal: ");
    Serial.println(tempFLOAT);
  }
  if (EEPROM.get(EEPROMAddresses::SSR_FAIL_THRESHOLD, tempBYTE) != calValues.ssrFailThreshold)
  {
    Serial.print("SSR Fail Threshold: ");
    Serial.println(tempBYTE);
  }
  if (EEPROM.get(EEPROMAddresses::CURR_OFFSET, tempDOUBLE) != calValues.emonCurrOffset)
  {
    Serial.print("Current Offset: ");
    Serial.println(tempDOUBLE);
  }
  if (EEPROM.get(EEPROMAddresses::SSR_ARMED, tempBOOL) != sensorValues.ssrArmed)
  {
    Serial.print("SSR Armed: ");
    Serial.println(tempBOOL);
  }
  if (EEPROM.get(EEPROMAddresses::PID1_MODE, tempBOOL) != pid1.mode)
  {
    Serial.print("PID1 Mode: ");
    Serial.println(tempBOOL);
  }
  if (EEPROM.get(EEPROMAddresses::PID2_MODE, tempBOOL) != pid2.mode) {
    Serial.print("PID2 Mode: ");
    Serial.println(tempBOOL);
  }
  if (EEPROM.get(EEPROMAddresses::PID3_MODE, tempBOOL) != pid3.mode) {
    Serial.print("PID3 Mode: ");
    Serial.println(tempBOOL);
  }
  if (EEPROM.get(EEPROMAddresses::PID2_ADAPTIVE_MODE, tempBOOL) != pid2.adaptiveMode)
  {
    Serial.print("PID2 Adaptive Mode: ");
    Serial.println(tempBOOL);
  }
  if (EEPROM.get(EEPROMAddresses::PID1_ADAPTIVE_MODE, tempBOOL) != pid1.adaptiveMode)
  {
    Serial.print("PID1 Adaptive Mode: ");
    Serial.println(tempBOOL);
  }
  if (EEPROM.get(EEPROMAddresses::S_DEBUG, tempBOOL) != configValues.sDebug)
  {
    Serial.print("S Debug: ");
    Serial.println(tempBOOL);
  }
  if (EEPROM.get(EEPROMAddresses::PID1_ALARM_THRESHOLD, tempINT) != pid1.alarmThreshold)
  {
    Serial.print("PID1 Alarm Threshold: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::PID2_ALARM_THRESHOLD, tempINT) != pid2.alarmThreshold)
  {
    Serial.print("PID2 Alarm Threshold: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::PID3_ALARM_THRESHOLD, tempINT) != pid3.alarmThreshold)
  {
    Serial.print("PID3 Alarm Threshold: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::PRESSURE2_OFFSET, tempINT) != calValues.pressure2Offset)
  {
    Serial.print("Pressure2 Offset: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::PRESSURE2_CAL, tempFLOAT) != calValues.pressure2Cal)
  {
    Serial.print("Pressure2 Cal: ");
    Serial.println(tempFLOAT);
  }
  if (EEPROM.get(EEPROMAddresses::PRESSURE3_OFFSET, tempINT) != calValues.pressure3Offset)
  {
    Serial.print("Pressure3 Offset: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::PRESSURE3_CAL, tempFLOAT) != calValues.pressure3Cal) {
    Serial.print("Pressure3 Cal: ");
    Serial.println(tempFLOAT);
  }
  if (EEPROM.get(EEPROMAddresses::PRESSURE4_OFFSET, tempINT) != calValues.pressure4Offset) {
    Serial.print("Pressure4 Offset: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::PRESSURE4_CAL, tempFLOAT) != calValues.pressure4Cal)
  {
    Serial.print("Pressure4 Cal: ");
    Serial.println(tempFLOAT);
  }
  if (EEPROM.get(EEPROMAddresses::VCC_CURRENT_OFFSET, tempINT) != calValues.VccCurrentOffset)
  {
    Serial.print("Vcc Current Offset: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::VCC_CURRENT_MULTIPLIER, tempFLOAT) != calValues.VccCurrentMultiplier)
  {
    Serial.print("Vcc Current Multiplier: ");
    Serial.println(tempFLOAT);
  }
  if (EEPROM.get(EEPROMAddresses::OLED_line1_SENSORID_ADDR, tempINT) != sensorValues.OLED_line1_SENSORID)
  {
    Serial.print("OLED Line1 SensorID: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::OLED_line2_SENSORID_ADDR, tempINT) != sensorValues.OLED_line2_SENSORID)
  {
    Serial.print("OLED Line2 SensorID: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::PID1_SENSORID_ADDR, tempINT) != sensorValues.PID1_SENSORID_VAR)
  {
    Serial.print("PID1 SensorID: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::PID2_SENSORID_ADDR, tempINT) != sensorValues.PID2_SENSORID_VAR)
  {
    Serial.print("PID2 SensorID: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::PID3_SENSORID_ADDR, tempINT) != sensorValues.PID3_SENSORID_VAR)
  {
    Serial.print("PID3 SensorID: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::OLED_line3_SENSORID_ADDR, tempINT) != sensorValues.OLED_line3_SENSORID)
  {
    Serial.print("OLED Line3 SensorID: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::OLED_line4_SENSORID_ADDR, tempINT) != sensorValues.OLED_line4_SENSORID)
  {
    Serial.print("OLED Line4 SensorID: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::OLED_line5_SENSORID_ADDR, tempINT) != sensorValues.OLED_line5_SENSORID)
  {
    Serial.print("OLED Line5 SensorID: ");
    Serial.println(tempINT);
  }
  if (EEPROM.get(EEPROMAddresses::OLED_line6_SENSORID_ADDR, tempINT) != sensorValues.OLED_line6_SENSORID)
  {
    Serial.print("OLED Line6 SensorID: ");
    Serial.println(tempINT);
  }
  Serial.println("End Sanity Check EEPROM");
}

float getThermistor(const int pinVar)
{
  float thermistorResistance = voltageDivider(pinVar, 981.0);
  // https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
  const float A = 1.680995265e-3;
  const float B = 2.392149905e-4;
  const float C = 1.594237047e-7;

  float logR = log(thermistorResistance);
  float Kelvin = 1.0 / (A + B * logR + C * logR * logR * logR);
  float tempF = (Kelvin - 273.15) * CELSIUS_TO_FAHRENHEIT_FACTOR + CELSIUS_TO_FAHRENHEIT_OFFSET;
  Serial.print("Thermistor ");
  Serial.print(pinVar);
  Serial.print(" Resistance: ");
  Serial.print(thermistorResistance);
  Serial.print(" Temp: ");
  Serial.println(tempF);
  return tempF;
}

void displayKnightRider() {
  static int position = 0;
  static int direction = 1;

  display.drawLine(0, SCREEN_HEIGHT - 4, SCREEN_WIDTH, SCREEN_HEIGHT - 4, SSD1306_BLACK); // Clear previous line
  display.drawLine(position, SCREEN_HEIGHT - 4, position + 4, SCREEN_HEIGHT - 4, SSD1306_WHITE); // Draw new line

  position += direction;
  if (position <= 0 || position >= SCREEN_WIDTH - 4) {
    direction = -direction; // Change direction
  }
}
