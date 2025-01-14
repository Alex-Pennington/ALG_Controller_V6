#include <Arduino.h>
#include "HX711.h"
#include <PID_v1.h>
#include <EEPROM.h>
#include "controller.h"
#include <OneWire.h>

#define VERSION "0.6.0.1"
#define SENDDELAY 50
#define MY_NODE_ID 201
#define MY_RADIO_RF24
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_RF24_CE_PIN 49
#define MY_RF24_CS_PIN 53
//#define MY_DEBUG
#include <MySensors.h>

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
  int scale_stabilisingtime = 10000;
  bool toACK = false;
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
PIDConfig pid1 = {false, 150, 0.5, 0.005, 0, 1, 0, 0, 10, false, 0, 0, 0};
PIDConfig pid2 = {false, 150, 0.35, 0, 0, 5, 0, 0, 10, false, 0, 0, 0};
PIDConfig pid3 = {false, 120, 1.2, 0.01, 0, 0, 0, 0, 10, false, 0, 0, 0};

struct CalibrationValues {
    float pressure1Cal = 20.77;
    int pressure1Offset = 425;
    float pressure2Cal = 20.77;
    int pressure2Offset = 425;
    float emonCal = 0.128;
    byte ssrFailThreshold = 2;
    double currOffset = 5.28;
    float zeroOffsetScale = 403361;
    float scaleCal = 53200;
    byte aScale =  255; // times is a parameter of type byte that specifies the number of times the raw value should be read and averaged.
    int mScale  = 1000; //scale multiplier
    int rScale =  1000; //rate scalar coefficient
};
CalibrationValues calValues;

struct SteinhartConfig {
    const long seriesResistor = 100000;
    const long nominalResistance = 100000;
    const int nominalTemperature = 25;
    const int bCoefficient = 3950;
};
SteinhartConfig thermistorConfig;

struct EmonVars {
    unsigned long sampleTime = 0;
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

//Working Variables
static unsigned long SensorLoop_timer = 0;
static float dC = 0.0;
static float dC2 = 0.0;
static float dC3 = 0.0;
static float valueScale = 0;
static float value_oldScale = 0;
static unsigned long oldtimeScale = 0;
static float kgpsScale = 0; //rate of change of scale in kilograms per second
static long AREF_V = 0;
static bool ssrArmed = false;
float pressure1Var = 0;
float pressure2Var = 0;


//Pin Definitions
#define HX711_dout 9 //
#define HX711_sck 10 //
#define ElementPowerPin3 25
#define SSRArmed_PIN 29
#define speakerPin 39 //
#define ElementPowerPin 41
#define ElementPowerPin2 43
#define DS18B20_PIN 44 //
#define SteinhartEnable 22
#define SteinhartPin A3 //
#define Pressure1PIN A15 //
#define Pressure2PIN A15 //
#define emon_Input_PIN A9
#define Thermistor1PIN A1 //
#define Thermistor2PIN A2 //

// MySensors Child IDs
enum CHILD_ID {
  T0 = 0,
  T1 = 1,
  T2 = 2,
  HUM = 3,
  Duty = 4,
  Scale = 5,
  Duty2 = 6,
  T3 = 7,
  SSR = 9,
  LOAD_MEMORY = 10,
  T4 = 11,
  P1 = 12,
  P1Cal = 13,
  ScaleRate = 14,
  ScaleTare = 15,
  ScaleOffset = 16,
  ScaleRtAccu = 17,
  ScaleUntMag = 18,
  ScaleAvg = 19,
  PIDMODE_1 = 20,
  PIDSETPOINT_1 = 21,
  AdaptiveSP_1 = 22,
  PIDkP0_1 = 23,
  PIDkI0_1 = 24,
  PIDkD0_1 = 25,
  PIDkP1_1 = 26,
  PIDkI1_1 = 27,
  PIDkD1_1 = 28,
  AdaptiveMode_1 = 29,
  EDC = 31,
  Info = 32,
  ScaleCal = 40,
  PIDMODE_2 = 60,
  PIDSETPOINT_2 = 61,
  AdaptiveSP_2 = 62,
  PIDkP0_2 = 63,
  PIDkI0_2 = 64,
  PIDkD0_2 = 65,
  PIDkP1_2 = 66,
  PIDkI1_2 = 67,
  PIDkD1_2 = 68,
  AdaptiveMode_2 = 69,
  EDC_2 = 71,
  Press1Offset = 73,
  RMS = 74,
  Curr_CAL = 75,
  SSRFail_Threshhold = 76,
  SSRFail_Alarm = 77,
  curr_OFFSET = 78,
  LCD_VAR1 = 1,
  LCD_VAR2 = 2,
  LCD_PID1 = 9,
  LCD_PID2 = 10,
  LCD_Condenser = 11,
  LCD_NODE_ID = 10,
  LCD_INFO = 32,
  LCD_Platter = 33,
  LCD_PID3 = 34,
  PIDMODE_3 = 80,
  PIDSETPOINT_3 = 81,
  Duty3 = 82,
  PIDkP0_3 = 83,
  PIDkI0_3 = 84,
  PIDkD0_3 = 85,
  EDC_3 = 91,
  s_debug = 92,
  BoardVoltage = 93,
  PID1_Threshold = 94,
  PID2_Threshold = 95,
  PID3_Threshold = 96
};

//MySensors Message Definitions
MyMessage msgPIDMODE(CHILD_ID::PIDMODE_1, V_STATUS);
MyMessage msgPIDSETPOINT(CHILD_ID::PIDSETPOINT_1, V_TEMP);
MyMessage msgKp(CHILD_ID::PIDkP0_1, V_LEVEL);
MyMessage msgKi(CHILD_ID::PIDkI0_1, V_LEVEL);
MyMessage msgKd(CHILD_ID::PIDkD0_1, V_LEVEL);
MyMessage msgEDC(CHILD_ID::EDC, V_PERCENTAGE);
MyMessage msgPIDMODE_2(CHILD_ID::PIDMODE_2, V_STATUS);
MyMessage msgPIDSETPOINT_2(CHILD_ID::PIDSETPOINT_2, V_TEMP);
MyMessage msgKp_2(CHILD_ID::PIDkP0_2, V_LEVEL);
MyMessage msgKi_2(CHILD_ID::PIDkI0_2, V_LEVEL);
MyMessage msgKd_2(CHILD_ID::PIDkD0_2, V_LEVEL);
MyMessage msgEDC_2(CHILD_ID::EDC_2, V_PERCENTAGE);
MyMessage msgPIDMODE_3(CHILD_ID::PIDMODE_3, V_STATUS);
MyMessage msgPIDSETPOINT_3(CHILD_ID::PIDSETPOINT_3, V_TEMP);
MyMessage msgKp_3(CHILD_ID::PIDkP0_3, V_LEVEL);
MyMessage msgKi_3(CHILD_ID::PIDkI0_3, V_LEVEL);
MyMessage msgKd_3(CHILD_ID::PIDkD0_3, V_LEVEL);
MyMessage msgEDC_3(CHILD_ID::EDC_3, V_PERCENTAGE);
MyMessage msgINFO(CHILD_ID::Info, V_TEXT);
MyMessage msgTemp0(CHILD_ID::T0, V_TEMP);
MyMessage msgTemp1(CHILD_ID::T1, V_TEMP);
MyMessage msgTemp2(CHILD_ID::T2, V_TEMP);
MyMessage msgHumidity(CHILD_ID::HUM, V_HUM);
MyMessage msgScale(CHILD_ID::Scale, V_WEIGHT);
MyMessage msgTemp3(CHILD_ID::T3, V_TEMP);
MyMessage msgTemp4(CHILD_ID::T4, V_TEMP);
//MyMessage msgDBG(CHILD_ID::DBG, V_TEXT);
MyMessage msgSSR(CHILD_ID::SSR, V_STATUS);
MyMessage msgSSR2(CHILD_ID::LOAD_MEMORY, V_STATUS);
MyMessage msgLCD1(CHILD_ID::LCD_VAR1, V_TEXT);
MyMessage msgLCD2(CHILD_ID::LCD_VAR2, V_TEXT);
MyMessage msgLCD_Platter(CHILD_ID::LCD_Platter, V_TEXT);
MyMessage msgPID1(CHILD_ID::LCD_PID1, V_STATUS);
MyMessage msgPID2(CHILD_ID::LCD_PID2, V_STATUS);
MyMessage msgPID3(CHILD_ID::LCD_PID3, V_STATUS);
MyMessage msgLCDScale(3, V_WEIGHT);
MyMessage msgLCDPressure(17, V_PRESSURE);
MyMessage msgLCDINFO(CHILD_ID::LCD_INFO, V_TEXT);
MyMessage msgLCDCondenser(CHILD_ID::LCD_Condenser, V_TEXT);
MyMessage msgPressure1(CHILD_ID::P1, V_PRESSURE);
MyMessage msgPressCal1(CHILD_ID::P1Cal, V_LEVEL);
MyMessage msgScaleRate(CHILD_ID::ScaleRate, V_WEIGHT);
MyMessage msgScaleTare(CHILD_ID::ScaleTare, V_STATUS);
MyMessage msgScaleUntMag(CHILD_ID::ScaleUntMag, V_STATUS);
MyMessage msgScaleRtAccu(CHILD_ID::ScaleRtAccu, V_STATUS);
MyMessage msgScaleOffset(CHILD_ID::ScaleOffset, V_LEVEL);
MyMessage msgPress1Offset(CHILD_ID::Press1Offset, V_LEVEL);
MyMessage msgRMS(CHILD_ID::RMS, V_LEVEL);
MyMessage msgCurCAL(CHILD_ID::Curr_CAL, V_LEVEL);
MyMessage msgSSRFailAlarm(CHILD_ID::SSRFail_Alarm, V_STATUS);
MyMessage msgBoardVoltage(CHILD_ID::BoardVoltage, V_LEVEL);

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
void sendSensors();
float Steinhart();
void AllStop();
void StoreEEPROM();
void getEEPROM();
void FactoryResetEEPROM();
void printConfig();
long getBandgap(void);
void receive(const MyMessage & message);
void sendInfo(String);
void DS18B20();
float getThermistor(int);
float readPressure(int pin, int offset, float cal);

void presentation()
{
  //Send the sensor node sketch version information to the gateway
  sendSketchInfo("Controller", VERSION);

  Serial.begin(115200);

  if (configValues.sDebug) {Serial.println("Presenting Sensors");}
  present(CHILD_ID::PIDMODE_1, S_BINARY, "PID1 Lower");
  present(CHILD_ID::PIDSETPOINT_1, S_TEMP, "PID1 Setpoint");
  present(CHILD_ID::PIDkP0_1, S_LIGHT_LEVEL, "constKp");
  present(CHILD_ID::PIDkI0_1, S_LIGHT_LEVEL, "constKi");
  present(CHILD_ID::PIDkD0_1, S_LIGHT_LEVEL, "constKd");
  present(CHILD_ID::PIDkP1_1, S_LIGHT_LEVEL, "aggKp");
  present(CHILD_ID::PIDkI1_1, S_LIGHT_LEVEL, "aggKi");
  present(CHILD_ID::PIDkD1_1, S_LIGHT_LEVEL, "aggKd");
  present(CHILD_ID::AdaptiveMode_1, S_BINARY, "Adaptive");
  present(CHILD_ID::AdaptiveSP_1, S_TEMP, "Adaptive SP");
  present(CHILD_ID::EDC, S_DIMMER, "L dC");
  present(CHILD_ID::PIDMODE_2, S_BINARY, "PID2 Upper");
  present(CHILD_ID::PIDSETPOINT_2, S_TEMP, "PID2 Setpoint");
  present(CHILD_ID::PIDkP0_2, S_LIGHT_LEVEL, "constKp_2");
  present(CHILD_ID::PIDkI0_2, S_LIGHT_LEVEL, "constKi_2");
  present(CHILD_ID::PIDkD0_2, S_LIGHT_LEVEL, "constKd_2");
  present(CHILD_ID::PIDkP1_2, S_LIGHT_LEVEL, "aggKp_2");
  present(CHILD_ID::PIDkI1_2, S_LIGHT_LEVEL, "aggKi_2");
  present(CHILD_ID::PIDkD1_2, S_LIGHT_LEVEL, "aggKd_2");
  present(CHILD_ID::AdaptiveMode_2, S_BINARY, "Adaptive_2");
  present(CHILD_ID::AdaptiveSP_2, S_TEMP, "Adaptive SP_2");
  present(CHILD_ID::EDC_2, S_DIMMER, "U dC");
  present(CHILD_ID::PIDMODE_3, S_BINARY, "PID3 Platter");
  present(CHILD_ID::PIDSETPOINT_3, S_TEMP, "PID3 Setpoint");
  present(CHILD_ID::PIDkP0_3, S_LIGHT_LEVEL, "constKp_3");
  present(CHILD_ID::PIDkI0_3, S_LIGHT_LEVEL, "constKi_3");
  present(CHILD_ID::PIDkD0_3, S_LIGHT_LEVEL, "constKd_3");
  present(CHILD_ID::EDC_3, S_DIMMER, "P dC");
  present(CHILD_ID::Info, S_INFO, "Debug Info");
  present(CHILD_ID::T0, S_TEMP, "Condenser T0");
  present(CHILD_ID::T1, S_TEMP, "Upper Spool T1");
  present(CHILD_ID::T2, S_TEMP, "Amb Temp");
  present(CHILD_ID::T3, S_TEMP, "Platter");
  present(CHILD_ID::T4, S_TEMP, "Lower Spool T2");
  present(CHILD_ID::HUM, S_HUM, "Amb Hum");
  present(CHILD_ID::Duty, S_DIMMER, "Duty Cycle");
  present(CHILD_ID::Scale, S_WEIGHT, "Ref Scale");
  present(CHILD_ID::Duty2, S_DIMMER, "Duty Cycle2");
  present(CHILD_ID::Duty3, S_DIMMER, "Duty Cycle3");
  present(CHILD_ID::SSR, S_BINARY, "SSR Armed");
  present(CHILD_ID::LOAD_MEMORY, S_BINARY, "Load Memory");
  present(CHILD_ID::ScaleCal, S_LIGHT_LEVEL, "Scale Cal");
  present(CHILD_ID::P1, S_BARO, "Pressure 1");
  present(CHILD_ID::P1Cal, S_LIGHT_LEVEL, "PresCal 1");
  present(CHILD_ID::ScaleRate, S_WEIGHT, "Scale Delta/T");
  present(CHILD_ID::ScaleTare, S_BINARY, "Tare Scale");
  present(CHILD_ID::ScaleUntMag, S_BINARY, "Scale g/Kg");
  present(CHILD_ID::ScaleRtAccu, S_BINARY, "Rate g/Kg");
  present(CHILD_ID::ScaleOffset, S_LIGHT_LEVEL, "Scale Zero Offset");
  present(CHILD_ID::ScaleAvg, S_BINARY, "Scale Samples");
  present(CHILD_ID::Press1Offset, S_LIGHT_LEVEL, "P1 Zero Offset");
  present(CHILD_ID::RMS, S_LIGHT_LEVEL, "RMS Power");
  present(CHILD_ID::Curr_CAL, S_LIGHT_LEVEL, "Cur CAL");
  present(CHILD_ID::SSRFail_Threshhold, S_LIGHT_LEVEL, "SSRFail Threshhold");
  present(CHILD_ID::SSRFail_Alarm, S_BINARY, "SSR FAIL ALARM");
  present(CHILD_ID::curr_OFFSET, S_LIGHT_LEVEL, "Current Offset");
  present(CHILD_ID::s_debug, S_BINARY, "Serial3 Debug");
  present(CHILD_ID::BoardVoltage, S_LIGHT_LEVEL, "Board Voltage");
  present(CHILD_ID::PID1_Threshold, S_TEMP, "PID1A");
  present(CHILD_ID::PID2_Threshold, S_TEMP, "PID2A");
  present(CHILD_ID::PID3_Threshold, S_TEMP, "PID3A");

  if (configValues.sDebug) {
    Serial.println("Presentation Complete");
  }
}

void setup() {
  getEEPROM();
  LoadCell.begin(HX711_dout, HX711_sck);
  pinMode(Thermistor1PIN, INPUT);
  pinMode(Thermistor2PIN, INPUT);
  pinMode(ElementPowerPin3, OUTPUT);
  pinMode(SSRArmed_PIN, OUTPUT);
  pinMode(ElementPowerPin, OUTPUT);
  pinMode(ElementPowerPin2, OUTPUT);
  pinMode(speakerPin, OUTPUT);
  pinMode(SteinhartEnable, OUTPUT);
  pinMode(SteinhartPin, INPUT);
  pinMode(Pressure1PIN, INPUT);
  pinMode(Pressure2PIN, INPUT);
  pinMode(emon_Input_PIN, INPUT);

}

void loop() {
  
  DutyCycleLoop();

  AREF_V = getBandgap();

  if ( (millis() - SensorLoop_timer) > (unsigned long)configValues.SENSORLOOPTIME)  {
    DS18B20();
    TempAlarm();
    emon();
    getScale();
    pressure1Var = readPressure(Pressure1PIN, calValues.pressure1Offset, calValues.pressure1Cal);
    if (configValues.sDebug) { calValues.pressure2Offset = calValues.pressure1Offset; calValues.pressure2Cal = calValues.pressure1Cal;} //remove this line when sensor is installed
    pressure2Var = readPressure(Pressure2PIN, calValues.pressure2Offset, calValues.pressure2Cal);

    sendSensors();
    SensorLoop_timer = millis();
  }
}

void sendSensors()
{
  // Send Sensors
  send(msgTemp0.set(ds18b20Values[0].F, 1), configValues.toACK);
  wait(SENDDELAY);
  send(msgTemp1.set(ds18b20Values[1].F, 1), configValues.toACK);
  wait(SENDDELAY);
  wait(SENDDELAY);
  send(msgTemp3.set(steinhartValues.steinhart, 1), configValues.toACK);
  wait(SENDDELAY);
  send(msgTemp4.set(ds18b20Values[2].F, 1), configValues.toACK);
  wait(SENDDELAY);
  wait(SENDDELAY);
  send(msgScale.set(valueScale, 1), configValues.toACK);
  wait(SENDDELAY);
  send(msgRMS.set(emonVars.rms, 1), configValues.toACK);
  wait(SENDDELAY);
  send(msgSSR.set(ssrArmed), configValues.toACK);
  wait(SENDDELAY);
  send(msgEDC.set(pid1.output, 2), configValues.toACK);
  wait(SENDDELAY);
  send(msgEDC_2.set(pid2.output, 2), configValues.toACK);
  wait(SENDDELAY);
  send(msgEDC_3.set(pid3.output, 2), configValues.toACK);
  wait(SENDDELAY);
  send(msgBoardVoltage.set((float)AREF_V, 2), configValues.toACK);
  wait(SENDDELAY);
  send(msgPressure1.set(pressure2Var, 2), configValues.toACK);
  wait(SENDDELAY);
  // send(msgPressure2.set(pressure2Var, 2), configValues.toACK);
  // wait(SENDDELAY);
  send(msgScaleRate.set(kgpsScale, 2), configValues.toACK);
  wait(SENDDELAY);

  if (configValues.sDebug)
  {
    Serial.println("----------------------------------------");
    Serial.print("Sensor Data Time (ms) ");
    Serial.println(millis());
    Serial.print("Condenser T0: ");
    Serial.println(ds18b20Values[0].F);
    Serial.print("Upper Spool T1: ");
    Serial.println(ds18b20Values[1].F);
    Serial.print("lower Spool T2: ");
    Serial.println(ds18b20Values[2].F);
    Serial.print("Platter Temp: ");
    Serial.println(Steinhart());
    Serial.print("Thermistor 1 Temp: ");
    Serial.println(getThermistor(Thermistor1PIN));
    Serial.print("Thermistor 2 Temp: ");
    Serial.println(getThermistor(Thermistor2PIN));
    Serial.print("Scale : ");
    Serial.println(valueScale);
    Serial.print("Pressure 1: ");
    Serial.println(pressure1Var);
    Serial.print("Pressure 2: ");
    Serial.println(pressure2Var);
    Serial.print("Current : ");
    Serial.println(emonVars.rms);
    Serial.print("MEGA Voltage : ");
    Serial.println(AREF_V);
  }
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

void emon()
{
  // SSRFail-Emon
  digitalWrite(ElementPowerPin, HIGH);
  digitalWrite(ElementPowerPin2, HIGH);
  digitalWrite(ElementPowerPin3, HIGH);
  float sum = 0;

  emonVars.sampleTime = millis();

  int N = 1000;
  for (int i = 0; i < N; i++)
  {
    float current = calValues.emonCal * (analogRead(emon_Input_PIN) - 512); // in amps I presume
    sum += current * current;                                               // sum squares
    wait(1);
  }
  emonVars.rms = sqrt(sum / N) - calValues.currOffset;
  if (int(emonVars.rms) > int(calValues.ssrFailThreshold))
  {
    red_alert();
    AllStop();
    sendInfo("SSR Failed");
  }
  else
  {
    digitalWrite(ElementPowerPin, dutyCycle[0].element);
    digitalWrite(ElementPowerPin2, dutyCycle[1].element);
    digitalWrite(ElementPowerPin3, dutyCycle[2].element);
    //      digitalWrite(SSRArmed_PIN, SSRArmed);
  }
  for (int i = 0; i < N; i++)
  {
    float current = calValues.emonCal * (analogRead(emon_Input_PIN) - 512); // in amps I presume
    sum += current * current;                                               // sum squares
    wait(1);
  }  emonVars.rms = sqrt(sum / N) - calValues.currOffset;
}

void getScale()
{
  // Scale
  value_oldScale = valueScale;
  if (LoadCell.is_ready())
  {
    valueScale = LoadCell.get_units(calValues.aScale) * calValues.mScale;
  }
  else
  {
    Serial.println("Load cell not ready");
    valueScale = value_oldScale; // or handle the error as needed
  }
  if (valueScale < 0)
  {
    valueScale = 0;
  }
  kgpsScale = calValues.rScale * ((valueScale - value_oldScale) / ((millis() - oldtimeScale) / 1000));
  oldtimeScale = millis();
}

float readPressure(int pin, int offset, float cal) {
    int RawADCavg = 0;
    int i = 0;
    for (i = 0; i < 10; i++) {
      wait(10);
      RawADCavg += analogRead(Pressure1PIN);
    }
    float avgADC1 = (float)RawADCavg / 10.0;
    float offsetCorrected1 = avgADC1 - (float)offset;
    return offsetCorrected1 * (1.0 / cal);
}

float Steinhart() {
  digitalWrite(SteinhartEnable, HIGH);
  for (int n = 0; n < 10; n++)
  {
    wait(10); //this increased resolution signifigantly
    steinhartValues.adcValue += analogRead(SteinhartPin);
  }
  digitalWrite(SteinhartEnable, LOW);
  steinhartValues.adcValue /= 10;

  steinhartValues.resistance = ((thermistorConfig.seriesResistor + thermistorConfig.nominalResistance) * (1 / (1023 / steinhartValues.adcValue - 1)));
  steinhartValues.steinhart =  steinhartValues.resistance / thermistorConfig.nominalResistance; // (R/Ro)
  steinhartValues.steinhart = log(steinhartValues.steinhart); // ln(R/Ro)
  steinhartValues.steinhart /= thermistorConfig.bCoefficient; // 1/B * ln(R/Ro)
  steinhartValues.steinhart += 1.0 / (thermistorConfig.nominalTemperature + 273.15); // + (1/To)
  steinhartValues.steinhart = 1.0 / steinhartValues.steinhart; // Invert
  steinhartValues.steinhart -= 273.15; // convert to C
  steinhartValues.steinhart = (((steinhartValues.steinhart * 9 ) / 5 ) + 32);
  return steinhartValues.steinhart;
}

float getThermistor(const int pinVar) {
  int ADCvalue = 0;
        const float seriesResistor = 981.0;
        for (int n = 0; n < 10; n++){
            delayMicroseconds(10); //this increased resolution signifigantly
            ADCvalue += analogRead(pinVar);
        }
        ADCvalue /= 10;
        float Va3 = (1.0 - ((float)ADCvalue / 1023.0)) * AREF_V;
        float thermistorResistance = (seriesResistor * Va3) / (AREF_V - Va3);
        //https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
        const float A = 1.499168475e-3;
        const float B = 2.766247366e-4;
        const float C = 0.2413822162e-7;

        float logR = log(thermistorResistance);
        float Kelvin = 1.0 / (A + B * logR + C * logR * logR * logR);
        float tempF = (Kelvin - 273.15) * (9.0/5.0) + 32.0;
        /* Debug Values
        Serial.print("ADCvalue ");
        Serial.println(ADCvalue);
        Serial.print("Va3 ");
        Serial.println(Va3);
        Serial.print("thermistorResistance ");
        Serial.println(thermistorResistance);
        Serial.print("A ");
        Serial.println((double)A,10);
        Serial.print("B ");
        Serial.println((double)B,10);
        Serial.print("C ");
        Serial.println((double)C,10);
        Serial.print("logR ");
        Serial.println(logR);
        Serial.print("Kelvin ");
        Serial.println(((double)Kelvin),10);
        Serial.print("tempF ");
        Serial.println((double)tempF,10);
        Serial.println("------------------------");
        */
        return tempF;
}

void DutyCycleLoop() {
  for (int i = 0; i < 3; i++) {
    if ((dutyCycle[i].loopTime > 0 && ssrArmed == true) && (i == 0 ? pid1.mode : (i == 1 ? pid2.mode : pid3.mode))) {
      if (!dutyCycle[i].element) {
        if ((millis() - dutyCycle[i].onTime) > (static_cast<unsigned long>(dutyCycle[i].loopTime) * 1000)) {
          dutyCycle[i].offTime = millis();
          dutyCycle[i].onTime = 0;
          digitalWrite(i == 0 ? ElementPowerPin : (i == 1 ? ElementPowerPin2 : ElementPowerPin3), HIGH);
          dutyCycle[i].element = HIGH;
        }
      } else {
        if ((millis() - dutyCycle[i].offTime) > ((1U - static_cast<unsigned long>(dutyCycle[i].loopTime)) * 1000)) {
          dutyCycle[i].onTime = millis();
          dutyCycle[i].offTime = 0;
          digitalWrite(i == 0 ? ElementPowerPin : (i == 1 ? ElementPowerPin2 : ElementPowerPin3), LOW);
          dutyCycle[i].element = LOW;
        }
      }
    } else {
      digitalWrite(i == 0 ? ElementPowerPin : (i == 1 ? ElementPowerPin2 : ElementPowerPin3), HIGH);
      dutyCycle[i].onTime = 0;
      dutyCycle[i].element = HIGH;
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
  send(msgPIDMODE.set(false), configValues.toACK);
  wait(SENDDELAY);
  send(msgPIDMODE_2.set(false), configValues.toACK);
  wait(SENDDELAY);
  send(msgPIDMODE_3.set(false), configValues.toACK);
  wait(SENDDELAY);
  send(msgSSRFailAlarm.set(true), configValues.toACK);
  wait(SENDDELAY);
  send(msgSSR.set(false), configValues.toACK);
}

void StoreEEPROM() {
  EEPROM.put(0, calValues.zeroOffsetScale);
  EEPROM.put(4, calValues.pressure1Offset);
  EEPROM.put(6, calValues.scaleCal);
  EEPROM.put(10, calValues.pressure1Cal);
  EEPROM.put(14, pid1.setpoint);
  EEPROM.put(18, pid1.kp);
  EEPROM.put(22, pid1.ki);
  EEPROM.put(26, pid1.kd);
  EEPROM.put(30, pid1.aggKp);
  EEPROM.put(34, pid1.aggKi);
  EEPROM.put(38, pid1.aggKd);
  EEPROM.put(42, pid2.setpoint);
  EEPROM.put(46, pid2.kp);
  EEPROM.put(50, pid2.ki);
  EEPROM.put(54, pid2.kd);
  EEPROM.put(58, pid2.aggKp);
  EEPROM.put(62, pid2.aggKi);
  EEPROM.put(66, pid2.aggKd);
  EEPROM.put(70, pid1.aggSP);
  EEPROM.put(71, pid2.aggSP);
  EEPROM.put(72, pid3.setpoint);
  EEPROM.put(76, pid3.kp);
  EEPROM.put(80, pid3.ki);
  EEPROM.put(84, pid3.kd);
  EEPROM.put(88, calValues.emonCal);
  EEPROM.put(92, calValues.ssrFailThreshold);
  EEPROM.put(96, calValues.currOffset);
  EEPROM.put(100, ssrArmed);
  EEPROM.put(101, pid1.mode);
  EEPROM.put(102, pid2.mode);
  EEPROM.put(103, pid3.mode);
  EEPROM.put(104, pid2.adaptiveMode);
  EEPROM.put(105, pid1.adaptiveMode);
  EEPROM.put(106, configValues.sDebug);
  EEPROM.put(107, pid1.alarmThreshold);
  EEPROM.put(109, pid2.alarmThreshold);
  EEPROM.put(111, pid3.alarmThreshold);
  EEPROM.put(114, pid3.alarmThreshold);
  EEPROM.put(116, calValues.pressure2Offset);
  EEPROM.put(118, calValues.pressure2Cal);



  printConfig();
}

void getEEPROM() {
  EEPROM.get(0, calValues.zeroOffsetScale);
  LoadCell.set_offset(calValues.zeroOffsetScale);
  EEPROM.get(4, calValues.pressure1Offset);
  EEPROM.get(6, calValues.scaleCal);
  EEPROM.get(10, calValues.pressure1Cal);
  EEPROM.get(14, pid1.setpoint);
  EEPROM.get(18, pid1.kp);
  EEPROM.get(22, pid1.ki);
  EEPROM.get(26, pid1.kd);
  EEPROM.get(30, pid1.aggKp);
  EEPROM.get(34, pid1.aggKi);
  EEPROM.get(38, pid1.aggKd);
  EEPROM.get(42, pid2.setpoint);
  EEPROM.get(46, pid2.kp);
  EEPROM.get(50, pid2.ki);
  EEPROM.get(54, pid2.kd);
  EEPROM.get(58, pid2.aggKp);
  EEPROM.get(62, pid2.aggKi);
  EEPROM.get(66, pid2.aggKd);
  EEPROM.get(70, pid1.aggSP);
  EEPROM.get(71, pid2.aggSP);
  EEPROM.get(72, pid3.setpoint);
  EEPROM.get(76, pid3.kp);
  EEPROM.get(80, pid3.ki);
  EEPROM.get(84, pid3.kd);
  EEPROM.get(88, calValues.emonCal);
  EEPROM.get(92, calValues.ssrFailThreshold);
  EEPROM.get(96, calValues.currOffset);
  EEPROM.get(100, ssrArmed);
  EEPROM.get(101, pid1.mode);
  EEPROM.get(102, pid2.mode);
  EEPROM.get(103, pid3.mode);
  EEPROM.get(104, pid2.adaptiveMode);
  EEPROM.get(105, pid1.adaptiveMode);
  EEPROM.get(106, configValues.sDebug);
  EEPROM.get(107, pid1.alarmThreshold);
  EEPROM.get(109, pid2.alarmThreshold);
  EEPROM.get(111, pid3.alarmThreshold);
  EEPROM.get(114, calValues.pressure2Offset);
  EEPROM.get(116, calValues.pressure2Cal);
  printConfig();
}

void FactoryResetEEPROM() {
  // Reset Calibration Values
  calValues.pressure1Cal = 20.77;
  calValues.pressure1Offset = 425;
  calValues.emonCal = 0.128;
  calValues.ssrFailThreshold = 2;
  calValues.currOffset = 5.28;
  calValues.zeroOffsetScale = 403361;
  calValues.scaleCal = 53200;
  calValues.aScale = 255;
  calValues.mScale = 1000; // multiplier
  calValues.rScale = 1000; // rate multiplier
  calValues.pressure2Cal = 1;
  calValues.pressure2Offset = 0;

  // Reset PID Configurations
  pid1.mode = false;
  pid1.setpoint = 150;
  pid1.kp = 0.500;
  pid1.ki = 0.005;
  pid1.kd = 0;
  pid1.aggKp = 1;
  pid1.aggKi = 0;
  pid1.aggKd = 0;
  pid1.aggSP = 10;
  pid1.adaptiveMode = false;
  pid1.alarmThreshold = 190;

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

  pid3.mode = false;
  pid3.setpoint = 120;
  pid3.kp = 1.2;
  pid3.ki = 0.01;
  pid3.kd = 0;
  pid3.alarmThreshold = 190;

  // Reset other configurations
  ssrArmed = false;
  configValues.sDebug = true;
  StoreEEPROM();
  printConfig();
}

void printConfig() {
  Serial.print("zeroOffsetScale: ");
  Serial.println(calValues.zeroOffsetScale);
  Serial.print("pressure1Offset: ");
  Serial.println(calValues.pressure1Offset);
  Serial.print("pressure1Cal: ");
  Serial.println(calValues.pressure1Cal);
  Serial.print("pressure2Offset: ");
  Serial.println(calValues.pressure2Offset);
    Serial.print("pressure2Cal: ");
  Serial.println(calValues.pressure2Cal);
  Serial.print("scaleCal: ");
  Serial.println(calValues.scaleCal);
  Serial.print("PID1 Setpoint: ");
  Serial.println(pid1.setpoint);
  Serial.print("PID1 kp: ");
  Serial.println(pid1.kp);
  Serial.print("PID1 ki: ");
  Serial.println(pid1.ki);
  Serial.print("PID1 kd: ");
  Serial.println(pid1.kd);
  Serial.print("PID1 aggKp: ");
  Serial.println(pid1.aggKp);
  Serial.print("PID1 aggKi: ");
  Serial.println(pid1.aggKi);
  Serial.print("PID1 aggKd: ");
  Serial.println(pid1.aggKd);
  Serial.print("PID2 Setpoint: ");
  Serial.println(pid2.setpoint);
  Serial.print("PID2 kp: ");
  Serial.println(pid2.kp);
  Serial.print("PID2 ki: ");
  Serial.println(pid2.ki);
  Serial.print("PID2 kd: ");
  Serial.println(pid2.kd);
  Serial.print("PID2 aggKp: ");
  Serial.println(pid2.aggKp);
  Serial.print("PID2 aggKi: ");
  Serial.println(pid2.aggKi);
  Serial.print("PID2 aggKd: ");
  Serial.println(pid2.aggKd);
  Serial.print("PID1 aggSP: ");
  Serial.println(pid1.aggSP);
  Serial.print("PID2 aggSP: ");
  Serial.println(pid2.aggSP);
  Serial.print("PID3 Setpoint: ");
  Serial.println(pid3.setpoint);
  Serial.print("PID3 kp: ");
  Serial.println(pid3.kp);
  Serial.print("PID3 ki: ");
  Serial.println(pid3.ki);
  Serial.print("PID3 kd: ");
  Serial.println(pid3.kd);
  Serial.print("emonCal: ");
  Serial.println(calValues.emonCal);
  Serial.print("ssrFailThreshold: ");
  Serial.println(calValues.ssrFailThreshold);
  Serial.print("currOffset: ");
  Serial.println(calValues.currOffset);
  Serial.print("ssrArmed: ");
  Serial.println(ssrArmed);
  Serial.print("PID1 mode: ");
  Serial.println(pid1.mode);
  Serial.print("PID2 mode: ");
  Serial.println(pid2.mode);
  Serial.print("PID3 mode: ");
  Serial.println(pid3.mode);
  Serial.print("PID2 adaptiveMode: ");
  Serial.println(pid2.adaptiveMode);
  Serial.print("PID1 adaptiveMode: ");
  Serial.println(pid1.adaptiveMode);
  Serial.print("sDebug: ");
  Serial.println(configValues.sDebug);
  Serial.print("PID1 alarmThreshold: ");
  Serial.println(pid1.alarmThreshold);
  Serial.print("PID2 alarmThreshold: ");
  Serial.println(pid2.alarmThreshold);
  Serial.print("PID3 alarmThreshold: ");
  Serial.println(pid3.alarmThreshold);
}

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

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
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
        EEPROM.put(0, calValues.zeroOffsetScale);
        LoadCell.set_offset(calValues.zeroOffsetScale);
        send(msgScaleTare.set(false), configValues.toACK);
        wait(SENDDELAY);
        send(msgScaleOffset.set(calValues.zeroOffsetScale, 1));
        wait(SENDDELAY);
        if (configValues.sDebug) { Serial.print("Scale Offset = "); Serial.println(calValues.zeroOffsetScale); }
        sendInfo("Scale Tare");
      }
      break;
    case CHILD_ID::ScaleOffset:
      calValues.zeroOffsetScale = message.getFloat();
      EEPROM.put(0, calValues.zeroOffsetScale);
      LoadCell.set_offset(calValues.zeroOffsetScale);
      break;
    case CHILD_ID::ScaleUntMag:
      calValues.mScale = message.getBool() ? 1 : 1000;
      break;
    case CHILD_ID::ScaleAvg:
      calValues.aScale = message.getBool() ? 255 : 128;
      break;
    case CHILD_ID::ScaleRtAccu:
      calValues.rScale = message.getBool() ? 1000 : 1;
      break;
    case CHILD_ID::Duty:
      dC = message.getFloat() / 100.0;
      break;
    case CHILD_ID::Duty2:
      dC2 = message.getFloat() / 100.0;
      break;
    case CHILD_ID::Duty3:
      dC3 = message.getFloat() / 100.0;
      break;
    case CHILD_ID::SSR:
      ssrArmed = message.getBool();
      EEPROM.put(100, ssrArmed);
      digitalWrite(SSRArmed_PIN, ssrArmed);
      break;
    case CHILD_ID::Press1Offset:
      calValues.pressure1Offset = message.getInt();
      EEPROM.put(4, calValues.pressure1Offset);
      break;
    case CHILD_ID::ScaleCal:
      calValues.scaleCal = message.getFloat();
      LoadCell.set_scale(calValues.scaleCal);
      EEPROM.put(6, calValues.scaleCal);
      break;
    case CHILD_ID::P1Cal:
      calValues.pressure1Cal = message.getFloat();
      EEPROM.put(10, calValues.pressure1Cal);
      break;
    case CHILD_ID::PIDMODE_1:
      pid1.mode = message.getBool();
      myPID1.SetMode(pid1.mode ? AUTOMATIC : MANUAL);
      EEPROM.put(101, pid1.mode);
      break;
    case CHILD_ID::PIDSETPOINT_1:
      pid1.setpoint = message.getFloat();
      EEPROM.put(14, pid1.setpoint);
      break;
    case CHILD_ID::PIDkP0_1:
      pid1.kp = message.getFloat();
      EEPROM.put(18, pid1.kp);
      break;
    case CHILD_ID::PIDkI0_1:
      pid1.ki = message.getFloat();
      EEPROM.put(22, pid1.ki);
      break;
    case CHILD_ID::PIDkD0_1:
      pid1.kd = message.getFloat();
      EEPROM.put(26, pid1.kd);
      break;
    case CHILD_ID::PIDkP1_1:
      pid1.aggKp = message.getFloat();
      EEPROM.put(30, pid1.aggKp);
      break;
    case CHILD_ID::PIDkI1_1:
      pid1.aggKi = message.getFloat();
      EEPROM.put(34, pid1.aggKi);
      break;
    case CHILD_ID::PIDkD1_1:
      pid1.aggKd = message.getFloat();
      EEPROM.put(38, pid1.aggKd);
      break;
    case CHILD_ID::AdaptiveSP_1:
      pid1.aggSP = message.getByte();
      EEPROM.put(70, pid1.aggSP);
      break;
    case CHILD_ID::AdaptiveMode_1:
      pid1.adaptiveMode = message.getBool();
      EEPROM.put(105, pid1.adaptiveMode);
      break;
    case CHILD_ID::PIDMODE_2:
      pid2.mode = message.getBool();
      myPID2.SetMode(pid2.mode ? AUTOMATIC : MANUAL);
      EEPROM.put(102, pid2.mode);
      break;
    case CHILD_ID::PIDSETPOINT_2:
      pid2.setpoint = message.getFloat();
      EEPROM.put(42, pid2.setpoint);
      break;
    case CHILD_ID::PIDkP0_2:
      pid2.kp = message.getFloat();
      EEPROM.put(46, pid2.kp);
      break;
    case CHILD_ID::PIDkI0_2:
      pid2.ki = message.getFloat();
      EEPROM.put(50, pid2.ki);
      break;
    case CHILD_ID::PIDkD0_2:
      pid2.kd = message.getFloat();
      EEPROM.put(54, pid2.kd);
      break;
    case CHILD_ID::PIDkP1_2:
      pid2.aggKp = message.getFloat();
      EEPROM.put(58, pid2.aggKp);
      break;
    case CHILD_ID::PIDkI1_2:
      pid2.aggKi = message.getFloat();
      EEPROM.put(62, pid2.aggKi);
      break;
    case CHILD_ID::PIDkD1_2:
      pid2.aggKd = message.getFloat();
      EEPROM.put(66, pid2.aggKd);
      break;
    case CHILD_ID::AdaptiveSP_2:
      pid2.aggSP = message.getByte();
      EEPROM.put(71, pid2.aggSP);
      break;
    case CHILD_ID::AdaptiveMode_2:
      pid2.adaptiveMode = message.getBool();
      EEPROM.put(104, pid2.adaptiveMode);
      break;
    case CHILD_ID::PIDMODE_3:
      pid3.mode = message.getBool();
      myPID3.SetMode(pid3.mode ? AUTOMATIC : MANUAL);
      EEPROM.put(103, pid3.mode);
      break;
    case CHILD_ID::PIDSETPOINT_3:
      pid3.setpoint = message.getFloat();
      EEPROM.put(72, pid3.setpoint);
      break;
    case CHILD_ID::PIDkP0_3:
      pid3.kp = message.getFloat();
      EEPROM.put(76, pid3.kp);
      break;
    case CHILD_ID::PIDkI0_3:
      pid3.ki = message.getFloat();
      EEPROM.put(80, pid3.ki);
      break;
    case CHILD_ID::PIDkD0_3:
      pid3.kd = message.getFloat();
      EEPROM.put(84, pid3.kd);
      break;
    case CHILD_ID::Curr_CAL:
      calValues.emonCal = message.getFloat();
      EEPROM.put(88, calValues.emonCal);
      break;
    case CHILD_ID::SSRFail_Threshhold:
      calValues.ssrFailThreshold = message.getByte();
      EEPROM.put(96, calValues.ssrFailThreshold);
      break;
    case CHILD_ID::curr_OFFSET:
      calValues.currOffset = message.getFloat();
      EEPROM.put(92, calValues.currOffset);
      break;
    case CHILD_ID::LOAD_MEMORY:
      if (message.getBool()) {
        StoreEEPROM();
      } else {
        getEEPROM();
      }
      break;
    case CHILD_ID::s_debug:
      configValues.sDebug = message.getBool();
      EEPROM.put(106, configValues.sDebug);
      break;
    case CHILD_ID::PID1_Threshold:
      pid1.alarmThreshold = message.getInt();
      EEPROM.put(107, pid1.alarmThreshold);
      break;
    case CHILD_ID::PID2_Threshold:
      pid2.alarmThreshold = message.getInt();
      EEPROM.put(109, pid2.alarmThreshold);
      break;
    case CHILD_ID::PID3_Threshold:
      pid3.alarmThreshold = message.getInt();
      EEPROM.put(111, pid3.alarmThreshold);
      break;
  }
}

void sendInfo(String payload) {
  send(msgINFO.set(payload.c_str()), configValues.toACK);
  wait(SENDDELAY);
  msgLCDINFO.setDestination(10);
  send(msgLCDINFO.set(payload.c_str()), configValues.toACK);
  wait(SENDDELAY);
}

void DS18B20() {
  //Serial.println("DS18B20()");
  byte addr[8];
  int count = 0;
   
  while (ds.search(addr)) {
    byte i;
    byte type_s;
    byte data[9];
    
    // Serial.print("ROM =");
    // for( i = 0; i < 8; i++) {
    //   Serial.write(' ');
    //   Serial.print(addr[i], HEX);
    // }
    
    switch (addr[0]) {
      case 0x10:
        // Serial.println("  Chip = DS18S20");  // or old DS1820
        type_s = 1;
        break;
      case 0x28:
        // Serial.println("  Chip = DS18B20");
        type_s = 0;
        break;
      case 0x22:
        // Serial.println("  Chip = DS1822");
        type_s = 0;
        break;
      default:
        // Serial.println("Device is not a DS18x20 family device.");
        return;
    } 

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 0);
    delay(10); 
    ds.reset();
    ds.select(addr);    
    ds.write(0xBE);
    for ( i = 0; i < 9; i++) { data[i] = ds.read(); }
    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10) {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    ds18b20Values[count].C = (float)raw / 16.0;
    ds18b20Values[count].F = ds18b20Values[count].C * 1.8 + 32.0;
    // Serial.print("Count = ");
    // Serial.print(count);
    // Serial.print("  Temperature = ");
    // Serial.print(ds18b20Values[count].C);
    // Serial.print(" Celsius, ");
    // Serial.print(ds18b20Values[count].F);
    // Serial.println(" Fahrenheit");
    count = count + 1;
}
  // Serial.println("No more addresses.");
  // Serial.println();
  ds.reset_search();
  delay(1000);
  return;
}