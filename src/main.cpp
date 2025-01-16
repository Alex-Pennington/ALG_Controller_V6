#include <Arduino.h>
#include "HX711.h"
#include <PID_v1.h>
#include <EEPROM.h>
#include "controller.h"
#include <OneWire.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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
  float scaleCal = 53200;
  byte aScale =  255; // times is a parameter of type byte that specifies the number of times the raw value should be read and averaged.
  int mScale  = 1; //scale multiplier
  int rScale =  1000; //rate scalar coefficient
  int VccCurrentOffset = 566;
  float VccCurrentMultiplier = 0.004887;
};;
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
unsigned long SensorLoop_timer = 0;
float dC = 0.0;
float dC2 = 0.0;
float dC3 = 0.0;
float valueScale = 0;
float value_oldScale = 0;
unsigned long oldtimeScale = 0;
float kgpsScale = 0; //rate of change of scale in kilograms per second
long AREF_V = 0;
bool ssrArmed = false;
float pressure1Var = 0;
float pressure2Var = 0;
float pressure3Var = 0;
float pressure4Var = 0;
float VccCurrent = 0;
extern unsigned int __heap_start;
extern void *__brkval;

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
#define Pressure3PIN A15 //
#define Pressure4PIN A15 //
#define emon_Input_PIN A9
#define VccCurrentSensor A0 //
#define Thermistor1PIN A1 //
#define Thermistor2PIN A2 //
#define Serial3_RX 15 //
#define Serial3_TX 14 //



/*Connector Pinouts
Pressure  1 5V, 2 GND, 3 SIG
Thermistor 1 SIG, 2 SIG
Steinhart 1 SIG, 2 SIG
HX711 1 E-, 2 A+, 3 A-, 4 E+
Serial 1 5V, 2 RX, 3 TX, 4 GND
*/

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
void serialPrintSensorData();
float Steinhart();
void AllStop();
void StoreEEPROM();
void getEEPROM();
void FactoryResetEEPROM();
void printConfig();
long getBandgap(void);
void sendInfo(String);
void DS18B20();
float getThermistor(int);
float readPressure(int pin, int offset, float cal);
void displayLine(const char* line);
int freeMemory();

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

void setup() {
  Serial.begin(115200);
  getEEPROM();
  LoadCell.begin(HX711_dout, HX711_sck);
  LoadCell.set_scale(calValues.scaleCal);
  LoadCell.set_offset(calValues.zeroOffsetScale);
  LoadCell.set_gain();
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
  pinMode(VccCurrentSensor, INPUT);

  //OLED
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {Serial.println(F("SSD1306 allocation failed"));}  
  displayLine("Booting...");
  delay(1000);
}

void loop() {
  DutyCycleLoop();
  

  if ( (millis() - SensorLoop_timer) > (unsigned long)configValues.SENSORLOOPTIME)  {
    Serial.println("-");
    AREF_V = getBandgap();

    getVccCurrent(); 
    DS18B20();
    emon(); 
    getScale();

    char buffer[16];
    dtostrf(valueScale, 6, 2, buffer);
    displayLine(buffer);

    pressure1Var = readPressure(Pressure1PIN, calValues.pressure1Offset, calValues.pressure1Cal);
    pressure2Var = readPressure(Pressure2PIN, calValues.pressure2Offset, calValues.pressure2Cal);
    pressure3Var = readPressure(Pressure3PIN, calValues.pressure3Offset, calValues.pressure3Cal);
    pressure4Var = readPressure(Pressure4PIN, calValues.pressure4Offset, calValues.pressure4Cal);
    
    TempAlarm();
    serialPrintSensorData();
    SensorLoop_timer = millis();
  }
}

void getVccCurrent()
{
  for (int i = 0; i < 10; i++)
  {
    VccCurrent += analogRead(VccCurrentSensor);
    delay(10); // Small delay for better averaging
  }
  VccCurrent /= 10;                                                                        // Average the readings
  VccCurrent = (VccCurrent - calValues.VccCurrentOffset) * calValues.VccCurrentMultiplier; //
  return;
}

void serialPrintSensorData()
{
  if (configValues.sDebug) {
    Serial.print("Time: ");
    Serial.println((float)millis()/1000.0/60.0/60.0,2);
    Serial.print("VccCurrent: ");
    Serial.println(VccCurrent);
    Serial.print("AREF_V: ");
    Serial.println(AREF_V);
    Serial.print("Pressure1: ");
    Serial.println(pressure1Var);
    Serial.print("Pressure2: ");
    Serial.println(pressure2Var);
    Serial.print("Pressure3: ");
    Serial.println(pressure3Var);
    Serial.print("Pressure4: ");
    Serial.println(pressure4Var);
    Serial.print("Scale: ");
    Serial.println(valueScale);
    Serial.print("kgpsScale: ");
    Serial.println(kgpsScale);
    for (int i = 0; i < static_cast<int>(sizeof(ds18b20Values) / sizeof(ds18b20Values[0])); i++) {
      Serial.print("DS18B20 Sensor ");
      Serial.print(i);
      Serial.print(" Temperature (C): ");
      Serial.println(ds18b20Values[i].C);
      Serial.print("DS18B20 Sensor ");
      Serial.print(i);
      Serial.print(" Temperature (F): ");
      Serial.println(ds18b20Values[i].F);
    }
    Serial.print("Emon RMS: ");
    Serial.println(emonVars.rms);
    Serial.print("Steinhart Temperature (F): ");
    Serial.println(steinhartValues.steinhart);
  }
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

void emon()
{
  // SSRFail-Emon
  digitalWrite(ElementPowerPin, HIGH);
  digitalWrite(ElementPowerPin2, HIGH);
  digitalWrite(ElementPowerPin3, HIGH);
  float sum = 0;

  emonVars.sampleTime = millis();

  int N = 1000;
  for (int i = 0; i < N; i++)  {
    float current = calValues.emonCal * (analogRead(emon_Input_PIN) - 512); // in amps I presume
    sum += current * current;                                               // sum squares
    delay(10);
  }
  emonVars.rms = sqrt(sum / N) - calValues.currOffset;
  if (int(emonVars.rms) > int(calValues.ssrFailThreshold))  {
    red_alert();
    AllStop();
    sendInfo("SSR Failed");
  }  else  {
    digitalWrite(ElementPowerPin, dutyCycle[0].element);
    digitalWrite(ElementPowerPin2, dutyCycle[1].element);
    digitalWrite(ElementPowerPin3, dutyCycle[2].element);
    //      digitalWrite(SSRArmed_PIN, SSRArmed);
  }
  for (int i = 0; i < N; i++)  {
    float current = calValues.emonCal * (analogRead(emon_Input_PIN) - 512); // in amps I presume
    sum += current * current;                                               // sum squares
    delay(10);
  }
  emonVars.rms = sqrt(sum / N) - calValues.currOffset;
  return;
}

void getScale()
{
  value_oldScale = valueScale;
  if (LoadCell.is_ready())
  {
    valueScale = LoadCell.get_units(10) * calValues.mScale;
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

  // Compensate for Seebeck effect using temperature sensor one
  // float tempCompensation = ds18b20Values[0].C * 0.0002; // Adjust the factor as needed
  // valueScale += tempCompensation;

  kgpsScale = calValues.rScale * ((valueScale - value_oldScale) / ((millis() - oldtimeScale) / 1000));
  oldtimeScale = millis();
  return;
}

float readPressure(int pin, int offset, float cal) {
  int RawADCavg = 0;
  int i = 0;
  for (i = 0; i < 10; i++) {
    delay(10);
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
    delay(10); //this increased resolution signifigantly
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
            delay(10);
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
  Serial.println("!!! TEMP ALARM !!!");
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
  EEPROM.get(EEPROMAddresses::SSR_ARMED, ssrArmed);
  EEPROM.get(EEPROMAddresses::PID1_MODE, pid1.mode);
  EEPROM.get(EEPROMAddresses::PID2_MODE, pid2.mode);
  EEPROM.get(EEPROMAddresses::PID3_MODE, pid3.mode);
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
  calValues.pressure1Cal = 20.77;
  calValues.pressure1Offset = 425;
  calValues.pressure2Cal = 20.77;
  calValues.pressure2Offset = 425;
  calValues.pressure3Cal = 20.77;
  calValues.pressure3Offset = 425;
  calValues.pressure4Cal = 20.77;
  calValues.pressure4Offset = 425;
  calValues.emonCal = 0.128;
  calValues.ssrFailThreshold = 2;
  calValues.currOffset = 5.28;
  calValues.zeroOffsetScale = 403361;
  calValues.scaleCal = 53200;
  calValues.aScale = 255;
  calValues.mScale = 1;
  calValues.rScale = 1000;
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
  configValues.scale_stabilisingtime = 10000;
  configValues.toACK = false;

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
  Serial.print("pressure3Offset: ");
  Serial.println(calValues.pressure3Offset);
  Serial.print("pressure3Cal: ");
  Serial.println(calValues.pressure3Cal);
  Serial.print("pressure4Offset: ");
  Serial.println(calValues.pressure4Offset);
  Serial.print("pressure4Cal: ");
  Serial.println(calValues.pressure4Cal);
  Serial.print("scaleCal: ");
  Serial.println(calValues.scaleCal);
  Serial.print("aScale: ");
  Serial.println(calValues.aScale);
  Serial.print("mScale: ");
  Serial.println(calValues.mScale);
  Serial.print("rScale: ");
  Serial.println(calValues.rScale);
  Serial.print("VccCurrentOffset: ");
  Serial.println(calValues.VccCurrentOffset);
  Serial.print("VccCurrentMultiplier: ");
  Serial.println(calValues.VccCurrentMultiplier);
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
  Serial.print("PID3 aggKp: ");
  Serial.println(pid3.aggKp);
  Serial.print("PID3 aggKi: ");
  Serial.println(pid3.aggKi);
  Serial.print("PID3 aggKd: ");
  Serial.println(pid3.aggKd);
  Serial.print("PID3 aggSP: ");
  Serial.println(pid3.aggSP);
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
  Serial.print("PID3 adaptiveMode: ");
  Serial.println(pid3.adaptiveMode);
  Serial.print("sDebug: ");
  Serial.println(configValues.sDebug);
  Serial.print("PID1 alarmThreshold: ");
  Serial.println(pid1.alarmThreshold);
  Serial.print("PID2 alarmThreshold: ");
  Serial.println(pid2.alarmThreshold);
  Serial.print("PID3 alarmThreshold: ");
  Serial.println(pid3.alarmThreshold);
  Serial.print("SENSORLOOPTIME: ");
  Serial.println(configValues.SENSORLOOPTIME);
  Serial.print("scale_stabilisingtime: ");
  Serial.println(configValues.scale_stabilisingtime);
  Serial.print("toACK: ");
  Serial.println(configValues.toACK);
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

  delay(10); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void sendInfo(String payload) {
  Serial.print("INFO : ");
  Serial.println(payload);
}

void DS18B20() {
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
