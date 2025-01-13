#include <Arduino.h>
#include "HX711.h"
#include <max6675.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include "controller.h"


#define VERSION "0.6.1.1"

//Structs

struct ConfigurationValues {
  int SENSORLOOPTIME = 23000;
  bool sDebug = true;
  int scale_stabilisingtime = 10000;
  bool toACK = false;
};

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

struct CalibrationValues {
    float pressCal1 = 20.77;
    int press1Offset = 425;
    float emonCal = 0.128;
    byte ssrFailThreshold = 2;
    double currOffset = 5.28;
    float zeroOffsetScale = 403361;
    float scaleCal = 53200;
    int aScale =  10000;
    int mScale  = 1000;
    int rScale =  1000;
};

struct ThermistorConfig {
    const long seriesResistor = 100000;
    const long nominalResistance = 100000;
    const int nominalTemperature = 25;
    const int bCoefficient = 3950;
};

struct EmonVars {
    unsigned long sampleTime = 0;
    bool ssrFail = false;
    int ssrFailCount = 0;
    float rms = 0.0;
};

struct DutyCycle {
    bool element = false;
    int loopTime = 10; // Duty cycle total loop time in seconds.
    unsigned long onTime = 0;
    unsigned long offTime = 0;
};

ConfigurationValues configValues;

ThermistorConfig thermistorConfig;

struct SteinhartValues {
  float resistance = 0.0;
  float adcValue = 0;
  float steinhart = 0;
};

SteinhartValues steinhartValues;

CalibrationValues calValues;

PIDConfig pid1 = {false, 150, 0.5, 0.005, 0, 1, 0, 0, 10, false, 0, 0, 0};
PIDConfig pid2 = {false, 150, 0.35, 0, 0, 5, 0, 0, 10, false, 0, 0, 0};
PIDConfig pid3 = {false, 120, 1.2, 0.01, 0, 0, 0, 0, 10, false, 0, 0, 0};

DutyCycle dutyCycle[3];

EmonVars emonVars;

//Working Variables
static unsigned long SensorLoop_timer = 0;
static float dC = 0.0;
static float dC2 = 0.0;
static float dC3 = 0.0;
static float valueScale = 0;
static float value_oldScale = 0;
static unsigned long oldtimeScale = 0;
static float kgpsScale = 0;
static long AREF_V = 0;
static bool ssrArmed = false;

//Pin Definitions
#define PressurePIN A15
#define NTCPin A13
#define NTCEnable 45
#define emon_Input_PIN A9
#define SSRArmed_PIN  29
#define HX711_dout 9
#define HX711_sck 10
#define ElementPowerPin 41
#define ElementPowerPin2 43
#define ElementPowerPin3 25
#define thermo0DO 32
#define thermo0CS 31
#define thermo0CLK 30
#define thermo1DO 35
#define thermo1CS 34
#define thermo1CLK 33
#define thermo2DO 48
#define thermo2CS 47
#define thermo2CLK 46
#define speakerPin 39

//Scale
HX711 LoadCell;

//Thermocouples
MAX6675 thermocouple0(thermo0CLK, thermo0CS, thermo0DO);
MAX6675 thermocouple1(thermo1CLK, thermo1CS, thermo1DO);
MAX6675 thermocouple2(thermo2CLK, thermo2CS, thermo2DO);

// MySensors Child IDs
enum CHILD_ID {
  T0 = 0,                // "Condenser T0"
  T1 = 1,                // "Upper Spool T1"
  T2 = 2,                // "Amb Temp"
  HUM = 3,               // "Amb Hum"
  Duty = 4,              // "Duty Cycle"
  Scale = 5,             // "Ref Scale"
  Duty2 = 6,             // "Duty Cycle2"
  T3 = 7,                // "Platter"
  SSR = 9,               // "SSR Armed"
  LOAD_MEMORY = 10,      // "Load Memory"
  T4 = 11,               // "Lower Spool T2"
  P1 = 12,               // "Pressure 1"
  P1Cal = 13,            // "PresCal 1"
  ScaleRate = 14,        // "Scale Delta/T"
  ScaleTare = 15,        // "Tare Scale"
  ScaleOffset = 16,      // "Scale Zero Offset"
  ScaleRtAccu = 17,      // "Rate g/Kg"
  ScaleUntMag = 18,      // "Scale g/Kg"
  ScaleAvg = 19,         // "Scale Accuracy"
  PIDMODE_1 = 20,        // "PID1 Lower"
  PIDSETPOINT_1 = 21,    // "PID1 Setpoint"
  AdaptiveSP_1 = 22,     // "Adaptive SP"
  PIDkP0_1 = 23,         // "constKp"
  PIDkI0_1 = 24,         // "constKi"
  PIDkD0_1 = 25,         // "constKd"
  PIDkP1_1 = 26,         // "aggKp"
  PIDkI1_1 = 27,         // "aggKi"
  PIDkD1_1 = 28,         // "aggKd"
  AdaptiveMode_1 = 29,   // "Adaptive"
  EDC = 31,              // "L dC"
  Info = 32,             // "Debug Info"
  ScaleCal = 40,         // "Scale Cal"
  PIDMODE_2 = 60,        // "PID2 Upper"
  PIDSETPOINT_2 = 61,    // "PID2 Setpoint"
  AdaptiveSP_2 = 62,     // "Adaptive SP_2"
  PIDkP0_2 = 63,         // "constKp_2"
  PIDkI0_2 = 64,         // "constKi_2"
  PIDkD0_2 = 65,         // "constKd_2"
  PIDkP1_2 = 66,         // "aggKp_2"
  PIDkI1_2 = 67,         // "aggKi_2"
  PIDkD1_2 = 68,         // "aggKd_2"
  AdaptiveMode_2 = 69,   // "Adaptive_2"
  EDC_2 = 71,            // "U dC"
  Press1Offset = 73,     // "P1 Zero Offset"
  RMS = 74,              // "RMS Power"
  Curr_CAL = 75,         // "Cur CAL"
  SSRFail_Threshhold = 76,// "SSRFail Threshhold"
  SSRFail_Alarm = 77,    // "SSR FAIL ALARM"
  curr_OFFSET = 78,      // "Current Offset"
  PIDMODE_3 = 80,        // "PID3 Platter"
  PIDSETPOINT_3 = 81,    // "PID3 Setpoint"
  Duty3 = 82,            // "Duty Cycle3"
  PIDkP0_3 = 83,         // "constKp_3"
  PIDkI0_3 = 84,         // "constKi_3"
  PIDkD0_3 = 85,         // "constKd_3"
  EDC_3 = 91,            // "P dC"
  s_debug = 92,          // "Serial3 Debug"
  BoardVoltage = 93,     // "Board Voltage"
  PID1_Threshold = 94,   // "PID1A"
  PID2_Threshold = 95,   // "PID2A"
  PID3_Threshold = 96    // "PID3A"
};

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
void receiveSerial();
void sendInfo(String);

void setup() {
  LoadCell.begin(HX711_dout, HX711_sck);
  Serial3.begin(115200);
}
 
void loop() {
  DutyCycleLoop();

  AREF_V = getBandgap();

  if ( (millis() - SensorLoop_timer) > (unsigned long)configValues.SENSORLOOPTIME)  {
    //Temp Alarm
    if (pid1.alarmThreshold < pid1.input) {
      sendInfo("PID1!");
      red_alert();
      AllStop();
    }
    if (pid2.alarmThreshold < pid2.input) {
      sendInfo("PID2!");
      red_alert();
      AllStop();
    }
    if (pid3.alarmThreshold < pid3.input) {
      sendInfo("PID3!");
      red_alert();
      AllStop();
    }

    //SSRFail-Emon
    digitalWrite(ElementPowerPin, HIGH);
    digitalWrite(ElementPowerPin2, HIGH);
    digitalWrite(ElementPowerPin3, HIGH);
    float sum = 0;

    emonVars.sampleTime = millis();

    int N = 1000;
    for (int i = 0; i < N; i++) {
      float current = calValues.emonCal * (analogRead(emon_Input_PIN) - 512) ;  // in amps I presume
      sum += current * current ;  // sum squares
      delayMicroseconds(1);
    }
    emonVars.rms = sqrt (sum / N) - calValues.currOffset;
    if (int(emonVars.rms) > int(calValues.ssrFailThreshold)) {
      red_alert();
      AllStop();
      sendInfo("SSR Failed");
    } else {
      digitalWrite(ElementPowerPin, dutyCycle[0].element);
      digitalWrite(ElementPowerPin2, dutyCycle[1].element);
      digitalWrite(ElementPowerPin3, dutyCycle[2].element);
      //      digitalWrite(SSRArmed_PIN, SSRArmed);
    }
    for (int i = 0; i < N; i++) {
      float current = calValues.emonCal * (analogRead(emon_Input_PIN) - 512) ;  // in amps I presume
      sum += current * current ;  // sum squares
      delayMicroseconds(1);
    }
    emonVars.rms = sqrt (sum / N) - calValues.currOffset;

    //Scale
    value_oldScale = valueScale;
    valueScale = LoadCell.get_units(calValues.aScale) * calValues.mScale;
    if (valueScale < 0) {
      valueScale = 0;
    }
    kgpsScale = calValues.rScale * ((valueScale - value_oldScale) / ((millis() - oldtimeScale) / 1000));
    oldtimeScale = millis();


    //Send Sensors
    Serial3.print("Temp0: ");
    Serial3.println(thermocouple0.readFahrenheit());
    Serial3.print("Temp1: ");
    Serial3.println(thermocouple1.readFahrenheit());
    Serial3.print("Temp3: ");
    Serial3.println(steinhartValues.steinhart);
    Serial3.print("Temp4: ");
    Serial3.println(thermocouple2.readFahrenheit());
    Serial3.print("Scale: ");
    Serial3.println(valueScale);
    Serial3.print("RMS: ");
    Serial3.println(emonVars.rms);
    Serial3.print("SSR Armed: ");
    Serial3.println(ssrArmed);
    Serial3.print("PID1 Output: ");
    Serial3.println(pid1.output);
    Serial3.print("PID2 Output: ");
    Serial3.println(pid2.output);
    Serial3.print("PID3 Output: ");
    Serial3.println(pid3.output);
    Serial3.print("Board Voltage: ");
    Serial3.println((float)AREF_V);



    int RawADCavg = 0;
    int i = 0;
    for (i = 0; i < 10; i++) {
      delayMicroseconds(10);
      RawADCavg += analogRead(PressurePIN);
    }
    float tempPress = (((float)RawADCavg / 10.0 )  - calValues.press1Offset ) * (1 / calValues.pressCal1);
    Serial3.print("Pressure 1: ");
    Serial3.println(tempPress, 2);
    Serial3.print("Scale Rate: ");
    Serial3.println(kgpsScale, 2);
    if (configValues.sDebug) {
      Serial3.println("----------------------------------------");
      Serial3.print("Sensor Data Time (ms) ");
      Serial3.println(millis());
      Serial3.print("Condenser T0: ");
      Serial3.println(thermocouple0.readFahrenheit());
      Serial3.print("Upper Spool T1: ");
      Serial3.println(thermocouple1.readFahrenheit());
      Serial3.print("lower Spool T2: ");
      Serial3.println(thermocouple2.readFahrenheit());
      Serial3.print("Amb Temp DHT: ");
      Serial3.print("Platter Temp: ");
      Serial3.println(Steinhart());
      Serial3.print("Humidity DHT: ");
      Serial3.print("Scale : ");
      Serial3.println(valueScale);
      Serial3.print("Pressure 1: ");
      Serial3.println(tempPress);
      Serial3.print("Current : ");
      Serial3.println(emonVars.rms);
      Serial3.print("MEGA Voltage : ");
      Serial3.println(AREF_V);
    }
    SensorLoop_timer = millis();
  }
}

float Steinhart() {
  digitalWrite(NTCEnable, HIGH);
  for (int n = 0; n < 10; n++)
  {
    delayMicroseconds(10); //this increased resolution signifigantly
    steinhartValues.adcValue += analogRead(NTCPin);
  }
  digitalWrite(NTCEnable, LOW);
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
  Serial3.print("PIDMODE: ");
  Serial3.println(pid1.mode);
  Serial3.print("PIDMODE_2: ");
  Serial3.println(pid2.mode);
  Serial3.print("PIDMODE_3: ");
  Serial3.println(pid3.mode);
  Serial3.print("SSRFailAlarm: ");
  Serial3.println(emonVars.ssrFail);
  Serial3.print("SSR: ");
  Serial3.println(ssrArmed);
}

void StoreEEPROM() {
  EEPROM.put(0, calValues.zeroOffsetScale);
  EEPROM.put(4, calValues.press1Offset);
  EEPROM.put(6, calValues.scaleCal);
  EEPROM.put(10, calValues.pressCal1);
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
  printConfig();
}

void getEEPROM() {
  EEPROM.get(0, calValues.zeroOffsetScale);
  EEPROM.get(4, calValues.press1Offset);
  EEPROM.get(6, calValues.scaleCal);
  EEPROM.get(10, calValues.pressCal1);
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
  printConfig();
}

void FactoryResetEEPROM() {
  // Reset Calibration Values
  calValues.pressCal1 = 20.77;
  calValues.press1Offset = 425;
  calValues.emonCal = 0.128;
  calValues.ssrFailThreshold = 2;
  calValues.currOffset = 5.28;
  calValues.zeroOffsetScale = 403361;
  calValues.scaleCal = 53200;
  calValues.aScale = 10000;
  calValues.mScale = 1000; // multiplier
  calValues.rScale = 1000; // rate multiplier

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
  Serial3.print("zeroOffsetScale: ");
  Serial3.println(calValues.zeroOffsetScale);
  Serial3.print("press1Offset: ");
  Serial3.println(calValues.press1Offset);
  Serial3.print("scaleCal: ");
  Serial3.println(calValues.scaleCal);
  Serial3.print("pressCal1: ");
  Serial3.println(calValues.pressCal1);
  Serial3.print("PID1 Setpoint: ");
  Serial3.println(pid1.setpoint);
  Serial3.print("PID1 kp: ");
  Serial3.println(pid1.kp);
  Serial3.print("PID1 ki: ");
  Serial3.println(pid1.ki);
  Serial3.print("PID1 kd: ");
  Serial3.println(pid1.kd);
  Serial3.print("PID1 aggKp: ");
  Serial3.println(pid1.aggKp);
  Serial3.print("PID1 aggKi: ");
  Serial3.println(pid1.aggKi);
  Serial3.print("PID1 aggKd: ");
  Serial3.println(pid1.aggKd);
  Serial3.print("PID2 Setpoint: ");
  Serial3.println(pid2.setpoint);
  Serial3.print("PID2 kp: ");
  Serial3.println(pid2.kp);
  Serial3.print("PID2 ki: ");
  Serial3.println(pid2.ki);
  Serial3.print("PID2 kd: ");
  Serial3.println(pid2.kd);
  Serial3.print("PID2 aggKp: ");
  Serial3.println(pid2.aggKp);
  Serial3.print("PID2 aggKi: ");
  Serial3.println(pid2.aggKi);
  Serial3.print("PID2 aggKd: ");
  Serial3.println(pid2.aggKd);
  Serial3.print("PID1 aggSP: ");
  Serial3.println(pid1.aggSP);
  Serial3.print("PID2 aggSP: ");
  Serial3.println(pid2.aggSP);
  Serial3.print("PID3 Setpoint: ");
  Serial3.println(pid3.setpoint);
  Serial3.print("PID3 kp: ");
  Serial3.println(pid3.kp);
  Serial3.print("PID3 ki: ");
  Serial3.println(pid3.ki);
  Serial3.print("PID3 kd: ");
  Serial3.println(pid3.kd);
  Serial3.print("emonCal: ");
  Serial3.println(calValues.emonCal);
  Serial3.print("ssrFailThreshold: ");
  Serial3.println(calValues.ssrFailThreshold);
  Serial3.print("currOffset: ");
  Serial3.println(calValues.currOffset);
  Serial3.print("ssrArmed: ");
  Serial3.println(ssrArmed);
  Serial3.print("PID1 mode: ");
  Serial3.println(pid1.mode);
  Serial3.print("PID2 mode: ");
  Serial3.println(pid2.mode);
  Serial3.print("PID3 mode: ");
  Serial3.println(pid3.mode);
  Serial3.print("PID2 adaptiveMode: ");
  Serial3.println(pid2.adaptiveMode);
  Serial3.print("PID1 adaptiveMode: ");
  Serial3.println(pid1.adaptiveMode);
  Serial3.print("sDebug: ");
  Serial3.println(configValues.sDebug);
  Serial3.print("PID1 alarmThreshold: ");
  Serial3.println(pid1.alarmThreshold);
  Serial3.print("PID2 alarmThreshold: ");
  Serial3.println(pid2.alarmThreshold);
  Serial3.print("PID3 alarmThreshold: ");
  Serial3.println(pid3.alarmThreshold);
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

/**
 * @brief Handles incoming serial messages and updates configuration or calibration values accordingly.
 * 
 * This function reads a message from Serial3, parses it, and updates various configuration or calibration
 * values based on the parsed message. The message format is expected to be "sensor:value". Depending on the
 * sensor ID, different actions are taken, such as updating EEPROM, setting offsets, or changing PID parameters.
 * 
 * The function supports the following sensor IDs and their corresponding actions:
 * - CHILD_ID::ScaleTare: Tares the scale and updates the zero offset.
 * - CHILD_ID::ScaleOffset: Sets the scale offset.
 * - CHILD_ID::ScaleUntMag: Sets the scale unit magnitude.
 * - CHILD_ID::ScaleAvg: Sets the scale averaging factor.
 * - CHILD_ID::ScaleRtAccu: Sets the scale rate accuracy.
 * - CHILD_ID::Duty: Sets the duty cycle.
 * - CHILD_ID::Duty2: Sets the second duty cycle.
 * - CHILD_ID::Duty3: Sets the third duty cycle.
 * - CHILD_ID::SSR: Arms or disarms the SSR and updates EEPROM.
 * - CHILD_ID::Press1Offset: Sets the pressure sensor 1 offset and updates EEPROM.
 * - CHILD_ID::ScaleCal: Sets the scale calibration factor and updates EEPROM.
 * - CHILD_ID::P1Cal: Sets the pressure sensor 1 calibration factor and updates EEPROM.
 * - CHILD_ID::PIDMODE_1: Sets the mode of PID controller 1 and updates EEPROM.
 * - CHILD_ID::PIDSETPOINT_1: Sets the setpoint of PID controller 1 and updates EEPROM.
 * - CHILD_ID::PIDkP0_1: Sets the proportional gain of PID controller 1 and updates EEPROM.
 * - CHILD_ID::PIDkI0_1: Sets the integral gain of PID controller 1 and updates EEPROM.
 * - CHILD_ID::PIDkD0_1: Sets the derivative gain of PID controller 1 and updates EEPROM.
 * - CHILD_ID::PIDkP1_1: Sets the aggressive proportional gain of PID controller 1 and updates EEPROM.
 * - CHILD_ID::PIDkI1_1: Sets the aggressive integral gain of PID controller 1 and updates EEPROM.
 * - CHILD_ID::PIDkD1_1: Sets the aggressive derivative gain of PID controller 1 and updates EEPROM.
 * - CHILD_ID::AdaptiveSP_1: Sets the adaptive setpoint of PID controller 1 and updates EEPROM.
 * - CHILD_ID::AdaptiveMode_1: Sets the adaptive mode of PID controller 1 and updates EEPROM.
 * - CHILD_ID::PIDMODE_2: Sets the mode of PID controller 2 and updates EEPROM.
 * - CHILD_ID::PIDSETPOINT_2: Sets the setpoint of PID controller 2 and updates EEPROM.
 * - CHILD_ID::PIDkP0_2: Sets the proportional gain of PID controller 2 and updates EEPROM.
 * - CHILD_ID::PIDkI0_2: Sets the integral gain of PID controller 2 and updates EEPROM.
 * - CHILD_ID::PIDkD0_2: Sets the derivative gain of PID controller 2 and updates EEPROM.
 * - CHILD_ID::PIDkP1_2: Sets the aggressive proportional gain of PID controller 2 and updates EEPROM.
 * - CHILD_ID::PIDkI1_2: Sets the aggressive integral gain of PID controller 2 and updates EEPROM.
 * - CHILD_ID::PIDkD1_2: Sets the aggressive derivative gain of PID controller 2 and updates EEPROM.
 * - CHILD_ID::AdaptiveSP_2: Sets the adaptive setpoint of PID controller 2 and updates EEPROM.
 * - CHILD_ID::AdaptiveMode_2: Sets the adaptive mode of PID controller 2 and updates EEPROM.
 * - CHILD_ID::PIDMODE_3: Sets the mode of PID controller 3 and updates EEPROM.
 * - CHILD_ID::PIDSETPOINT_3: Sets the setpoint of PID controller 3 and updates EEPROM.
 * - CHILD_ID::PIDkP0_3: Sets the proportional gain of PID controller 3 and updates EEPROM.
 * - CHILD_ID::PIDkI0_3: Sets the integral gain of PID controller 3 and updates EEPROM.
 * - CHILD_ID::PIDkD0_3: Sets the derivative gain of PID controller 3 and updates EEPROM.
 * - CHILD_ID::Curr_CAL: Sets the current calibration factor and updates EEPROM.
 * - CHILD_ID::SSRFail_Threshhold: Sets the SSR fail threshold and updates EEPROM.
 * - CHILD_ID::curr_OFFSET: Sets the current offset and updates EEPROM.
 * - CHILD_ID::LOAD_MEMORY: Loads or stores EEPROM values based on the message value.
 * - CHILD_ID::s_debug: Sets the debug mode and updates EEPROM.
 * - CHILD_ID::PID1_Threshold: Sets the alarm threshold for PID controller 1 and updates EEPROM.
 * - CHILD_ID::PID2_Threshold: Sets the alarm threshold for PID controller 2 and updates EEPROM.
 * - CHILD_ID::PID3_Threshold: Sets the alarm threshold for PID controller 3 and updates EEPROM.
 * 
 * If the debug mode is enabled (configValues.sDebug), the received message is printed to Serial3.
 */
void receiveSerial() {
  if (Serial3.available() > 0) {
    String message = Serial3.readStringUntil('\n');
    if (configValues.sDebug) {
      Serial3.print("Received: ");
      Serial3.println(message);
    }

    int sensor = message.substring(0, message.indexOf(':')).toInt();
    String value = message.substring(message.indexOf(':') + 1);

    switch (sensor) {
      case CHILD_ID::ScaleTare:
        if (value.toInt()) {
          if (configValues.sDebug) Serial3.print("Tare ... ");
          calValues.zeroOffsetScale = LoadCell.read_average(100);
          if (configValues.sDebug) Serial3.println(" done.");
          EEPROM.put(0, calValues.zeroOffsetScale);
          LoadCell.set_offset(calValues.zeroOffsetScale);
          sendInfo("Scale Tare");
        }
        break;
      case CHILD_ID::ScaleOffset:
        calValues.zeroOffsetScale = value.toFloat();
        EEPROM.put(0, calValues.zeroOffsetScale);
        LoadCell.set_offset(calValues.zeroOffsetScale);
        break;
      case CHILD_ID::ScaleUntMag:
        calValues.mScale = value.toInt() ? 1 : 1000;
        break;
      case CHILD_ID::ScaleAvg:
        calValues.aScale = value.toInt() ? 100000 : 10000;
        break;
      case CHILD_ID::ScaleRtAccu:
        calValues.rScale = value.toInt() ? 1000 : 1;
        break;
      case CHILD_ID::Duty:
        dC = value.toFloat() / 100.0;
        break;
      case CHILD_ID::Duty2:
        dC2 = value.toFloat() / 100.0;
        break;
      case CHILD_ID::Duty3:
        dC3 = value.toFloat() / 100.0;
        break;
      case CHILD_ID::SSR:
        ssrArmed = value.toInt();
        EEPROM.put(100, ssrArmed);
        digitalWrite(SSRArmed_PIN, ssrArmed);
        break;
      case CHILD_ID::Press1Offset:
        calValues.press1Offset = value.toInt();
        EEPROM.put(4, calValues.press1Offset);
        break;
      case CHILD_ID::ScaleCal:
        calValues.scaleCal = value.toFloat();
        LoadCell.set_scale(calValues.scaleCal);
        EEPROM.put(6, calValues.scaleCal);
        break;
      case CHILD_ID::P1Cal:
        calValues.pressCal1 = value.toFloat();
        EEPROM.put(10, calValues.pressCal1);
        break;
      case CHILD_ID::PIDMODE_1:
        pid1.mode = value.toInt();
        myPID1.SetMode(pid1.mode ? AUTOMATIC : MANUAL);
        EEPROM.put(101, pid1.mode);
        break;
      case CHILD_ID::PIDSETPOINT_1:
        pid1.setpoint = value.toFloat();
        EEPROM.put(14, pid1.setpoint);
        break;
      case CHILD_ID::PIDkP0_1:
        pid1.kp = value.toFloat();
        EEPROM.put(18, pid1.kp);
        break;
      case CHILD_ID::PIDkI0_1:
        pid1.ki = value.toFloat();
        EEPROM.put(22, pid1.ki);
        break;
      case CHILD_ID::PIDkD0_1:
        pid1.kd = value.toFloat();
        EEPROM.put(26, pid1.kd);
        break;
      case CHILD_ID::PIDkP1_1:
        pid1.aggKp = value.toFloat();
        EEPROM.put(30, pid1.aggKp);
        break;
      case CHILD_ID::PIDkI1_1:
        pid1.aggKi = value.toFloat();
        EEPROM.put(34, pid1.aggKi);
        break;
      case CHILD_ID::PIDkD1_1:
        pid1.aggKd = value.toFloat();
        EEPROM.put(38, pid1.aggKd);
        break;
      case CHILD_ID::AdaptiveSP_1:
        pid1.aggSP = value.toInt();
        EEPROM.put(70, pid1.aggSP);
        break;
      case CHILD_ID::AdaptiveMode_1:
        pid1.adaptiveMode = value.toInt();
        EEPROM.put(105, pid1.adaptiveMode);
        break;
      case CHILD_ID::PIDMODE_2:
        pid2.mode = value.toInt();
        myPID2.SetMode(pid2.mode ? AUTOMATIC : MANUAL);
        EEPROM.put(102, pid2.mode);
        break;
      case CHILD_ID::PIDSETPOINT_2:
        pid2.setpoint = value.toFloat();
        EEPROM.put(42, pid2.setpoint);
        break;
      case CHILD_ID::PIDkP0_2:
        pid2.kp = value.toFloat();
        EEPROM.put(46, pid2.kp);
        break;
      case CHILD_ID::PIDkI0_2:
        pid2.ki = value.toFloat();
        EEPROM.put(50, pid2.ki);
        break;
      case CHILD_ID::PIDkD0_2:
        pid2.kd = value.toFloat();
        EEPROM.put(54, pid2.kd);
        break;
      case CHILD_ID::PIDkP1_2:
        pid2.aggKp = value.toFloat();
        EEPROM.put(58, pid2.aggKp);
        break;
      case CHILD_ID::PIDkI1_2:
        pid2.aggKi = value.toFloat();
        EEPROM.put(62, pid2.aggKi);
        break;
      case CHILD_ID::PIDkD1_2:
        pid2.aggKd = value.toFloat();
        EEPROM.put(66, pid2.aggKd);
        break;
      case CHILD_ID::AdaptiveSP_2:
        pid2.aggSP = value.toInt();
        EEPROM.put(71, pid2.aggSP);
        break;
      case CHILD_ID::AdaptiveMode_2:
        pid2.adaptiveMode = value.toInt();
        EEPROM.put(104, pid2.adaptiveMode);
        break;
      case CHILD_ID::PIDMODE_3:
        pid3.mode = value.toInt();
        myPID3.SetMode(pid3.mode ? AUTOMATIC : MANUAL);
        EEPROM.put(103, pid3.mode);
        break;
      case CHILD_ID::PIDSETPOINT_3:
        pid3.setpoint = value.toFloat();
        EEPROM.put(72, pid3.setpoint);
        break;
      case CHILD_ID::PIDkP0_3:
        pid3.kp = value.toFloat();
        EEPROM.put(76, pid3.kp);
        break;
      case CHILD_ID::PIDkI0_3:
        pid3.ki = value.toFloat();
        EEPROM.put(80, pid3.ki);
        break;
      case CHILD_ID::PIDkD0_3:
        pid3.kd = value.toFloat();
        EEPROM.put(84, pid3.kd);
        break;
      case CHILD_ID::Curr_CAL:
        calValues.emonCal = value.toFloat();
        EEPROM.put(88, calValues.emonCal);
        break;
      case CHILD_ID::SSRFail_Threshhold:
        calValues.ssrFailThreshold = value.toInt();
        EEPROM.put(96, calValues.ssrFailThreshold);
        break;
      case CHILD_ID::curr_OFFSET:
        calValues.currOffset = value.toFloat();
        EEPROM.put(92, calValues.currOffset);
        break;
      case CHILD_ID::LOAD_MEMORY:
        if (value.toInt()) {
          StoreEEPROM();
        } else {
          getEEPROM();
        }
        break;
      case CHILD_ID::s_debug:
        configValues.sDebug = value.toInt();
        EEPROM.put(106, configValues.sDebug);
        break;
      case CHILD_ID::PID1_Threshold:
        pid1.alarmThreshold = value.toInt();
        EEPROM.put(107, pid1.alarmThreshold);
        break;
      case CHILD_ID::PID2_Threshold:
        pid2.alarmThreshold = value.toInt();
        EEPROM.put(109, pid2.alarmThreshold);
        break;
      case CHILD_ID::PID3_Threshold:
        pid3.alarmThreshold = value.toInt();
        EEPROM.put(111, pid3.alarmThreshold);
        break;
    }
  }
}

void sendInfo(String payload) {
  Serial3.print("INFO: ");
  Serial3.println(payload);
}