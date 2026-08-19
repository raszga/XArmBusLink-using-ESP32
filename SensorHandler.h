

#ifndef SENSORHANDLER_H
#define SENSORHANDLER_H

#include <Arduino.h>
#include "aux_.h"
#include <Wire.h>
#include "esp32-hal-gpio.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
//#include "I2C_Sensors.h"

#define CONVERSIONS_PER_PIN 10
//#define SDA 21
//#define SCL 22

// 1. Basic System Globals
extern std::vector<int> iS;
extern const uint8_t max_iS;
extern uint8_t adc_pins[];
extern uint8_t adc_pins_count;
extern volatile bool adc_coversion_done;
extern uint8_t digi_pins[];
extern uint8_t digi_pins_count;
extern uint8_t sensor_count;
extern String deviceKey;
extern unsigned long int pulsarTime;
extern adc_continuous_data_t* result;
extern String outString;

// 2. Type Definitions (Must come before Sensor struct)
typedef int (*SensorReadFunction)(void* arg);
typedef bool (*SensorInitFunction)(void* arg);

// 3. The Sensor Structure (Must come before extern instances)
struct Sensor {
  String Name = "-1XXX";
  bool isBooted = false;
  int Pin = -1;
  int Set = INPUT_PULLUP;
  int Norm = 1;
  bool analogue = false;
  int Vmin = 0;
  int Vmax = 1023;
  int V = -1;
  int Vbatt = -2;
  int Pow = -1;
  int normPow = 0;
  bool active = false;
  bool soft = false;
  bool softStage = true;
  String sMSG = "-1XXX";
  String Name0 = Name;
  bool i2c = false;
  uint8_t i2cAddress = 0x00;
  bool Publish = true;

  SensorReadFunction pointerRead = nullptr;
  SensorInitFunction pointerInit = nullptr;

  void Init();
  void Reset();
  bool Check();
  void Read();
  void ReadBatt();
  void PowON();
  void PowOFF();
  void Activate();
  void deActivate();
  String Response();
};

// 4. NOW we can declare the Extern Sensor Objects
extern Sensor Analog32;
extern Sensor Analog33;
extern Sensor Analog34;
extern Sensor Analog35;
extern Sensor Analog36;
extern Sensor Analog39;

extern Sensor Digi5;
extern Sensor Digi12;
extern Sensor Digi13;
extern Sensor Digi14;
extern Sensor Digi16;
extern Sensor Digi17;
extern Sensor Digi18;
extern Sensor Digi19;
extern Sensor Digi23;
extern Sensor Digi25;
extern Sensor Digi26;
extern Sensor Digi27;

// New BMP280 Sensors
extern Sensor BMP280_Temp;
extern Sensor BMP280_Press;

extern Sensor* AllSensors[];
extern uint8_t sensor_count;
extern bool dbg;

// 5. Function Prototypes
void ARDUINO_ISR_ATTR adcComplete();
long int ESP32getChipId();
String readAnalogs(bool sh = false, bool st = false);
String updateESP32Analogs();
void generateDeviceKey();
void initSensors();
void checkSensors();
void respondSensors();
void ChkI2CSensHardw();
bool I2C_deviceExists(uint8_t address);
bool checkProto(void* arg);
bool CheckPulsar(void* arg);
int CheckPulsarRead(void* arg);

#endif