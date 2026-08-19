
#include "SensorHandler.h"

const uint8_t max_iS = 40;
volatile bool adc_coversion_done = false;
adc_continuous_data_t* result = NULL;
String outString = "";
std::vector<int> iS(max_iS, -1);

// DEFINITIONS OF ARRAYS (Fixes Undefined Reference)
uint8_t adc_pins[] = { 32, 33, 34, 35, 36, 39 };
uint8_t adc_pins_count = sizeof(adc_pins) / sizeof(uint8_t);

uint8_t digi_pins[] = { 4, 5, 12, 13, 14, 18, 19, 23, 25, 26, 27 };
uint8_t digi_pins_count = sizeof(digi_pins) / sizeof(uint8_t);

String deviceKey = "";
unsigned long int pulsarTime = millis();

// --- SENSOR DEFINITIONS ---

Sensor Analog32 = { .Name = "Front_FW0_RW", .Pin = 32, .analogue = true, .Vmin = 0, .Vmax = 4095, .Publish = true };
Sensor Analog33 = { .Name = "Front_RTH_LFT0", .Pin = 33, .analogue = true, .Vmin = 0, .Vmax = 4095, .Publish = true };
Sensor Analog34 = { .Name = "Rigth_FW0_RW", .Pin = 34, .analogue = true, .Vmin = 00, .Vmax = 4095, .Publish = true };
Sensor Analog35 = { .Name = "Rigth_RTH0_LFT", .Pin = 35, .analogue = true, .Vmin = 00, .Vmax = 4095, .Publish = true };
Sensor Analog36 = { .Name = "LeftFW0_RW", .Pin = 36, .analogue = true, .Vmin = 00, .Vmax = 4095, .Publish = true };
Sensor Analog39 = { .Name = "Left_LFT_RTH0", .Pin = 39, .analogue = true, .Vmin = 00, .Vmax = 4095, .Publish = true };

Sensor Digi4 = { .Name = "TogleRigth", .Pin = 4, .Set = INPUT_PULLUP, .Norm = 1, .analogue = false, .Publish = true };
Sensor Digi5 = { .Name = "PressLeft", .Pin = 5, .Set = INPUT_PULLUP, .Norm = 1, .analogue = false, .Publish = true };
Sensor Digi12 = { .Name = "PressFront", .Pin = 12, .Set = INPUT_PULLUP, .Norm = 1, .analogue = false, .Publish = true };
Sensor Digi13 = { .Name = "Digi13", .Pin = 13, .Set = INPUT_PULLUP, .Norm = 1, .analogue = false, .Publish = true };
Sensor Digi14 = { .Name = "TRB", .Pin = 14, .Set = INPUT_PULLUP, .Norm = 1, .analogue = false, .Publish = true };  // dbg
Sensor Digi18 = { .Name = "PressRigth", .Pin = 18, .Set = INPUT_PULLUP, .Norm = 1, .analogue = false, .Publish = false };
Sensor Digi19 = { .Name = "Conf", .Pin = 19, .Set = INPUT_PULLUP, .Norm = 1, .analogue = false, .Publish = true };
Sensor Digi23 = { .Name = "Digi23", .Pin = 23, .Set = INPUT_PULLUP, .Norm = 1, .analogue = false, .Publish = true };
Sensor Digi25 = { .Name = "TRA", .Pin = 25, .Set = INPUT_PULLUP, .Norm = 1, .analogue = false, .Publish = true };
Sensor Digi26 = { .Name = "Push", .Pin = 26, .Set = INPUT_PULLUP, .Norm = 1, .analogue = false, .Publish = true };
Sensor Digi27 = { .Name = "Back", .Pin = 27, .Set = INPUT_PULLUP, .Norm = 1, .analogue = false, .Publish = true };

Sensor* AllSensors[] = {
  &Analog32,
  &Analog33,
  &Analog34,
  &Analog35,
  &Analog36,
  &Analog39,
  &Digi4,
  &Digi5,
  &Digi12,
  &Digi13,
  &Digi14,
  &Digi18,
  &Digi19,
  &Digi23,
  &Digi25,
  &Digi26,
  &Digi27,
  nullptr
};

uint8_t sensor_count = sizeof(AllSensors) / sizeof(uint8_t);

void generateDeviceKey() {
  uint64_t mac = ESP.getEfuseMac();
  deviceKey = String((uint32_t)(mac & 0xFFFFFF));
}

void ARDUINO_ISR_ATTR adcComplete() {
  adc_coversion_done = true;
}

void Sensor::Init() {
  if (!isBooted) {
    Name0 = Name;
    Name += " " + deviceKey;
    isBooted = true;
  }
  V = -1;
  if (pointerInit != nullptr) {
    pointerInit(i2c ? (void*)&i2cAddress : (void*)&Vmax);
  }
  if (!i2c && !soft) {
    if (Pow != -1) {
      pinMode(Pow, OUTPUT);
      digitalWrite(Pow, normPow);
    }
    if (!analogue) pinMode(Pin, Set);
  }
  active = false;
}

void Sensor::Read() {
  readAnalogs();
  if (!active) {
    V = -1;
    return;
  }
  if (pointerRead != nullptr) {
    V = pointerRead(i2c ? (void*)&i2cAddress : (void*)&Vmax);
    return;
  }
  if (Pin >= 0 && Pin < max_iS) V = iS[Pin];
}

bool Sensor::Check() {
  Read();
  if (i2c || analogue) return ((Vmin <= V) && (V <= Vmax));
  if (soft) return softStage;
  return (V == Norm);
}

String Sensor::Response() {
  Read();
  bool isOk = Check();
  buildMessage(sMSG, Name.c_str(), V, isOk, Publish);
  return sMSG;
}

void Sensor::Activate() {
  active = true;
}
void Sensor::deActivate() {
  active = false;
}
void Sensor::PowON() {
  if (active && Pow != -1) digitalWrite(Pow, !normPow);
}
void Sensor::PowOFF() {
  if (active && Pow != -1) digitalWrite(Pow, normPow);
}
void Sensor::Reset() {
  Name = Name0;
  active = false;
  V = -1;
}
/*****************************************************************************/
String readAnalogs(bool sh, bool st) {
  for (int i = 0; i < digi_pins_count; i++) {
    int dPin = digi_pins[i];
    iS[dPin] = digitalRead(dPin);
  }
  if (adc_coversion_done == true) {
    adc_coversion_done = false;
    if (analogContinuousRead(&result, 0)) {
      outString = "";
      for (int i = 0; i < adc_pins_count; i++) {
        int aPin = adc_pins[i];
        iS[aPin] = result[i].avg_read_raw;
        if (st) { outString += (i == 0 ? "*," : ",") + String(iS[aPin]); }
      }
      if (st) {
        for (int j = 0; j < digi_pins_count; j++) {
          outString += "," + String(iS[digi_pins[j]]);
        }
      }
    }
  }
  if (sh) {
    Serial.println("ReadAnalogsResults===========================");
    for (int i = 0; i < max_iS; i++) {
      if (iS[i] != -1) {
        Serial.print(i);
        Serial.print("...");
        Serial.println(iS[i]);
      }
    }
  }
  return outString;
}
/*****************************************************************************/
void initSensors() {
  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_11db);
  analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, 20000, &adcComplete);
  analogContinuousStart();
  for (int i = 0; i < digi_pins_count; i++) pinMode(digi_pins[i], INPUT_PULLUP);
  for (int i = 0; AllSensors[i] != nullptr; i++) {
    AllSensors[i]->Init();
    AllSensors[i]->Activate();
  }
}
/*****************************************************************************/
void checkSensors() {
  for (int i = 0; AllSensors[i] != nullptr; i++) AllSensors[i]->Check();
}
void respondSensors() {
  for (int i = 0; AllSensors[i] != nullptr; i++) AllSensors[i]->Response();
}
/*****************************************************************************/
bool I2C_deviceExists(uint8_t address) {
  Wire.begin(SDA, SCL);
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}
/*****************************************************************************/
void ChkI2CSensHardw() {
  int activeCount = 0;
  int totalSensors = 0;
  while (AllSensors[totalSensors] != nullptr) {
    totalSensors++;
  }
  for (int i = 0; i < totalSensors; i++) {
    if (AllSensors[i] == nullptr) continue;

    if (AllSensors[i]->i2c) {
      AllSensors[i]->active = I2C_deviceExists(AllSensors[i]->i2cAddress);
    } else {
      AllSensors[i]->active = true;
    }
  }
  /*****************************************************************************/
  for (int i = 0; i < totalSensors; i++) {
    if (AllSensors[i] != nullptr && AllSensors[i]->active) {
      Sensor* temp = AllSensors[activeCount];
      AllSensors[activeCount] = AllSensors[i];
      AllSensors[i] = temp;
      activeCount++;
    }
  }
  for (int i = activeCount; i < totalSensors; i++) {
    AllSensors[i] = nullptr;
  }
}
/*****************************************************************************/
int CheckPulsarRead(void* arg) {
  return 1;
}
/*****************************************************************************/