#include <Wire.h>

#define BNO055_ADDRESS 0x29
#define BNO055_INT_PIN 15

volatile bool dataReady = false;

void IRAM_ATTR onDataReady() {
  dataReady = true;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(BNO055_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BNO055_ADDRESS, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void setPage(uint8_t page) {
  writeRegister(0x07, page); // PAGE_ID
}

void setMode(uint8_t mode) {
  writeRegister(0x3D, 0x00); // CONFIG mode
  delay(25);
  writeRegister(0x3D, mode);
  delay(25);
}

void checkRegister(const char* name, uint8_t reg, uint8_t value, uint8_t expected) {
  Serial.print(name);
  Serial.print(" (0x");
  Serial.print(reg, HEX);
  Serial.print("): 0x");
  Serial.print(value, HEX);
  if (value == expected) {
    Serial.println(" [OK]");
  } else {
    Serial.print(" [ERRO] esperado: 0x");
    Serial.println(expected, HEX);
  }
}

void printSensorConfig() {
  Serial.println("\n--- BNO055 CONFIG CHECK ---");

  uint8_t page_id = readRegister(0x07);
  uint8_t opr_mode = readRegister(0x3D);
  uint8_t acc_config = readRegister(0x08);
  uint8_t int_en = 0, int_msk = 0, sys_trigger = 0, int_sta = 0;

  setPage(1);
  int_en = readRegister(0x10);
  int_msk = readRegister(0x0F);
  setPage(0);

  sys_trigger = readRegister(0x3F);
  int_sta = readRegister(0x37);

  checkRegister("PAGE_ID", 0x07, page_id, 0x00);
  checkRegister("OPR_MODE", 0x3D, opr_mode, 0x01);
  checkRegister("ACC_CONFIG", 0x08, acc_config, 0x1C);
  checkRegister("INT_EN", 0x10, int_en, 0x01);
  checkRegister("INT_MSK", 0x0F, int_msk, 0x01);

  // SYS_TRIGGER pode ser 0x00 ou 0x40 (RST_INT set/reset), aceitamos ambos como OK
  Serial.print("SYS_TRIGGER (0x3F): 0x");
  Serial.print(sys_trigger, HEX);
  if (sys_trigger == 0x00 || sys_trigger == 0x40) {
    Serial.println(" [OK]");
  } else {
    Serial.println(" [ERRO] esperado: 0x00 ou 0x40");
  }

  Serial.print("INT_STA (0x37): 0x");
  Serial.println(int_sta, HEX);
  // INT_STA muda dinamicamente, pode ser 0x01 ou outro valor dependendo do momento

  Serial.println("----------------------------\n");
  delay(5000);
}

float alpha = 0.1; // coeficiente EMA (0.0 a 1.0)
float filteredAx = 0, filteredAy = 0, filteredAz = 0;
bool firstSample = true;

void addSampleEMA(float ax, float ay, float az, float &outAx, float &outAy, float &outAz) {
  if (firstSample) {
    filteredAx = ax;
    filteredAy = ay;
    filteredAz = az;
    firstSample = false;
  } else {
    filteredAx = alpha * ax + (1 - alpha) * filteredAx;
    filteredAy = alpha * ay + (1 - alpha) * filteredAy;
    filteredAz = alpha * az + (1 - alpha) * filteredAz;
  }
  outAx = filteredAx;
  outAy = filteredAy;
  outAz = filteredAz;
}

void setup() {
  Serial.begin(921600);
  Wire.begin();
  Wire.setClock(400000);

  delay(500);

  writeRegister(0x3E, 0x00); // Normal Mode
  delay(50);

  setPage(1);
  writeRegister(0x10, 0x01); // INT_EN: ACC_BSX_DRDY (bit 0)
  writeRegister(0x0F, 0x01); // INT_MSK: encaminha ao pino INT
  writeRegister(0x08, 0x1C); // ACC_CONFIG: ±2g, 1000Hz, normal mode // 0b11100
  setPage(0);  

  setMode(0x01); // ACCONLY
  delay(50);

  pinMode(BNO055_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BNO055_INT_PIN), onDataReady, RISING);

  printSensorConfig();
}

unsigned long lastStats = 0;
uint32_t sampleCount = 0;

void loop() {
  uint8_t int_sta = readRegister(0x7);  
  if (int_sta & 0x01) {
    dataReady = false;

    uint8_t data[6];
    Wire.beginTransmission(BNO055_ADDRESS);
    Wire.write(0x08);
    Wire.endTransmission(false);
    Wire.requestFrom(BNO055_ADDRESS, (uint8_t)6);
    for (int i = 0; i < 6 && Wire.available(); i++) {
      data[i] = Wire.read();
    }

    int16_t ax = (int16_t)((data[1] << 8) | data[0]);
    int16_t ay = (int16_t)((data[3] << 8) | data[2]);
    int16_t az = (int16_t)((data[5] << 8) | data[4]);

    float fax = ax * 0.00981;
    float fay = ay * 0.00981;
    float faz = az * 0.00981;

    float filteredAx, filteredAy, filteredAz;
    addSampleEMA(fax, fay, faz, filteredAx, filteredAy, filteredAz);

    uint32_t t_us = micros();

    Serial.print(t_us);
    Serial.print('\t');
    Serial.print(filteredAx, 6);
    Serial.print('\t');
    Serial.print(filteredAy, 6);
    Serial.print('\t');
    Serial.println(filteredAz, 6);

    sampleCount++;

    // Resetar o pino de interrupção no BNO055
    writeRegister(0x3F, 0x40);
  }

  if (millis() - lastStats >= 1000) {
    Serial.print("Samples/sec: ");
    Serial.println(sampleCount);
    sampleCount = 0;
    lastStats = millis();
  }
}
