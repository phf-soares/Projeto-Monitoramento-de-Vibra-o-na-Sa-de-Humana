#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

/*Enderecos dos Registradores Acelerometro*/
const uint8_t POWER_MODE_ADDR = 0x3E;
const uint8_t OPERATION_MODE_ADDR = 0x3D;
const uint8_t ACC_CONFIG_ADDR = 0x08;
const uint8_t ACCEL_DATA_X_LSB_ADDR = 0x08;
const uint8_t LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28;
const uint8_t MAG_DATA_X_LSB_ADDR = 0X0E;
const uint8_t SELFTEST_RESULT_ADDR = 0x36;
const uint8_t ACC_OFFSET_X_LSB_ADDR = 0x55;
const uint8_t PAGE_ID_ADDR = 0x07;
const uint8_t AXIS_MAP_SIGN_ADDR = 0x42;
//const uint8_t AXIS_MAP_CONFIG_ADDR = 0x41;

/*Valores de Configuracoes Acelerometro*/
const uint8_t POWER_NORMAL_MODE = 0x00;
const uint8_t POWER_LOW_MODE = 0X01;
const uint8_t POWER_SUSPEND_MODE = 0X02;
const uint8_t OPERATION_NDOF = 0x0C;
const uint8_t OPERATION_ACCONLY = 0X01;
const uint8_t ACC_CONFIG = 0b00011101; //normal power, 1000Hz, +-4g
const uint8_t PAGE_ID_0 = 0x00;
const uint8_t PAGE_ID_1 = 0x01;
const uint8_t AXIS_MAP_SIGN = 0b010;
//const uint8_t AXIS_MAP_CONFIG = 0x18;

double accx, accy, accz;

const unsigned long AMOSTRAGEM_MICROS = 1000;
//const unsigned long BAUD_RATE = 115200; //taxa de trasmissao em bits por segundo (bps)
const unsigned long BAUD_RATE = 921600; 
//const unsigned long BAUD_RATE = 2000000;
//const unsigned long BAUD_RATE = 1000000;

const uint8_t I2C_ADDR = 0x29;  //0x29 endereco i2c
const uint32_t SETTING_TIME_MS = 500;

uint32_t start_time;
double tempo_s;
uint32_t current_time = 0;  // in ms seconds
uint32_t previous_time = 0; // in ms seconds
uint32_t count = 0; // samples counter
char dataMessage[50]; // max 50 chars per sample/line

const int AMOSTRAS = 10;
double bufferX[AMOSTRAS];
double bufferY[AMOSTRAS];
double bufferZ[AMOSTRAS];
uint32_t Ts, SumTs, MeanTs;
int coleta, show;
uint32_t time_coleta = 0;

void setup() {
  // Transmissao
  Serial.begin(BAUD_RATE);
  delay(SETTING_TIME_MS);

  // Inicio protocolo I2C
  Wire.begin();  
  delay(SETTING_TIME_MS);

  // Power On Self Test (POST)
  POST_test();

  // Configuracao sensor
  init_SENSOR();

  start_time = micros();
  Serial.println("SAIU SETUP");
}

void loop() {
  /*
  //Serial.println("LOOP");
  current_time = micros();

  if (current_time - previous_time >= AMOSTRAGEM_MICROS) {
  //Serial.println("IF");  
  // Accelerometer
  for (int coleta=0; coleta < AMOSTRAS; coleta++) {
    //Serial.println("FOR I");
    // Leitura sequencial dos sensores (registradores)
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(ACCEL_DATA_X_LSB_ADDR);
    Wire.endTransmission(false); // False will send a restart, keeping the connection active
    Wire.requestFrom(I2C_ADDR, 6, true);
    bufferX[coleta] = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;
    bufferY[coleta] = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;
    bufferZ[coleta] = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;
    time_coleta = micros();
    Ts = time_coleta - current_time;
    Serial.print(" Ts= ");  Serial.println(Ts/1000.00);
    SumTs += Ts;    
  }
  coleta = 0;
  MeanTs = SumTs/(AMOSTRAS*1000);

  for (int show=0; show < AMOSTRAS; show++) {
    //Serial.print("FOR J");
    Serial.print(" Accel_X= ");  Serial.print(bufferX[show]);
    Serial.print(" Accel_Y= ");  Serial.print(bufferY[show]);
    Serial.print(" Accel_Z= ");  Serial.println(bufferZ[show]);
    Serial.print(" MeanTs= ");  Serial.println(MeanTs);
    delay(MeanTs);    
  }
  show = 0;

  previous_time = current_time;
  }
  */

  // Leitura sequencial dos sensores (registradores)
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(ACCEL_DATA_X_LSB_ADDR);
    Wire.endTransmission(false); // False will send a restart, keeping the connection active
    Wire.requestFrom(I2C_ADDR, 6, true);
    accx = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;
    accy = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;
    accz = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;
    Serial.print(" Accel_X= ");  Serial.print(accx);
    Serial.print(" Accel_Y= ");  Serial.print(accy);
    Serial.print(" Accel_Z= ");  Serial.println(accz);
    delay(1);


}

void POST_test() {  
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(SELFTEST_RESULT_ADDR);
  Wire.endTransmission(false);
  Wire.requestFrom(I2C_ADDR, 1, true);
  uint8_t POST_result = Wire.read();
  bool bit0_ACC_result = ((POST_result>>0)&1);
  if (bit0_ACC_result == 0) {
    Serial.print("ACELEROMETRO FALHOU");
    // mandar alerta SMS;
    delay(SETTING_TIME_MS);
    exit(0);
  } else
  delay(SETTING_TIME_MS);
}

void init_SENSOR() {
  //page 0
  //readI2C(PAGE_ID_ADDR,"PAGE_ID");
  //Serial.println("Expected: 0"); 
  //power mode
  writeI2C(POWER_MODE_ADDR, POWER_NORMAL_MODE);
  readI2C(POWER_MODE_ADDR,"POWER_MODE");
  Serial.println("Expected: 0");
  //orientar sinais dos eixos
  //writeI2C(AXIS_MAP_SIGN_ADDR, AXIS_MAP_SIGN);
  readI2C(AXIS_MAP_SIGN_ADDR,"AXIS_MAP_SIGN");
  //Serial.println("Expected: 10");
  //offsets
    /*
    writeI2C(0x59, 0x00);
    readI2C(0x59, "Z_OFF_LSB");
    Serial.println("Expected: 0"); 
    writeI2C(0x5A, 0x00);
    readI2C(0x5A, "Z_OFF_MSB");
    Serial.println("Expected: 0"); 
    */
  //trocar para page 1
  writeI2C(PAGE_ID_ADDR, PAGE_ID_1);
  readI2C(PAGE_ID_ADDR,"PAGE_ID");
  Serial.println("Expected: 1"); 
  //acc config
  writeI2C(ACC_CONFIG_ADDR, ACC_CONFIG);
  readI2C(ACC_CONFIG_ADDR,"ACC_CONFIG");
  Serial.println("Expected: 11101"); 
  //trocar de volta para page 0
  writeI2C(PAGE_ID_ADDR, PAGE_ID_0);
  readI2C(PAGE_ID_ADDR,"PAGE_ID");
  Serial.println("Expected: 0"); 
  //modo operacao sensor
    /*
    writeI2C(OPERATION_MODE_ADDR, OPERATION_NDOF);
    readI2C(OPERATION_MODE_ADDR,"OPERATION_MODE");
    Serial.println("Expected: 1100");
    */
  //writeI2C(OPERATION_MODE_ADDR, OPERATION_NDOF);
  writeI2C(OPERATION_MODE_ADDR, OPERATION_ACCONLY);
  readI2C(OPERATION_MODE_ADDR,"OPERATION_MODE");
  Serial.println("Expected: 1, ACCONLY");
  //Serial.println("Expected: 1100, NDOF");
  delay(5000);

  /*
  // PAGE 0
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x59); //Z-OFF-LSB
  Wire.write(0x01);
  Wire.endTransmission();
  delay(SETTING_TIME_MS);
  
  // PAGE 0
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(0x5A); //Z-OFF-MSB
  Wire.write(0x01);
  Wire.endTransmission();
  delay(SETTING_TIME_MS);
  
  // PAGE 0
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(ACC_OFFSET_X_LSB_ADDR);
  Wire.endTransmission(false); // False will send a restart, keeping the connection active
  Wire.requestFrom(I2C_ADDR, 6, true);  
  double xoff = (int16_t)(Wire.read() | Wire.read() << 8);  // m/s^2
  double yoff = (int16_t)(Wire.read() | Wire.read() << 8);  // m/s^2
  double zoff = (int16_t)(Wire.read() | Wire.read() << 8);
  Serial.print("X_OFF= ");     Serial.print(xoff,10);
  Serial.print(" Y_OFF= ");     Serial.print(yoff,10);
  Serial.print(" Z_OFF= ");     Serial.println(zoff,10);
  */
}

void writeI2C( const uint8_t registerAddr, const uint8_t writeConfig ) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(registerAddr);  
  Wire.write(writeConfig);
  Wire.endTransmission();
  delay(SETTING_TIME_MS);
}

void readI2C( const uint8_t registerAddr, const String mode) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(registerAddr);
  Wire.endTransmission(false); // False will send a restart, keeping the connection active
  Wire.requestFrom(I2C_ADDR, 1, true);
  uint8_t registerContent = Wire.read();
  Serial.print(mode);
  Serial.print(": ");
  Serial.println(registerContent,BIN);
}

void appendFile(fs::FS &fs, const char *path, const char *message) {  
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {    
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {    
  } else {
    Serial.println("Write failed");
  }
  file.close();
}
