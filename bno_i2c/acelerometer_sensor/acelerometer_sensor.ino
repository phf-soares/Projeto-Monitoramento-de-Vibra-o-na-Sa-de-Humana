#include <Wire.h>

const uint8_t POWER_MODE_ADDR = 0x3E;
const uint8_t POWER_NORMAL_MODE = 0x00;
const uint8_t POWER_LOW_MODE = 0X01;
const uint8_t POWER_SUSPEND_MODE = 0X02;
const uint8_t OPERATION_MODE_ADDR = 0x3D;
const uint8_t OPERATION_NDOF = 0x0C;
const uint8_t Operation_ACCONLY = 0X01;
const uint8_t ACCEL_DATA_X_LSB_ADDR = 0x08;
const uint8_t LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28;
const uint8_t MAG_DATA_X_LSB_ADDR = 0X0E;
const uint8_t SELFTEST_RESULT_ADDR = 0x36;

float Yaw, Roll, Pitch, magx, magy, magz, accx, accy, accz, gyrox, gyroy, gyroz, q0, q1, q2, q3, Roll2, Pitch2, Yaw2, LIAx, LIAy, LIAz, GRVx, GRVy, GRVz;

const uint8_t I2C_ADDR = 0x29;  //0x29 endereco i2c
const unsigned long BAUD_RATE = 9600; //taxa de trasmissao em bits por segundo (bps)
const uint32_t SETTING_TIME_MS = 500;

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
  Sensor_config();
}

void loop() {  
  // Leitura sequencial dos sensores (registradores)
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(ACCEL_DATA_X_LSB_ADDR);
  Wire.endTransmission(false); // False will send a restart, keeping the connection active
  Wire.requestFrom(I2C_ADDR, 6, true);
  // Accelerometer
  accx = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
  accy = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
  accz = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
  
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(MAG_DATA_X_LSB_ADDR);
  Wire.endTransmission(false);
  Wire.requestFrom(I2C_ADDR, 26, true);
  // Magnetometer
  magx = (int16_t)(Wire.read() | Wire.read() << 8) / 16.00;  // mT
  magy = (int16_t)(Wire.read() | Wire.read() << 8) / 16.00;  // mT
  magz = (int16_t)(Wire.read() | Wire.read() << 8) / 16.00;  // mT
  // Gyroscope
  gyrox = (int16_t)(Wire.read() | Wire.read() << 8) / 16.00;  // Dps
  gyroy = (int16_t)(Wire.read() | Wire.read() << 8) / 16.00;  // Dps
  gyroz = (int16_t)(Wire.read() | Wire.read() << 8) / 16.00;  // Dps
  // Euler Angles
  Yaw = (int16_t)(Wire.read() | Wire.read() << 8) / 16.00;    //in Degrees unit
  Roll = (int16_t)(Wire.read() | Wire.read() << 8) / 16.00;   //in Degrees unit
  Pitch = (int16_t)(Wire.read() | Wire.read() << 8) / 16.00;  //in Degrees unit
  // Quaternions
  q0 = (int16_t)(Wire.read() | Wire.read() << 8) / (pow(2, 14));  //unit less
  q1 = (int16_t)(Wire.read() | Wire.read() << 8) / (pow(2, 14));  //unit less
  q2 = (int16_t)(Wire.read() | Wire.read() << 8) / (pow(2, 14));  //unit less
  q3 = (int16_t)(Wire.read() | Wire.read() << 8) / (pow(2, 14));  //unit less
  //Convert Quaternions to Euler Angles
  Yaw2 = (atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (pow(q2, 2) + pow(q3, 2)))) * 180 / PI;
  Roll2 = (asin(2 * (q0 * q2 - q3 * q1))) * 180 / PI;
  Pitch2 = (atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (pow(q1, 2) + pow(q2, 2)))) * 180 / PI;

  //Linear (Dynamic) & Gravitational (Static) Acceleration
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(LINEAR_ACCEL_DATA_X_LSB_ADDR);
  Wire.endTransmission(false);
  Wire.requestFrom(I2C_ADDR, 12, true);
  LIAx = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
  LIAy = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
  LIAz = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
  GRVx = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
  GRVy = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
  GRVz = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2

  // Print data
  Serial.print("Accel_X= ");
  Serial.print(accx);
  Serial.print(" Accel_Y= ");
  Serial.print(accy);
  Serial.print(" Accel_Z= ");
  Serial.println(accz);
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

void Sensor_config() {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(POWER_MODE_ADDR);
  Wire.write(POWER_NORMAL_MODE);
  Wire.endTransmission();
  delay(SETTING_TIME_MS);

  Wire.beginTransmission(I2C_ADDR);
  Wire.write(OPERATION_MODE_ADDR);
  Wire.write(OPERATION_NDOF);
  Wire.endTransmission();
  delay(SETTING_TIME_MS);
}
