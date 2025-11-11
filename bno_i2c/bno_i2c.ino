#include <Wire.h>

const int Power_mode_addr = 0x3E;
const int Power_normal_mode = 0x00;
const int Power_low_mode = 0X01;
const int Power_suspend_mode = 0X02;
const int Operation_mode_addr = 0x3D;
const int Operation_NDOF = 0x0C;
const int Operation_ACCONLY = 0X01;
const int ACCEL_DATA_X_LSB_ADDR = 0x08;
const int LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28;

float Yaw, Roll, Pitch, magx, magy, magz, accx, accy, accz, gyrox, gyroy, gyroz, q0, q1, q2, q3, Roll2, Pitch2, Yaw2, LIAx, LIAy, LIAz, GRVx, GRVy, GRVz;

const int I2C_addr = 0x29;  //0x29 endereco i2c
const long Baud_rate = 115200; //taxa de trasmissao em bits por segundo (bps)
const unsigned long Setting_time_ms = 100;

void setup() {

  // Inicio protocolo I2C
  Wire.begin();
  delay(Setting_time_ms);

  // Configuracao sensor
  Wire.beginTransmission(I2C_addr);
  Wire.write(Power_mode_addr);
  Wire.write(Power_normal_mode);
  Wire.endTransmission();
  delay(Setting_time_ms);

  Wire.beginTransmission(I2C_addr);
  Wire.write(Operation_mode_addr);
  Wire.write(Operation_NDOF);
  Wire.endTransmission();
  delay(Setting_time_ms);

  // Transmissao
  Serial.begin(Baud_rate);
  delay(Setting_time_ms);
}

void loop() {

  // Leitura sequencial dos sensores (registradores)
  Wire.beginTransmission(I2C_addr);
  Wire.write(ACCEL_DATA_X_LSB_ADDR);
  Wire.endTransmission(false); // False will send a restart, keeping the connection active
  Wire.requestFrom(I2C_addr, 32, true);
  // Accelerometer
  accx = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
  accy = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
  accz = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
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

  //Linear (Dynamic) & Gravitational (static) Acceleration
  Wire.beginTransmission(I2C_addr);
  Wire.write(LINEAR_ACCEL_DATA_X_LSB_ADDR);
  Wire.endTransmission(false);
  Wire.requestFrom(I2C_addr, 12, true);
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
