#include <Wire.h> // 1
#include <SoftwareSerial.h> // 2 e 3
#include "FS.h" // 4
#include "SD.h" // 4
#include "SPI.h" // 4

/*Modos do Acelerometro*/
const uint8_t POWER_MODE_ADDR = 0x3E;
const uint8_t POWER_NORMAL_MODE = 0x00;
const uint8_t POWER_LOW_MODE = 0X01;
const uint8_t POWER_SUSPEND_MODE = 0X02;
const uint8_t OPERATION_MODE_ADDR = 0x3D;
const uint8_t OPERATION_NDOF = 0x0C;
const uint8_t OPERATION_ACCONLY = 0X01;

/*Endereços dos registradores do Acelerometro*/
const uint8_t ACCEL_DATA_X_LSB_ADDR = 0x08;
const uint8_t LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28;
const uint8_t MAG_DATA_X_LSB_ADDR = 0X0E;
const uint8_t SELFTEST_RESULT_ADDR = 0x36;

/*Constantes do Modulo Comunicacao GSM*/
const int GMS_CONFIRM = 26;

/*EDITAVEIS - TO DO*/
/*Editaveis do Acelerometro*/
const uint8_t I2C_ADDR = 0x29;  //0x29 endereco i2c
const unsigned long BAUD_RATE = 9600; //taxa de trasmissao em bits por segundo (bps)
const uint32_t SETTING_TIME_MS = 500;

/*Editaveis do Modulo Comunicacao GSM*/
const int PORT_UART_RX = 16;
const int PORT_UART_TX = 17;
const String SMS_NUMBER = "+5531998610040"; //Número de telefone que irá receber a mensagem, “ZZ” corresponde ao código telefônico do pais e “XXXXXXXXXXX” corresponde ao número de telefone com o DDD
const String ALERT_TEXT = "Vibracao nociva detectada";
const int MIN_DUR_BETWEEN_ALARM = 30*1000; // duracao entre alertas em ms

/*Editaveis da Comunicacao HTTP*/
String apn = "zap.vivo.com.br"; //APN (Acess Point Name) da operadora do cartao SIM
String url_string = "http://api.thingspeak.com/update?api_key=RN0VM260LCRNDMBR"; //url para envio dos dados
const unsigned long AMOSTRAGEM_RMS = 1000;

/*Editaveis do Cartao SD*/
const unsigned long AMOSTRAGEM_MS = 6; // 166 Hz
const uint32_t TOTAL_TIME = 3600* 10 * 1000; // in ms

/*Editaveis de Metricas*/
const long LIMITE_ACC = 1.1; // m/s^2
const long LIMITE_VDV = 21.0; // m/s^(1.75)

/*Variaveis*/
float accx, accy, accz, LIAx, LIAy, LIAz, GRVx, GRVy, GRVz;

bool alert = 0;
unsigned long last_alert = 0;
unsigned long time_alert = 0;

long current_time, previous_time;
String data_x, data_y, data_z;

uint16_t x,y,z;
uint32_t start_time;
float tempo_s;
uint32_t current_time = 0;  // in ms seconds
uint32_t previous_time = 0; // in ms seconds
uint32_t count = 0; // samples counter
bool done = false; // will be set to true after total time
char dataMessage[50]; // max 50 chars per sample/line

//rms
float sum_x_square = 0;
float x_square, x_rms;
float sum_y_square = 0;
float y_square, y_rms;
float sum_z_square = 0;
float z_square, z_rms;

SoftwareSerial mySerial(PORT_UART_RX, PORT_UART_TX);

void setup() {
  /*TRANSMISSAO*/
  Serial.begin(BAUD_RATE);
  delay(SETTING_TIME_MS);

  /*ACELEROMETRO*/  
  // Inicio protocolo I2C
  Wire.begin();  
  delay(SETTING_TIME_MS);
  // Power On Self Test (POST)
  POST_test();
  // Configuracao sensor
  Sensor_config();

  /*MODULO GSM*/
  mySerial.begin(BAUD_RATE); //Inicializa comunicação entre o SIM800L e o ESP32
  init_WEB();

  /*CARTAO SD*/
  init_SD();
  start_time = millis();
}

void loop() {

  current_time = millis();
  if (!done && current_time - previous_time >= AMOSTRAGEM_MS) {

    /*ACELEROMETRO*/  
    // Leitura sequencial dos sensores (registradores)
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(ACCEL_DATA_X_LSB_ADDR);
    Wire.endTransmission(false); // False will send a restart, keeping the connection active
    Wire.requestFrom(I2C_ADDR, 6, true);
    // Accelerometer
    accx = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
    accy = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2
    accz = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00;  // m/s^2  
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

    /*CARTAO SD*/
    tempo_s = current_time/1000.0;
    sprintf( dataMessage, "%.3f,%i,%i,%i\n" , tempo_s, accx, accy, accz); // build the char array    
    appendFile(SD, "/data.txt", dataMessage);
    count++; // samples counter
    
      if( current_time - start_time >= TOTAL_TIME) {
        Serial.print( "Start time: ");    Serial.println( start_time); 
        Serial.print( "End time: ");      Serial.println( current_time); 
        Serial.print( "Total time (s): ");Serial.println( (current_time - start_time) / 1000.0); 
        Serial.print( "Total samples: "); Serial.println( count);      
        done = true;
        Serial.println("end");
      }     

    /*WEB*/ // falta teste de delay de transmissao
    // Weight


    // Root mean square (RMS) por segundo ou running
    x_square = accx*accx;
    sum_x_square += x_square; 
    x_rms = sqrt( sum_x_square / count);

    y_square = accy*accy;
    sum_y_square += y_square; 
    y_rms = sqrt( sum_y_square / count);

    z_square = accz*accz;
    sum_z_square += z_square; 
    z_rms = sqrt( sum_z_square / count);

    /*
    // Vibration dose value (VDV)
    float x_four = x_square*x_square;
    float sum_x_four += x_four;
    float VDVx = sqrt( sqrt( sum_x_four ));
    */

    if (current_time - previous_time >= AMOSTRAGEM_RMS) {
      data_x = String(x_rms);
      data_y = String(y_rms);
      data_z = String(z_rms);
      EnviaDados(data_x, data_y, data_z);
      sum_x_square = 0; // reset do calculo de rms
      sum_y_square = 0;
      sum_z_square = 0;
    }

    /*SMS*/
    /*
    //Aceleração normalizada na direção I - indicador de saude
    float kx = 1.4;
    float ky = 1.4;
    float kz = 1.0;
    float Ti = 6/1000;
    float T = current_time - start_time;
    float x_squareT = 0;
    float x_squareT = accx*accx*Ti;
    float sum_x_squareT += x_squareT;
    float Axt = kx * sqrt ( sum_x_squareT / T );
    */

    if (x_rms > LIMITE_ACC || y_rms > LIMITE_ACC || z_rms > LIMITE_ACC) {
      alert = 1;    
      time_alert = millis();    
    } else { alert = 0;}

    if (alert && time_alert - last_alert >=  MIN_DUR_BETWEEN_ALARM) {
      init_GMS();
      last_alert = time_alert;
    }

    /*AMOSTRAGEM*/
    previous_time = current_time;

  }

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

void read_GMS() {  
  while (mySerial.available()) //Verifica se a comunicação serial está disponível
  {
    Serial.write(mySerial.read()); //Realiza leitura serial dos dados de entrada ESP32
  }
  delay(500); //Intervalo de 0,5 segundos
}

void init_GMS() {  
  mySerial.println("AT"); //Teste de conexão 
  read_GMS(); //Chamada da função read_GMS()
  
  mySerial.println("AT+CMGF=1"); //Configuração do modo SMS text
  read_GMS(); //Chamada da função read_GMS()  

  mySerial.println("AT+CMGS=\"" + SMS_NUMBER + "\"\r");
  read_GMS(); //Chamada da função read_GMS()
  
  mySerial.print(ALERT_TEXT); //Texto que será enviado para o usúario
  read_GMS(); //Chamada da função read_GMS()
  
  mySerial.write(GMS_CONFIRM); //confirmação das configurações e envio dos dados para comunicação serial (ASCII code of CTRL+Z)
}

void init_WEB() {
  Serial.println("Inicializando HTTP");
  delay(2000);
  Serial.println("OK");
  mySerial.flush();
  Serial.flush();

  //inicializar o serviço GPRS
  mySerial.println("AT+CGATT?");
  delay(100);
  toSerial();

  // Configurações da Portadora
  mySerial.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  delay(2000);
  toSerial();

  // Configurações da Portadora
  mySerial.println("AT+SAPBR=3,1,APN" + apn);
  delay(2000);
  toSerial();

  // Configurações da Portadora
  mySerial.println("AT+SAPBR=0,1");
  delay(2000);
  mySerial.println("AT+SAPBR=1,1");
  delay(2000);
  toSerial();
}

void EnviaDados(String x, String y, String z) {
  // Inicializando o serviço HTTP
  mySerial.println("AT+HTTPINIT");
  delay(2000);
  toSerial();

  // Setando a URL
  String sendURL = url_string + "&field1=" + x + "&field2=" + y + "&field3=" + z;
  mySerial.println("AT+HTTPPARA=URL," + sendURL);
  delay(20000);
  toSerial();

  // Tipo requisição: 0=GET, 1=POST, 2=HEAD
  mySerial.println("AT+HTTPACTION=1");
  delay(6000);
  toSerial();

  // Lendo a resposta do servidor
  mySerial.println("AT+HTTPREAD");
  delay(10000);
  toSerial();

  mySerial.println("");
  mySerial.println("AT+HTTPTERM");
  toSerial();
  delay(300);
}

void toSerial() {
  while (mySerial.available() != 0) {
    Serial.write(mySerial.read());
  }
}

void init_SD() {
  #ifdef REASSIGN_PINS
    SPI.begin(sck, miso, mosi, cs);
    if (!SD.begin(cs)) {
  #else
    if (!SD.begin()) {
  #endif
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);  
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
   
  appendFile(SD, "/data.txt", "Inicio:\n");
  readFile(SD, "/data.txt");  
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
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

float RMS_value( float value,  uint32_t samples) {
  value_square = value*value;
  sum_square += value_square; 
  return value_rms = sqrt( sum_square / samples);
}