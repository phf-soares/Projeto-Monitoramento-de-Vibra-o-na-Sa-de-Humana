#include <Wire.h> // 1
#include <SoftwareSerial.h> // 2 e 3
#include "FS.h" // 4
#include "SD.h" // 4
#include "SPI.h" // 4
#include "math.h"

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

typedef struct {
    double b0, b1, b2; // Feedforward coefficients
    double a1, a2;     // Feedback coefficients
    double z1, z2;     // Internal state variables (delays)
} IIR_Biquad_Filter;

typedef struct {
    double sum_square, sum_four;
    double max;
    double factorK;  
} Acc_Metric;

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

/*Constantes do Modulo Comunicacao GSM*/
const int GMS_CONFIRM = 26;

/*Editaveis de Metricas*/
const double FREQ_AMOSTRAGEM_SENSOR = 250.0; // Hz
const int AMOSTRAS_SEC = int(FREQ_AMOSTRAGEM_SENSOR);
uint32_t TRANSIENT_FILTER_TIME = int(FREQ_AMOSTRAGEM_SENSOR)*10; // 10 sec de amostras
const unsigned int RESET = 0;

/*Coeficientes do Filtro*/
const double Q1 = 1/sqrt(2);
const double Q2 = 1/sqrt(2);
const double Q4 = 0.63;
const double Q5 = 0.91;
const double Q6 = 0.91;

const double F1 = 0.4;
const double F2 = 100;
const double F3WD = 2;
const double F4WD = 2;
const double F3WK = 12.5;
const double F4WK = 12.5;
const double F5 = 2.37;
const double F6 = 3.3;

const double W1 = 2*tan(M_PI*F1/FREQ_AMOSTRAGEM_SENSOR);
const double W2 = 2*tan(M_PI*F2/FREQ_AMOSTRAGEM_SENSOR);
const double W3WD = 2*tan(M_PI*F3WD/FREQ_AMOSTRAGEM_SENSOR);
const double W4WD = 2*tan(M_PI*F4WD/FREQ_AMOSTRAGEM_SENSOR);
const double W3WK = 2*tan(M_PI*F3WK/FREQ_AMOSTRAGEM_SENSOR);
const double W4WK = 2*tan(M_PI*F4WK/FREQ_AMOSTRAGEM_SENSOR);
const double W5 = 2*tan(M_PI*F5/FREQ_AMOSTRAGEM_SENSOR);
const double W6 = 2*tan(M_PI*F6/FREQ_AMOSTRAGEM_SENSOR);

// High Pass
const double A0H = 4*Q1 + 2*W1 + W1*W1;
const double A1H = (2*W1*W1 - 8*Q1)/A0H;
const double A2H = (4*Q1 - 2*W1 + W1*W1)/A0H;

const double B0H = (4*Q1)/A0H;
const double B1H = (-8*Q1)/A0H;
const double B2H = (4*Q1)/A0H;

// Low Pass
const double A0L = 4*Q2 + 2*W2 + W2*W2*Q2;
const double A1L = (2*W2*W2*Q2 - 8*Q2)/A0L;
const double A2L = (4*Q2 - 2*W2 + W2*W2*Q2)/A0L;

const double B0L = (W2*W2*Q2)/A0L;
const double B1L = (2*W2*W2*Q2)/A0L;
const double B2L = (W2*W2*Q2)/A0L;

// Transition Filter for Wd
const double A0TWD = 4*Q4 + 2*W4WD + W4WD*W4WD*Q4;
const double A1TWD = (2*W4WD*W4WD*Q4 - 8*Q4)/A0TWD;
const double A2TWD = (4*Q4 - 2*W4WD + W4WD*W4WD*Q4)/A0TWD;

const double B0TWD = (W4WD*W4WD*Q4 + 2*(Q4*W4WD*W4WD)/W3WD)/A0TWD;
const double B1TWD = (2*W4WD*W4WD*Q4)/A0TWD;
const double B2TWD = (W4WD*W4WD*Q4 - 2*(Q4*W4WD*W4WD)/W3WD)/A0TWD;

// Transition Filter for Wk
const double A0TWK = 4*Q4 + 2*W4WK + W4WK*W4WK*Q4;
const double A1TWK = (2*W4WK*W4WK*Q4 - 8*Q4)/A0TWK;
const double A2TWK = (4*Q4 - 2*W4WK + W4WK*W4WK*Q4)/A0TWK;

const double B0TWK = (W4WK*W4WK*Q4 + 2*(Q4*W4WK*W4WK)/W3WK)/A0TWK;
const double B1TWK = (2*W4WK*W4WK*Q4)/A0TWK;
const double B2TWK = (W4WK*W4WK*Q4 - 2*(Q4*W4WK*W4WK)/W3WK)/A0TWK;

// Step Filter
const double A0S = (4*Q6 + 2*W6 + W6*W6*Q6)/Q5;
const double A1S = ((2*W6*W6*Q6 - 8*Q6)/Q5)/A0S;
const double A2S = ((4*Q6 - 2*W6 + W6*W6*Q6)/Q5)/A0S;

const double B0S = ((4*Q5 + 2*W5+ W5*W5*Q5)/Q6)/A0S;
const double B1S = ((2*W5*W5*Q5 - 8*Q5)/Q6)/A0S;
const double B2S = ((4*Q5 - 2*W5 + W5*W5*Q5)/Q6)/A0S;

/*Coeficientes de Metricas*/
const double VDL_USEFUL = 9.0;
const double LIMITE_ACC_EAV = 0.5; // m/s^2
const double LIMITE_VDV_EAV = 9.1; // m/s^(1.75)
const double LIMITE_ACC_ELV = 1.15; // m/s^2 - BR (1.1)
const double LIMITE_VDV_ELV = 21.0; // m/s^(1.75)
const double KX = 1.4;
const double KY = 1.4;
const double KZ = 1.0;
const double T0_REF_8H_SEC = 28800; // sec

////////*EDITAVEIS - TO DO*////////
/*Editaveis do Acelerometro*/
const uint8_t I2C_ADDR = 0x29;  //0x29 endereco i2c
const unsigned long BAUD_RATE = 9600; //taxa de trasmissao em bits por segundo (bps)
//const unsigned long BAUD_RATE = 115200;
const uint32_t SETTING_TIME_MS = 500;

/*Editaveis do Modulo Comunicacao GSM*/
const int PORT_UART_RX = 16;
const int PORT_UART_TX = 17;
const String SMS_NUMBER = "+5531998610040"; //Número de telefone que irá receber a mensagem, “ZZ” corresponde ao código telefônico do pais e “XXXXXXXXXXX” corresponde ao número de telefone com o DDD
const String ALERT_TEXT_EAV = "Reduza a vibracao";
const String ALERT_TEXT_ELV = "PARE imediatamente, vibracao nociva a saude";
const String ALERT_TEXT_CF = "VDV mais indicado para avaliacao de vibracao";
const int MIN_DUR_BETWEEN_ALARM = 30*1000; // duracao entre alertas em ms

/*Editaveis da Comunicacao HTTP*/
const String apn = "zap.vivo.com.br"; //APN (Acess Point Name) da operadora do cartao SIM
const String url_string = "http://api.thingspeak.com/update?api_key=RN0VM260LCRNDMBR"; //url para envio dos dados
const unsigned long TEMPO_ATUALIZACAO_WEB = 15*1000;

/*Editaveis do Cartao SD*/
const unsigned long AMOSTRAGEM_MS = int(FREQ_AMOSTRAGEM_SENSOR)/1000.0; // 1000 Hz
const uint32_t TOTAL_TIME = 20 * 60*1000; // 20 min in ms

/*Variaveis*/
double accx, accy, accz, LIAx, LIAy, LIAz, GRVx, GRVy, GRVz;
String data_x, data_y, data_z;
double tempo_s;
char dataMessage[50]; // max 50 chars per sample/line

uint32_t start_time = 0;
uint32_t current_time = 0;  // in ms seconds
uint32_t previous_time = 0; // in ms seconds
uint32_t update_time = 0;
uint32_t count = 0; // samples counter
bool done = false; // will be set to true after total time

uint32_t stable_samples = 0;
double time_stable_sec = 1/FREQ_AMOSTRAGEM_SENSOR;
double datax, x_WD, datay, y_WD, dataz, z_WK;
double x_rms, y_rms, z_rms, x_av8, y_av8, z_av8, av8;
double x_cf, y_cf, z_cf, cf;
double x_vdv, y_vdv, z_vdv, vdv8, x_vdv8, y_vdv8, z_vdv8;

bool alert = false;
unsigned long last_alert = 0;
unsigned long time_alert = 0;

IIR_Biquad_Filter HighPassX, HighPassY, HighPassZ;
IIR_Biquad_Filter LowPassX, LowPassY, LowPassZ;
IIR_Biquad_Filter Transition_WDX, Transition_WDY;
IIR_Biquad_Filter Transition_WKZ;
IIR_Biquad_Filter StepZ;

Acc_Metric ProcessX, ProcessY, ProcessZ;

SoftwareSerial mySerial(PORT_UART_RX, PORT_UART_TX);

void setup() {  
  /*VARIAVEIS*/
  init_var();

  /*TRANSMISSAO*/
  Serial.begin(BAUD_RATE);
  delay(SETTING_TIME_MS);
  Serial.println("Inicializando configuracoes");

  /*ACELEROMETRO*/    
  Wire.begin(); // Inicio protocolo I2C
  delay(SETTING_TIME_MS);  
  POST_test(); // Power On Self Test (POST)  
  init_SENSOR(); // Configuracao sensor

  /*MODULO GSM*/
  delay(45000);
  mySerial.begin(BAUD_RATE); //Inicializa comunicação entre o SIM800L e o ESP32
  init_WEB();

  /*CARTAO SD*/
  init_SD();

  /*DADOS*/
  start_time = millis();
  Serial.println("Configuracoes finalizadas"); 
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

    /*CARTAO SD*/ // TALVEZ salvar a cada 1000 dados
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

    /*PROCESSAMENTO DE DADOS*/    
    datax = (double)accx;
    x_WD = IIR_Cascade_WD(&HighPassX, &LowPassX, &Transition_WDX, datax);
    datay = (double)accy;
    y_WD = IIR_Cascade_WD(&HighPassY, &LowPassY, &Transition_WDY, datay);
    dataz = (double)accz;
    z_WK = IIR_Cascade_WK(&HighPassZ, &LowPassZ, &Transition_WKZ, &StepZ, dataz);

    if (count >= TRANSIENT_FILTER_TIME) {          
      stable_samples++;
      time_stable_sec = stable_samples/FREQ_AMOSTRAGEM_SENSOR;

      x_rms = RMS_value(&ProcessX, x_WD, stable_samples);
      y_rms = RMS_value(&ProcessY, y_WD, stable_samples);
      z_rms = RMS_value(&ProcessZ, z_WK, stable_samples);

      x_av8 = Acc_normalization(&ProcessX, x_rms, time_stable_sec);
      y_av8 = Acc_normalization(&ProcessY, y_rms, time_stable_sec);
      z_av8 = Acc_normalization(&ProcessZ, z_rms, time_stable_sec);
      av8 = worstAxe(x_av8, y_av8, z_av8);

      x_cf = CF_value(&ProcessX, x_WD, x_rms);
      y_cf = CF_value(&ProcessY, y_WD, y_rms);
      z_cf = CF_value(&ProcessZ, z_WK, z_rms);
      cf = worstAxe(x_cf, y_cf, z_cf);

      x_vdv = VDV_value(&ProcessX, x_WD);
      y_vdv = VDV_value(&ProcessY, y_WD);
      z_vdv = VDV_value(&ProcessZ, z_WK);

      x_vdv8 = VDV_normalization(&ProcessX, x_vdv, time_stable_sec);
      y_vdv8 = VDV_normalization(&ProcessY, y_vdv, time_stable_sec);
      z_vdv8 = VDV_normalization(&ProcessZ, z_vdv, time_stable_sec);
      vdv8 = worstAxe(x_vdv8, y_vdv8, z_vdv8);

      // WEB // falta teste de delay de transmissao
      if (current_time - update_time >= TEMPO_ATUALIZACAO_WEB) {      
      Serial.println( "Enviando dados WEB"); 
      String data_av = String(av8,4);
      String data_cf = String(cf,2);
      String data_vdv = String(vdv8,4);
      Serial.print( "AV(8): ");    Serial.println( data_av); 
      Serial.print( "CF: ");    Serial.println( data_cf); 
      Serial.print( "VDV(8): ");    Serial.println( data_vdv);      
      EnviaDados(data_av, data_cf, data_vdv);
      update_time = current_time;
      }

      //SMS      
      String alert_type = "";    
      //AVALIACAO DAS METRICAS           
      if (cf > VDL_USEFUL) {
        alert = true;
        alert_type = ALERT_TEXT_CF;   
      }
      if (alert && time_alert - last_alert >=  MIN_DUR_BETWEEN_ALARM) {
        init_GMS(alert_type);
        last_alert = time_alert;
      }

      if (EvaluateVibration(av8, vdv8) == 2) {
        alert = true;
        alert_type = ALERT_TEXT_ELV;  
        time_alert = millis();    
      }    
      if (EvaluateVibration(av8, vdv8) == 1) {
        alert = true;
        alert_type = ALERT_TEXT_EAV;  
        time_alert = millis();
      } else {alert = false;}
      if (alert && time_alert - last_alert >=  MIN_DUR_BETWEEN_ALARM) {
        init_GMS(alert_type);
        last_alert = time_alert;
      }
    }
    /*AMOSTRAGEM*/
    previous_time = current_time;
  }
}

void init_var() {
  
  IIR_Biquad_init(&HighPassX, B0H, B1H, B2H, A1H, A2H);
  IIR_Biquad_init(&LowPassX, B0L, B1L, B2L, A1L, A2L);
  IIR_Biquad_init(&Transition_WDX, B0TWD, B1TWD, B2TWD, A1TWD, A2TWD);

  IIR_Biquad_init(&HighPassY, B0H, B1H, B2H, A1H, A2H);
  IIR_Biquad_init(&LowPassY, B0L, B1L, B2L, A1L, A2L);
  IIR_Biquad_init(&Transition_WDY, B0TWD, B1TWD, B2TWD, A1TWD, A2TWD);

  IIR_Biquad_init(&HighPassZ, B0H, B1H, B2H, A1H, A2H);
  IIR_Biquad_init(&LowPassZ, B0L, B1L, B2L, A1L, A2L);
  IIR_Biquad_init(&Transition_WKZ, B0TWK, B1TWK, B2TWK, A1TWK, A2TWK);
  IIR_Biquad_init(&StepZ, B0S, B1S, B2S, A1S, A2S);

  Acc_Metric_init(&ProcessX, KX);
  Acc_Metric_init(&ProcessY, KY);
  Acc_Metric_init(&ProcessZ, KZ);

  uint32_t start_time = 0;
  bool done = false; // will be set to true after total time  
  uint32_t current_time = 0;  // in ms seconds
  uint32_t previous_time = 0; // in ms seconds
  uint32_t update_time = 0;
  uint32_t count = 0; // samples counter  
  uint32_t stable_samples = 0;
  bool alert = false;
  unsigned long last_alert = 0;
  unsigned long time_alert = 0;
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
    delay(SETTING_TIME_MS);
    init_GMS("FALHA NO ACELEROMETRO");
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
  //readI2C(POWER_MODE_ADDR,"POWER_MODE");
  //Serial.println("Expected: 0");
  //orientar sinais dos eixos
  writeI2C(AXIS_MAP_SIGN_ADDR, AXIS_MAP_SIGN);
  //readI2C(AXIS_MAP_SIGN_ADDR,"AXIS_MAP_SIGN");
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
  //readI2C(PAGE_ID_ADDR,"PAGE_ID");
  //Serial.println("Expected: 1"); 
  //acc config
  writeI2C(ACC_CONFIG_ADDR, ACC_CONFIG);
  //readI2C(ACC_CONFIG_ADDR,"ACC_CONFIG");
  //Serial.println("Expected: 11101"); 
  //trocar de volta para page 0
  writeI2C(PAGE_ID_ADDR, PAGE_ID_0);
  //readI2C(PAGE_ID_ADDR,"PAGE_ID");
  //Serial.println("Expected: 0"); 
  //modo operacao sensor
    /*
    writeI2C(OPERATION_MODE_ADDR, OPERATION_NDOF);
    readI2C(OPERATION_MODE_ADDR,"OPERATION_MODE");
    Serial.println("Expected: 1100");
    */
  writeI2C(OPERATION_MODE_ADDR, OPERATION_ACCONLY);
  //readI2C(OPERATION_MODE_ADDR,"OPERATION_MODE");
  //Serial.println("Expected: 1");
  //delay(30000);

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

void read_GMS() {  
  while (mySerial.available()) //Verifica se a comunicação serial está disponível
  {
    Serial.write(mySerial.read()); //Realiza leitura serial dos dados de entrada ESP32
  }
  delay(500); //Intervalo de 0,5 segundos
}

void init_GMS( const String alert ) {  
  mySerial.println("AT"); //Teste de conexão 
  read_GMS(); //Chamada da função read_GMS()
  
  mySerial.println("AT+CMGF=1"); //Configuração do modo SMS text
  read_GMS(); //Chamada da função read_GMS()  

  mySerial.println("AT+CMGS=\"" + SMS_NUMBER + "\"\r");
  read_GMS(); //Chamada da função read_GMS()

  mySerial.print(alert); //Texto que será enviado para o usúario
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

void EnviaDados( const String x, const String y, const String z ) {
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
    init_GMS("FALHA NO CARTAO SD");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    init_GMS("SEM CARTAO SD");
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

  uint32_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);  
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  char indicador_sd[40];
  sprintf(indicador_sd, "Iniciando coleta de dados. PORCENTAGEM DO SD: %llu de 100\n", (SD.usedBytes() / SD.totalBytes()) );  
  init_GMS(String(indicador_sd));

  //writeFile(SD, "/data.txt", "NOVO REGISTRO:\n");
  appendFile(SD, "/data.txt", "NOVO REGISTRO:\n");
  //readFile(SD, "/data.txt");  
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
  //Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    init_GMS("FALHA AO ABRIR O ARQUIVO DE DADOS");
    return;
  }
  if (file.print(message)) {
    //Serial.println("File written");
  } else {
    Serial.println("Write failed");
    init_GMS("FALHA NA ESCRITA DE DADOS");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    init_GMS("FALHA AO ABRIR O ARQUIVO DE DADOS");
    return;
  }
  if (file.print(message)) {
    //Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
    init_GMS("FALHA NA ESCRITA DE DADOS");
  }
  file.close();
}

void IIR_Biquad_init(IIR_Biquad_Filter *filter, const double b0, const double b1, const double b2, const double a1, const double a2) {
    filter->b0 = b0;
    filter->b1 = b1;
    filter->b2 = b2;
    filter->a1 = a1;
    filter->a2 = a2;
    filter->z1 = 0.0; // Initialize state variables to zero
    filter->z2 = 0.0;
}

double IIR_Biquad_process(IIR_Biquad_Filter *filter, const double input) {
    double output = filter->b0 * input + filter->z1;
    filter->z1 = filter->b1 * input + filter->z2 - filter->a1 * output;
    filter->z2 = filter->b2 * input - filter->a2 * output;
    return output;
}

double IIR_Cascade_WD(IIR_Biquad_Filter *HighPass, IIR_Biquad_Filter *LowPass, IIR_Biquad_Filter *Transition_WD, const double input) {
  double out1 = IIR_Biquad_process(HighPass, input);    
  double out2 = IIR_Biquad_process(LowPass, out1);    
  double outFiltered = IIR_Biquad_process(Transition_WD, out2); 
  return outFiltered;
}

double IIR_Cascade_WK(IIR_Biquad_Filter *HighPass, IIR_Biquad_Filter *LowPass, IIR_Biquad_Filter *Transition_WK,  IIR_Biquad_Filter *Step, const double input) {
  double out1 = IIR_Biquad_process(HighPass, input);    
  double out2 = IIR_Biquad_process(LowPass, out1);    
  double out3 = IIR_Biquad_process(Transition_WK, out2);
  double outFiltered = IIR_Biquad_process(Step, out3);
  return outFiltered;
}

void Acc_Metric_init(Acc_Metric *metric, const double k) {
  metric->sum_square = 0.0;
  metric->sum_four = 0.0;
  metric->max = 0.0;
  metric->factorK = k;
}

double RMS_value(Acc_Metric *metric, const double input, const uint32_t samples) {
  metric->sum_square += input*input;
  double rms = sqrt( metric->sum_square/samples );
  return rms;
}

double CF_value(Acc_Metric *metric, const double input, const double rms) {
  double modulo = abs(input);
  if (modulo > metric->max) {
    metric->max = modulo;
  }
  double cf = metric->max/rms;  
  return cf;
}

double VDV_value(Acc_Metric *metric, const double input) {
  metric->sum_four += pow(input, 4);
  double vdv = pow(metric->sum_four, 0.25);
  return vdv;
}

double worstAxe(const double x, const double y, const double z) {
  double temp = max(x,y);
  double worst = max(z, temp);
  return worst;
}

double Acc_normalization(Acc_Metric *metric, const double acc_rms, const double time_passed_sec) {
  double av8 = acc_rms*metric->factorK*sqrt(time_passed_sec/T0_REF_8H_SEC);
  return av8;
}

double VDV_normalization(Acc_Metric *metric, const double vdv, const double time_passed_sec) {
  double vdv8 = vdv*metric->factorK*pow(time_passed_sec/T0_REF_8H_SEC, 0.25);
  return vdv8;
}

unsigned int EvaluateVibration( const double Av, const double VDVv ) {
  if (Av > LIMITE_ACC_ELV || VDVv > LIMITE_VDV_ELV) {
    return 2;
  }
  if (Av > LIMITE_ACC_EAV || VDVv > LIMITE_VDV_EAV) {
    return 1;
  }
  else
   return 0;
}


