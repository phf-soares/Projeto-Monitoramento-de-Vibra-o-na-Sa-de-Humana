#include <Wire.h> // 1
#include <SoftwareSerial.h> // 2 e 3
#include "FS.h" // 4
#include "SD.h" // 4
#include "SPI.h" // 4
#include "math.h"

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

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

/*Editaveis de Metricas*/
const double FREQ_AMOSTRAGEM_SENSOR = 125.0; // Hz
const int AMOSTRAS_SEC = int(FREQ_AMOSTRAGEM_SENSOR);
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
const double A1H = 2*W1*W1 - 8*Q1;
const double A2H = 4*Q1 - 2*W1 + W1*W1;

const double B0H = 4*Q1;
const double B1H = -8*Q1;
const double B2H = 4*Q1;

// Low Pass
const double A0L = 4*Q2 + 2*W2 + W2*W2*Q2;
const double A1L = 2*W2*W2*Q2 - 8*Q2;
const double A2L = 4*Q2 - 2*W2 + W2*W2*Q2;

const double B0L = W2*W2*Q2;
const double B1L = 2*W2*W2*Q2;
const double B2L = W2*W2*Q2;

// Transition Filter for Wd
const double A0TWD = 4*Q4 + 2*W4WD + W4WD*W4WD*Q4;
const double A1TWD = 2*W4WD*W4WD*Q4 - 8*Q4;
const double A2TWD = 4*Q4 - 2*W4WD + W4WD*W4WD*Q4;

const double B0TWD = W4WD*W4WD*Q4 + 2*(Q4*W4WD*W4WD)/W3WD;
const double B1TWD = 2*W4WD*W4WD*Q4;
const double B2TWD = W4WD*W4WD*Q4 - 2*(Q4*W4WD*W4WD)/W3WD;

// Transition Filter for Wk
const double A0TWK = 4*Q4 + 2*W4WK + W4WK*W4WK*Q4;
const double A1TWK = 2*W4WK*W4WK*Q4 - 8*Q4;
const double A2TWK = 4*Q4 - 2*W4WK + W4WK*W4WK*Q4;

const double B0TWK = W4WK*W4WK*Q4 + 2*(Q4*W4WK*W4WK)/W3WK;
const double B1TWK = 2*W4WK*W4WK*Q4;
const double B2TWK = W4WK*W4WK*Q4 - 2*(Q4*W4WK*W4WK)/W3WK;

// Step Filter
const double A0S = (4*Q6 + 2*W6 + W6*W6*Q6)/Q5;
const double A1S = (2*W6*W6*Q6 - 8*Q6)/Q5;
const double A2S = (4*Q6 - 2*W6 + W6*W6*Q6)/Q5;

const double B0S = (4*Q5 + 2*W5+ W5*W5*Q5)/Q6;
const double B1S = (2*W5*W5*Q5 - 8*Q5)/Q6;
const double B2S = (4*Q5 - 2*W5 + W5*W5*Q5)/Q6;

/*Coeficientes de Metricas*/
const double VDL_USEFUL = 9.0;
const double LIMITE_ACC_EAV = 0.5; // m/s^2
const double LIMITE_VDV_EAV = 9.1; // m/s^(1.75)
const double LIMITE_ACC_ELV = 1.15; // m/s^2 - BR (1.1)
const double LIMITE_VDV_ELV = 21.0; // m/s^(1.75)
const double KX = 1.4;
const double KY = 1.4;
const double T0_REF_8H_SEC = 28800; // sec

////////*EDITAVEIS - TO DO*////////
/*Editaveis do Acelerometro*/
const uint8_t I2C_ADDR = 0x29;  //0x29 endereco i2c
//const unsigned long BAUD_RATE = 9600; //taxa de trasmissao em bits por segundo (bps)
const unsigned long BAUD_RATE = 115200;
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
const unsigned long AMOSTRAGEM_RMS = 1000;

/*Editaveis do Cartao SD*/
const unsigned long AMOSTRAGEM_MS = 1; // 1000 Hz
const uint32_t TOTAL_TIME = 20 * 60*1000; // 20 min in ms

/*Variaveis*/
double accx, accy, accz, LIAx, LIAy, LIAz, GRVx, GRVy, GRVz;
String data_x, data_y, data_z;
double tempo_s;
char dataMessage[50]; // max 50 chars per sample/line

uint32_t start_time = 0;
uint32_t current_time = 0;  // in ms seconds
uint32_t previous_time = 0; // in ms seconds
uint32_t count = 0; // samples counter
bool done = false; // will be set to true after total time

unsigned int endereco_vec = 0;
double accx_vec[AMOSTRAS_SEC]={0}; // setup
double accy_vec[AMOSTRAS_SEC]={0}; // setup
double accz_vec[AMOSTRAS_SEC]={0}; // setup
unsigned int n_sample = 0; // void setup
unsigned int grouped_samples_per_sec = 0; // void setup
double sum_square_zrms = 0; // void setup
double sum_square_yrms = 0; // void setup
double sum_square_xrms = 0; // void setup
double cf = 0;
double Av8 = 0;
double VDVv = 0;

bool alert = false;
unsigned long last_alert = 0;
unsigned long time_alert = 0;

SoftwareSerial mySerial(PORT_UART_RX, PORT_UART_TX);

void setup() {
  /*VARIAVEIS*/
  init_var();

  /*TRANSMISSAO*/
  Serial.begin(BAUD_RATE);
  delay(SETTING_TIME_MS);

  /*ACELEROMETRO*/    
  Wire.begin(); // Inicio protocolo I2C
  delay(SETTING_TIME_MS);  
  POST_test(); // Power On Self Test (POST)  
  init_SENSOR(); // Configuracao sensor

  /*MODULO GSM*/
  delay(20000);
  mySerial.begin(BAUD_RATE); //Inicializa comunicação entre o SIM800L e o ESP32
  init_WEB();

  /*CARTAO SD*/
  init_SD();

  /*DADOS*/
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
    // Accelerometer - TALVEZ MUDAR PRA double PRA GANHAR PRECISAO
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
    // falta ver como processar esse array ?      
    accx_vec[n_sample] = (double)accx;
    accy_vec[n_sample] = (double)accy;
    accz_vec[n_sample] = (double)accz;
    n_sample++;    
    
    if (n_sample == AMOSTRAS_SEC) {
      n_sample = RESET;
      grouped_samples_per_sec++;
      // Frequency Weighting
      // Wd - eixo X e Y
      Serial.print( "X(0): ");    Serial.println( accx_vec[0]);

      double xh[AMOSTRAS_SEC]={0};
      size_t n = NELEMS(xh);           
      filterIIR_2nd_order(A0H,A1H,A2H,B0H,B1H,B2H, xh, accx_vec, n);
      Serial.print( "xh(0): ");    Serial.println( xh[0]); 

      double xhl[AMOSTRAS_SEC]={0};      
      filterIIR_2nd_order(A0L,A1L,A2L,B0L,B1L,B2L, xhl, xh, n);
      Serial.print( "xhl(0): ");    Serial.println( xhl[0]); 

      double x_WD[AMOSTRAS_SEC]={0};
      filterIIR_2nd_order(A0TWD,A1TWD,A2TWD,B0TWD,B1TWD,B2TWD, x_WD, xhl, n);
      Serial.print( "x_WD(0): ");    Serial.println( x_WD[0]);
      
      double yh[AMOSTRAS_SEC]={0};
      filterIIR_2nd_order(A0H,A1H,A2H,B0H,B1H,B2H, yh, accy_vec, n);
      double yhl[AMOSTRAS_SEC]={0};
      filterIIR_2nd_order(A0L,A1L,A2L,B0L,B1L,B2L, yhl, yh, n);      
      double y_WD[AMOSTRAS_SEC]={0};
      filterIIR_2nd_order(A0TWD,A1TWD,A2TWD,B0TWD,B1TWD,B2TWD, y_WD, yhl, n);

      // Wk - eixo Z
      double zh[AMOSTRAS_SEC]={0};
      filterIIR_2nd_order(A0H,A1H,A2H,B0H,B1H,B2H, zh, accz_vec, n);
      double zhl[AMOSTRAS_SEC]={0};
      filterIIR_2nd_order(A0L,A1L,A2L,B0L,B1L,B2L, zhl, zh, n);
      double zhlt[AMOSTRAS_SEC]={0};
      filterIIR_2nd_order(A0TWK,A1TWK,A2TWK,B0TWK,B1TWK,B2TWK, zhl, zhl, n);
      double z_WK[AMOSTRAS_SEC]={0};
      filterIIR_2nd_order(A0S,A1S,A2S,B0S,B1S,B2S, z_WK, zhlt, n);

      // Root mean square (RMS)
      double x_Rrms = RunningRMS_value(x_WD, n);
      Serial.print( "x_Rrms: ");    Serial.println( x_Rrms);
      double y_Rrms = RunningRMS_value(y_WD, n);
      double z_Rrms = RunningRMS_value(z_WK, n);
      
      // RMS Continuo - Somando Intervalos
      unsigned int T_running_sec = grouped_samples_per_sec;      
      
      sum_square_xrms += x_Rrms*x_Rrms;
      double axeq = sqrt( sum_square_xrms/T_running_sec );
      Serial.print( "axeq: ");    Serial.println( axeq);
      
      sum_square_yrms += y_Rrms*y_Rrms;
      double ayeq = sqrt( sum_square_yrms/T_running_sec );
      
      sum_square_zrms += z_Rrms*z_Rrms;
      double azeq = sqrt( sum_square_zrms/T_running_sec );

      // Av(8)
      double ax8 = axeq*KX*sqrt(T_running_sec/T0_REF_8H_SEC);
      Serial.print( "ax8: ");    Serial.println( ax8);
      double ay8 = ayeq*KY*sqrt(T_running_sec/T0_REF_8H_SEC);
      double az8 = azeq*sqrt(T_running_sec/T0_REF_8H_SEC);

      double temp = max(ay8,az8);
      Av8 = max(ax8,temp);
      Serial.print("Av(8)= ");
      Serial.println(Av8);

      // Crest Factor (CF)      
      double CFx = CF_value(x_WD, axeq, n);
      double CFy = CF_value(y_WD, ayeq, n);
      double CFz = CF_value(z_WK, azeq, n);

      temp = max(CFy,CFz);
      cf = max(CFx,temp);
      Serial.print("CF= ");
      Serial.println(cf);

      // Vibration dose value (VDV)
      double VDVx = VDV_value(x_WD, n);
      double VDVy = VDV_value(y_WD, n);
      double VDVz = VDV_value(z_WK, n);

      // VDVv
      double VDVx8 = VDVx*KX*sqrt( sqrt(T_running_sec/T0_REF_8H_SEC) );
      double VDVy8 = VDVy*KY*sqrt( sqrt(T_running_sec/T0_REF_8H_SEC) );
      double VDVz8 = VDVz*sqrt( sqrt(T_running_sec/T0_REF_8H_SEC) );

      temp = max(VDVy8,VDVz8);
      VDVv = max(VDVx8,temp);
      Serial.print("VDV= ");
      Serial.println(VDVv);

      delay(100000);     

      /*WEB*/ // falta teste de delay de transmissao
      if (current_time - previous_time >= AMOSTRAGEM_RMS) {
      String data_av = String(Av8,4);
      String data_cf = String(cf,2);
      String data_vdv = String(VDVv,4);
      EnviaDados(data_av, data_cf, data_vdv);      
      }
      
      /*SMS*/ 
      
      String alert_type = "";    
      /*AVALIACAO DAS METRICAS*/
           
      if (cf > VDL_USEFUL) {
        alert = true;
        alert_type = ALERT_TEXT_CF;   
      }
      if (alert && time_alert - last_alert >=  MIN_DUR_BETWEEN_ALARM) {
        init_GMS(alert_type);
        last_alert = time_alert;
      }

      if (EvaluateVibration(Av8, VDVv) == 2) {
        alert = true;
        alert_type = ALERT_TEXT_ELV;  
        time_alert = millis();    
      }    
      if (EvaluateVibration(Av8, VDVv) == 1) {
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
  // declarar como global tambem? falta
  uint32_t start_time = 0;
  uint32_t current_time = 0;  // in ms seconds
  uint32_t previous_time = 0; // in ms seconds
  uint32_t count = 0; // samples counter
  bool done = false; // will be set to true after total time

  unsigned int endereco_vec = 0;   
  double accx_vec[AMOSTRAS_SEC] = {0}; // setup
  double accy_vec[AMOSTRAS_SEC] = {0}; // setup
  double accz_vec[AMOSTRAS_SEC] = {0}; // setup
  unsigned int n_sample = 0; // void setup
  unsigned int grouped_samples_per_sec = 0; // void setup
  double sum_square_zrms = 0; // void setup
  double sum_square_yrms = 0; // void setup
  double sum_square_xrms = 0; // void setup
  double cf = 0;
  double Av8 = 0;
  double VDVv = 0;


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

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);  
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  char indicador_sd[40];
  sprintf(indicador_sd, "PORCENTAGEM DO SD: %llu per100\n", (SD.usedBytes() / SD.totalBytes()) );  
  //init_GMS(String(indicador_sd));

  writeFile(SD, "/data.txt", "NOVO REGISTRO:\n");
  //appendFile(SD, "/data.txt", "NOVO REGISTRO:\n");
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

double RunningRMS_value( const double data[], size_t n ) {  
  double sum_square = 0;
  for (int j = 0; j < n; j++) {
    sum_square += (data[j]*data[j]);    
  }  
  return sqrt( sum_square / n );
}

double CF_value( const double data[], const double data_rms, size_t size ) {
  double maxValue = abs(data[0]);  
  double modulo_atual = 0;
  for (int j = 1; j < size ; j++) {
    modulo_atual = abs(data[j]);
    if ( modulo_atual > maxValue ) {
      maxValue = modulo_atual;      
    }    
  }  
  return maxValue/data_rms;
}

double VDV_value( const double data[], size_t size ) {  
  double sum_four = 0;
  for (int j = 0; j < size; j++) {
    sum_four += ( pow(data[j], 4) );
  }  
  return sqrt( sqrt(sum_four) );
}

void filterIIR_2nd_order(const double a0, const double a1, const double a2, const double b0, const double b1, const double b2, double y[], const double x[], size_t size ) {       
  y[0] = 1/a0*( b0*x[0]);
  y[1] = 1/a0*( b0*x[1] + b1*x[0] - a1*y[0]);
  for (int n = 2; n < size; n++) {
    y[n] = 1/a0*( b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2] );
    Serial.print( "y: ");    Serial.println( y[n] );
  }    
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


