#include <SoftwareSerial.h>  //Adiciona da biblioteca SoftwareSerial.h

const int PORT_UART_RX = 16;
const int PORT_UART_TX = 17;
const int AMOSTRAGEM_MS = 1 * 1000;  // duracao entre alertas em ms

String apn = "zap.vivo.com.br";
String url_string = "http://api.thingspeak.com/update?api_key=RN0VM260LCRNDMBR";

float x, y, z;
long current_time, previous_time;
String data_x, data_y, data_z;

SoftwareSerial mySerial(PORT_UART_RX, PORT_UART_TX);  //Cria objeto mySerial passando como parâmetro as portas digitais

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
  Serial.println("Inicializando…");
  delay(2000);
  Serial.println("Prontinho…");
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

void loop() {
  x = 12;  // sensor values, hardcoded
  y = 54;
  z = 16;

  data_x = String(x);
  data_y = String(y);
  data_z = String(z);
  EnviaDados(data_x, data_y, data_z);
  delay(30000);
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