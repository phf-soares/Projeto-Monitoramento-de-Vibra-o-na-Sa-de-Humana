#include <SoftwareSerial.h> //Adiciona da biblioteca SoftwareSerial.h

const int PORT_UART_RX = 16;
const int PORT_UART_TX = 17;
const int AMOSTRAGEM_MS = 1*1000; // duracao entre alertas em ms

String apn = "internet";
String apn_u = "";
String apn_p = "";

String url_string1 = "https://api.thingspeak.com/update?api_key=RN0VM260LCRNDMBR&field1";
String url_string2 = "https://api.thingspeak.com/update?api_key=RN0VM260LCRNDMBR&field2";
String url_string3 = "https://api.thingspeak.com/update?api_key=RN0VM260LCRNDMBR&field3";

float x,y,z;
long current_time, previous_time;
String data_x, data_y, data_z;

SoftwareSerial mySerial(PORT_UART_RX, PORT_UART_TX); //Cria objeto mySerial passando como parâmetro as portas digitais

void testPostRequest(String jsonToSend){
  //Example format of JSON:
  // String jsonToSend="{\"uploadedAt\":\"2023-06-26T20:18:22.826Z\",\"data\":[{\"unit\":\"C\",\"reading\":31}]}";
  
  sendCommand("AT");
  ShowSerialData();
  sendCommand("AT+CIPSHUT");
  ShowSerialData();
  delay(500);
  sendCommand("AT+SAPBR=0,1");
  delay(2000);
  
  ShowSerialData();
  sendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  ShowSerialData();
  sendCommand("AT+SAPBR=3,1,\"APN\",\"zap.vivo.com.br\"");
  ShowSerialData();
  sendCommand("AT+SAPBR=1,1");
  delay(2000);
  ShowSerialData();
  sendCommand("AT+HTTPINIT");
  delay(1000);
  ShowSerialData();
  sendCommand("AT+HTTPPARA=\"CID\",1");
  ShowSerialData();

  sendCommand("AT+HTTPPARA=\"URL\",\"http://api.thingspeak.com/update?api_key=RN0VM260LCRNDMBR&field1=25\""); // deu certo

  //char url_string1[] = "http://api.thingspeak.com/update?api_key=RN0VM260LCRNDMBR&field1";
  //char sendURL[] = url_string1+"="+jsonToSend;
  //char buffer[] = "=";  
  //snprintf(buffer, sizeof(buffer), "%s%s", jsonToSend);
  //snprintf(url_string1, sizeof(url_string1), "%s%s", buffer);
  //sendCommand("AT+HTTPPARA=URL," + String(url_string1).c_str());  

  ShowSerialData();
  //sendCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  sendCommand("AT+HTTPPARA=\"CONTENT\",\"application/x-www-form-urlencoded\"");  
  ShowSerialData();
  sendCommand(("AT+HTTPDATA=" + String(jsonToSend.length()) + ",20000").c_str());
  delay(6000);
  mySerial.println(jsonToSend);
  delay(16000);
  ShowSerialData();
  sendCommand("AT+HTTPACTION=1");
  delay(20000);
  ShowSerialData();
  sendCommand("AT+HTTPREAD");
  ShowSerialData();
  sendCommand("AT+HTTPTERM");
  ShowSerialData();
  sendCommand("AT+CIPSHUT");
  ShowSerialData();
}

void sendCommand(const char* command) {
  Serial.print("C: ");
  Serial.println(command);
  mySerial.println(command);
  delay(1000);
}

String ShowSerialData() {
 
  String response="";
  while (mySerial.available()) {
    response = mySerial.readString();
    Serial.println("Res from mySerial:");
    Serial.println(response);
  }
  return response;
  // delay(1000);
}

void setup() {
  int baudRate = 9600;
  Serial.begin(baudRate); // Serial monitor
  mySerial.begin(baudRate); // GSM module
  // delay(1000);
  String jsonToSend="{\"uploadedAt\":\"2023-06-26T20:18:22.826Z\",\"data\":[{\"unit\":\"C\",\"reading\":31}]}";
  float x = 12; // sensor values, hardcoded
  //float y = 54;
  //float z = 16;

  data_x = String(x);
  //data_y = String(y);
  //data_z = String(z);
  //please give some time for the SIM card to register on the network before running this command
  delay(7000);
  testPostRequest(data_x);

}

void loop() {
  
  
}

void gms_http_post (String url, String postdata) {
  String sendURL = url+"="+postdata;
  Serial.println(" --- Start GPRS & HTTP ---");
  gsm_send_serial("AT+SAPBR=1,1"); //Opens the GPRS context to establish the connection. Expected response: "OK" if the connection is successfully established.
  gsm_send_serial("AT+SAPBR=2,1"); // IP? Querying bearer?
  gsm_send_serial("AT+HTTPINIT"); //Initializes the HTTP service on the module. Expected response: "OK" if the initialization is successful.
  gsm_send_serial("AT+HTTPPARA=CID,1"); //Sets the HTTP context identifier to 1. Expected response: "OK" if the parameter is set successfully.
  gsm_send_serial("AT+HTTPPARA=URL," + sendURL); //Sets the URL for the server with the actual URL of the server you want to send the GET request to. Expected response: "OK" if the URL is set successfully.
  
  //gsm_send_serial("AT+HTTPPARA=CONTENT,application/x-www-form-urlencoded"); //Indicates that the data will be in XXX format. Expected value: OK
  gsm_send_serial("AT+HTTPDATA=192,5000"); //Prepares the module to receive the XXX data. Length, Time in ms to wait the data to be received.
  gsm_send_serial(postdata); //Send just the XXX data. Expected response: OK If the number of characters are lower or higher than expected the command will fail with 'ERROR'.
  gsm_send_serial("AT+HTTPACTION=1"); //antes (1) Initiates the HTTP POST request. Expected response: "OK", number of characters in HTTP response followed by the HTTP response code (e.g. "+HTTPACTION:42,201")
  gsm_send_serial("AT+HTTPREAD"); //Reads the HTTP response from the server. Expected response: The response data from the server. usually contains the json data you just transmitted with some additional details or error information in case the request returns anything other than 200 or 201 status code.
  gsm_send_serial("AT+HTTPTERM"); //Terminates the HTTP service on the module. Expected response: "OK" if the termination is successful.
  gsm_send_serial("AT+HTTPSAPBR=0,1"); // Closing bearer?
}

void gsm_config_gprs() {
  Serial.println("--- CONFIG GPRS ---");
  gsm_send_serial("AT+SAPBR=3,1,Contype,GPRS"); //Sets the connection type to GPRS. Expected response: "OK" if the command is successful.
  gsm_send_serial("AT+SAPBR=3,1,APN" + apn); //Sets the Access Point Name (APN) for the GPRS connection. APN for your network provider. Expected response: "OK" if the command is successful.
  if (apn_u != "") {
    gsm_send_serial("AT+SAPBR=3,1,USER" + apn_u);
  }
  if (apn_p != "") {
    gsm_send_serial("AT+SAPBR=3,1,PWD" + apn_p);
  }
}

void gsm_send_serial (String command) {
  Serial.println("Send ->:" + command);
  mySerial.println(command);
  read_GMS();
  Serial.println();
}

void init_GMS() {
  mySerial.begin(9600); //Inicializa comunicação entre o SIM800L e o ESP32
  delay(1000);  //Intervalo de 1 segundo
  Serial.println("--- TESTE AT ---");
  mySerial.println("AT"); //Teste de conexão, espera OK
  read_GMS(); //Chamada da função read_GMS()
  delay(30000);
}

void read_GMS() {  
  while (mySerial.available()) //Verifica se a comunicação serial está disponível
  {
    Serial.write(mySerial.read()); //Realiza leitura serial dos dados de entrada ESP32
  }
  delay(500); //Intervalo de 0,5 segundos
}

void gsm_send_serial2 (String command) {
  Serial.println("Send ->:" + command);
  mySerial.println(command);
  long wtimer = millis();
  while (wtimer + 3000 > millis()) {
    while (mySerial.available()) {
      Serial.write(mySerial.read());
    }
  }
  Serial.println();
}