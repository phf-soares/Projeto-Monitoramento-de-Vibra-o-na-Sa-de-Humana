#include <SoftwareSerial.h> //Adiciona da biblioteca SoftwareSerial.h

const int PORT_UART_RX = 16;
const int PORT_UART_TX = 17;
const String SMS_NUMBER = "+5531998610040"; //Número de telefone que irá receber a mensagem, “ZZ” corresponde ao código telefônico do pais e “XXXXXXXXXXX” corresponde ao número de telefone com o DDD
const String ALERT_TEXT = "Vibracao nociva detectada";
const int GMS_CONFIRM = 26;
const int MIN_DUR_BETWEEN_ALARM = 30*1000; // duracao entre alertas em ms

bool alert = 0;
bool test = 1;
unsigned long last_alert = 0;
unsigned long time_alert = 0;

SoftwareSerial mySerial(PORT_UART_RX, PORT_UART_TX); //Cria objeto mySerial passando como parâmetro as portas digitais

void setup() {

  Serial.begin(9600); //Inicializa a comunicação serial
  
}

void loop() {

  if (test) {
    alert = 1;    
    time_alert = millis();    
  } else { alert = 0;}

  if (alert && time_alert - last_alert >=  MIN_DUR_BETWEEN_ALARM) {
    init_GMS();
    last_alert = time_alert;
  }  
    
}

void read_GMS() {  
  while (mySerial.available()) //Verifica se a comunicação serial está disponível
  {
    Serial.write(mySerial.read()); //Realiza leitura serial dos dados de entrada ESP32
  }
  delay(500); //Intervalo de 0,5 segundos
}

void init_GMS() {
  mySerial.begin(9600); //Inicializa comunicação entre o SIM800L e o ESP32
  delay(1000);  //Intervalo de 1 segundo
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