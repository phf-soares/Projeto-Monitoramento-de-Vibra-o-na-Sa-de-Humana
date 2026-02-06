# Projeto Monitoramento de Vibração na Saúde Humana

Fluxograma básico:

<img width="1760" height="1360" alt="_Fluxograma (1)" src="https://github.com/user-attachments/assets/d9bbccdf-5cc7-49ee-875b-6cf5d5d7f4b4" />


Esquemático:

<img width="1738" height="1249" alt="image" src="https://github.com/user-attachments/assets/fabed7d5-461e-411c-85cf-12f9ced1f36b" />


Código compilado em Arduino IDE com biblioteca para esp32.

Protocolo I2C entre:

1) Módulo sensor BNO055

![image](https://github.com/user-attachments/assets/376cb56f-e07f-475c-9e07-f9a22ab78216)

2) ESP32-WROOM-32D OU DEVKIT-V1


![image](https://github.com/user-attachments/assets/45c2b9af-bc6b-4dd6-a4b6-040efd92e512)

Montagem:
![image](https://github.com/user-attachments/assets/1353824b-a9ea-47d3-8f77-ff2a4f163949)

Protocolo SPI entre ESP32 e:

3) Módulo SD:


![image](https://github.com/user-attachments/assets/3b610a80-6c44-49eb-a397-15c0167fdf9d)

Montagem:
![image](https://github.com/user-attachments/assets/ac5387a5-40da-4f57-bd0b-2beca50ef739)
Feito por mim: https://wokwi.com/projects/434601176080578561

*SD:*

Capacidade 32 GB

Alimentado pelo USB.

Protocolo UART entre ESP32 e:

4) Módulo GSM:

![image](https://github.com/user-attachments/assets/16489fec-9d19-476c-877f-8954d327c72b)

Fonte: https://www.youtube.com/watch?v=THCJWWsyh10

Cartão SIM 2G

Montagem:

![image](https://github.com/user-attachments/assets/190d87d6-d7bf-4c1e-8c5d-8c385ba7c2de)
Feito por mim: https://wokwi.com/projects/435414590476866561


Alimentação
-

- Alimentação portátil por bateria e carregador da mesma:

![image](https://github.com/user-attachments/assets/2655918b-aa83-43d6-be84-c99d01dfc830)

Fonte: https://www.usinainfo.com.br/carregador-de-bateria/shield-v3-carregador-de-bateria-18650-com-usb-saida-5v-3v-e-protecao-de-sobrecarga-ou-descarga-excessiva-5319.html

Exemplo da alimentação

![image](https://github.com/user-attachments/assets/20aa0b39-3b69-42b3-8f20-2ac998f9312b)

Fonte: https://youtu.be/J86yQlJO4w0?si=M3n3m-WgvZsvyjYb


- Bateria recarregável:

Bateria 18650 Lithium (Li-ion) 3.7V / 2600 mAh - FLEXGOLD FX-L2600 Flat-top

![image](https://github.com/user-attachments/assets/967cf584-5a9a-4877-998e-19b579f174dc)

Display WEB
-
Comunicação e display WEB -  enviando para ThingSpeak via GPRS

Configuração de canal no ThingSpeak, teste com PuTTY do HTTP requests via GPRS.

https://thingspeak.mathworks.com/channels/3007277

Aplicativo Android
-

Biblioteca Android que implementa MQTT: Eclipse Paho Android Service
 - Android Studio,  Android SDK (SDK for 24)
O Android Studio é o ambiente de desenvolvimento integrado (IDE) oficial para o desenvolvimento de apps Android. Com base no editor de código e nas ferramentas para desenvolvedores avançados do IntelliJ IDEA. (https://developer.android.com/studio/intro?hl=pt-br)

MQTT Broker
- ThingSpeak MQTT Broker

MQTT Client
- App Android

Código: https://github.com/phf-soares/MQTTClientSample_Android

Calibração e testes:
-

Código: https://github.com/phf-soares/BNO055-ESP32-PlatformIO








