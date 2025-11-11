#define BNO055_INT_PIN 15
volatile bool irq_triggered = false;

void IRAM_ATTR onDataReady() {
  irq_triggered = true;
}

void setup() {
  Serial.begin(115200);
  //pinMode(BNO055_INT_PIN, INPUT_PULLUP);  // ou apenas INPUT, dependendo do hardware
  pinMode(BNO055_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BNO055_INT_PIN), onDataReady, FALLING);
  Serial.println("Teste interrupcao pino");
}

void loop() {
  if (irq_triggered) {
    irq_triggered = false;
    Serial.println("Interrupcao detectada!");
  }
}
