#include"Anchor.h"

const uint8_t PIN_SCK = 18;
const uint8_t PIN_SS = 4;
const uint8_t PIN_RST = 15;
const uint8_t PIN_IRQ = 17;
const uint8_t PIN_TX = 26;
const uint8_t PIN_RX = 27;
byte data[256];

SemaphoreHandle_t mutex;

TaskHandle_t SerialTaskHandle;

void serialSendFunc(void*);

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, PIN_TX, PIN_RX);
  delay(1000);

  Anchor::init(PIN_SS, PIN_IRQ, PIN_RST);
  Anchor::printDeviceIdentifier(Serial);

  Serial.println("anchor");

  xTaskCreatePinnedToCore(serialSendFunc, "task", 1000, &Serial2, 1, NULL, 0);
}

void loop() {
    Anchor::run();
}

void serialSendFunc(void* arg){
  HardwareSerial serial = *(HardwareSerial *)arg;
  while(1){
    /*char buffer[20];
    double distance = Anchor::getDistance();
    dtostrf(distance, 10, 3, buffer);
    serial.println(buffer);*/
    Anchor::getCirData(data);
    serial.write(data, 256);
    //Serial.println("anchor");
    Serial.println(*(int16_t*)&data[0]);
    delay(1000);  // 1초 대기
  }
}