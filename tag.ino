#include"Tag.h"

const uint8_t PIN_SCK = 18;
const uint8_t PIN_SS = 4;
const uint8_t PIN_RST = 15;
const uint8_t PIN_IRQ = 17;
const uint8_t PIN_TX = 26;
const uint8_t PIN_RX = 27;

SemaphoreHandle_t mutex;

TaskHandle_t SerialTaskHandle;

void serialSendFunc(void*);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, PIN_TX, PIN_RX);
  delay(1000);

  Tag::init(PIN_SS, PIN_IRQ, PIN_RST);
  Tag::printDeviceIdentifier(Serial);
}

void loop() {
    Tag::run();
}