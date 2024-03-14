#ifndef _ANCHOR_H_
#define _ANCHOR_H_

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>

#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_ACK 3
#define RANGE_NAK 255
#define LEN_DATA 17
#define DEVICE_ADDRESS 1
#define ANTENNA_DELAY 16346

const device_configuration_t DEFAULT_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

const interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
    true,
    true,
    true,
    false,
    true
};

namespace Anchor{
  void init(int, int, int, int, int);
  void run();
  void printDeviceIdentifier(const HardwareSerial&);
  double getDistance();
};

namespace Anchor{
  // private
  namespace {
    volatile byte expectedMsgId = POLL;
    volatile boolean sentAck = false;
    volatile boolean receivedAck = false;
    volatile double distance = 0.0;
    boolean protocolFailed = false;
    uint64_t timePollSent;
    uint64_t timePollReceived;
    uint64_t timePollAckSent;
    uint64_t timePollAckReceived;
    uint64_t timeRangeSent;
    uint64_t timeRangeReceived;
    uint64_t timeComputedRange; 

    // read and write distance
    SemaphoreHandle_t mutex;
    
    byte data[LEN_DATA];
    uint32_t lastActivity;
    uint32_t resetPeriod = 250;
    uint16_t replyDelayTimeUS = 3000;
    uint16_t successRangingCount = 0;
    uint32_t rangingCountPeriod = 0;
    float samplingRate = 0;
      
    void receiver() {
      DW1000Ng::forceTRxOff();
      DW1000Ng::startReceive();
    }
    void noteActivity() {
      lastActivity = millis();
    }
    void resetInactive() {
      expectedMsgId = POLL;
      receiver();
      noteActivity();
    }
    void sendPollAck() {
      data[0] = POLL_ACK;
      DW1000Ng::setTransmitData(data, LEN_DATA);
      DW1000Ng::startTransmit();
    }
    void sendRangeAck(float curRange) {
      data[0] = RANGE_ACK;
      memcpy(data + 1, &curRange, 4);
      DW1000Ng::setTransmitData(data, LEN_DATA);
      DW1000Ng::startTransmit();
    }
    void sendRangeNak(){
      data[0] = RANGE_NAK;
      DW1000Ng::setTransmitData(data, LEN_DATA);
      DW1000Ng::startTransmit();
    }
    void handleSent() {
      sentAck = true;
    }
    void handleReceived() {
      receivedAck = true;
    }
  }
  //public 
  void init(int pinSS, int pinIrq, int pinRst){
    DW1000Ng::initialize(pinSS, pinIrq, pinRst);
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);
    DW1000Ng::setDeviceAddress(DEVICE_ADDRESS);
    DW1000Ng::setAntennaDelay(ANTENNA_DELAY);
    DW1000Ng::attachSentHandler(handleSent);
    DW1000Ng::attachReceivedHandler(handleReceived);
    mutex = xSemaphoreCreateMutex();
    receiver();
    noteActivity();
    rangingCountPeriod = millis();
  }
  void run(){
    int32_t curMillis = millis();
    if (!sentAck && !receivedAck) {
        if (curMillis - lastActivity > resetPeriod)
            resetInactive();
        return;
    }
    if (sentAck) {
        sentAck = false;
        byte msgId = data[0];
        if (msgId == POLL_ACK) {
            timePollAckSent = DW1000Ng::getTransmitTimestamp();
            noteActivity();
        }
        DW1000Ng::startReceive();
    }
    if(receivedAck){
      receivedAck =  false;
      DW1000Ng::getReceivedData(data, LEN_DATA);
      byte msgId = data[0];
      if(msgId != expectedMsgId)
        protocolFailed = true;
      if (msgId == POLL) {
        protocolFailed = false;
        timePollReceived = DW1000Ng::getReceiveTimestamp();
        expectedMsgId = RANGE;
        sendPollAck();
        noteActivity();
      }
      else if(msgId == RANGE){
        timeRangeReceived = DW1000Ng::getReceiveTimestamp();
        expectedMsgId = POLL;
        if(!protocolFailed){
          timePollSent = DW1000NgUtils::bytesAsValue(data + 1, LENGTH_TIMESTAMP);
          timePollAckReceived = DW1000NgUtils::bytesAsValue(data + 6, LENGTH_TIMESTAMP);
          timeRangeSent = DW1000NgUtils::bytesAsValue(data + 11, LENGTH_TIMESTAMP);
          
          double temp = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                                                            timePollReceived, 
                                                            timePollAckSent, 
                                                            timePollAckReceived, 
                                                            timeRangeSent, 
                                                            timeRangeReceived);
  
          xSemaphoreTake(mutex, portMAX_DELAY);
          distance = temp;
          xSemaphoreGive(mutex);
          distance = DW1000NgRanging::correctRange(distance);
                
          String rangeString = "Range: "; rangeString += distance; rangeString += " m";
          Serial.println(rangeString);
          sendRangeAck(distance * DISTANCE_OF_RADIO_INV);
          if (curMillis - rangingCountPeriod > 1000) {
            samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
            rangingCountPeriod = curMillis;
          }
        }
        sendRangeNak();
      }
    }
  }
  void printDeviceIdentifier(HardwareSerial& serial){
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    serial.print("Device ID: ");
    serial.println(msg);
  }
  double getDistance(){
    double ret;
    xSemaphoreTake(mutex, portMAX_DELAY);
    ret = distance;
    xSemaphoreGive(mutex);
    return ret;
  }
}

#endif
