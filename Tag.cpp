/*
 * MIT License
 * 
 * Copyright (c) 2018 Michele Biondi, Andrea Salvatori
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "Tag.h"
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>

namespace Tag{
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
    void sendPoll() {
      data[0] = POLL;
      DW1000Ng::setTransmitData(data, LEN_DATA);
      DW1000Ng::startTransmit();
    }
    void sendRange() {
      data[0] = RANGE;

      byte futureTimeBytes[LENGTH_TIMESTAMP];

	    timeRangeSent = DW1000Ng::getSystemTimestamp();
	    timeRangeSent += DW1000NgTime::microsecondsToUWBTime(replyDelayTimeUS);
      DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
      DW1000Ng::setDelayedTRX(futureTimeBytes);
      timeRangeSent += DW1000Ng::getTxAntennaDelay();

      DW1000NgUtils::writeValueToBytes(data + 1, timePollSent, LENGTH_TIMESTAMP);
      DW1000NgUtils::writeValueToBytes(data + 6, timePollAckReceived, LENGTH_TIMESTAMP);
      DW1000NgUtils::writeValueToBytes(data + 11, timeRangeSent, LENGTH_TIMESTAMP);
      DW1000Ng::setTransmitData(data, LEN_DATA);
      DW1000Ng::startTransmit(TransmitMode::DELAYED);
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
    DW1000Ng::setNetworkId(NETWORK_ID);
    DW1000Ng::setAntennaDelay(ANTENNA_DELAY);
    DW1000Ng::attachSentHandler(handleSent);
    DW1000Ng::attachReceivedHandler(handleReceived);
    mutex = xSemaphoreCreateMutex();
    sendPoll();
    noteActivity();
    rangingCountPeriod = millis();
  }
  void run(){
    if (!sentAck && !receivedAck) {
      // check if inactive
      if (millis() - lastActivity > resetPeriod) {
          resetInactive();
      }
      return;
    }
    // continue on any success confirmation
    if (sentAck) {
      sentAck = false;
      DW1000Ng::startReceive();
    }
    if (receivedAck) {
      receivedAck = false;
      // get message and parse
      DW1000Ng::getReceivedData(data, LEN_DATA);
      byte msgId = data[0];
      if (msgId != expectedMsgId) {
        // unexpected message, start over again
        //Serial.print("Received wrong message # "); Serial.println(msgId);
        expectedMsgId = POLL_ACK;
        sendPoll();
        return;
      }
      if (msgId == POLL_ACK) {
        timePollSent = DW1000Ng::getTransmitTimestamp();
        timePollAckReceived = DW1000Ng::getReceiveTimestamp();
        expectedMsgId = RANGE_ACK;
        sendRange();
        noteActivity();
      } else if (msgId == RANGE_ACK) {
        expectedMsgId = POLL_ACK;
        float curRange;
        memcpy(&curRange, data + 1, 4);
        delay(100);
        sendPoll();
        noteActivity();
      } else if (msgId == RANGE_NAK) {
        expectedMsgId = POLL_ACK;
        sendPoll();
        noteActivity();
      }
    }
  }
  void printDeviceIdentifier(HardwareSerial& serial){
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    serial.print("Device ID: ");
    serial.println(msg);
  }
}
