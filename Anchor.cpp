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
#include "Anchor.h"
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgConstants.hpp>
#include <DW1000NgRegisters.hpp>
#include <SPIporting.hpp>

namespace Anchor{
  // private
  namespace {
    volatile byte expectedMsgId = POLL;
    volatile boolean sentAck = false;
    volatile boolean receivedAck = false;
    double distance = 0.0;
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
    
    constexpr uint16_t LEN_FP_INDEX = 2;
    constexpr uint16_t CIR_FP_INDEX_SUB = 0x05;
    constexpr uint16_t ACC_MEM = 0x25;
    constexpr uint16_t LEN_CIR = 4;
    constexpr uint16_t LEN_PMSC = 4;
    constexpr uint16_t FACE_BIT = 6;
    constexpr uint16_t AMCE_BIT = 15;
    uint32_t lastActivity;
    uint32_t resetPeriod = 250;
    uint16_t replyDelayTimeUS = 3000;
    uint16_t successRangingCount = 0;
    uint32_t rangingCountPeriod = 0;
    float samplingRate = 0;
    uint8_t _ss = 0xff;
    byte data[LEN_DATA];
    byte cirDataBytes[LEN_CIR * 64];
      
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
    void _readBytesFromRegister(byte cmd, uint16_t offset, byte data[], uint16_t data_size) {
			byte header[3];
			uint8_t headerLen = 1;
			
			// build SPI header
			if(offset == NO_SUB) {
				header[0] = READ | cmd;
			} else {
				header[0] = READ_SUB | cmd;
				if(offset < 128) {
					header[1] = (byte)offset;
					headerLen++;
				} else {
					header[1] = RW_SUB_EXT | (byte)offset;
					header[2] = (byte)(offset >> 7);
					headerLen += 2;
				}
			}
			SPIporting::readFromSPI(_ss, headerLen, header, data_size, data);
		}
    void _writeBytesToRegister(byte cmd, uint16_t offset, byte data[], uint16_t data_size) {
			byte header[3];
			uint8_t headerLen = 1;
			
			// TODO proper error handling: address out of bounds
			// build SPI header
			if(offset == NO_SUB) {
				header[0] = WRITE | cmd;
			} else {
				header[0] = WRITE_SUB | cmd;
				if(offset < 128) {
					header[1] = (byte)offset;
					headerLen++;
				} else {
					header[1] = RW_SUB_EXT | (byte)offset;
					header[2] = (byte)(offset >> 7);
					headerLen += 2;
				}
			}
			
			SPIporting::writeToSPI(_ss, headerLen, header, data_size, data);
		}
  }
  //public 
  void init(int pinSS, int pinIrq, int pinRst){
    _ss = pinSS;
    DW1000Ng::initialize(pinSS, pinIrq, pinRst);
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);
    DW1000Ng::setDeviceAddress(DEVICE_ADDRESS);
    DW1000Ng::setAntennaDelay(ANTENNA_DELAY);
    DW1000Ng::attachSentHandler(handleSent);
    DW1000Ng::attachReceivedHandler(handleReceived);
    byte pmscBytes[LEN_PMSC];
    _readBytesFromRegister(PMSC, PMSC_CTRL0_SUB, pmscBytes, LEN_PMSC);
    pmscBytes[0] |= 1 << FACE_BIT;
    pmscBytes[1] |= 1 << (AMCE_BIT - sizeof(byte));
    _writeBytesToRegister(PMSC, PMSC_CTRL0_SUB, pmscBytes, LEN_PMSC);
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

          byte fpIndex[LEN_FP_INDEX];
          _readBytesFromRegister(RX_TIME, CIR_FP_INDEX_SUB, fpIndex, LEN_FP_INDEX);
          uint16_t f = ((uint16_t)fpIndex[0] | ((uint16_t)fpIndex[1] << 8)) >> 6;          
          String fpIndexString = "fpIndex : "; fpIndexString += f;
          //Serial.println(fpIndexString);

          int16_t c = 0;
          byte cirDataBytesTemp[LEN_CIR * 64 + 1];
          _readBytesFromRegister(ACC_MEM, f * LEN_CIR, cirDataBytesTemp, LEN_CIR * 64 + 1);
  
          xSemaphoreTake(mutex, portMAX_DELAY);
          distance = temp;
          memcpy(cirDataBytes, &cirDataBytesTemp[1], LEN_CIR * 64);
          xSemaphoreGive(mutex);
          distance = DW1000NgRanging::correctRange(distance);

          byte cirPwrBytes[LEN_CIR_PWR];
          _readBytesFromRegister(RX_FQUAL, CIR_PWR_SUB, cirPwrBytes, LEN_CIR_PWR);

          int16_t real = *(int16_t*)(&cirDataBytes[0]);
          int16_t imag = *(int16_t*)(&cirDataBytes[2]);
          c = sqrt(real * real + imag * imag);
          String fpCirString = "fpCir : "; fpCirString += c;
          //Serial.println(fpCirString);
                
          String rangeString = "Range: "; rangeString += distance; rangeString += " m";
          rangeString += "\t RX power: "; rangeString += DW1000Ng::getReceivePower(); rangeString += " dBm";
          //Serial.println(rangeString);
          sendRangeAck(distance * DISTANCE_OF_RADIO_INV);
          if (curMillis - rangingCountPeriod > 1000) {
            rangingCountPeriod = curMillis;
          }
        }
        else{
          sendRangeNak();
        }
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
  byte* getCirData(byte* dst){
    byte* ret = nullptr;
    xSemaphoreTake(mutex, portMAX_DELAY);
    memcpy(dst, cirDataBytes, LEN_CIR * 64);
    xSemaphoreGive(mutex);
    ret = dst;
    return ret;
  }
}