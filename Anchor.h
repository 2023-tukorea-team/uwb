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
  void init(int, int, int);
  void run();
  void printDeviceIdentifier(HardwareSerial&);
  double getDistance();
};

#endif
