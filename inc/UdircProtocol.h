#pragma once

#include <memory>

//
// This file contains the reverse engineered wire protocol for the
// udirc-WiFi-18BB64 drone.
//
// This is not 100% correct or complete, but it gets the job done...
#define UDIRC_MSG_TYPE_STATUS    0x01
#define UDIRC_MSG_TYPE_VIDEO     0x03
#define UDIRC_MSG_TYPE_COMMAND   0x0a
#define UDIRC_MSG_TYPE_STATUS2   0x0b


// A Message is the unit of transport, which is pretty much 1:1 
// to a UDP packet
struct UdircMsgHeader {
  uint16_t magicNumber;
  uint8_t  type;
  uint16_t _unknown;     // version?
  uint16_t length;
  uint8_t  payload[0];
} __attribute__((packed));

// A Frame is a bit more structured.  Not all messages contain
// frames.
// TODO not all frame lenghts may be known statically.. but they 
// are for now.
template<int L>
struct UdircFrame {
  int8_t sof = 0x66;     // 0x66
  std::array<uint8_t, L> data;
  int8_t checksum;       // just XOR(data)
  int8_t eof = 0x99;     // 0x99
} __attribute__((packed));

template<int L>
uint8_t udircChecksum(std::array<uint8_t, L> data) {
  uint8_t r = 0x00;
  for (auto c : data) {
    r ^= c;
  }
  return r;
}

#define UDIRC_CMD_ENABLE  0x80
#define UDIRC_CMD_DISABLE 0x40
#define UDIRC_CMD_LAND    0x20
#define UDIRC_CMD_TAKEOFF 0x10
#define UDIRC_CMD_HISPEED 0x08
#define UDIRC_CMD_PRESENT 0x04 // not sure what this is... always on
#define UDIRC_CMD_02      0x02 
#define UDIRC_CMD_01      0x01
struct UdircCommand {
  uint8_t roll      = 0x80;
  uint8_t pitch     = 0x80;
  uint8_t throttle  = 0x10;
  uint8_t yaw       = 0x80;
  uint8_t trimRoll  = 0x80;
  uint8_t trimPitch = 0x80;
  uint8_t trimYaw   = 0x80;
  uint8_t cmd       = UDIRC_CMD_PRESENT;
} __attribute__((packed));

// A video frame is fragmented
struct UdircVideoFragment {
  uint8_t   _unknown;
  uint16_t  sequence;
  uint8_t   _pad3[3];
  uint8_t   _pad1;   // could be something
  uint8_t   _pad34[34];
  uint16_t  fragIndex;
  uint16_t  fragCount;
  uint16_t  fragLength;
  uint8_t   payload[0];
} __attribute__((packed));
