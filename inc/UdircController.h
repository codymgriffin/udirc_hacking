#pragma once

#include <iostream>
#include <vector>

#include "UdircProtocol.h"
#include "Utilities.h"

using namespace std::chrono_literals;

typedef std::chrono::high_resolution_clock Clock;
typedef std::vector<uint8_t> Buffer;

// The UdircController translates high-level control events to 
// lower-level UdircCommands.
struct UdircController {
  UdircController() {
    // When we instantiate the controller, we assert the 'enable' bit
    // for 1 second.  No idea if this is necessary... 
    enable();
  }

  // The UDIRC_CMD_PRESENT bit seems to be set all the time, but we 
  // explicitly set it here... No idea what it means.
  void takeoff() {
    _command.cmd = UDIRC_CMD_PRESENT | UDIRC_CMD_TAKEOFF;
    _lastCmdTime = _clock.now();
  }

  void land() {
    _command.cmd = UDIRC_CMD_PRESENT | UDIRC_CMD_LAND;
    _lastCmdTime = _clock.now();
  }

  void enable() {
    _command.cmd = UDIRC_CMD_PRESENT | UDIRC_CMD_ENABLE;
    _lastCmdTime = _clock.now();
  }

  void disable() {
    _command.cmd = UDIRC_CMD_PRESENT | UDIRC_CMD_DISABLE;
    _lastCmdTime = _clock.now();
  }

  // enforce that when we apply a delta it stays between 0-255
  // this is a bit much - could probably just do this all in
  // floating point and cast at the end...
  void dclamp(uint8_t& byte, int8_t delta) {
    if (delta > 0) {
      delta = std::min(int8_t(255 - byte), delta);
    } else {
      delta = std::max(int8_t(-byte), delta);
    }
    
    byte += delta;
  }

  void throttle(int8_t delta) {
    dclamp(_command.throttle, delta);
    _lastCtrTime = _clock.now();
  }

  void yaw(int8_t delta) {
    dclamp(_command.yaw, delta);
    _lastCtrTime = _clock.now();
  }

  void pitch(int8_t delta) {
    dclamp(_command.pitch, delta);
    _lastCtrTime = _clock.now();
  }

  void roll(uint8_t delta) {
    dclamp(_command.roll, delta);
    _lastCtrTime = _clock.now();
  }

  void trimRoll(int8_t delta) {
    _command.trimRoll += delta;  
  }

  void trimYaw(int8_t delta) {
    _command.trimYaw += delta;  
  }

  void trimPitch(int8_t delta) {
    _command.trimPitch += delta;  
  }

  const UdircCommand& command() {
    // We only want to assert our cmd flags for 1s
    auto timeSinceLastCmd = ms(_clock.now(), _lastCmdTime);
    auto timeSinceLastCtr = ms(_clock.now(), _lastCtrTime);
    if (timeSinceLastCmd > 1000) {
      _command.cmd = UDIRC_CMD_PRESENT;
    }

    if (timeSinceLastCmd > 10) {
      // really bad 'decay' function based on a really bad 'sgn' function
      _command.throttle += (0x80 - _command.throttle)>0?1:-1;
      _command.yaw      += (0x80 - _command.yaw)     >0?1:-1;
      _command.pitch    += (0x80 - _command.pitch)   >0?1:-1;
      _command.roll     += (0x80 - _command.roll)    >0?1:-1;

      // If we are close, just pin to the 'neutral' position
      // This prevents us from oscillating around 
      if (std::abs(_command.yaw - 0x80) < 2) {
        _command.yaw = 0x80;
      }
      if (std::abs(_command.pitch - 0x80) < 2) {
        _command.pitch = 0x80;
      }
      if (std::abs(_command.roll - 0x80) < 2) {
        _command.roll = 0x80;
      }
      if (std::abs(_command.throttle - 0x80) < 2) {
        _command.throttle = 0x80;
      }
    }

    return _command;
  }

  const bool neutral() const {
    return _command.yaw      == 0x80 && 
           _command.pitch    == 0x80 && 
           _command.roll     == 0x80 && 
           _command.throttle == 0x80 &&
           _command.cmd      == UDIRC_CMD_PRESENT;
  }

  static std::string display(const UdircCommand& command) {
    std::stringstream s;
    s << std::hex << std::setfill('0') << std::setw(2);
    s << "  throttle: " << int(command.throttle) << std::endl;
    s << "       yaw: " << int(command.yaw) << std::endl;
    s << "     pitch: " << int(command.pitch) << std::endl;
    s << "      roll: " << int(command.roll) << std::endl;
    s << "  trim yaw: " << int(command.trimYaw) << std::endl;
    s << "trim pitch: " << int(command.trimPitch) << std::endl;
    s << " trim roll: " << int(command.trimRoll) << std::endl;
    s << "       cmd: " << int(command.cmd) << std::endl;
    s << std::dec << std::setw(0) << std::setfill(' ');
    return s.str();
  }

private:
  Clock             _clock;
  UdircCommand      _command;
  Clock::time_point _lastCmdTime;
  Clock::time_point _lastCtrTime;
};
