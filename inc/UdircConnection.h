#pragma once

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <queue>
#include <chrono>

#include <boost/asio.hpp>

#include "UdircProtocol.h"
#include "Utilities.h"

using namespace std::chrono_literals;
using boost::asio::ip::udp;

typedef std::vector<uint8_t> Buffer;

// The UdircConnection takes care of the actual communication with the drone.
// This includes details such as serialization, transport and any protocol state.
struct UdircConnection {
  typedef std::vector<uint8_t> UdircBuffer;
  UdircConnection(boost::asio::io_service& io)
  : _io(io)
  , _socket(_io, udp::endpoint(udp::v4(), 0))
  , _drone(boost::asio::ip::address::from_string("192.168.0.1"), 40000)
  , _bufferSize(1500) 
  , _running(true) 
  {
    _socket.non_blocking(true);
    _thread = std::thread(&UdircConnection::loop, this);
  }

  ~UdircConnection() {
    _running = false;
    _thread.join();
  }

  // A really simple heartbeat which is sent at 1Hz from controller
  // to the drone
  static Buffer buildHeartbeat() {
    return {0x63, 0x63, 0x01, 0x00, 0x00, 0x00, 0x00};
  }

  // We can build up our commands + checksums here
  Buffer static buildCommand(UdircCommand cmd) {
    UdircFrame<sizeof(cmd)> frame;
    Buffer buf(sizeof(frame) + sizeof(UdircMsgHeader));
    UdircMsgHeader* msg = reinterpret_cast<UdircMsgHeader*>(buf.data());
    std::memcpy(frame.data.data(), &cmd, sizeof(cmd));

    frame.checksum = udircChecksum<frame.data.size()>(frame.data);

    msg->magicNumber = 0x6363;
    msg->type = UDIRC_MSG_TYPE_COMMAND;
    msg->length = sizeof(frame);

    std::memcpy(msg->payload, &frame, sizeof(frame));
    return buf;
  }


  // Send a simple heartbeat
  void sendHeartbeat() {
    auto hb = UdircConnection::buildHeartbeat();
    DUMP(heartbeat, hb);
    _socket.send_to(boost::asio::buffer(hb), _drone);
  }

  // Send a UDIRC command
  void sendCommand(UdircCommand cmd) {
    auto c = UdircConnection::buildCommand(cmd);
    DUMP(command, c);
    _socket.send_to(boost::asio::buffer(c), _drone);
  }

  // Poll for any received messages
  void receiveData() {
    std::vector<uint8_t> b(_bufferSize);
    boost::system::error_code error = boost::asio::error::would_block;
    auto recvd = _socket.receive_from(boost::asio::buffer(b), _drone, 0, error);
    if (recvd == 0) {
      // Nothing to see here...  
      return;
    }
    b.resize(recvd);

    auto msg = reinterpret_cast<UdircMsgHeader*>(b.data());
    switch (msg->type) {
      case UDIRC_MSG_TYPE_STATUS: 
        // These status messages don't seem too informative
        // There is a simple text 'udirc-WiFi-18BB64', and some
        // other data that seems static.
        DUMP(status1, b);
        break;
      case UDIRC_MSG_TYPE_VIDEO: 
        // This is a video fragment, but there are some issues here
        // still.  In particular, the video fragments can be 
        // reassembled into something which should be a proper JPEG 
        // (begins w/ FFD8 and ends with FFD9), but they seem to be 
        // missing data...
        DUMP(video_fragment, b);
        reassembleVideoFragment(msg);
        break;
      case UDIRC_MSG_TYPE_CMD_ACK: 
        // This messages appears to be an ACK to commands, and
        // could contain altitude/battery information.
        DUMP(status2, b);
        break;
      default:
        throw std::runtime_error("unknown message type: " + std::to_string(msg->type));
    }
  } 

  // Attempt to reassemble MJPEG fragments
  void reassembleVideoFragment(const UdircMsgHeader* b) {
    auto v = reinterpret_cast<const UdircVideoFragment*>(b->payload);
    // restart if we are at the first fragment
    if (v->fragIndex == 1) {
      _videoBuffer.clear();
    }

    // TODO We should attempt to handle missing or out-of-order packets.  

    // add the fragment to the list
    _videoBuffer.insert(_videoBuffer.end(), v->payload, v->payload + v->fragLength);
  
    // If this is the last fragment, fire our callback with the complete buffer
    if (v->fragIndex == v->fragCount) {
      onVideo(_videoBuffer);
    }
  }

  // Run a synchronous loop - spam heartbeats, commands and then poll any video data
  void loop() {
    // We need to send a heartbeat first.
    sendHeartbeat();

    // Initialize our clock and timestamps
    Clock clock;
    auto currentTime = clock.now();
    auto lastHbTime = currentTime;
    auto lastCmdTime = currentTime;
    auto lastTime = currentTime;

    while(_running) {
      currentTime = clock.now();

      // send heartbeat (no more than 1hz)
      if (ms(currentTime, lastHbTime) >= 1000) {
        sendHeartbeat();
        lastHbTime = currentTime;
      }

      // send command (no more than 20hz)
      if (ms(currentTime, lastCmdTime) >= 50) {
        std::unique_lock<std::mutex>(_mutex);
        sendCommand(_cmd);
        lastCmdTime = currentTime;
      }

      // recv any data
      receiveData();
  
      // let's not get too crazy...
      std::this_thread::sleep_for(1ms);

      lastTime = currentTime;
    }
  }

  // queue up any received video 
  void onVideo(UdircBuffer& buffer) {
    static int frame = 0;
    
    try {
      auto image = cv::imdecode(cv::Mat(buffer), 1);
      if (!image.empty()) {
        std::unique_lock<std::mutex>(_mutex);
        _video = image;
        //std::ofstream output("frames/frame_" + std::to_string(frame), std::ios::out | std::ios::binary);
        //output.write((char*)&buffer[0], buffer.size());
      }
    } catch (std::exception e) {
      std::cerr << e.what() << std::endl;
    }

    DUMP(jpeg, buffer);

    frame++;
  }

  //--------------------------------------------------------------------------

  // queue up commands
  void command(const UdircCommand cmd) {
    _cmd = cmd;
  }

  // allow caller to dequeue any video frames
  cv::Mat video() {
    std::unique_lock<std::mutex>(_mutex);
    return _video;
  }

private:
  boost::asio::io_service& _io;
  udp::socket              _socket;
  udp::endpoint            _drone;
  UdircBuffer              _videoBuffer;
  size_t                   _bufferSize;

  std::thread              _thread;
  std::atomic<bool>        _running;
  std::mutex               _mutex;
  UdircCommand             _cmd;
  cv::Mat                  _video;
};
