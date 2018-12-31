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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "UdircProtocol.h"
#include "Hexdump.hpp"

using namespace std::chrono_literals;
using boost::asio::ip::udp;

typedef std::chrono::high_resolution_clock Clock;
typedef std::vector<uint8_t> Buffer;

auto ms = [](auto&& a, auto&& b) { 
  return std::chrono::duration_cast<
    std::chrono::milliseconds>(a - b).count();
};


#define DEBUG false
#define DUMP(label, buffer) if (DEBUG) {\
    std::cerr << #label << std::endl;\
    std::cerr << Hexdump(buffer.data(), buffer.size()) << std::endl;\
}\

// A really simple heartbeat which is sent at 1Hz from controller
// to the drone
Buffer buildHeartbeat() {
  return {0x63, 0x63, 0x01, 0x00, 0x00, 0x00, 0x00};
}

// We can build up our commands + checksums here
Buffer buildCommand(UdircCommand cmd) {
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

  // Send a simple heartbeat
  void sendHeartbeat() {
    auto hb = buildHeartbeat();
    DUMP(heartbeat, hb);
    _socket.send_to(boost::asio::buffer(hb), _drone);
  }

  // Send a UDIRC command
  void sendCommand(UdircCommand cmd) {
    auto c = buildCommand(cmd);
    DUMP(command, c);
    _socket.send_to(boost::asio::buffer(c), _drone);
  }

  // Poll for an received messages - only attempt to reassemble video for now
  void receiveData() {
    std::vector<uint8_t> b(_bufferSize);
    boost::system::error_code error = boost::asio::error::would_block;
    auto recvd = _socket.receive_from(boost::asio::buffer(b), _drone, 0, error);
    if (recvd == 0) {
      std::cerr << "nothing to read..." << std::endl;
      return;
    }
    b.resize(recvd);

    auto msg = reinterpret_cast<UdircMsgHeader*>(b.data());
    switch (msg->type) {
      case UDIRC_MSG_TYPE_STATUS: 
        DUMP(status1, b);
        break;
      case UDIRC_MSG_TYPE_VIDEO: 
        DUMP(video_fragment, b);
        reassembleVideoFragment(msg);
        break;
      case UDIRC_MSG_TYPE_STATUS2: 
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

    //std::cerr << "on fragment " << v->fragIndex << " of " << v->fragCount << std::endl;

    // TODO hanle out-of-order stuff

    // add the fragment to the list
    _videoBuffer.insert(_videoBuffer.end(), v->payload, v->payload + v->fragLength);
  
    // If this is the last fragment, fire our callback
    if (v->fragIndex == v->fragCount) {
      onVideo(_videoBuffer);
    }
  }

  // Run a synchronous loop - spam heartbeats, commands and then poll any video data
  void loop() {
    Clock clock;
    auto currentTime = clock.now();

    // We need to send a heartbeat first.
    sendHeartbeat();
    auto lastHbTime = currentTime;

    auto lastTime = currentTime;

    while(_running) {
      currentTime = clock.now();

      auto dt = ms(currentTime, lastTime);
      //std::cerr << "dt = " << std::dec << dt << std::endl;

      // send heartbeat (no more than once a second... I hate chrono)
      if (ms(currentTime, lastHbTime) >= 1000) {
        sendHeartbeat();
        lastHbTime = currentTime;
      }

      // send command (we need to lock here)
      {
      std::unique_lock<std::mutex>(_mutex);
      sendCommand(_cmd);
      }

      // recv any data
      receiveData();
      std::this_thread::sleep_for(5ms);

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
      DUMP(jpeg, buffer);
    }

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


// The UdircController translates high-level control events to 
// lower-level UdircCommands.
struct UdircController {
  enum EventType {
    CMD_LAND,
    CMD_TAKEOFF,
    CMD_ENABLE,
    CMD_DISABLE,
    THROTTLE,
    YAW,
    PITCH,
    ROLL,
    COUNT_NUM_EVENTS // this must be last!
  };

  UdircController() {
    enable();
  }

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

  void throttle(uint8_t delta) {
    _command.throttle += delta;  
    _lastCmdTime = _clock.now();
  }

  void yaw(uint8_t delta) {
    _command.yaw += delta;  
    _lastCmdTime = _clock.now();
  }

  void pitch(uint8_t delta) {
    _command.pitch += delta;  
    _lastCmdTime = _clock.now();
  }

  void roll(uint8_t delta) {
    _command.roll += delta;  
    _lastCmdTime = _clock.now();
  }

  void trimRoll(uint8_t delta) {
    _command.trimRoll += delta;  
  }

  const UdircCommand& command() {
    // We only want to assert our cmd flags for 1s
    auto timeSinceLastCmd = ms(_clock.now(), _lastCmdTime);
    if (timeSinceLastCmd > 800) {
      _command.cmd = UDIRC_CMD_PRESENT;
      _command.yaw = 0x80;
      _command.pitch = 0x80;
      _command.roll = 0x80;
      _command.throttle = 0x80;
    }

    return _command;
  }

  static std::string display(const UdircCommand& command) {
    std::stringstream s;
    s << "  throttle: " << int(command.throttle)/128. << std::endl;
    s << "       yaw: " << int(command.yaw - 128)/128. << std::endl;
    s << "     pitch: " << int(command.pitch - 128)/128. << std::endl;
    s << "      roll: " << int(command.roll - 128)/128. << std::endl;
    s << "  trim yaw: " << int(command.trimYaw) << std::endl;
    s << "trim pitch: " << int(command.trimPitch) << std::endl;
    s << " trim roll: " << int(command.trimRoll) << std::endl;
    s << "       cmd: " << int(command.cmd) << std::endl;
    return s.str();
  }

private:
  Clock             _clock;
  UdircCommand      _command;
  Clock::time_point _lastCmdTime;
};

int main(int argc, char* argv[])
{
  try
  {
    // we'll drive the even loop manually for now
    boost::asio::io_service io;
    UdircConnection conn(io);
    UdircController ctrl;

    cv::namedWindow("udirc", cv::WINDOW_AUTOSIZE);

    cv::Mat previousFrame = cv::Mat();
    while(true) {
      auto frame = conn.video();
      if (!frame.empty()) {

        // We need to convert to greyscale float for phase correlation
        cvtColor(frame, frame, CV_RGB2GRAY);
        frame.convertTo(frame, CV_32FC1, 1./255.);

        cv::imshow("udirc", frame);
        // If we have a previous frame
        if (!previousFrame.empty()) {
          std::cerr << previousFrame.type() << std::endl;
          auto diff = phaseCorrelate(previousFrame, frame);  
        
          // due to a bunch of corruption, we filter aggressively here
          if (norm(diff) < 10. && norm(diff) > 0.1) {
            if (diff.x > 0) {
              //std::cout << "left" << std::endl;
              ctrl.trimRoll(-3);
            }
            if (diff.x < 0) {
              //std::cout << "right" << std::endl;
              ctrl.trimRoll(+3);
            }
  /*
            if (diff.y < 0) {
              std::cout << "down" << std::endl;
            }
            if (diff.y > 0) {
              std::cout << "up" << std::endl;
            }
  */
          }
        }
        
        previousFrame = frame;
      }

      auto k = cv::waitKey(1);
      const uint8_t SENSITIVITY = 0x50;
      switch (k) {
        case ' ':
          ctrl.disable();
          break;
        case 'q':
          ctrl.throttle(-SENSITIVITY);
          break;
        case 'e':
          ctrl.throttle(SENSITIVITY);
          break;
        case 's':
          ctrl.pitch(-SENSITIVITY);
          break;
        case 'w':
          ctrl.pitch(SENSITIVITY);
          break;
        case 'a':
          ctrl.roll(-SENSITIVITY);
          break;
        case 'd':
          ctrl.roll(SENSITIVITY);
          break;
        case 't':
          ctrl.takeoff();
          break;
        case 'l':
          ctrl.land();
          break;
        default:
          break;
      }

      auto command = ctrl.command();
      std::cout << ctrl.display(command);
      conn.command(command);

    }
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
