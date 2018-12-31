#include <iostream>

#include <boost/asio.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "UdircConnection.h"
#include "UdircController.h"

int main(int argc, char* argv[])
{
  try
  {
    // The UdircConnection takes care of communicating with the drone.  This includes
    // any state/timing as well as the actual wire protocol and data transport.
    boost::asio::io_service io;
    UdircConnection conn(io);

    // The UdircController takes care of generating an appropriate command stream from 
    // user-input.  For example, when a 'takeoff' command is given, the controller will
    // ensure that the UDIRC_CMD_TAKEOFF bit is asserted for at least 1 second.
    //
    // The UdircController also ensures that the controller returns to a neutral state 
    // after some time.
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
          if (ctrl.neutral() && norm(diff) < 10. && norm(diff) > 0.1) {
            if (diff.x > 0) {
              //std::cout << "left" << std::endl;
              ctrl.trimRoll(-3);
            }
            if (diff.x < 0) {
              //std::cout << "right" << std::endl;
              ctrl.trimRoll(+3);
            }
          }
        }
        
        previousFrame = frame;
      }

      auto k = cv::waitKey(1);
      const uint8_t SENSITIVITY = 0x10;
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
        case 'S':
          ctrl.trimPitch(-2);
          break;
        case 'W':
          ctrl.trimPitch(2);
          break;
        case 'A':
          ctrl.trimRoll(-2);
          break;
        case 'D':
          ctrl.trimRoll(2);
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
