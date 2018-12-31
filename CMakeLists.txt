cmake_minimum_required(VERSION 3.13)
project(udirc_example)
set(CMAKE_CXX_STANDARD 14)

find_package(Boost COMPONENTS system REQUIRED)
find_package(OpenCV COMPONENTS highgui REQUIRED)

add_executable(
  udirc_example
  UdircExample.cpp)

target_include_directories(
  udirc_example
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
)

target_link_libraries(
  udirc_example
 
  PRIVATE Boost::system opencv_highgui
)
