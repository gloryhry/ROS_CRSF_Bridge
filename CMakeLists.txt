cmake_minimum_required(VERSION 3.0.2)
project(crsf_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(crsf_control_node
  src/main.cpp
  src/crsf_protocol.cpp
  src/serial_port.cpp
  src/crsf_node.cpp
)

target_link_libraries(crsf_control_node
  ${catkin_LIBRARIES}
  pthread
)
