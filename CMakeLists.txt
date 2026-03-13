cmake_minimum_required(VERSION 3.0.2)
project(crsf_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  CrsfFrame.msg
  CrsfLinkStatistics.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(crsf_control_node
  src/main.cpp
  src/crsf_protocol.cpp
  src/crsf_frame_stream_parser.cpp
  src/crsf_telemetry_decoder.cpp
  src/serial_port.cpp
  src/crsf_node.cpp
)

add_dependencies(crsf_control_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(crsf_control_node
  ${catkin_LIBRARIES}
  pthread
)
