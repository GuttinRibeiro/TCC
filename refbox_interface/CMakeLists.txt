cmake_minimum_required(VERSION 3.5)
project(refbox_interface)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

cmake_policy(SET CMP0071 NEW)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Protobuf REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS src/referee.proto)

add_executable(refbox_node src/refbox_node.cpp ${PROTO_SRCS} ${PROTO_HDRS})
ament_target_dependencies(refbox_node rclcpp)

target_include_directories(refbox_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(refbox_node Qt5::Widgets ${Protobuf_LIBRARIES})

install(TARGETS refbox_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()