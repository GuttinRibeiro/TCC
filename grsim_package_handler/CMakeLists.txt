cmake_minimum_required(VERSION 3.5)
project(grsim_package_handler)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(grsim_node src/grsim_node.cpp)

target_include_directories(grsim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(grsim_node Qt5::Widgets)

install(TARGETS grsim_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
