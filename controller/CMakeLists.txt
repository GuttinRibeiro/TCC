cmake_minimum_required(VERSION 3.5)
project(controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ctr_msgs REQUIRED)
find_package(vision REQUIRED)

add_executable(ssl_controller src/ssl_robot_ctr.cpp src/controller/controller.cpp
    src/utils/vector.cpp src/controller/ssl_controller.cpp
    src/utils/utils.cpp)
ament_target_dependencies(ssl_controller rclcpp ctr_msgs)

add_executable(worldmap src/map.cpp src/map/map_node.cpp src/map/worldmap.hpp src/utils/vector.cpp)
ament_target_dependencies(worldmap rclcpp ctr_msgs vision)

target_link_libraries(ssl_controller Qt5::Widgets)
target_link_libraries(worldmap Qt5::Widgets)


install(TARGETS ssl_controller worldmap
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
