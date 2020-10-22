cmake_minimum_required(VERSION 3.5)
project(ctr_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(ctr_msgs
   srv/State.srv
   srv/Elementrequest.srv
   srv/Inforequest.srv
   srv/Fieldinformationrequest.srv
   msg/Command.msg
   msg/Robot.msg
   msg/Visionpkg.msg
   msg/Position.msg
   msg/Velocity.msg
   msg/Navigation.msg
   msg/Path.msg
   msg/State.msg
   action/Nav.action
   DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
