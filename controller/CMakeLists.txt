cmake_minimum_required(VERSION 3.5)
project(controller)

set(OpenGL_GL_PREFERENCE LEGACY)
set(USE_QT5 TRUE CACHE BOOL "Use Qt5 instead of Qt4")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenGL REQUIRED)

if(USE_QT5)
        set(CMAKE_INCLUDE_CURRENT_DIR ON)
        set(CMAKE_AUTOMOC ON)
        find_package(Qt5Core REQUIRED)
        find_package(Qt5Widgets REQUIRED)
        #find_package(Qt5Network REQUIRED)
        find_package(Qt5OpenGL REQUIRED)
        # replaced by automoc
        macro(qt4_wrap_cpp VARNAME)
                set(${VARNAME} "")
        endmacro()
        # wrap functions
        macro(qt4_wrap_ui)
                qt5_wrap_ui(${ARGN})
        endmacro()
        macro(qt4_add_resources)
                qt5_add_resources(${ARGN})
        endmacro()
        set(QT_LIBRARIES "")
else()
        find_package(Qt4 4.8.0 COMPONENTS QtCore QtGui QtOpenGL QtNetwork REQUIRED)
        include ( ${QT_USE_FILE} )
endif()

find_package(rclcpp REQUIRED)
find_package(ctr_msgs REQUIRED)

## generate moc files for graphicalClient2
qt4_wrap_cpp(GCLIENT_MOC_SRCS
    soccerview.h
)

## shared qt wrappings

qt4_wrap_cpp (SHARED_MOC_SRCS
        ${SHARED_HEADERS}
)

qt4_add_resources(SHARED_RC_SRCS
        ${SHARED_RC}
)

## build the common code
add_library(sslvision ${SHARED_MOC_SRCS} ${SHARED_RC_SRCS} ${CC_PROTO} ${SHARED_SRCS})

set (libs ${QT_LIBRARIES}
        ${OPENGL_gl_LIBRARY}
        ${OPENGL_glu_LIBRARY}
)

target_link_libraries(sslvision ${libs})
if(USE_QT5)
        qt5_use_modules(sslvision Widgets)
endif()
set (libs ${libs} sslvision)

# Build worldmap
add_executable(worldmap ${GCLIENT_MOC_SRCS} src/map.cpp src/map/map_node.cpp src/utils/entity.cpp src/map/worldmap.hpp src/utils/vector.cpp
    src/map/gui/gltext.cc src/map/gui/soccerview.cc src/map/gui/util/field.cc src/map/exithandler.cc)
ament_target_dependencies(worldmap rclcpp ctr_msgs)

target_link_libraries(worldmap ${libs})
if(USE_QT5)
        qt5_use_modules(worldmap Widgets OpenGL)
endif()

# Build controller
add_executable(ssl_controller src/ssl_robot_ctr.cpp src/utils/entity.cpp src/controller/controller.cpp
    src/utils/vector.cpp src/controller/ssl_controller.cpp
    src/utils/utils.cpp src/utils/infobus.cpp src/controller/state.cpp)
ament_target_dependencies(ssl_controller rclcpp ctr_msgs)

target_link_libraries(ssl_controller ${libs})
if(USE_QT5)
        qt5_use_modules(ssl_controller Widgets)
endif()

# Build navigation
add_executable(ssl_navigation src/ssl_nav_node.cpp src/utils/entity.cpp src/navigation/navigation.cpp
    src/utils/vector.cpp src/utils/utils.cpp src/utils/infobus.cpp
    src/navigation/navalg/navigation_algorithm.cpp src/navigation/navalg/pf.cpp
    src/navigation/ctralg/control_algorithm.cpp src/navigation/ctralg/discrete_pid.cpp)
ament_target_dependencies(ssl_navigation rclcpp rclcpp ctr_msgs)

target_link_libraries(ssl_navigation ${libs})
if(USE_QT5)
        qt5_use_modules(ssl_navigation Widgets)
endif()

# Install
install(TARGETS worldmap ssl_controller ssl_navigation
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
