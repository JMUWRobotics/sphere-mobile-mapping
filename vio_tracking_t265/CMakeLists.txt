cmake_minimum_required(VERSION 3.0.2)
project(vio_tracking_t265)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  tf 
  tf2
  std_msgs
  state_estimator_msgs
  image_transport
)

if (WIN32)
find_package(realsense2 CONFIG REQUIRED)
else()
find_package(realsense2 2.50.0)
endif()

if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS geometry_msgs sensor_msgs roscpp message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${realsense2_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS}
)

if (WIN32)
else()
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")
endif()

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(auto_reconnect src/auto_reconnect.cpp)
add_executable(center_extrinsics src/center_extrinsics.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(auto_reconnect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(auto_reconnect state_estimator_msgs_generate_messages_cpp)
add_dependencies(center_extrinsics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(center_extrinsics state_estimator_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(auto_reconnect
   ${realsense2_LIBRARY}
   ${catkin_LIBRARIES}
)

target_link_libraries(center_extrinsics
   ${realsense2_LIBRARY}
   ${catkin_LIBRARIES}
)
