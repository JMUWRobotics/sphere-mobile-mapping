cmake_minimum_required(VERSION 2.8.3)
project(imu_odom_phidgets)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  state_estimator_msgs
)

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
   CATKIN_DEPENDS geometry_msgs sensor_msgs roscpp message_runtime
)

add_compile_options(-std=c++11)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(imuJasper src/imuJasper.cpp)
add_dependencies(imuJasper state_estimator_msgs_generate_messages_cpp)
list( APPEND CMAKE_INSTALL_RPATH "/opt/glibc/lib")
target_link_libraries(imuJasper -lphidget22 -lm -L/opt/glibc/lib -dynamic-linker=/opt/glibc/lib/ld-linux-x86-64.so.2 ${catkin_LIBRARIES})
