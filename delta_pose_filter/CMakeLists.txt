cmake_minimum_required(VERSION 3.0.2)
project(delta_pose_filter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  tf
  message_runtime
  state_estimator_msgs
)

find_package(Eigen3 REQUIRED)

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
#  LIBRARIES delta_pose_filter
   CATKIN_DEPENDS message_runtime state_estimator_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS} 
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(filter_node src/delta_filter_node.cpp)
add_executable(vizualizer src/tf_odom_to_maps.cpp)
add_executable(kalman_filter_node src/kalman_filter_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(kalman_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(kalman_filter_node ${catkin_EXPORTED_TARGETS} state_estimator_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(filter_node
  ${catkin_LIBRARIES}
)

target_link_libraries(vizualizer
  ${catkin_LIBRARIES}
)

target_link_libraries(kalman_filter_node
  ${catkin_LIBRARIES}
)