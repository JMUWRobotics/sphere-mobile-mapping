cmake_minimum_required(VERSION 3.0.2)
project(lio_sphere)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FindOpenMP)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
roscpp
nodelet
pluginlib
lio_gf_nodelet_manager
sensor_msgs
geometry_msgs
nav_msgs
pcl_ros
tf2_ros
state_estimator_msgs
evaluation_msgs
)

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lio_sphere_core lio_sphere_nodelet
#  CATKIN_DEPENDS roscpp nodelet pluginlib  sensor_msgs geometry_msgs nav_msgs pcl_ros tf2_ros state_estimator_msgs evaluation_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
 src/external/ikd-Tree
 src/external/robin-map/include
 src/external/kiss_icp
 src/external/nanoflann/include
 src/GraphSLAM
)

add_library(lio_sphere_core
src/lio_node.cpp
src/distortion.cpp
src/helpers.cpp
src/external/ikd-Tree/ikd_Tree.cpp
src/external/kiss_icp/Threshold.cpp
src/registration_gicp.cpp
src/GraphSLAM/graph_slam.cpp
src/GraphSLAM/graph.cpp
src/GraphSLAM/scan.cpp
)

target_link_libraries(lio_sphere_core
   ${catkin_LIBRARIES}
   Eigen3::Eigen
   TBB::tbb
)

add_executable(lio_node
src/lio_node_main.cpp
)

target_link_libraries(lio_node
  lio_sphere_core
)

add_library(lio_sphere_nodelet
  src/lio_nodelet.cpp
)

target_link_libraries(lio_sphere_nodelet
  lio_sphere_core
  ${catkin_LIBRARIES}
)

install(TARGETS lio_node lio_sphere_core lio_sphere_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)