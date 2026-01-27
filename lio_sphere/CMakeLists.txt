cmake_minimum_required(VERSION 3.0.2)
project(lio_sphere)
set(CMAKE_BUILD_TYPE "Release")


include(FindOpenMP)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
geometry_msgs
nav_msgs
pcl_ros
tf2_ros
state_estimator_msgs
)

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lio_sphere
#  CATKIN_DEPENDS other_catkin_pkg
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

add_executable(lio_node
src/lio_node.cpp
src/distortion.cpp
src/helpers.cpp
src/external/ikd-Tree/ikd_Tree.cpp
src/external/kiss_icp/VoxelHashMap.cpp
src/external/kiss_icp/Threshold.cpp
src/registration.cpp
src/transformation_comparison.cpp
src/registration_gicp.cpp
src/GraphSLAM/graph_slam.cpp
src/GraphSLAM/graph.cpp
src/GraphSLAM/scan.cpp
)
add_executable(lio_comp
src/time_comp.cpp
)

target_link_libraries(lio_node
   ${catkin_LIBRARIES}
   Eigen3::Eigen
   TBB::tbb
 )
target_link_libraries(lio_comp
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

#############
## Install ##
#############


#############
## Testing ##
#############