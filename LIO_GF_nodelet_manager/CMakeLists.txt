cmake_minimum_required(VERSION 3.0.2)
project(lio_gf_nodelet_manager)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
)

catkin_package()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)