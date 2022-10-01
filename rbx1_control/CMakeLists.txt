cmake_minimum_required(VERSION 2.8.3)
project(rbx1_control)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
)

catkin_package()

find_package(roslaunch)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
