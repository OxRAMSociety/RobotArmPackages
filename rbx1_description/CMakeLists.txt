cmake_minimum_required(VERSION 2.8.3)
project(rbx1_description)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS
    gazebo_ros
)

find_package(roslaunch)

install(DIRECTORY meshes launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
