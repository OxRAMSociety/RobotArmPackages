cmake_minimum_required(VERSION 3.0.2)
project(rbx1_hardware_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    message_generation
)

# find_package(Boost REQUIRED system filesystem date_time thread)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ArmJointState.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES rbx1_hardware_interface
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
#  DEPENDS system_lib
  DEPENDS message_runtime
)

###########
## Build ##
###########

# include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

## Declare a C++ executable
add_executable(angles_to_steps src/angles_to_steps.cpp)

target_link_libraries(angles_to_steps ${catkin_LIBRARIES})