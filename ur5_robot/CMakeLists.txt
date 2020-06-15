################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(ur5_robot)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  sensor_msgs
)
find_package(PCL REQUIRED)
################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
## Generate services in the 'srv' folder
add_service_files(
   FILES
   TfPair.srv
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencv_example2
   CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
include_directories(include 
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}) 



################################################################################
# Install
################################################################################
install(DIRECTORY launch ur5_robot param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
