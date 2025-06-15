cmake_minimum_required(VERSION 3.0.2)
project(display_module)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(PkgConfig)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python scripts
catkin_install_python(PROGRAMS
  src/show_stats_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()
