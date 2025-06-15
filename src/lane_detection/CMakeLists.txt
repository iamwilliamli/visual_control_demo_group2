cmake_minimum_required(VERSION 3.0.2)
project(lane_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  message_generation
)

catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/lane_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)