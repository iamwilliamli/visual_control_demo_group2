cmake_minimum_required(VERSION 3.0.2)
project(button_mode_switcher)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/button_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
