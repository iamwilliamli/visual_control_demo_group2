cmake_minimum_required(VERSION 3.0.2)
project(paul_alarm)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  message_generation
)

# Add message files
add_message_files(
  FILES
  DualImage.msg
)

# Generate added messages and services
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

# Ensure that the package depends on message generation
catkin_package(
  CATKIN_DEPENDS message_runtime
)

# Declare a catkin package
catkin_package()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/dual_image_subscriber_node.py
  src/alarm_omni.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)