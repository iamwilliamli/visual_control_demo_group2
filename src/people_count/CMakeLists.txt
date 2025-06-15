cmake_minimum_required(VERSION 3.0.2)
project(people_count)

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
  MotorCommand.msg
  MotorCommandStatus.msg
  DualImages.msg
)

# Generate added messages and services
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
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
  src/dual_images_subscriber_node_alex.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
