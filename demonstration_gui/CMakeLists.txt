cmake_minimum_required(VERSION 3.0.2)
project(demonstration_gui)

## Find dependencies
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add Python scripts
catkin_install_python(PROGRAMS
  src/demonstration_gui_application.py  # Replace with your main Python application file
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
