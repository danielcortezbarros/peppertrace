cmake_minimum_required(VERSION 3.0.2)
project(pepper_demonstration_recorder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

# add_message_files(
#   FILES
#   JointCommands.msg
# )

# add_service_files(
#   FILES
#   file.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

install(PROGRAMS
  src/pepper_demonstration_recorder/pepperRecordingApplication.py
  src/pepper_data_processing/bag_reader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
