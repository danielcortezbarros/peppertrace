cmake_minimum_required(VERSION 3.0.2)
project(programming_by_demonstration)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

# ## Declare message files (if any)
# add_message_files(
#   FILES
#   # Add message files shared across the package here
# )

# ## Declare service files (if any)
# add_service_files(
#   FILES
#   # Add service files shared across the package here
# )

## Generate added messages and services
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

## Declare the catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

## Include all subdirectories
add_subdirectory(demonstration_gui)
add_subdirectory(demonstration_recorder)
add_subdirectory(skeletal_model)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
