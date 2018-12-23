cmake_minimum_required(VERSION 2.8.3)
project(hector_quadrotor_smart_drone)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  genmsg 
  actionlib_msgs
  actionlib
)
add_action_files(DIRECTORY action FILES Drone.action)
generate_messages(DEPENDENCIES actionlib_msgs)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hector_keyboard_teleop
#  CATKIN_DEPENDS geometry_msgs rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


