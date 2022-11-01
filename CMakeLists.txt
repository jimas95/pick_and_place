cmake_minimum_required(VERSION 3.0.2)
project(pick_and_place)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    message_generation
    rospy
    std_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  go_to.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  pickAndPlace.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs 
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
 CATKIN_DEPENDS actionlib_msgs message_runtime std_msgs 
)

###########
## Build ##
###########

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

catkin_install_python(PROGRAMS
  nodes/brick_builder.py
  nodes/action_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)