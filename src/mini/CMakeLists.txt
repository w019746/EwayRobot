cmake_minimum_required(VERSION 2.8.3)
project(mini)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
  tf
)
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	controlOutput.msg
 	naviLoc.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(control src/control.cpp)
target_link_libraries(control ${catkin_LIBRARIES})
add_dependencies(control mini_generate_messages_cpp)

add_executable(nav src/nav.cpp)
target_link_libraries(nav ${catkin_LIBRARIES})
add_dependencies(nav mini_generate_messages_cpp)

add_executable(SLAM_offline src/SLAM_offline.cpp)
target_link_libraries(SLAM_offline ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(SLAM_offline mini_generate_messages_cpp)

