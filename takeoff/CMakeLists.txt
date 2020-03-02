cmake_minimum_required(VERSION 2.8.3)
project(takeoff)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  gazebo_msgs
  std_srvs
  ardrone_autonomy
  tf
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GroundtruthAltitude.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  takeoff
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## Specify libraries to link a library or executable target against

add_executable(takeoff src/takeoff)
target_link_libraries(takeoff ${catkin_LIBRARIES})

add_executable(robot_state_broadcaster src/robot_state_broadcaster.cpp)
target_link_libraries(robot_state_broadcaster ${catkin_LIBRARIES})
add_dependencies(robot_state_broadcaster takeoff_gencpp)

add_executable(ardrone_teleop src/ardrone_joystick.cpp)
target_link_libraries(ardrone_teleop ${catkin_LIBRARIES})
