cmake_minimum_required(VERSION 2.8.3)
project(summit_xl_pad)

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    nav_msgs 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    diagnostic_updater
    )
    
add_service_files(DIRECTORY srv 
   FILES
    enable_disable_pad.srv
)

generate_messages(
   DEPENDENCIES
   robotnik_msgs sensor_msgs summit_xl_pad
 )

catkin_package(
  CATKIN_DEPENDS 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    message_runtime
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)

add_executable(summit_xl_pad src/summit_xl_pad.cpp)

add_dependencies(summit_xl_pad summit_xl_pad_generate_messages_cpp)

target_link_libraries(summit_xl_pad
   ${catkin_LIBRARIES}
 )

