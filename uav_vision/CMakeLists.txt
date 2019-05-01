cmake_minimum_required(VERSION 2.8.3)
project(uav_vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  OpenCV
  image_view
  tf
  std_srvs
  cvg_sim_msgs
  ardrone_autonomy
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS thread)


catkin_package(
  INCLUDE_DIRS include
)


include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS}) 
include_directories(${Boost_INCLUDE_DIRS})
include_directories(include)



#########################################################################################################
# platform_detection NODE
add_executable(platform_detection src/platform_detection.cpp)
target_link_libraries(platform_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# platform_detection_real NODE
# add_executable(platform_detection_real src/platform_detection_real.cpp)
# target_link_libraries(platform_detection_real ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#########################################################################################################

#########################################################################################################
# indicator_detection NODE
# add_executable(indicator_detection src/indicator_detection.cpp)
# target_link_libraries(indicator_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# platform_tracking_real NODE
#add_executable(platform_tracking_real src/platform_tracking_real.cpp)
#target_link_libraries(platform_tracking_real ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
#########################################################################################################

#########################################################################################################
# platform_tracking NODE
add_executable(platform_tracking src/platform_tracking.cpp)
target_link_libraries(platform_tracking ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# platform_tracking_real NODE
# add_executable(platform_tracking_real src/platform_tracking_real.cpp)
# target_link_libraries(platform_tracking_real ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
#########################################################################################################



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS vision vision_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vision.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

