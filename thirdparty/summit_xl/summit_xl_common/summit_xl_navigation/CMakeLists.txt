cmake_minimum_required(VERSION 2.8.3)
project(summit_xl_navigation)

find_package(catkin REQUIRED COMPONENTS)

catkin_package( CATKIN_DEPENDS )

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY launch config maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
