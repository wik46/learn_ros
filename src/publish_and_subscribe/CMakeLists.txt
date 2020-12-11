cmake_minimum_required(VERSION 3.0.2)
project(publish_and_subscribe)

find_package(catkin REQUIRED roscpp geometry_msgs turtlesim)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## These two below lines should be copied for each distinc executable.
## This needs to be the name of the executable and all the source files
## needed to compile it. NB! Executable name needs to be same as name
## inside ros::init();
add_executable(pub_out src/pub_out.cpp)
add_executable(sub_in src/sub_in.cpp)

## Add the name of you executable
target_link_libraries(pub_out ${catkin_LIBRARIES})
## You need this line for each executable you package has.
target_link_libraries(sub_in ${catkin_LIBRARIES})
