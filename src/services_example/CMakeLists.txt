cmake_minimum_required(VERSION 3.0.2)
project(services_example)

## is used, also find other catkin packages
find_package (catkin REQUIRED roscpp turtlesim)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
)

## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## The recommended prefix ensures that target names across packages don't collide
add_executable(client_user_options src/client_example.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( client_user_options
   ${catkin_LIBRARIES}
)

