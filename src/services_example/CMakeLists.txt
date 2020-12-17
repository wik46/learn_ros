cmake_minimum_required(VERSION 3.0.2)
project(services_example)

## is used, also find other catkin packages
find_package (catkin REQUIRED roscpp turtlesim geometry_msgs)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
)

## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## The recommended prefix ensures that target names across packages don't collide
## 1.
add_executable(client_user_options src/client_example.cpp)
## 2.
add_executable(server_move_turtle src/server_example.cpp)

## 1.
## Specify libraries to link a library or executable target against
target_link_libraries( client_user_options 
   ${catkin_LIBRARIES}
)
## 2,
## This is used for my server node.
target_link_libraries(
	server_move_turtle
	${catkin_LIBRARIES}
)
