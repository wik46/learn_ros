cmake_minimum_required(VERSION 3.0.2)
project(example_publisher)


find_package(catkin REQUIRED roscpp geometry_msgs)
## Declare as a catking package.
catkin_package()

## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
## Each cpp file needs this statement.
## 1.
add_executable(publish_velocity src/pub_vel.cpp)
## 2.
add_executable(anon_node src/anon_node.cpp)
## 1.
target_link_libraries( publish_velocity
   ${catkin_LIBRARIES}
 )
## 2. 
target_link_libraries( anon_node
	${catkin_LIBRARIES}
)
