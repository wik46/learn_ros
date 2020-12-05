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

add_executable(publish_velocity src/pub_vel.cpp)

target_link_libraries(publish_velocity
   ${catkin_LIBRARIES}
 )
