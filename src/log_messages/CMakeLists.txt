cmake_minimum_required(VERSION 3.0.2)
project(log_messages)

find_package(catkin REQUIRED roscpp)

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
## Remember that the name of the executable should agreee with
## the name that you specified inside the node. 
add_executable(log_messages1 src/basic.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(log_messages1
   ${catkin_LIBRARIES}
 )

