cmake_minimum_required(VERSION 3.0.2)
project(pub_sub_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp gazebo_ros gazebo_msgs)

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(example_sub_LinkStates src/sub1.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(example_sub_LinkStates
   ${catkin_LIBRARIES}
 )
