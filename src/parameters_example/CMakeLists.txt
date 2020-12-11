cmake_minimum_required(VERSION 3.0.2)
project(parameters_example)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp std_srvs)
catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## The recommended prefix ensures that target names across packages don't collide
add_executable(parameter_ex1 src/parameters1.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(parameter_ex1
   ${catkin_LIBRARIES}
)

