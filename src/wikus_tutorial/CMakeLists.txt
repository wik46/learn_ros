cmake_minimum_required(VERSION 3.0.2)
project(wikus_tutorial)

## Add roscpp and geometry_msgs.
find_package(catkin REQUIRED roscpp geometry_msgs)

catkin_package()

## Uncimment all the code here.
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Uncomment this line of code.
## 'publisher_exe': Is the name of the executable that will be created from this package.
## 'pub_vel.cpp': The name of the .cpp file that the executable needs to compile the program.
## ** Note that if the executable needs more than one .cpp file you add them in list seperated by a
## 		space.
## NB!! This must be the same name that you used in ros::init()
add_executable(publisher_name src/pub_vel.cpp)

## Here you tell the build system the name of the executable.
target_link_libraries(publisher_name
  ${catkin_LIBRARIES}
)
