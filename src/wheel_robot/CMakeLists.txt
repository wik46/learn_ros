cmake_minimum_required(VERSION 3.0.2)
project(wheel_robot)

find_package(catkin REQUIRED
	roscpp ## We need this to add the c++ compiler
	sensor_msgs ## because we include sensor_msgs/JointState.h
	geometry_msgs ## because we include geometry_msgs/TransformStamped.h
	tf2_ros ## We use transform frames.	
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wheel_robot
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# (We just uncomment this from the original file.)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Remember that the name of the executable and the name that you
## set for the node inside the ros::init() method needs to be the same.
# Here you tell the compiler the name of the executable and where to find
# the executable. The file will come with a standard way, but you can
# rename the executable if you change the name of it and the source
# file.
# For each different node in a package, you need to define this.
add_executable(circle_mover src/circle_mover.cpp)

# Here you specify the name of the executable again.
target_link_libraries(circle_mover
  ${catkin_LIBRARIES}
)

