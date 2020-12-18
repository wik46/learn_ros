cmake_minimum_required(VERSION 3.0.2)
project(rviz_visual)

## Here we define all the packages that our package needs in order
## to visualize the rover arm.
find_package(catkin REQUIRED COMPONENTS
  roscpp roslaunch joint_state_publisher robot_state_publisher gazebo_ros
  rviz
)

catkin_package(
)
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

