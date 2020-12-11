// Author: Wikus Jansen van Rensburg
// Date: December 5, 2020.
// Description: This is an example of creating a subscriber 
//  		program that will take in the geometry_msgs 
//		output them to the screen

// This is to get the basic functionality needed to
// make this cpp file a ros node.
#include <ros/ros.h>
// This is the first type of data that this program is going
// to subscribe to
// The turtlesim_node publishes on the /turtle1/pose topic.
// How to find this header file?
/* 1. Find the topic you need to subcribe from. (rostopic list)
   2. Find the type of message that the topic uses. (rostopic info 'topic-name')
   3. Find the primitive data types used by the msg (rosmsg show 'message-name')
   4. The last command, in this case, returned 'turtlesim/Pose'
      The naming convention of Ros is to now just add a .h
 */
#include <turtlesim/Pose.h> 
// This is needed because the program is going to subscribe
// from a topic with the /turtle1/cmd_vel topic that
// uses these message types.
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
	// These two steps are needed in any ros node program.
	// First step is to initialize the node.
	// The string is the name of the node.
	ros::init(argc, argv, "custom_subscriber");
	// This creates the nodehandler.
	// This nodehandle object is used to make this node a publisher or a subcriber.
	ros::NodeHandle nh;
	

}
