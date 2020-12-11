/*
	Author: Lodewyk Jansen van Rensburg
	Description: This is a simple hello world program
			in ROS.
		     We create a node that will print out 
			'hello world'

*/

// 1. This is a header file that will be included in all the Ros programs.
// This headerfile defines standard Ros classes.
#include <ros/ros.h>

int main(int argc, char** argv){
	// 2. This initializes the Ros system.
	ros::init(argc, argv, "hello_ros");
	// It initializes the Ros client library. 
	/* The last parameter is the name of the node.*/
	
	// 3. This turns our cpp file into a ros node.
	ros::NodeHandle nh;

	// 4. This sends outpus to various locations
	// 	one including the console.
	ROS_INFO_STREAM("Hello ROS !!!");
}
