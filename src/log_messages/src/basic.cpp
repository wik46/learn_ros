/*
	Author: Wikus Jansen van Renburg
	Description: This is program to learn one of the basic
			methods to used ros log messages
		     There is five types of messages
		     base on the severity of the message.
*/

// This is the header needed for basic ros functionality.
#include <ros/ros.h>

int main(int argc, char** argv){
	// These first two lines of condes turn this 
	// cpp file into a ros node.
	// Remember that this is the name of the node
	// 	and this must be the name of the executble
	// defined inside the CMakelist.txt.
	ros::init(argc, argv, "log_messages1");
	// We also need to create a node handler object.
	ros::NodeHandle nh;
	// Now this will set the rate at which messages are published.
	// For this program we will only send log messages.
	ros::Rate rate(10); // Publishes at a rate of 10hz.
	// This will be our loop to publish constant log messages.
	for(int i = 1; ros::ok(); i++){
		
		// 1. Type one logs message.
		if(i % 2 == 0){
			ROS_INFO_STREAM( i << " is divisible by 2");
		}else if(i % 3 == 0){
			ROS_WARN_STREAM(i << " is divisible by 3");
		}else if(i % 5 == 0){
			ROS_ERROR_STREAM(i << "is divisible by 5");
		}else if(i % 7 == 0){
			ROS_FATAL_STREAM(i << " is divisible by 7");
		}else{
		//	ROS_DEBUG_STREAM(i << " was printed by debug");
		}
	// This is used so that ros sends message according 
	// to the rate that we specified above.
		rate.sleep();
	}
}
