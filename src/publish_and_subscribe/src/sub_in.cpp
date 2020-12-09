/*
	Author: Wikus Jansen van Rensburg
	Date: December 6, 2020.
	Description:* This is a simple subcriber node
		      that will subscribe to the /turtle1/pose
		      topic that sends turtlesim/Pose messages.
		    * It will also subcribe to the 
			that are being published by the
			pub_out node defined in this source file
		        geometry_msgs/Twist.

*/
// Just for basic input and output.
#include <iostream>
// A C++ header to format ouput, std::setw and std::setprecision.
#include <iomanip>
// For basic ros functionality.
#include <ros/ros.h>
// This is to gain access to the message type I wnat to subcribe to.
#include <turtlesim/Pose.h>
// 2nd message type that the node will subscribe to.
#include <geometry_msgs/Twist.h>

// A subcriber node is a bit different thatn a publisher node in that
// it needs a call back function. This is a function that accepts a const
// reference to the message type class. In this example it is turtlesim::Pose.
// This function should be passed as the third argument int the .subscriber method.

/*
	Also take note that we very seldom call the .subcriber() method.
	We only define it and tell ros to continue subcribing untill we stop
	the program. We almost never call it explicitly.
	2 Ways to let ros call our subcriber.
		I. ros::spin() : Which continues untill the node is killed.
		II. ros::spinOnce(): This will only call it once.
			Therefore to use it, write
			while(ros::ok()){
				ros::spinOnce();
			}
	The second way is more helpful if we have other tasks to do as well.
*/
// This is the class that we have to work with.
// The turtlesim node is publishing these position of the turtle on the
// turtle1/pose topic.

/*
	This is a cpp class with 5 member variables/ fields.
	Message type: turtlesim/Pose structure
			float32 x
			float32 y
			float32 theta
			float32 linear_velocity
			float32 angular_velocity
*/

// This function is ourcall back function.
void get_data_from_topic( const turtlesim::Pose& p){
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
	<< "turtlesim::Pose.x: " << p.x << ", turtlesim::Pose.y: "
	<< p.y <<
	", turtlesim::Pose.linear_velocity: " << p.linear_velocity
	<< ", turtlesim::angular_velocity: " << p.angular_velocity);
	std::cout << std::endl;
	std::cout << "Easy to see:"<< std::endl;
	std::cout << "Position: (" << p.x << ", " << p.y << 
	") and direction: " << p.theta << std::endl;
	return;
}


int main( int argc, char** argv){
	// This turns the cpp file into a ros node.
	ros::init( argc, argv, "sub_in"); // Nb the string is the name of the node.
	// Creating a handler object.
	ros::NodeHandle nh;
	// Here is the difference between a node that subcribes from a topic
	// compared to one that publishes on a topic.
	// 1. "turlte1/pose" it the topic name we subcribe to.
	// 2. Is the queu size: The number of messages we can store.
	// 3. Is ou callback function that is used in subcribers but not publishers.
	ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, get_data_from_topic);
	// Now this is the ros loop
	// Here Ros takes over and will call our callback function untill we kill
	// the node.
	// The callback function is executed each time a new message arrives.
	while(ros::ok()){
		ros::spinOnce();
	}
}
