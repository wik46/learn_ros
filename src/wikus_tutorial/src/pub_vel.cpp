// Author: Wikus
// Learning resource: A Gentle Introduction to Ros.
// Date: December 4, 2020.

/*
	This is a program used to study how to publish
	messages onto topics. In particular, we are going
	to publish geometry_msgs/Twist on the /turtle1/cmd_vel
	topic. We are going to prompt the user on the velocity
	and direction in which the turtle should move.
*/


#include <iostream>

// This is need for all the basic ros node functionality.
#include <ros/ros.h>
// This is needed to use the geometry message.
// Format '<package-name/type-name.h>'
// The purpose of this header is to define the cpp class
// that contains the fields needed. The class is defined in the 
// namespace after the package.
// Therefore to used the data class, geomety_msgs::Twist.
// This class has to upper level classes. (linear and angular)
// Each of these two have 3 fields each which are all floats.
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv){
	// Standard in roscpp to turn this code into a Ros node.
	ros::init(argc, argv, "publisher_name"); // Name of the node.
	ros::NodeHandle nh;
	// Now we create a ros publisher object.
	// Publisher is a class that lives in the ros namespace.
	// We use the advertise method defined in the ros::NodeHandle class.
	// We need to specify the message type as a template argument.
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000); // Name of the topic and the size of the queue

	// Now we start with the ros loop.
	// This will publish a message at a rate of 2Hz
	ros::Rate rate(2); // This determines the rate at which the messages
			// are going to be published.

	while(ros::ok()){ // This will continue untill node is shutdown.
	// Create an instance of a geometry_msgs::Twist
	// so that we can pass the correct data that needs to be published.
	// Note that our publisher object can only publish geometry_msgs::Twist
	// therefore it is the datatype that needs to be passed as an argument
	// to the publisher method.
	geometry_msgs::Twist msg;
	// This is only so that I can see how to publish.
	std::cout << "Enter the velocity:";
	std::cin >> msg.linear.x; // 
	std::cout << "Enter the angle tow turn ( -1 to 1):";
	std::cin >> msg.angular.z; // 
	// Here we publish the message to the topic 'turtle1/cmd_vel'
	pub.publish(msg);
	// This is only to see what messages 'data' are being publish.
	ROS_INFO_STREAM("Veloctity sent: msg.linear.x: " << 
	msg.linear.x << " and msg.angular.z: " << msg.angular.z);
	// Wait untill it is time for another iteration.
	rate.sleep();
	}
}
