/*
	Author: Wikus Jansen van Rensburg
	Description: This programis used ot learn basic concepts
			in Ros.
		     This program will prompt the user to enter
		     geometry_msgs/Twist and it will publish them
		     on a topic that sub_in.cpp subcribes to.
*/

// Needed to gain access to basic ros functionality
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv){
	// This turns the cpp file intor a rosnode
	ros::init(argc, argv, "pub_out"); /* NB! This is the name of the node*/
	// We need a NodeHandler to work with during the program.
	ros::NodeHandle nh;
	// With this code we are turning this file into a publisher.
	// Here we specify the message type, the topic name, and the que size
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

	// We create a rate object that will loop at 2 hz.
	ros::Rate rate(2);
	// This loop wil continue to run untill the program terminates.
	while(ros::ok()){
		// Here we create a variable to take in the data.
		geometry_msgs::Twist msg;
		// Taking simple input
		std::cout << "Enter msg.linear.x: ";
		std::cin >> msg.linear.x;
		std::cout << "Enter msg.angular.z: ";
		std::cin >> msg.angular.z;
		// Here we publish the message on the topic.
		pub.publish( msg);
		// This logs the messages;
		// This is only to see what messages 'data' are being publish.
		ROS_INFO_STREAM("Veloctity sent: msg.linear.x: " << 
		msg.linear.x << " and msg.angular.z: " << msg.angular.z);
		
		// This lets the loop wait to make correct timing.
		rate.sleep();

	}
}
