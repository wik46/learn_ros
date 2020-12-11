/*
	Author: Wikus
	Date: December 10, 2020
	Description: This is a program to show how to use the
			third argument in ros::init to make it possible
			to run nodes with the same name.
*/

/*
	Names in ros: Resource graph name.
	-----------------------------------
	Nodes, topics, services, and parameters are all refered to as graph
	resources. All graph resource are represented by a string. They are used
	all over ros. ros::init needs the name of the node, the member Publisher needs
	the topic (graph resource) inside its constructor. We call graph resource from
	the commandline.

	1. Global names:
	2. Relative names:
	3. Private names:
	4. Anonymus names: Ros does not allow nodes in the same package to run with the
			    same names. But if we add ros::init_options::AnonymousName,
			    ros will append to the node name so that we dont have nameing
				conflicts.
*/

// Needed for all basic ros functionality.
#include <ros/ros.h>

int main(int argc, char** argv){
	// These two lines of code is needed to turn ros into a ros node.
	// The last argument will append values to the string name of the node
	// so that we can call this node as many times as we like without naming conflicts
	ros::init(argc, argv, "anon_node", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
	// Publish at 10hz
	ros::Rate rate(10);
	
	while(ros::ok()){
		// This is only used to show me the name of the node.
		// What ros does is append the clocktim to the end of the node.
		ROS_INFO_STREAM( "Name of the node: " << ros::this_node::getName());
		// This regulates the time the node takes to do something
		rate.sleep();
	}
}
