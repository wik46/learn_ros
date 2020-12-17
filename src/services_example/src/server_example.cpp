/* **************************************************************************************
 * Author: Wikus Jansen van Rensburg
 * Date: December 14, 2020.
 * Description: This is a program written to study how to create a server node in ros. 
 * **************************************************************************************/

/*
	1. A client node sends a request to a server: A client calls the server node.
	2. The server node takes the data, uses it and sends a repsonse
	   back to the client based apon the programmer's implementation.
	3. A server node is very similar to a subscriber node in that it needs
	   a callback function that seldomly gets called explicitly, and 
	   where we need to hand over control to ros with ros::spinOnce() inside
	   a while loop.	
*/

/*
	A message gets stored in a .msg file.
		- It has various different fields inside the message.
	A service type gets stored in a .srv file.
		- It has various different fields in the file.
		- The request type is seperated from the reponse type
		  with a '--------'.
*/

// Remember the naming conventions: package-name::service-type,
//				    package-name::service-type::reponse,
//				    package-name::service-type::request.

/*
	This server node will be called using the commandline. This is very simliar
	to use
*/

// 1. This header is needed to gain access to basic ros functionality.
#include <ros/ros.h>
// 2. This header is needed becuase this is the srv type that the 
// client will use as a request. 
#include <turtlesim/TeleportRelative.h>
// 3. This is the geometry messages that I will publish after the the user called
//    this service.
#include <geometry_msgs/Twist.h>


// This is used to get the data from the request.
geometry_msgs::Twist msg;

/* **************************************************************************************
 * Function name: 
 * Description: This function is the callback function used by the server node
	        to accpet the request that a client sends when calling the server and
		to reply with a reponse.
 * Parameters: turtlesim::TeleportRelative::Request&, 
		turtlesim::TeleportRelative::Reponse&.
 * Pre-conditions: This function must return bool, to indicate wheter the request was 
		   successful.
 * **************************************************************************************/
bool take_in(
turtlesim::TeleportRelative::Request& req, turtlesim::TeleportRelative::Response& resp)
{
	ROS_INFO_STREAM("Request data received: req.linear :" <<
	req.linear << " and req.angular : " << req.angular << std::endl);
	// Now we save the data the we received from the client request
	// in msg data format.
	msg.linear.x = 0 + req.linear;
	msg.angular.z = 0 + req.angular;
	// This part wil inform the user if they entered an unsuccesul
	// value for the call.
	// For example purposes, an unsuccessful value is a value outside
	// (0,10) for linear velocity.
	if(req.linear < 0 || req.linear > 10){
		ROS_WARN_STREAM("Entered linear velocity outside range (0,10).");
		return false;
	}else{
		return true;
	}

}

int main(int argc, char** argv){
	// These two statements turn my cpp program into a ros node.
	ros::init(argc, argv, "server_move_turtle");
	ros::NodeHandle nh;
	
	// This creates the service object.
	// We need to spcify the type of srv that the service will accpet.
	ros::ServiceServer move_server = 
	nh.advertiseService("custom_move_service", take_in);
	// We also create a publisher to do the work that the client's request
	// asked for.
	ros::Publisher pub = 
	nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);
	// We can set the rate at wich the publisher will publish to the topic
	// cmd_vel
	ros::Rate rate(3); // 3 hz

	// This loop is very similar to the subscriber loop.
	// We dont call the callback function explicitly, we rather let 
	// ros take over the program.

	while(ros::ok()){
		// This publishes the message of type geometry_msgs/Twist
		// on the turtle1/cmd_vel topic.
		// Inside the callback function we set the correct values for
		// msg.	
		pub.publish(msg);
		// This means that the callback function will be executed
		// once for each iteration of the loop.
		ros::spinOnce();

		// This regulates the speed of the loop.
		rate.sleep();
	}
}

