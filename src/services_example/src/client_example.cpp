/*
	Author: Wikus Jansen van Rensburg
	Date: December 12, 2020
	Description: This is a simple program taken from the
			Gentle Introduction to Ross handbook.
		     It is used to practise creating a client node
		     in ros.
*/

/*
	Services are very similar to topics in that it is used 
	as a channel of communication between nodes. The big difference
	is that when a node publishes on a topic, the node is not
	aware of any other nodes subscribing to the topic.

	With services, we have a client node that sends a message (request)
	to a server node. The server node acts apon the data and then sends
	a message (reponse) back to the client node that requested the 
	service.
	
	Client <-> Server.
	A client calls a service.
	
	Example: Topic and message.
		 Services and service message type.
		 (service message = request + reponse )
*/

// This header file is needed for all ros nodes.
#include <ros/ros.h>
// 1. This is the service message type that we wil use.
#include <turtlesim/Spawn.h>
// 2. This is the second type of service message we wil use.
#include <turtlesim/Kill.h>
// This is to use cout and cin.
#include <iostream>


int main(int argc, char** argv){
	// Turning this cpp file into a ros node.
	ros::init(argc, argv, "client_user_options");
	// Creating a node handler.
	ros::NodeHandle nh;
	// 1. Now we turn a normal node into a client node.
	// Create a client object.
	// Ths string passed as argument must be a graph resoure name of the
	// service we want to call. The client object must be created with
	// the correct service type using templates.
	ros::ServiceClient various_client = nh.serviceClient<turtlesim::Spawn>("spawn");
	// 2. Now we create a requist and a reponse object to store the data
	// that the request sends, and to save the data received from the response.
	// The service data type defined insdide the .srv file has two parts
	// to it seperated by a '------' line. The top half is all the fields inside
	// the request part and below the line is all the fields of the reponse part.
	turtlesim::Spawn::Request req;
	turtlesim::Spawn::Response resp;

	
	// This loop will let the node keep running untill it is shutdown
	while(ros::ok()){
		// This is only used so that the user enters the position of the new turtle
		// and the name.
		std::cout << "Enter the position to spawn a new turtle" << std::endl;
		std::cout << "x-position: ";
		std::cin >> req.x;
		std::cout << "y-position: ";
		std::cin >> req.y;
		std::cout << "theta-position: ";
		std::cin >> req.theta;
		std::cout << "name :";
		std::cin >> req.name;
		// Now we send the request to the server and wait for a reponse.
		// We pass in the data the is sent as a request and we provide
		// a variable to store the data received from the request.
		// This call wont return untill the service is complete.
		bool success = various_client.call(req, resp);
		// We make sure that the call was succussful.
		if( success ){
			// This wil informg the user that the new turtle is spawned.
			ROS_INFO_STREAM("Spawn successful, turtlename: " << resp.name);
		}else{
		
			ROS_ERROR_STREAM("Your call to spwan a turtle failed.");
		}
	}
	
}
