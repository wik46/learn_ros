/* ***************************************************************************************
 *	Author: Wikus Jansen van Rensburg
 *	Date: December 10, 2020
 *	Description: This is a node inside a package dedicated
 *			to study how parameters work in ros.
 *	EXE name: parameter_ex1
 *
 * **************************************************************************************/

/*
	Parameters are similar to messages, but because ... they work
	best for values that do not change frequently. This example will
	get the current parameters used for the background color for the 
	turtlesim_node and set the new background colors according to human
	input.
	--------------------------
	The two main functions introduced inside this file is
	// param-name is the name of the parameter.
	// The value can be any bool, int, double, string
	ros::param::set(param-name, value)
	// The variable must be a reference to a variable
	ros::param::get(param-name, variable)
	-------------------------------
	Notes on services:(See package about services for more info)
		- After a node's parameters are set, (depending how the node
			is defined), you need to call a service that tells the
			parameter server that this parameters are being changed.
		  If you dont call the service is this example ,then the colors
		  of the turtlesim background wont change. 
*/
// This header is needed in all ros nodes.
#include <ros/ros.h>
// More will be explained on services in the services package.
#include <std_srvs/Empty.h>
//This is just for basic input and output.
#include <iostream>

int main(int argc, char** argv){
	// These two lines turn the cpp file into a rosnode.
	ros::init(argc, argv, "parameter_ex1");
	// 
	ros::NodeHandle nh;
	// This will get the current parameters set for the three
	// parameters.
	// These variables need to be passed by reference because the values
	// of the parameters will be store in them.
	int bcolor_b, bcolor_g, bcolor_r;	
	bool ok1 = ros::param::get("background_b", bcolor_b);
	/*Remember to use relative names*/
	bool ok2 = ros::param::get("background_g", bcolor_g);
	/*Remember to use relative names*/
	bool ok3 = ros::param::get("background_r", bcolor_r);
	/*Remember to use relative names*/
	// If any if the parameters were not succesfully obtained we kill the program.
	if(!ok1 || !ok2 || !ok3){
		ROS_ERROR_STREAM("Unsuccessfull get");
	}
	
	// This will ouput the current background colors that are set for the three 
	// parameters specified.
	ROS_INFO_STREAM(
		"Current parameters set: " <<
		"background_b: " << bcolor_b << 
		", background_g: " << bcolor_g << 
		", background_r: " << bcolor_r 
	);
	
	// Now before we can set the new parameters we first need to make sure
	// the 'clear' service is available for use.
	// We wait untill the clear service becomes available which indicates
	// that the turtlesim node has started up.
	ros::service::waitForService("clear"); // remember to use relative names.
	// Now that we know that our service is ready, we can set the new parameters
	// and call the service to update the new parameters.
	
	// Simple program to ask the user what the parameters should be.
	std::cout << "Enter parameter for 'background_b': ";
	std::cin >> bcolor_b; 
	std::cout << "Enter parameter for 'background_g': ";
	std::cin >> bcolor_g; 
	std::cout << "Enter parameter for 'background_r': ";
	std::cin >> bcolor_r; 
	// Now we set the new parameters.
	// This function want the name of the parameter and the value we want to set it.
	ros::param::set("background_b", bcolor_b);
	ros::param::set("background_g", bcolor_g);
	ros::param::set("background_r", bcolor_r);
	// But there wil not be a change in the background color.
	// We first need to call the clear service. More on services in the services
	// package.
	// 1. We create an instance of a ServiceClient that will call the service
	// clear/
	ros::ServiceClient clearClient = nh.serviceClient<std_srvs::Empty>("/clear");
	// This is just an empty service.
	std_srvs::Empty srv;
	clearClient.call(srv);

	// This will only print out that the program is finished.
	std::cout << "** End of node. **" << std::endl;
}
