/*
	Description: This is a program where I try to subcribe to
			any topic that the gazebo_ros node
			pubishes to.
*/

// This is needed for all the basic ros functionality.
#include <ros/ros.h>
// This is the single topic I am goind to subsriber to.
#include <gazebo_msgs/LinkStates.h>
/* Th key differenc between a pubisher and a subcriber is that s subcriber
	needs a callback function. */
// See function definition below.
void get_subscriber_data(const gazebo_msgs::LinkStates& ls);


int main(int argc, char** argv){
	/// These first two steps are neccesary for 
	// turning the cpp file into a ros node.
	ros::init(argc, argv, "example_sub_LinkStates");//NB! This needs to be name of executable.
	// This is a nodehandler object.
	ros::NodeHandle nh;
	// This first argument is the topic we subsribe to.
	ros::Subscriber sub = nh.subscribe("/gazebo/link_states", 1000, get_subscriber_data);

//	ros::Rate rate(10); // Spcifies a rate of 10hz at which messages
				// are going to be subscribed.					
	// We seldom call the subscriber node explicitly.
	// We let ros take care of it.
	while(ros::ok()){
		ros::spinOnce();
	}


}

/*
	Function name: 
	Description:
*/
//	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
//	<< "turtlesim::Pose.x: " << p.x << ", turtlesim::Pose.y: "
//	<< p.y <<
void get_subscriber_data(const gazebo_msgs::LinkStates& ls)
{
//	ROS_INFO_STREAM("Name: " << ls.name);
	// This is data inside the gazebo_msgs::LinkState instance.
	ROS_INFO_STREAM(	
		" *obj.twist[0].linear.x: " << ls.twist[0].linear.x << '\n'
	<<	" *obj.pose[0].position.x" << ls.pose[0].position.x << '\n'


	);
}


/*
	Subcribing to the topic:
	The message (data type): gazebo_msgs/LinkStates 
	Structure of the mesagee type .
		- string[] name
		- geometry_msgs/Pose[]
		- geometry_msgs/Point position
			float64 x
			float64 y
			float64 z

*/
