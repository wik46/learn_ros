
// ######################## STILL IN PROGRESS ##############################


/* *************************************************************************
 * Author: WIkus Jansen van Rensburg
 * Date: Decemeber 20, 2020
 * Description: This is an example takin from the learning resource
 *		specified in the readme.txt file.
 *		This is a simple program to move the arm we create in
 *		urdf file using a node.
 * ************************************************************************/

// We include the necessary header files.
/* 1. This is always need for the basic functionality such as ros::init()
	and the NodeHandler. */
#include <ros/ros.h>
/* 2. This is because we send sensor messages to the joints. */
#include <sensor_msgs/JointState.h>
/* 3. This is to get access to the transform messages that we want to broadcast.
	We need to gain access to the TransformBroadcaster that makes it easier
	for us to transform to publish transform data to the tf2_ros/
    *** Inspecting wheter to use tf or tf2_ros
 */
#include <geometry_msgs/TransformStamped.h>
// This is to gain access to the transform broadcaster.
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char** argv){
	// This is to turn this cpp file into a ros node.
	// The string entered is the name of the node and should correspond
	// to the executable file defined inside the Cmakelist.txt file.
	/* The purpose of this arm is to move the node in circles. */
	ros::init(argc, argv, "cirlce_mover"); 
	// We create a node handler object. This will be used to tell ros if this node
	// is a publisher, subscriber, client, server etc.
	ros::NodeHandle nh;
	// Now we need to publish the joint states as we go.
	// We need to specify the type of messages tho publish and on which topic to publish
	// the messages.
	ros::Publisher joint_pub 
	= nh.advertise<sensor_msgs::JointState>("joint_state", 1);
	// We create a transform broadcaster as well.
	tf2_ros::TransformBroadcaster broadcaster;
	// This will specify the loop at which the messages are being published.
	// The ros::sleep() methos will know how to regulate the while loop.
	ros::Rate loop_rate(30); // 30 hz.	

	// We create the messages needed. This is like createing the data-types needed.
	// These are all classes with various fields. To gain more information
	// of the messages (datatypes) use the ros commandline tool. (rosmsg)
	/* You can list all the messages using 'rosmsg list', you can inspect all the
	   fields (member variables of the class) using 'rosmsg info message-name'.
	*/
	geometry_msgs::TransformStamped odom_trans;
	sensor_msgs::JointState joint_state;
	// Now we create a new frame called odom.
	odom_trans.header.frame_id = "odom";	
	// We also set the child frame to the base_link we specified in the urdf.
	// All our links are children of the base_link, so if we know the position of
	// the base_link relative to our transform frame, we know the position of all our
	// links relative the the base_link and the to the transform frame.
	odom_trans.child_fram_id = "base_link";
	// We need the degrees.
	const double degree = M_PI/180;
	/*
		Here we define the current state of the robot.
	*/
	// Used to specify the increase.
	double inc = 0.005, base_to_arm_base_inc = 0.005, arm_1_to_arm_base_inc = 0.005,
	arm_2_to_arm_1_inc = 0.005, gripper_inc = 0.005, tip_inc = 0.005;
	// Used to set the current robot state.
	double base_to_arm_base = 0, arm_1_to_arm_base = 0, arm_2_to_arm_1, gripper = 0,
	tip = 0;

	// We tell our joint_state instance about all of our joints. Notice that we specify
	// all elven joints but we dont move all eleven. See urdf description of the	
	// robot to know the type of joints that our robot uses.
	joint_state.name.resize(11);
	joint_state.position.resize(11);
	// Now that we have enough space for all the joints we specify them and we will
	// set their position inside the while loop. NB! These names must correspond to the
	// names of the joints specified in the urdf file.
	/* We set the positions of all the wheel_to_base joints at a constant because we
		are not concerned about moving the wheels in this tutorial.
	*/
	joint_state.name[0] = "base_to_wheel_1"; /* type="fixed" */
	joint_state.position[0] = 0;
	joint_state.name[1] = "base_to_wheel_2"; /* type="fixed" */
	joint_state.position[1] = 0;
	joint_state.name[2] = "base_to_wheel_3"; /* type="fixed" */
	joint_state.position[2] = 0;
	joint_state.name[3] = "base_to_wheel_4"; /* type="fixed" */
	joint_state.position[3] = 0;
	
	joint_state.name[4] = "base_to_arm_base";/* type="continuous" */
	joint_state.name[5] = "arm_1_to_arm_base";/* type="revolute" */
	joint_state.name[6] = "arm_2_to_arm_1";/* type="revolute" */
	joint_state.name[7] = "right_gripper_joint";/* type=revolute */
	joint_state.name[8] = "right_tip_joint";/* type="fixed" */
	joint_state.name[9] = "left_gripper_joint";/* type="revolute"*/
	joint_state.name[10] = "left_tip_joint";/* type="fixed"*/

	// Now we enter the loop that is specified to iterate at a rate of 30 hz.	
	while(ros::ok()){
		// We need to tell ros the exact time when we set the joints positions.
		joint_state.header.stamp = ros::Time::now();
		// We specify the positions of the joints.
		joint_state.position[4] = base_to_arm_base;
		joint_state.position[5] = arm_1_to_arm_base;
		joint_state.position[6] = arm_2_to_arm_1;
		joint_state.position[7] = gripper;
		joint_state.position[8] = tip;
		joint_state.position[9] = gripper;
		joint_state.position[10] = tip;
		// We update the transforms.
		odom_trans.header.stamp = ros::Time::now();

		// This is what lets the loop iterate at our desired rate.
		loop_rate.sleep();
	}	
}





