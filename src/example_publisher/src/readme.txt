Author: Wikus Jansen van Rensburg.
Date: December 5, 2020.
===========================================================================
Description:
===========================================================================
	This is a readme to describe the two cpp files inside this directory
	1. pub_vel.cpp
		- Goal: This program publishes on the /turtle1/cmd_vel
			topic. I wrote a simple program that prompts
			the user to enter data needed to move the turtle.
		- Publish info:
			* topic: /turtle1/cmd_vel
			* msg: geometry_msgs/Twist
				#include <geometry_msgs/Twist.h>
		- Node name: publish_velocity

	2. sub_pose.cpp
		- Goal: This program will subscribe to the /turtle1/pose
			topic on which the turtlesim_node publishes.
		- Subcriber info:
			* topic: /turtle1/pose
			* msg: turtlesim/Pose

	3. Finding data types:
		1. Remember that you need to find the topic the node publishes/ subsribes on.
		*	rostopic list
		- This command will list all the topics that are currently running.
		2. Then you need to find the msg type that is published/ subcribed on the topic.
		* 	rostopic info 'topic-name'
		- This command will tell you information of the topic you enter.
		  In particular the type of msg that is used by the topic.
		3. Now we need to see what the message looks like.
		   That is we need to see what kind of data the message uses.
		* 	rosmsg show 'msg-type'
		- This will give the structure of the class so that we see the
		  primitive data types that we can work with.
