The urdf in this package is from the OSURC rover club. The urdf
for the robot was not created by me. I only defined the launch file
so that the urdf can be visualized in rviz.

The purpose of this package is the study the rover arm.

Date: December 16, 2020.
Description: The purpose of this package is to get an idea of how to
	     visualize the rover arm.
	     This was used as tutorial to get started with the arm.

This package has several aspects to it.
1. A urdf file that describes to the parameter server how our arm 
   looks. That is how many links (parts) out arm has, how the different
   links are joint together, and the limits that these joints can move.
   
2. There is a launch file that describes how to launch the various
   nodes needed to launch this visualization.


=====================================================
Inspecting what is happening.
=====================================================
Follow these steps.
* Make sure your environment and workspace is sourced.
	- If unsure source your environment by
		source /opt/ros/kinetic/setup.bash
	- Source you workspace by (from $ directory)
		source workspace-name/devel/setup.bash

1. Launch all the nodes needed for the visualization.
	roslaunch package-name launch-file model:="path-to-model.urdf"
	roslaunch rviz_visual visual.launch model:="urdf/mr_1920_arm_urdf_export.urdf"

2. Now make sure that you see the rover arm as well as a gui that displays
	all the moveable parts of the arm.

3. Now open a second terminal window (make sure bot env and workspace is soourced) 
  and then see what topics are available for use with the command 
	< rostopic list >
This command list all the topics.

4. Now take a closer look the joint_state_publisher.
   * Side note: Remember the power of tab completion.
   type in:
	rostopic echo joint (and then use tab completion to fill the rest).	
    The full command is:
	rostopic echo /joint_state_publisher
5. Make sure you can see the join_state_publisher_gui, the terminal where 
   you ran the nr 4. command. Now move the joints of the robot and see
   how the values that are being published changes. 

6. Next step, see if you can publish values using the commandline instead
	of the gui. 
