There are two robots inside this directory. 

a.) robot1.urdf
	This is a very basic robot that has a rectangular base and has
	4 wheels.

b.) robot1_with_arm.urdf
	This is a very basic robot that is built from robot1.urdf excpet
	that I have added an arm with 2 links and a gripper.

To visualize these robots in rviz use the following command.
(remember to be in the home directory of the package, otherwise you would
	need to specify a different path to the you robot decription urdf
	files.
)
	roslaunch wheel_robot display.launch model:="urdf/name-of-urdf.urdf"
