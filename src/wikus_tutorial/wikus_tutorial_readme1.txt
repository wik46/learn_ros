Author: Wikus Jansen van Rensburg
Date: December 5, 2020
Description: This is readme file that I wrote while studying the basics
		of Ros. Ros has a steap learning curve due to the
		unclear decriptions. I used the textbook "A Gentle Introduction to Ros"
		to learn the basic concepts. Please refer to this textbook for the
		exact meanings and definitions.

	Assuming you have Ros installed and a empty cakin workspace.
	
	Steps to create a publisher in Cpp and how to compile your program
	
	1. Go to you catkin workspace and create a 'src' directory.
	   This is where your packages will live. While I was studying
	   I used a seperate catkin workspace for learning ros. 
	* Make you ran:
		- source /opt/ros/kinetic/s
		And inside you catkin workspace
		- source devel/setup.bash
	
	2. Now create a package:
		'catkin_create_pkg package-name'
		* You can name your package anything you want just keep track of what you name it.

	3. You will see 2 files inside your newly created 'directory'.
	* 	We will used them in a moment.
	
	4. Now create a .cpp file where you will write the code for your program.
	-  * Refer to my file for descriptions on what each line of code does.

	5. Now you have a program living in a catkin workspace, our next step is to
	   compile the program. First we need to modify our 'package.xml' and 'CMakelist.txt'
	   files.
	5.1 package.xml:
		* Here we define all the other files that our program needs to use.
		a.) Because we use the ros/ros.h library we need to tell our program to
		    include the c compiler.
		b.) The turtlesim node subcribes (takes data) in the form of a geometry_msg/Twist.
		     Thefore, we need to specify that it is included inside our package.xml.			
	5.2 CMakelist.txt:
		* This is like creating a make file. We specify how our program should be
		  built.

	6. Always remember to build (compile) you program from the home directory of you workspace.
	* Go to you uppermost directory in your workspace and run 'catkin_make'
