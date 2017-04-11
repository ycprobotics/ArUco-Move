# Aruco_move_ros_pkg

1. Download aruco_ros and aruco_move from github
	
	a. Download repository on desktop or in home directory, will be moved later.

		->git clone https://github.com/plynn17/Aruco_move_ros_pkg.git

2. Using catkin_make_pkg create a pkg titled "aruco_move" and "aruco_ros"
	
	a. Navigate to the /src directory in your catkin workspace:

		->catkin_create_pkg aruco_ros roscpp

		->catkin_create_pkg aruco_move roscpp

3. Copy files from repository into respective folders in your catkin workspace
	
	a. From the /src directory and all sub-directories, copy all files in to the respectives directories.

	b. Replace files in the catkin workspace with files from the repository if they already exist.
	
4. Do a catkin_make
	
	a. In the catkin root directory:

		->catkin_make

5. Source your development environment
	
	a. In your .bashrc file add:

		->source /path/to/catkin/workspace/devel/setup.bash

6. Launch program
	
	a. Using the terminal on the robot's computer or an SSH shell:

		->roslaunch aruco_move aruco_move.launch
