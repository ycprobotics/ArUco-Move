/*
*
*
*
*
*/

#include <sstream>
#include "ros/ros.h"
#include "ArucoMove.h"

int main(int argc, char** argv){
	ros::init(argc,argv,"aruco_move");

	ArucoMove node;
	ROS_INFO("In the ArucoMove node\n");

	ros::spin();

	return 0;
}

