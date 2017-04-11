/*
*
*
*
*
*/
#ifndef ARUCO_MOVE_H
#define ARUCO_MOVE_H

#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

class ArucoMove
{
public:
	ArucoMove();
	static void subCallback(const geometry_msgs::TransfromStamped::ConstPtr& g_msg);
private:
	
	ros::NodeHandle n;
	ros::Subscriber sub_dist;
	ros::Publisher pub_vel;
};
#endif
