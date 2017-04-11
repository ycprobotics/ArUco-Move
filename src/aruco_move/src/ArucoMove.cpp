#include "ArucoMove.h"

ArucoMove::ArucoMove()
{
	pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
	sub_dist = n.subscribe("/aruco_single/transform",100,subCallback, this);
}

void ArucoMove::subCallback(const geometry_msgs::TransformStamped::ConstPtr& g_msg)
{
	double x = g_msg->transform.translation.x;
	double y = g_msg->transform.translation.y;
	double z = g_msg->transform.translation.z;
	double d = sqrt(x*x + y*y + z*z);

	if(d < lower_thres){
		ROS_INFO("Moving Backward: [%lf,%lf,%lf]: [%lf]\n",x,y,z,d);
	}
	else if(d > upper_thres){
		ROS_INFO("Moving Forward: [%lf,%lf,%lf]: [%lf]\n",x,y,z,d);
	}
	else{
		ROS_INFO("I AIN'T NO HOLLA BACK GIRL: [%lf,%lf,%lf]: [%lf]\n",x,y,z,d);
	}	
}
