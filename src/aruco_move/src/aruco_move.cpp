/* Follow an April Tagged robot, using the aruco_ros library
 *
 * TODO: collision avoidance, accurate following
 * TODO: class implementation
 * TODO: launch files, stripped down aruco
 * TODO: out of frame search, with timer
 */
	
#include <sstream>
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#define DIST_TH (0.10)
#define MIDPOINT (1.5)
#define THETA_TH (0.015)
#define CENTER (0.25)

#define SLOPE_V (0.3)
#define KPX_V (1.5)
#define KDX_V (0.05)
#define KIX_V (0.5)

#define SLOPE_A (8.0)

#define DEBUG 1

typedef struct Velocity3{
	double x;
	double y;
	double z;
}Velocity3;

/*Global Variables
 *
 */
const double EXPECTED_FACTOR = 0.35;
const double MAX_SPEED = 1.0;
const double MAX_ANGLE = 1.0;

ros::Publisher pub_vel;
ros::Subscriber sub_pose;
ros::Time prev_timestamp(0.0);
ros::Duration delta_t(0.0);
ros::Timer timer;

geometry_msgs::Point* prev_pos;

Velocity3 angular_velocity = {0.0,0.0,0.0};
Velocity3 robot_velocity = {0.0,0.0,0.0};
Velocity3 tracking_velocity = {0.0,0.0,0.0};

bool release_command = true;
bool out_of_frame = false;

/*
 * Methods
 */
geometry_msgs::Point* kalman_filter(geometry_msgs::Point rx_pos);
Velocity3* pid_filterv(Velocity3* new_velocity);
Velocity3* linear(double distance);
Velocity3* angular(double theta);
void initialize(ros::NodeHandle n);

void subscriber_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	out_of_frame = false;
	if(release_command){
		release_command = false;
		//get delta t
		delta_t = msg->header.stamp - prev_timestamp;
		prev_timestamp += delta_t;
		
		//pull and filter position
		geometry_msgs::Twist cmd;
		geometry_msgs::Point* position = kalman_filter(msg->pose.position);
	
		//filter output velocities
		double distance = sqrt(position->x * position->x + 
							   position->y * position->y +
							   position->z * position->z);
		Velocity3* velocity = linear(distance);
	
		double theta = acos(position->z / sqrt(position->x*position->x + position->z*position->z));
		Velocity3* angle_v = angular(theta);
	
		//assign output
		cmd.angular.x = angle_v->x;
		cmd.angular.y = angle_v->y;
		cmd.angular.z = angle_v->z;
		cmd.linear.x = velocity->x;
		cmd.linear.y = velocity->y;
		cmd.linear.z = velocity->z;
	
		//publish
		if(DEBUG) ROS_INFO("POSE: v[%lf,%lf,%lf]:d[%lf]:t[%lf]:@[%lf]\n",
							position->x,position->y,position->z,distance,theta,delta_t.toSec());
		pub_vel.publish(cmd);
	}
}
void timer_callback(const ros::TimerEvent& event)
{
	release_command = true;
	if(out_of_frame){
		geometry_msgs::Twist cmd;
		
		//cmd.angular.x = angular_velocity.x = 0.999*angular_velocity.x;
		//cmd.angular.y = angular_velocity.y = 0.999*angular_velocity.y;
		cmd.angular.z = angular_velocity.z = 0.99*angular_velocity.z;
		cmd.linear.x = robot_velocity.x = 0.99*robot_velocity.x;
		//cmd.linear.y = robot_velocity.y = 1.1*robot_velocity.y;
		//cmd.linear.z = robot_velocity.z = 1.1*robot_velocity.z;

		pub_vel.publish(cmd);
	}
	out_of_frame = true;
}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"aruco_move");

	ros::NodeHandle n;
	initialize(n);

	ros::spin();

	return 0;
}

void initialize(ros::NodeHandle n)
{
	ROS_INFO("Running the ArucoMove node\n");

	geometry_msgs::Point init_pos;
	init_pos.x = 0.0;
	init_pos.y = 0.0;
	init_pos.z = 0.0;
	prev_pos = &init_pos;

	sub_pose = n.subscribe("/aruco_single/pose",10,subscriber_callback);
	pub_vel = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 10);
	timer = n.createTimer(ros::Duration(0.1),timer_callback);
	
	ROS_INFO("Synchronizing Clock...\n");
	while(ros::Time::now() == ((ros::Time) 0)){};
	ROS_INFO("Clock Synchronized\n");
	
	prev_timestamp = ros::Time::now();
}

geometry_msgs::Point* kalman_filter(geometry_msgs::Point rx_pos)
{
	//TODO: find better way to deal with long out of frame delays
	/*if(delta_t.toSec() < 0.1){
		geometry_msgs::Point expected_pos;
	
		//find expected postion based on previous tracking velocity
		expected_pos.x = prev_pos->x + tracking_velocity.x*delta_t.toSec();
		expected_pos.y = prev_pos->y + tracking_velocity.y*delta_t.toSec();	
		expected_pos.z = prev_pos->z + tracking_velocity.z*delta_t.toSec();
		
		//declare new position based on weighed average of expected and recieved
		rx_pos.x = (1-EXPECTED_FACTOR)*rx_pos.x + EXPECTED_FACTOR*expected_pos.x;	
		rx_pos.y = (1-EXPECTED_FACTOR)*rx_pos.y + EXPECTED_FACTOR*expected_pos.y;
		rx_pos.z = (1-EXPECTED_FACTOR)*rx_pos.z + EXPECTED_FACTOR*expected_pos.z;
	
		//recalculate tracking velocity
		tracking_velocity.x = (rx_pos.x - prev_pos->x)/delta_t.toSec();
		tracking_velocity.y = (rx_pos.y - prev_pos->y)/delta_t.toSec();
		tracking_velocity.z = (rx_pos.z - prev_pos->z)/delta_t.toSec();

		prev_pos->x = rx_pos.x;
		prev_pos->y = rx_pos.y;
		prev_pos->z = rx_pos.z;
	}
	*/
	prev_pos->x = rx_pos.x;
	prev_pos->y = rx_pos.y;
	prev_pos->z = rx_pos.z;
	return prev_pos;
}

Velocity3* pid_filterv(Velocity3* new_velocity)
{
	double n_x = new_velocity->x;

	robot_velocity.x = KPX_V*(n_x) + 
					   KDX_V*(n_x - robot_velocity.x)/delta_t.toSec() + 
					   KIX_V*(n_x - robot_velocity.x)*delta_t.toSec();

	return &robot_velocity;
}

Velocity3* linear(double distance)
{
	Velocity3 new_velo = {0.0,0.0,0.0};

	//if distance is outside of theshold move into threshold
	if(distance > MIDPOINT + DIST_TH || distance < MIDPOINT - DIST_TH){
		new_velo.x = SLOPE_V*(distance - MIDPOINT);
	}
	
	//clamp max velocity
	if(new_velo.x > MAX_SPEED) new_velo.x = MAX_SPEED;
	else if(new_velo.x < (-1.0)*MAX_SPEED) new_velo.x = (-1.0)*MAX_SPEED;
	
	//smooth velocity
	return pid_filterv(&new_velo);
}

Velocity3* angular(double theta)
{
	Velocity3 ang_velo = {0.0,0.0,0.0};

	//if theta is ouside of threshold rotate into threshold
	if(theta < CENTER - THETA_TH || theta > CENTER + THETA_TH){
		ang_velo.z = SLOPE_A*(theta - CENTER);
	}
	
	//clamp max angular velocity
	if(ang_velo.z > MAX_ANGLE) ang_velo.z = MAX_ANGLE;
	else if(ang_velo.z < (-1.0)*MAX_ANGLE) ang_velo.z = (-1.0)*MAX_ANGLE;
	
	//smooth spin TODO: move to pid filter
	angular_velocity.z = (0.4*angular_velocity.z + 0.6*ang_velo.z);
	return &angular_velocity;
}
