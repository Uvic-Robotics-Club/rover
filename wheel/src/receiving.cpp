#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"


void wheelCallback(const geometry_msgs::AccelConstPtr& msg)
{
	ROS_INFO("S:%f, A:%.2f",msg->linear.x, msg->linear.y);
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "wheellistener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("motor_1",1000,wheelCallback);
	ROS_INFO("SUBSCRIBED TO WHEEL_1");
	ros::spin();
	return 0;
}
