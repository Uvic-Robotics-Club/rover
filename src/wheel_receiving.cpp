#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"

/*
 * This file simply echos out some of the data that is pushed from
 * the sending portion of the wheel. This is pretty much just a copy
 * of the basic pub/sub tutorials.
 * 
 * @see http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 * 
 */ 

void wheelCallback(const geometry_msgs::AccelConstPtr& msg)
{
	ROS_INFO("S:%f, A:%.2f",msg->linear.x, msg->linear.y);
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "wheellistener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("motor_12",1000,wheelCallback);
	ROS_INFO("SUBSCRIBED TO WHEEL_1");
	ros::spin();
	return 0;
}
