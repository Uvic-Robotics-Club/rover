#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Vector3.h"
#include <stdlib.h> // this includes the rand function
#include "MiniPID.h" // this includes the PID class

#include <sstream>

double Setpoint = 0;
double Actual = -10;

void setpointCallback(const std_msgs::Float32ConstPtr& msg)
{
	Setpoint = msg->data;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "wheelpub");

	ros::NodeHandle n;
	std::string m;
	n.param<std::string>("motor", m, "motor_1");

	ros::Publisher motor_1 = n.advertise<geometry_msgs::Accel>(m.c_str(), 1000);
	ros::Subscriber sub = n.subscribe("motor_1_speed",1000,setpointCallback);

	ros::Rate loop_rate(10);
	ROS_INFO("PUBLISHING FOR %s",m.c_str());
	MiniPID pid = MiniPID(0.6,0.1,0.1);
	double count = 0;
	
	while (ros::ok())
	{

		geometry_msgs::Accel msg;
		Actual += pid.getOutput(Actual,Setpoint);
		msg.linear.x = Actual;
		msg.linear.y = Setpoint;
		motor_1.publish(msg);
		ROS_INFO("%.1f", msg.linear.x);
		ros::spinOnce();

		loop_rate.sleep();
		count += 0.1;
	}


	return 0;
}
