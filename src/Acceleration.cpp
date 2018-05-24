#include "ros/ros.h" // needed for everything
#include "geometry_msgs/Accel.h" // needed for publishing data
#include "std_msgs/Float32.h"// needed for the setpoint callback

#include <sstream>
/*
 * Define all of the variables that sending node uses
 */

typedef ros::Publisher Publisher;
typedef ros::Subscriber Subscriber;
typedef geometry_msgs::Accel Accel;

// callback for setting the "speed" of the motor.
void setpointCallback(const std_msgs::Float32ConstPtr& msg)
{
	Setpoint = msg->data;
	ROS_INFO("Updating the setpoint to %.3f",Setpoint);
}


int main(int argc, char **argv)
{
	// initialize ros
	ros::init(argc, argv, "wheelpub");

	// grab the public and private node handle
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	// this is going to be the name of the motor
	std::string m;

	// If a name has been specified in the launch file then use it
	if(nh.hasParam("IMU_NUM")){
		nh.getParam("IMU_NUM", m);
		ROS_INFO("Creating a node with the name of %s",m.c_str());
	}

	// setup publisher of data and Subscriber of data
	Publisher pub = n.advertise<Accel>((m).c_str(), 1000);
	Subscriber sub = n.subscribe((m+"/setpoint").c_str(),1000,setpointCallback);

	// set how often you want the main loop to process in Hz
	ros::Rate loop_rate(10);


	while (ros::ok())
	{
		// make a new message
		Accel msg;


		// set the actual value and the setpoint to some elements of Accel
		// @info rosmsg show geometry_msgs/Accel
		msg.linear.x = Actual;
		msg.linear.y = Setpoint;

		// push the message to the data cloud
		pub.publish(msg);

		// if you want you can log it. I dont need it so its commented out
		//ROS_INFO("%.1f", msg.linear.x);

		// @see http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
		ros::spinOnce();

		// make sure you sleep for the right amount of time
		// use the loop_rate.sleep not delay as the ros package is better
		loop_rate.sleep();
	}

	// end of the program
	return 0;
}
