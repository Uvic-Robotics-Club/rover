#include "ros/ros.h" // needed for everything
#include "geometry_msgs/Accel.h" // needed for publishing data
#include "std_msgs/Float32.h"// needed for the setpoint callback
#include "wiringpi.h"

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

	wiringPiSetup();//library setup function
	pinMode(23,OUTPUT);//set pin GPX2.1 as output. Based on wiringPi

	// grab the public and private node handle
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	// this is going to be the name of the motor
	std::string m;

	// If a name has been specified in the launch file then use it
	if(nh.hasParam("IMU_NUM")){
		nh.getParam("IMU_NUM", m);
		ROS_INFO("Creating a motor with the name of %s",m.c_str());
	}

	// setup publisher of data and Subscriber of data
	Publisher pub = n.advertise<Accel>((m+"/position").c_str(), 1000);
	Subscriber sub = n.subscribe((m+"/speed").c_str(),1000,setpointCallback);

	// set how often you want the main loop to process in Hz
	ros::Rate loop_rate(10);

	bool pin_status = 1;

	while (ros::ok())
	{
		digitalWrite(23,pin_status); //write data(binary) to pin
		pin_status = !pin_status;

		// @see http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
		ros::spinOnce();

		// make sure you sleep for the right amount of time
		// use the loop_rate.sleep not delay as the ros package is better
		loop_rate.sleep();
	}

	// end of the program
	return 0;
}
