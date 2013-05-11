#include "ros/ros.h"
#include <stdio.h>
#include "sensor_msgs/Imu.h"
#include "imu.hpp"

std::string nodeName = "velmaheadimu";
sensor_msgs::Imu	msgImu;
IMU* ai;
std::string serialPortName;
ImuData ldata;

double angular_velocity_stdev_, angular_velocity_covariance_;
double linear_acceleration_covariance_, linear_acceleration_stdev_;
double orientation_covariance_, orientation_stdev_;

/**
* 
*/
int main(int argc, char **argv)
{

	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;
    n.param("serial_port", serialPortName, std::string("/dev/ttyUSB0"));

    n.param("linear_acceleration_stdev", linear_acceleration_stdev_, 0.0);
    n.param("orientation_stdev", orientation_stdev_, 0.0);
    n.param("angular_velocity_stdev", angular_velocity_stdev_, 0.0);

    double angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
	double orientation_covariance = orientation_stdev_ * orientation_stdev_;
	double linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;

	msgImu.linear_acceleration_covariance[0] = linear_acceleration_covariance;
	msgImu.linear_acceleration_covariance[4] = linear_acceleration_covariance;
	msgImu.linear_acceleration_covariance[8] = linear_acceleration_covariance;

	msgImu.angular_velocity_covariance[0] = angular_velocity_covariance;
	msgImu.angular_velocity_covariance[4] = angular_velocity_covariance;
	msgImu.angular_velocity_covariance[8] = angular_velocity_covariance;

	msgImu.orientation_covariance[0] = orientation_covariance;
	msgImu.orientation_covariance[4] = orientation_covariance;
	msgImu.orientation_covariance[8] = orientation_covariance;

	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu/data", 10);
	ros::Rate loop_rate(10);

	ROS_INFO("Starting node %s", nodeName.c_str());
	ROS_INFO("Connecting to %s ... ", serialPortName.c_str());

	ai = new IMU(serialPortName);
	if(ai->connected)
		ROS_INFO("connected");
	else
		ROS_ERROR("unable to connect");
	
	bool quit = false;
	while (ros::ok() && (!quit))
	{	// Prepare data to send
		
		// Building message

		ldata = ai->getReading();

	    msgImu.linear_acceleration.x = ldata.linearAcceleration[0];
	    msgImu.linear_acceleration.y = ldata.linearAcceleration[1];
	    msgImu.linear_acceleration.z = ldata.linearAcceleration[2];

	    msgImu.angular_velocity.x = ldata.angularVelocity[0];
	    msgImu.angular_velocity.y = ldata.angularVelocity[1];
	    msgImu.angular_velocity.z = ldata.angularVelocity[2];

//	    tf::Quaternion quat;
//	    (tf::Matrix3x3(-1,0,0,
//			 0,1,0,
//			 0,0,-1)*
//	    tf::Matrix3x3(orientation[0], orientation[3], orientation[6],
//			 orientation[1], orientation[4], orientation[7],
//			 orientation[2], orientation[5], orientation[8])).getRotation(quat);
//
//	    tf::quaternionTFToMsg(quat, data.orientation);

	    msgImu.orientation_covariance[0] = -1; // No orientation data

	    msgImu.header.stamp = ros::Time::now();

		imu_pub.publish(msgImu);

		ros::spinOnce();

		//loop_rate.sleep();
	}

	return 0;
}
