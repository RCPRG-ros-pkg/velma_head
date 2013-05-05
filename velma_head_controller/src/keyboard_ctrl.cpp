#include "ros/ros.h"
#include <stdio.h>
#include <cstring>
#include <termios.h>
#include "geometry_msgs/TwistStamped.h"

#define VSTEP 0.25 // ang velocity step [rad/s] 

std::string nodeName = "ptkeybctrl";

/**
* 
*/
int main(int argc, char **argv)
{
	geometry_msgs::TwistStamped	msgTwist;

	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	ros::Publisher js_pub = n.advertise<geometry_msgs::TwistStamped>("/head_vel", 10);
	ros::Rate loop_rate(10);

	ROS_INFO("Starting node %s", nodeName.c_str());
	
	char inp;
	float dx = 0, dy = 0;
	bool synchronized = false;
	// get the console in raw mode
	int kfd = 0;
	struct termios cooked, raw;
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON);
	raw.c_lflag &= ~(ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	raw.c_cc[VMIN] = 0;
	raw.c_cc[VTIME] = 0;
	tcsetattr(kfd, TCSANOW, &raw);
	// read() is non-blocking from now on and echo is off
	
	printf("Pan-tilt speed control: [A] x- [D] x+ [S] y- [W] y+ [space] stop\n\n");
	printf("Quit: [Q]\n\n");
	sleep(1);
	
	bool quit = false;
	while (ros::ok() && (!quit))
	{	// Prepare data to send
		if(read(0, (void*)&inp, 1)){
			switch(inp){
				case 'a':
				case 'A':
					dx += VSTEP;
					break;
				case 'd':
				case 'D':
					dx -= VSTEP;
					break;
				case 's':
				case 'S':
					dy += VSTEP;
					break;
				case 'w':
				case 'W':
					dy -= VSTEP;
					break;
				case ' ':
					dx = 0;
					dy = 0;
					break;
				case 'q':
				case 'Q':
					dx = 0;
					dy = 0;
					quit = true;
					break;
			}
		}
		
		// Building message
		msgTwist.twist.angular.z = dx;
		msgTwist.twist.angular.y = dy;
		msgTwist.header.stamp = ros::Time::now();
		js_pub.publish(msgTwist);

		ros::spinOnce();

		loop_rate.sleep();
	}

	// restore console settings
	raw.c_lflag |= ECHO | ICANON;
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	raw.c_cc[VMIN] = 0;
	raw.c_cc[VTIME] = 0;
	tcsetattr(kfd, TCSANOW, &raw);

	return 0;
}
