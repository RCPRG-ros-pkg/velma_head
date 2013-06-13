#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;

class RobotHead
{
private:
  PointHeadClient* point_head_client_;

public:
  //! Action client initialization 
  RobotHead()
  {
    //Initialize the client for the Action interface to the head controller
//    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
	point_head_client_ = new PointHeadClient("head_controller/point_head_action", true);

    //wait for head controller action server to come up 
    while(!point_head_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
  }

  ~RobotHead()
  {
    delete point_head_client_;
  }

  //! Points the high-def camera frame at a point in a given frame  
  void lookAt(std::string frame_id, double x, double y, double z)
  {
    //the goal message we will be sending
    control_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    //we are pointing the high-def camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "stereo_left_link";

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    //and go no faster than 1 rad/s
    goal.max_velocity = 1.0;

    //send the goal
    point_head_client_->sendGoal(goal);

    //wait for it to get there (abort after 2 secs to prevent getting stuck)
    point_head_client_->waitForResult(ros::Duration(5));
  }

  //! Shake the head from left to right n times  
  void shakeHead(int n)
  {
    int count = 0;
    while (ros::ok() && ++count <= n )
    {
      //Looks at a point forward (x=5m), slightly right (y=-1m), and 1m up
      lookAt("torso_link", 5.0, -1.0, 1);
      //Looks at a point forward (x=5m), slightly left (y=1m), and 1m up
      lookAt("torso_link", 5.0, 1.0, 1);
    }
  }

  //! Take a look around 032 lab
  void watch032(void)
  {
    while (ros::ok())
    {
        //Looks through the window
        lookAt("torso_link", 0.5, -2.0, 1.0);
        //Looks at Slon's desk
        lookAt("torso_link", 2.5, 2.5, 0);
        //Looks at Yoyek's / Banan's desk
        lookAt("torso_link", 3.5, 0, 0);
        //Looks at Wacek's desk
        lookAt("torso_link", 5.5, 2.5, 0);
        //Looks at the door
        lookAt("torso_link", 1.5, 3.0, 0.5);

        break;
    }
  }
};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");

  RobotHead head;
  head.watch032();
  head.shakeHead(3);
}

