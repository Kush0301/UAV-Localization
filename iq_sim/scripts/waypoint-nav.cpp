#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>

mavros_msgs::State current_state;
nav_msgs::Odometry current_pose;
geometry_msgs::Pose correction_vector;
geometry_msgs::Point local_offset_pose;
geometry_msgs::PoseStamped waypoint;

float current_heading;
float local_offset;
float correction_heading = 0;
float local_desired_heading; 



ros::Publisher local_pos_pub;
ros::Subscriber currentPos;
ros::Subscriber state_sub;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient takeoff_client;

struct waypoint{
	float x; 
	float y; 
	float z; 
	float theta; 
};
