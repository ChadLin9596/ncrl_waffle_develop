#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
// keyboard include
#include <tf/tf.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <termios.h>
// cpp include
#include <math.h>

geometry_msgs::Point goal; // global frame
geometry_msgs::Point waffle; // global frame
geometry_msgs::Point force_att; // local frame
geometry_msgs::Point force_rep; // local frame
geometry_msgs::Point force; // local frame
geometry_msgs::Twist cmd; // control waffle

ros::Subscriber sub_force;
ros::Publisher pub_cmd;

// state global variable
bool lock = false;
double begin;
double dur;
double end;

void force_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  if (!lock){
    lock = true;
    force_rep = *msg;
    ROS_INFO("force : %f , %f",force_rep.x,force_rep.y);

  }
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "pf_nd");
	ros::NodeHandle nh;
	goal.x = 5;
	goal.y = 0;

	sub_force = nh.subscribe<geometry_msgs::Point> ("/ncrl/repulsive/force",10,force_cb);
	pub_cmd = nh.advertise<geometry_msgs::Twist> ("/waffle1/cmd_vel",10);

	ros::spin();
}
