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

double K_l = 0.00001;
double K_a = 0.001;

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
void force_processing();

void force_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  if (!lock){
    lock = true;
    force_rep = *msg;
  }
  else
  {
    std::cout << "lock" << std::endl;
  }
  force_processing();
}

void force_processing()
{
  force.x = force_att.x - force_rep.x;
  force.y = force_att.y - force_rep.y;
  ROS_INFO("force : %f ,%f",force.x,force.y);
  cmd.linear.x = K_l*force.x;
  cmd.angular.z = K_a*force.y;
  ROS_INFO("cmd : %f , %f",cmd.linear.x,cmd.angular.z);
// =======set confine==========
  if (cmd.linear.x >= 0.7)
    cmd.linear.x = 0.7;
  else if (cmd.linear.x <= -0.7)
    cmd.linear.x = -0.7;
  else
    cmd.linear.x = cmd.linear.x;

  if (cmd.angular.z >= 1)
    cmd.angular.z = 1;
  else if (cmd.angular.z <= -1)
    cmd.angular.z = -1;
  else
    cmd.angular.z = cmd.angular.z;
// ============================
  pub_cmd.publish(cmd);
  lock = false;
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
