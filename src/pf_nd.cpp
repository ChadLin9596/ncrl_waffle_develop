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
double K_l = 0.001;
double K_a = 0.001;
double Ka = 10;
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
  // update local waffle frame
  cmd.linear.x = 0;cmd.angular.z = 0;
  end = ros::Time::now().toSec();
  dur = begin - end;
  // linear update
  goal.x = goal.x - cmd.linear.x*dur;
  goal.y = goal.y;
  double theta = cmd.angular.z;
  // angular update
  goal.x = goal.x*cos(-theta*dur) - goal.y*sin(-theta*dur);
  goal.y = goal.x*sin(-theta*dur) + goal.y*cos(-theta*dur);
  force_att.x = Ka*pow(goal.x,3);
  force_att.y = Ka*pow(goal.x,3);
  ROS_INFO("time : %f",dur);
  ROS_INFO("goal : %f , %f",goal.x,goal.y);
  // ========================
  force.x = force_att.x - force_rep.x;
  force.y = force_att.y - force_rep.y;
  ROS_INFO("force : %f ,%f",force.x,force.y);
  cmd.linear.x = K_l*force.x;
  cmd.angular.z = K_a*force.y;
  ROS_INFO("cmd : %f , %f",cmd.linear.x,cmd.angular.z);
  // =======set confine==========

  if (cmd.linear.x >= 0.6)
    cmd.linear.x = 0.6;
  else if (cmd.linear.x <= -0.6)
    cmd.linear.x = -0.6;
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
  begin = ros::Time::now().toSec();
  lock = false;
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "pf_nd");
	ros::NodeHandle nh;
	goal.x = 5;
	goal.y = 0;
  begin = ros::Time::now().toSec();
	sub_force = nh.subscribe<geometry_msgs::Point> ("/ncrl/repulsive/force",10,force_cb);
	pub_cmd = nh.advertise<geometry_msgs::Twist> ("/waffle1/cmd_vel",10);

	ros::spin();
}
