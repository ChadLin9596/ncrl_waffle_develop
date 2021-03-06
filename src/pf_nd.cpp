/*
 * Author : Chun-Jung Lin
 * Date : 28 05 2018
 * Brief : potenial function obstacle avoiding
*/

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
#include <iostream>
#include <cmath>
#include <cfloat>

geometry_msgs::Point goal; // global frame
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
double Ka_l = 100; // repulsive linear force gain 10
double Ka_a = 0.3; // repulsive angular force gain 1
double Kr_l = 5e-11;  // repulsive linear force gain 1e-11
double Kr_a = 0.6;   // repulsive angular force gain 2
void force_processing();

void force_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  if (!lock){
    lock = true;
    force_rep = *msg;
    if (isinf(force_rep.y))
      force_rep.y =99999999999;
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
  double linear = cmd.linear.x;
  double angular = cmd.angular.z;
  cmd.linear.x = 0;cmd.angular.z = 0;

  end = ros::Time::now().toSec();
  dur = end - begin;

  // linear update
  goal.x = goal.x - dur*linear;
  goal.y = goal.y;
  // angular update
  goal.x = goal.x*cos(-angular*dur) - goal.y*sin(-angular*dur);
  goal.y = goal.x*sin(-angular*dur) + goal.y*cos(-angular*dur);
  // declare attractive force
  //double X = gsqrt(pow(goal.x,3));
  //double Y = oal.ysqrt(pow(goal.y,3));
  force_att.x = pow(goal.x,3);
  force_att.y = pow(goal.y,3);

  ROS_INFO(" ");
  ROS_INFO("time      : %f",dur);
  ROS_INFO("goal      : %f , %f",goal.x,goal.y);
  ROS_INFO("att force : %f , %f",force_att.x,force_att.y);
  ROS_INFO("rep force : %f , %f",force_rep.x,force_rep.y);
  // ========================
  force.x = Ka_l*force_att.x - Kr_l*force_rep.x;
  force.y = Ka_a*force_att.y - Kr_a*force_rep.y;
  ROS_INFO("force : %f ,%f",force.x,force.y);
  cmd.linear.x = force.x;
  cmd.angular.z = force.y;
  // =======set confine==========

  if (cmd.linear.x >= 0.1)
    cmd.linear.x = 0.1;
  else if (cmd.linear.x <= -0.1)
    cmd.linear.x = -0.1;
  else
    cmd.linear.x = cmd.linear.x;

  if (cmd.angular.z >= 1)
    cmd.angular.z = 1;
  else if (cmd.angular.z <= -1)
    cmd.angular.z = -1;
  else
    cmd.angular.z = cmd.angular.z;
  ROS_INFO("cmd       : %f , %f",cmd.linear.x,cmd.angular.z);
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
