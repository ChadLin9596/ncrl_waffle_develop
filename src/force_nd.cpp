/*
 * Author : Chun-Jung Lin
 * Date : 27 05 2018
 * Brief : Use VLP-16 lidar to detect obstacle then calculate the repulsive force 
*/
#include <ncrl_waffle_develop/force_nd.h>

// define point cloud type
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

// declare point cloud pointer
PointCloudXYZ::Ptr cloud_XYZ (new PointCloudXYZ);
PointCloudXYZRGB::Ptr cloud_XYZRGB (new PointCloudXYZRGB);
geometry_msgs::Point force;
//declare publisher & subscriber
ros::Publisher pub_XYZRGB;
ros::Publisher pub_force;
ros::Subscriber sub_pointcloud;

// declare global variable
bool lock = false;
void pointcloud_processing(void);
int m = 2;

// caculate the repulsive force with the number and the distance of the points in ROI
void setConfine()
{
  int count = 0;
  force.x = 0;
  force.y = 0;
  for (size_t i = 0; i < cloud_XYZRGB->points.size(); i++)
  {
    float X = cloud_XYZRGB->points[i].x;
    float Y = cloud_XYZRGB->points[i].y;
    float Z = cloud_XYZRGB->points[i].z;
    float distance = sqrt(pow(X,2)+pow(Y,2));
    if (Z >= -0.1 && Z <= 0.1&& distance <= 1 && X > 0) // ROI
    {
      cloud_XYZRGB->points[i].r = 255;
      cloud_XYZRGB->points[i].g = 0;
      cloud_XYZRGB->points[i].b = 0;
      double nx = X;
      double ny = Y;
      if (nx >= 0)
          nx = 1;
      else
          nx = -1;
      if (ny >= 0)
          ny = 1;
      else
          ny = -1;
      force.x += nx/pow(X,m);
      force.y += ny/pow(Y,m);
      count += 1;
    }
  }
  if (count == 0)
  {
    force.x = 0;
    force.y = 0;
  }
  else
  {
  force.x = force.x/count;
  force.y = force.y/count;
  }
  ROS_INFO(" ");
  ROS_INFO("force: %f , %f count: %d",force.x,force.y,count);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if (!lock){
    lock = true;

    pcl::fromROSMsg (*input, *cloud_XYZ); //covert from ros type to pcl type
    copyPointCloud(*cloud_XYZ, *cloud_XYZRGB); //it will include frame information

    for (size_t i = 0; i < cloud_XYZRGB->points.size(); i++)
    {
        // white color
        cloud_XYZRGB->points[i].r = 255;
        cloud_XYZRGB->points[i].g = 255;
        cloud_XYZRGB->points[i].b = 255;
    }
    pointcloud_processing();
  }
  else
  {
    std::cout << "lock" << std::endl;
  }
}

void pointcloud_processing()
{
  setConfine();
  pub_force.publish(force);
  pub_XYZRGB.publish(*cloud_XYZRGB);
  lock = false;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "force_nd");
  ros::NodeHandle nh;

  sub_pointcloud = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, cloud_cb);
  pub_XYZRGB     = nh.advertise<PointCloudXYZRGB>         ("/ncrl/cloudpoint", 1);
  pub_force      = nh.advertise<geometry_msgs::Point>     ("/ncrl/repulsive/force",1);

  ros::spin ();
}
