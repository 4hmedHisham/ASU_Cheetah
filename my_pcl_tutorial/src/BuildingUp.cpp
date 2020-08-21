#include <ros/ros.h>
#include "std_msgs/String.h"
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// int main(int argc, char** argv)
// {
// // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
// // if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
// //   {
// //     PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
// //     return (-1);
// //   }

// //   ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
// //   sensor_msgs::PointCloud2 output;
// //   pcl::toROSMsg(*cloud, output);
// //   pub.publish (output);
// }
void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
 //ROS_INFO("inside callback");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("cloud_pcd", 1, cloud_callback);
  ros::spin();
}

