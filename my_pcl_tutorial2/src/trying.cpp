#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

//for testing ros with string
#include "std_msgs/String.h"
#include <pcl/filters/voxel_grid.h>

#include <std_msgs/Float32.h>

bool debugging=false;



//for debug only
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  ROS_INFO("I in: [%d]",2);
  double x=99;
  double y=94;
  double z=98;
  std_msgs::Float32 msg;
  //init pointcloud2 pcl change from rosmsg pc2 to pcl pc2 , then from pcl pc2 to pcl
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
//  sensor_msgs::Pointcloud out_pointcloud;
//  sensor_msgs::convertPointCloud2ToPointCloud(input_pointcloud, out_cloud)
pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
pcl_conversions::toPCL(*input, *cloud);

  //sensor_msgs::PointCloud2 output;
  // Publish the data
    // Access the data
    ROS_INFO("SIZE IS  : [%d]",temp_cloud->points.size ());
  for (std::size_t i = 0; i < temp_cloud->points.size (); ++i)
  {
    x=temp_cloud->points[i].x ;
    y=temp_cloud->points[i].y ;
    z=temp_cloud->points[i].z;
    ros::Duration(0.5).sleep();
    msg.data = x;
    if(true)
    {
    ROS_INFO("I heard z : [%d]",z);
    ROS_INFO("I heard x : [%d]",x);
     ROS_INFO("I heard y : [%d]",y);
    
    }

    //cloud->points[i].z = 1.0;
  }

}
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
void chatterCallback2(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard2: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");


  ros::NodeHandle n;

 //debuging
if (debugging)
{
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::Subscriber sub2 = n.subscribe("chatter2", 1000, chatterCallback2);
}


  ros::Subscriber sub3 = n.subscribe("cloud_pcd", 1, cloud_cb);
  int x=0;
  ROS_INFO("%s\n", "STARTEDDDDD");
  x++;

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}