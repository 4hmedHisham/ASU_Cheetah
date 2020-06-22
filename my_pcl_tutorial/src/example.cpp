#include <ros/ros.h>
#include "std_msgs/String.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sstream>
#include<string>



#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
pcl::visualization::PCLVisualizer::Ptr
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}
ros::Publisher pub;
bool once=true;
int twice=0;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  //pub.publish (output);
  if (twice<1)
  {
    std_msgs::String msg;

    //std::stringstream ss;
    
    msg.data = "Saved on new once bgad i guess";
    ROS_INFO("%s", msg.data.c_str());
  pcl::io::savePCDFile ("trying_smth_new_once_bgad_16_Jun.pcd", output);
  twice =twice+1;
  }
  
  // Initialize ROS
}

void load_file(std::string path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // Load bun0.pcd -- should be available with the PCL archive in test 
   pcl::PointCloud<pcl::PointXYZ> cloudnew;


  pcl::io::loadPCDFile (path, *cloud);
    // creates the visualization object and adds either our original cloud or all of the inliers
  // depending on the command line arguments specified.





}
void save_file()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::savePCDFile ("test_pcd.pcd", cloud);

}
int
main (int argc, char** argv)
{
    if(false)
    {
      std_msgs::String msg;

    //std::stringstream ss;
    
    msg.data = "WAZZAAAP";
    ROS_INFO("%s", msg.data.c_str());
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  //printf("START");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  msg.data = "3adet1";
    ROS_INFO("%s", msg.data.c_str());
  ros::Subscriber sub = nh.subscribe ("camera/depth/points", 1, cloud_cb);
  msg.data = "3adet2";
    ROS_INFO("%s", msg.data.c_str());

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  //printf("DONE");
  // Spin
  ros::spin ();
    }
    if(true)
    {
      load_file("trying_smth_new_oneshot_bgad.pcd");
    }
    if(false)
    {
      save_file();
    }
}