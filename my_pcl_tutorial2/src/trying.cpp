#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include<ros/ros.h>
// STL


// PCL
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>


#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;
 void filter_nan(pcl::PointCloud<pcl::PointXYZRGB> ::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB> ::Ptr outputCloud)
{
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
cloud->is_dense = false;
 std::vector<int> indices;
 pcl::removeNaNFromPointCloud(*cloud, *outputCloud, indices);
 std::cout << "size: " << outputCloud->points.size () << std::endl;
  // for (size_t i = 0; i < outputCloud->points.size (); ++i)
  //   std::cout << "" << i
	//       << "    " << outputCloud->points[i].x
  //             << " "    << outputCloud->points[i].y
  //             << " "    << outputCloud->points[i].z << std::endl;
  
}
void load_pcl(pcl::PointCloud<pcl::PointXYZRGB> ::Ptr input,std::string path)
{

if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path, *input) == -1) //* load the file 592732000.pcd
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    
  }
  std::cout << "Loaded "
            << input->width * input->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  
  
}
void segmenter(pcl::PointCloud<pcl::PointXYZRGB> ::Ptr segmented)
{
pcl::PointIndices::Ptr indices;
pcl::PointIndices indices_internal;
pcl::SACSegmentation<PointC> seg;

seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);

seg.setDistanceThreshold(0.01);
seg.setInputCloud(segmented);

Eigen::Vector3f axis;
axis << 0, -1, 0;
seg.setAxis(axis);
seg.setEpsAngle(pcl::deg2rad(10.0));

pcl::ModelCoefficients coeff;
seg.segment(indices_internal, coeff);


*indices = indices_internal;
 std::cout << *indices << std::endl ;

int index=NULL;

  for (size_t i = 0; i <indices->indices.size(); ++i)
    {
       segmented->points[indices->indices[i]].r=255;
    }
}
void save_pcd(pcl::PointCloud<pcl::PointXYZRGB>::Ptr SAVEEEE)
{
pcl::io::savePCDFile<pcl::PointXYZRGB>("colored.pcd",*SAVEEEE,false);

}
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  load_pcl(cloud,"725215000.pcd");
  filter_nan(cloud,outputCloud);
  segmenter(outputCloud);
  save_pcd(outputCloud);




  return (0);
}
