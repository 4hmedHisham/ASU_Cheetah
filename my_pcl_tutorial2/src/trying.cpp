#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// STL


// PCL
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("592732000.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

cloud->is_dense = false;



 std::vector<int> indices;
 pcl::removeNaNFromPointCloud(*cloud, *outputCloud, indices);
 std::cout << "size: " << outputCloud->points.size () << std::endl;
  for (size_t i = 0; i < outputCloud->points.size (); ++i)
    std::cout << "" << i
	      << "    " << outputCloud->points[i].x
              << " "    << outputCloud->points[i].y
              << " "    << outputCloud->points[i].z << std::endl;

  return (0);
}
