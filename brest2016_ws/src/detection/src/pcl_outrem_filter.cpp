#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

ros::Publisher pub;
float radiusSearch;
float minNeighbors;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Convert from Ros msg to pcl::PointCloud2
  pcl::PCLPointCloud2 pcl2_input;
  pcl_conversions::toPCL(*cloud_msg, pcl2_input);

  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Convert from pcl::PointCloud2 to pcl::PointCloud<PointXYZ>
  pcl::fromPCLPointCloud2(pcl2_input, *cloud);

  // Filter
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  // build the filter
  outrem.setInputCloud(cloud);
  outrem.setRadiusSearch(radiusSearch);
  outrem.setMinNeighborsInRadius (minNeighbors);
  // apply filter
  std::cerr << "Start filtering...";
  outrem.filter (*cloud_filtered);
  std::cerr << "done\n";
  

  // Convert back from pcl::PointCloud<PointXYZ> to pcl::PointCloud2
  pcl::PCLPointCloud2 pcl2_output;
  pcl::toPCLPointCloud2(*cloud_filtered, pcl2_output);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(pcl2_output, output);


  // Comparing log
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").\n";
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").\n";
  std::cerr << "----------------\n";
  // // Publish the data
  pub.publish (output);
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "outrem_filter");
  ros::NodeHandle nh;

  // Get parameter
  ros::param::get("~radiusSearch", radiusSearch);
  ros::param::get("~minNeighbors", minNeighbors);
  std::cerr << "got " << radiusSearch << "\n";

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}