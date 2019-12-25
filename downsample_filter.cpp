// This node receives a pointcloud and donwsamples it, using a voxel grid filter from the PCL library
// Alternatively this could be integrated into a service and be called only on request

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>


// Declaring the publisher and subscriber
ros::Publisher pub;
ros::Subscriber sub;

// Service callback; receives pointcloud, returns downsampled cloud
const std_msgs::String::ConstPtr& msg
bool service_callback (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input){

  ROS_INFO("Callback");

  // Declaring pointcloud object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud_pcl); // Convert from ROS message to PCL pointcloud datatype

  std::cout << "PointCloud has: " << cloud_pcl->points.size () << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1.5cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud_pcl);
  vg.setLeafSize (0.015f, 0.015f, 0.015f); // originally all 0.01f
  vg.filter (*cloud_filtered);
  std::cout << "Callback: PointCloud after donwsampling has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*


  // Publish the filtered cloud to a ROS topic
  sensor_msgs::PointCloud2::Ptr clusters (new sensor_msgs::PointCloud2); // Create a new PointCloud2 ROS message
  pcl::toROSMsg (*cloud_filtered , *filtered_cloud); // Convert from PCL pointcloud to PointCloud2 message
  clusters->header.frame_id = "/camera_link"; // ROS frame to publish message to
  clusters->header.stamp=ros::Time::now(); // Define message time stamp
  pub.publish (*filtered_cloud);

  return true;
}


int main (int argc, char** argv) {

  // Initialize node
  ros::init(argc, argv, "passthrough_node");

  ros::NodeHandle nh;

  // Subscriber that receives the original pointcloud
  sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("cloud_subscriber",  100, subscriber_callback);

  // Pointcloud publisher
  pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 100);

  ros::spin();

  return 0;
}
