#include "ros/ros.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

ros::Publisher filtered_pc_pub;
tf::TransformListener *tf_listener;


void filterCallback(const sensor_msgs::PointCloud2::ConstPtr& original_pc)
{

  sensor_msgs::PointCloud2::Ptr cloud_transformed(new sensor_msgs::PointCloud2());
  sensor_msgs::PointCloud2::Ptr cloud_filtered(new sensor_msgs::PointCloud2());
  sensor_msgs::PointCloud2::Ptr cloud_transformed_back(new sensor_msgs::PointCloud2());


  tf::StampedTransform transform;
  ros::Time now = ros::Time::now();
  tf_listener->waitForTransform ("/world", "/camera_rgb_optical_frame", now, ros::Duration(4.0));
  tf_listener->lookupTransform ("/world", "/camera_rgb_optical_frame", now, transform);


  Eigen::Matrix4f eigen_transform;
  pcl_ros::transformAsMatrix (transform, eigen_transform);
  pcl_ros::transformPointCloud (eigen_transform, *original_pc, *cloud_transformed);


  pcl::PassThrough<sensor_msgs::PointCloud2>pass;
  pass.setInputCloud(cloud_transformed);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-1.0, 0.15);
  pass.filter(*cloud_transformed);


  pass.setInputCloud(cloud_transformed);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.15, 1.0);
  pass.filter(*cloud_filtered);
  

  pcl_ros::transformPointCloud("/camera_rgb_optical_frame",
		      *cloud_filtered,
		      *cloud_transformed_back,
		      *tf_listener);
  cloud_transformed_back->header.frame_id = "/camera_rgb_optical_frame";

  
  filtered_pc_pub.publish(*cloud_transformed_back);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filteredpcserver");
  ros::NodeHandle n;
  tf::TransformListener tfl;
  tf_listener = &tfl;
  filtered_pc_pub = n.advertise<sensor_msgs::PointCloud2>("filtered_pc", 1);
  ros::Subscriber original_pc_sub = n.subscribe("/camera/depth_registered/points", 1, filterCallback);

  ros::spin();
  return 0;
}
