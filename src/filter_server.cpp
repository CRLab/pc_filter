#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

ros::Publisher filtered_pc_pub;
tf::TransformListener *tf_listener;


void filterCallback(const sensor_msgs::PointCloud2::ConstPtr& sensor_message_pc)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_y(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_z(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed_back(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PCLPointCloud2 original_pc2;
  pcl_conversions::toPCL(*sensor_message_pc, original_pc2);
  pcl::fromPCLPointCloud2(original_pc2, *original_pc);

  tf::StampedTransform transform;
  ros::Time now = ros::Time::now();
  tf_listener->waitForTransform ("/world", "/camera_rgb_optical_frame", now, ros::Duration(4.0));
  tf_listener->lookupTransform ("/world", "/camera_rgb_optical_frame", now, transform);

  Eigen::Matrix4f eigen_transform;
  pcl_ros::transformAsMatrix (transform, eigen_transform);
  pcl::transformPointCloud (*original_pc, *cloud_transformed, eigen_transform);
  
  pcl::PassThrough<pcl::PointXYZRGB >pass;
  pass.setInputCloud(cloud_transformed);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-1.0, 0.25);
  pass.filter(*cloud_filtered_x);

  pass.setInputCloud(cloud_filtered_x);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-100.0, 100.0);
  //pass.setFilterLimits (-0.0, 1.0);
  pass.filter(*cloud_filtered_y);

  pass.setInputCloud(cloud_filtered_y);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-100.01, 100.0);
  //pass.setFilterLimits (0.01, 1.0);
  pass.filter(*cloud_filtered_z);

  cloud_filtered_z->header.frame_id = "/world";
  pcl_ros::transformPointCloud("/camera_rgb_optical_frame",
		      *cloud_filtered_z,
		      *cloud_transformed_back,
		      *tf_listener);
  cloud_transformed_back->header.frame_id = "/camera_rgb_optical_frame";

  sensor_msgs::PointCloud2 cloud_transformed_back_msg;
  pcl::PCLPointCloud2 cloud_transformed_back_pc2;
  pcl::toPCLPointCloud2(*cloud_transformed_back, cloud_transformed_back_pc2);

  pcl_conversions::fromPCL(cloud_transformed_back_pc2, cloud_transformed_back_msg);
  filtered_pc_pub.publish(cloud_transformed_back_msg);

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
