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
ros::NodeHandle* n;


void filterCallback(const sensor_msgs::PointCloud2::ConstPtr& sensor_message_pc)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xy(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZRGB>());
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


  //float x_min = "/filtered_pc";
  float x_min = -0.2;
  float x_max = 0.23;
  n->getParam("/pc_filter/x_max", x_max);
  n->getParam("/pc_filter/x_min", x_min);
  pcl::PassThrough<pcl::PointXYZRGB >pass;
  pass.setInputCloud(cloud_transformed);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_min, x_max);
  pass.filter(*cloud_filtered_x);

  float y_min = -0.65;
  float y_max = 0.1;
  n->getParam("/pc_filter/y_min", y_min);
  n->getParam("/pc_filter/y_max", y_max);
  pass.setInputCloud(cloud_filtered_x);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_min, y_max);
  pass.filter(*cloud_filtered_xy);

  float z_min = 0.0;
  float z_max = 0.4;
  n->getParam("/pc_filter/z_min", z_min);
  n->getParam("/pc_filter/z_max", z_max);
  pass.setInputCloud(cloud_filtered_xy);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_min, z_max);
  pass.filter(*cloud_filtered_xyz);

//  pcl_ros::transformPointCloud("/camera_rgb_optical_frame",
//		      *cloud_filtered,
//		      *cloud_transformed_back,
//		      *tf_listener);
  cloud_filtered_xyz->header.frame_id = "/world";

  sensor_msgs::PointCloud2 cloud_transformed_back_msg;
  pcl::PCLPointCloud2 cloud_transformed_back_pc2;
  pcl::toPCLPointCloud2(*cloud_filtered_xyz, cloud_transformed_back_pc2);
  
  pcl_conversions::fromPCL(cloud_transformed_back_pc2, cloud_transformed_back_msg);
  filtered_pc_pub.publish(cloud_transformed_back_msg);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filteredpcserver");
  n = new ros::NodeHandle();
  tf::TransformListener tfl;
  tf_listener = &tfl;
  filtered_pc_pub = n->advertise<sensor_msgs::PointCloud2>("filtered_pc", 1);
  ros::Subscriber original_pc_sub = n->subscribe("/camera/depth_registered/points", 1, filterCallback);

  ros::spin();
  return 0;
}
