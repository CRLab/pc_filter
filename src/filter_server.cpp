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

float x_clip_min_;
float x_clip_max_;
float y_clip_min_;
float y_clip_max_;
float z_clip_min_;
float z_clip_max_;

void filterCallback(const sensor_msgs::PointCloud2ConstPtr& sensor_message_pc)
{
  ROS_INFO("Filtering PointCloud Callback");
  pcl::PCLPointCloud2 original_pc2;
  pcl_conversions::toPCL(*sensor_message_pc, original_pc2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromPCLPointCloud2(original_pc2, *original_pc);

  tf::StampedTransform transform;
  ros::Time now = ros::Time::now();
  tf_listener->waitForTransform ("/world", "/head_camera_rgb_optical_frame", now, ros::Duration(2.0));
  tf_listener->lookupTransform ("/world", "/head_camera_rgb_optical_frame", now, transform);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xy(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed_back(new pcl::PointCloud<pcl::PointXYZRGB>());

  original_pc->header.frame_id = "/head_camera_rgb_optical_frame";
  pcl_ros::transformPointCloud("/world",
          *original_pc,
          *cloud_transformed,
          *tf_listener);

  pcl::PassThrough<pcl::PointXYZRGB > pass;
  pass.setInputCloud(cloud_transformed);

  ROS_INFO("filtercloudsize before x: ");
  ROS_INFO_STREAM(cloud_transformed->size());

  pass.setFilterFieldName ("x");
  pass.setFilterLimits (x_clip_min_, x_clip_max_);
  pass.filter(*cloud_filtered_x);


  ROS_INFO("filtercloudsize before y: ");
  ROS_INFO_STREAM(cloud_filtered_x->size());
  
  pass.setInputCloud(cloud_filtered_x);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (y_clip_min_, y_clip_max_);
  pass.filter(*cloud_filtered_xy);

  pass.setInputCloud(cloud_filtered_xy);
  

  ROS_INFO("filtercloudsizei before z: ");
  ROS_INFO_STREAM(cloud_filtered_xy->size());
  
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_clip_min_, z_clip_max_);
  pass.filter(*cloud_filtered_xyz);

  cloud_filtered_xyz->header.frame_id = "/world";
  pcl_ros::transformPointCloud("/head_camera_rgb_optical_frame",
		      *cloud_filtered_xyz,
		      *cloud_transformed_back,
		      *tf_listener);

  cloud_transformed_back->header.frame_id = "/head_camera_rgb_optical_frame";

  pcl::PCLPointCloud2 cloud_transformed_back_pc2;
  pcl::toPCLPointCloud2(*cloud_transformed_back, cloud_transformed_back_pc2);

  sensor_msgs::PointCloud2 cloud_transformed_back_msg;
  pcl_conversions::fromPCL(cloud_transformed_back_pc2, cloud_transformed_back_msg);
  ROS_INFO("filtercloudsize: ");
  ROS_INFO_STREAM(cloud_transformed_back->size());
  filtered_pc_pub.publish(cloud_transformed_back_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filteredpcserver");
  ros::NodeHandle n;
  tf::TransformListener tfl;
  tf_listener = &tfl;

  n.getParam("/xpassthrough/filter_limit_min", x_clip_min_);
  n.getParam("/xpassthrough/filter_limit_max", x_clip_max_); 
  n.getParam("/ypassthrough/filter_limit_min", y_clip_min_);
  n.getParam("/ypassthrough/filter_limit_max", y_clip_max_); 
  n.getParam("/zpassthrough/filter_limit_min", z_clip_min_);
  n.getParam("/zpassthrough/filter_limit_max", z_clip_max_);

  ros::Time now = ros::Time::now();
  ROS_INFO("WAITING for Checkerboard detection to find world frame");
  tf_listener->waitForTransform ("/world", "/head_camera_rgb_optical_frame", now, ros::Duration(10.0));

  ros::Subscriber original_pc_sub = n.subscribe("/head_camera/depth_registered/points", 1, filterCallback);
  filtered_pc_pub = n.advertise<sensor_msgs::PointCloud2>("filtered_pc", 1);

  ros::spin();
  return 0;
}
