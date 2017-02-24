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

bool use_y_clip_;
bool use_z_clip_;
bool use_x_clip_;

void filterCallback(const sensor_msgs::PointCloud2ConstPtr& sensor_message_pc)
{
  pcl::PCLPointCloud2 original_pc2;
  pcl_conversions::toPCL(*sensor_message_pc, original_pc2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromPCLPointCloud2(original_pc2, *original_pc);

  tf::StampedTransform transform;
  ros::Time now = ros::Time::now();
  tf_listener->waitForTransform ("/world", "/kinect2_rgb_optical_frame", now, ros::Duration(4.0));
  tf_listener->lookupTransform ("/world", "/kinect2_rgb_optical_frame", now, transform);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_x(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xy(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_xyz(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed_back(new pcl::PointCloud<pcl::PointXYZRGB>());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud = cloud_transformed;

  original_pc->header.frame_id = "/kinect2_rgb_optical_frame";
  pcl_ros::transformPointCloud("/world",
          *original_pc,
          *cloud_transformed,
          *tf_listener);

  pcl::PassThrough<pcl::PointXYZRGB > pass;

  if(use_x_clip_) {
    pass.setInputCloud(current_cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_clip_min_, x_clip_max_);
    pass.filter(*cloud_filtered_x);

    current_cloud = cloud_filtered_x;
  }

  if(use_y_clip_) {
    pass.setInputCloud(current_cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_clip_min_, y_clip_max_);
    pass.filter(*cloud_filtered_xy);

    current_cloud = cloud_filtered_xy;
  }

  if(use_z_clip_) {
    pass.setInputCloud(current_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_clip_min_, z_clip_max_);
    pass.filter(*cloud_filtered_xyz);

    current_cloud = cloud_filtered_xyz;
  }

  cloud_filtered_xyz->header.frame_id = "/world";
  pcl_ros::transformPointCloud("/kinect2_rgb_optical_frame",
              *current_cloud,
		      *cloud_transformed_back,
              *tf_listener);

  cloud_transformed_back->header.frame_id = "/kinect2_rgb_optical_frame";

  pcl::PCLPointCloud2 cloud_transformed_back_pc2;
  pcl::toPCLPointCloud2(*cloud_transformed_back, cloud_transformed_back_pc2);

  sensor_msgs::PointCloud2 cloud_transformed_back_msg;
  pcl_conversions::fromPCL(cloud_transformed_back_pc2, cloud_transformed_back_msg);

  filtered_pc_pub.publish(cloud_transformed_back_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filteredpcserver");
  n = new ros::NodeHandle();
  tf::TransformListener tfl;
  tf_listener = &tfl;

  n->getParam("x_clip_min", x_clip_min_);
  n->getParam("x_clip_max", x_clip_max_);
  n->getParam("y_clip_min", y_clip_min_);
  n->getParam("y_clip_max", y_clip_max_);
  n->getParam("z_clip_min", z_clip_min_);
  n->getParam("z_clip_max", z_clip_max_);

  n->getParam("use_y_clip", use_y_clip_);
  n->getParam("use_z_clip", use_z_clip_);
  n->getParam("use_x_clip", use_x_clip_);
//Old setup
  x_clip_min_ = -0.18;
  x_clip_max_ = 0.6;
  y_clip_min_ = -0.65;
  y_clip_max_ = -0.15;
  z_clip_min_ = -0.1;
  z_clip_max_ = 0.35;

//New setup
  x_clip_min_ = -0.5;
  x_clip_max_ = -0.1;
  y_clip_min_ = 0.0;
  y_clip_max_ = 0.32;
  z_clip_min_ = -0.1;
  z_clip_max_ = 0.35;

  use_y_clip_ = true;
//  use_z_clip_ = true;
  use_x_clip_ = true;

  ros::Subscriber original_pc_sub = n->subscribe("/kinect2/hd/points", 1, filterCallback);
  filtered_pc_pub = n->advertise<sensor_msgs::PointCloud2>("filtered_pc", 1);

  ros::spin();
  return 0;
}
