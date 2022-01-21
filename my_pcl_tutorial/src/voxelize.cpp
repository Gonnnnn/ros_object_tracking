#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

// voxelized one
ros::Publisher pub;

void input (const sensor_msgs::PointCloud2ConstPtr& scan)
{
  //////Msg to pointcloud
  // 처음에 받아줄 때 pcl::PointXYZ 형태로 받아도 되는지 확인 후 주석 지우자
  pcl::PointCloud<pcl::PointXYZI>::Ptr src(new pcl::PointCloud<pcl::PointXYZI>);
  // ptr을 통해 각각의 pointcloud들을 참조한다. 값을 복사하거나 덮어씌우거나 할 수 있다.
  pcl::fromROSMsg(*scan, *src);

  pcl::PointCloud<pcl::PointXYZI>::Ptr voxelized(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;

  float voxel_size = 0.2;
  voxel_filter.setInputCloud (src);
  voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_filter.filter(*voxelized);

  pcl::PCLPointCloud2 voxelized_cloud;
  pcl::toPCLPointCloud2(*voxelized, voxelized_cloud);
  sensor_msgs::PointCloud2 voxelized_output;
  pcl_conversions::fromPCL(voxelized_cloud, voxelized_output);

  voxelized_output.header.frame_id = "velodyne";
  pub.publish(voxelized_output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "voxelize");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 100, input);

  // Create a ROS publisher for the output point cloud
  // pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points", 100);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_voxelized", 100);

  // Spin
  ros::spin ();
}