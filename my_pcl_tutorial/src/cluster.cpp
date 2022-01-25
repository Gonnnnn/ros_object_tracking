#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>

// ROS
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

// the ones below were being included in the src file i read through
// i didn't look into them but apparently they dont have to be included

// #include <pcl/features/normal_3d.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/kdtree/kdtree.h>

// #include <pcl/io/pcd_io.h>
// #include <pcl/common/common.h>
// #include <pcl/common/centroid.h>
// #include <pcl/common/transforms.h>
// #include <set>
// #include <boost/format.hpp>

// #include <pcl/filters/passthrough.h>
// #include <pcl/sample_consensus/ransac.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>

// #include <pcl/filters/voxel_grid.h>

using namespace std;

// Publish clustered PCL data
ros::Publisher pub;

void input (const sensor_msgs::PointCloud2ConstPtr& scan)
{
  //////Msg to pointcloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr src(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*scan, *src);

///////////////////////clustering 시작///////////////////////

  float clusterTolerance = 0.1;
  int minSize = 5;
  int maxSize = 2500;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(src);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  // cluter라고 판단할만한 point들의 간격
  ec.setClusterTolerance(1);
  // 한 cluster의 최소, 최대 포인트 개수
  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(1000);
  // 검색 방법이 tree라는 것을 명시해주는 것
  ec.setSearchMethod(tree);
  // clustering 결과를 입력할 pointer 집어넣기. 현 source 파일 기준 src
  ec.setInputCloud(src);
  ec.extract(cluster_indices);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
  for (pcl::PointIndices getIndices: cluster_indices)
    {
        // every loop, initialize a cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster (new pcl::PointCloud<pcl::PointXYZI>);
        // get each index in cluster in cluster_indices and push it into the cloud above
        for (int index : getIndices.indices)
            cloudCluster->points.push_back(src->points[index]);
        // set width, height and such --> https://pointclouds.org/documentation/singletonpcl_1_1_point_cloud.html
        // width
        //// it specifies the total number of points in the cloud
        //// or the width (total number of points in a row) of an organized PC dataset
        // height
        //// it specifies the height (total number of rows) of an organized PC dataset
        //// it is set to 1 for unorganized datasets(thus used to check whether a dataset is organized or not)
        // is_dense
        //// specifies if all the data in points is finite (true), or whether it might contain Inf/NaN values (false)
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        // push the cloud into clusters(a vector containing pcl::PointCloud<PointT>::Ptr)
        clusters.push_back(cloudCluster);
    }
///////////////////////clustering 끝///////////////////////

  try { 
    // 예외가 발생하면 예외를 던지는 영역
    if (clusters.size() == 0) throw clusters;
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp = clusters[0];
    pcl::PointCloud<pcl::PointXYZI> cloud1 = *temp;
    pcl::PCLPointCloud2 clustered_cloud;
    pcl::toPCLPointCloud2(cloud1, clustered_cloud);
    sensor_msgs::PointCloud2 clustered_output;
    pcl_conversions::fromPCL(clustered_cloud, clustered_output);

    clustered_output.header.frame_id = "velodyne";
    pub.publish(clustered_output);
  } catch (std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> exception) {// 던져진 예외를 잡는 영역
  // 예외 처리 영역
  std::cout << "there is no cluster" << std::endl;
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points_obstacles", 100, input);

  // Create a ROS publisher for the output point cloud
  // pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points", 100);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_clustered", 100);

  // Spin
  ros::spin ();
}