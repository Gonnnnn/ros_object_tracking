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

#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;

// Publish clustered PCL data
ros::Publisher pub;

void input (const sensor_msgs::PointCloud2ConstPtr& scan)
{
  //////Msg to pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
  // ptr을 통해 각각의 pointcloud들을 참조한다. 값을 복사하거나 덮어씌우거나 할 수 있다.
  pcl::fromROSMsg(*scan, *src);
  pcl::PointCloud<pcl::PointXYZ> src_cloud = *src;

///////////////////////clustering 과정 시작///////////////////////

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  // ptr로 받아야함. pointcloud로 받으니까 에러 뜨더라   
  tree->setInputCloud(src);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  // 1m의 포인트와 포인트 간의 간격? 이라고 한다
  ec.setClusterTolerance(1);
  // 한 cluster의 최소, 최대 포인트 개수
  ec.setMinClusterSize(4);
  ec.setMaxClusterSize(40);
  // 검색 방법이 tree라는 것을 명시해주는 것
  ec.setSearchMethod(tree);
  // clustering 결과를 입력할 pointer 집어넣기. 현 source 파일 기준 src
  ec.setInputCloud(src);

  ec.extract(cluster_indices);

  // 위 과정 후 아래 for문에서는 cluster_indices를 가지고 뭘 한다.
  // iterator을 선언해서 처음부터 끝까지 쭉 돌아주며 특정 작업을 하는데
  // 클래스(struct인지는 나도 모르지만.. 아마 클래스겠지) 내부 구조를 모르다보니 뭐 하는거지 잘은 모르겠다.
  // 확실한 것은 clutering결과를 total_cloud에 넣어주는 것

  pcl::PointCloud<pcl::PointXYZI> total_cloud;
  // iterator 선언 후 첫 원소부터 끝 원소까지 훑기
  // 예제 소스는 ++it이었다. 왜 그런지는 모르겠다 아직.
  int j = 0; // intensity에만 들어가는 변수. 왜 1씩 더해지며 들어가는지는 잘 모름
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++){
      for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++){
          pcl::PointXYZ point1 = src->points[*pit];
          pcl::PointXYZI point2;
          point2.x = point1.x, point2.y = point1.y, point2.z = point1.z;
          point2.intensity = (float)(j+1);
          total_cloud.push_back(point2);
      }
  j++;
  }

///////////////////////clustering 과정 시작///////////////////////

  pcl::PCLPointCloud2 clustered_cloud;
  pcl::toPCLPointCloud2(total_cloud, clustered_cloud);
  sensor_msgs::PointCloud2 clustered_output;
  pcl_conversions::fromPCL(clustered_cloud, clustered_output);

  clustered_output.header.frame_id = "velodyne";
  pub.publish(clustered_output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cluster");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points_roied", 100, input);

  // Create a ROS publisher for the output point cloud
  // pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points", 100);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points_clustered", 100);

  // Spin
  ros::spin ();
}