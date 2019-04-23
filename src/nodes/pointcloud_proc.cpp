#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <vector>
#include <string>

#include <iostream>
#include <pcl/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
std::vector<PointCloud::ConstPtr> PointCloud_Accumulator;
int lastSeqNumber = 0;
int count = 0;

void callback(const PointCloud::ConstPtr& msg)
{
  //ROS_INFO_STREAM(PointCloud_Accumulator.size() << " is divisible by 3." ) ;

  pcl::PointCloud<pcl::PointXYZRGB> cloud = *msg; //Dereference boost shared_ptr to get to actual PointCloud object

  std::string path = "/home/andre/scratchSLAM/src/image_pipeline/stereo_image_proc/Kinect_Pointclouds_room2";

  std::string filename = path + "/EuRoC_Pointcloud_" + std::to_string(count) + ".pcd";

  //std::string filename = "test1239123123.pcd";

  pcl::io::savePCDFileASCII (filename, cloud);
  count++;
  std::cout << "Saved " << cloud.points.size() << " data points to " << filename << std::endl;

  if (lastSeqNumber + 1 != msg->header.seq) {
    int skippedNum = msg->header.seq - (lastSeqNumber + 1);
    std::cout << "Dropping messages! Skipped " << skippedNum << " messages!" << std::endl;
  }

  lastSeqNumber = msg->header.seq;


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_proc");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth_registered/points", 1, callback);
  ros::spin();
}