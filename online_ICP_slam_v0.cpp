#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

#include <vector>
#include <string>

#include <iostream>
#include <pcl/io/pcd_io.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"

/*
void convert_cloud(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*input,pcl_pc2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
}
*/

// GLOBAL VARIABLES
pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);


void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{

	// pcl::PointCloud<pcl::PointXYZRGB> cloud = *msg; //Dereference boost shared_ptr to get to actual PointCloud object

	// pcl::PCLPointCloud2 pcl_pc2;
	// pcl_conversions::toPCL(*msg,pcl_pc2);
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	// pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

	// std::cout << "Received " << temp_cloud->points.size() << " data points " << std::endl;

	pcl::fromROSMsg(*msg, *temp_cloud);
	// pcl::io::savePCDFileASCII ("/home/andre/cloud.pcd", *temp_cloud);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pointcloud_proc");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/front/depth/points", 1, callback);
	ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/clouds", 10);

	// Loop at 30 Hz
	ros::Rate loop_rate(30);

	while (ros::ok()) {
		//ROS_INFO("Publishing: [%f], [%f]", pub_msg.linear.z, pub_msg.linear.x);
		pub.publish(*temp_cloud);

		ros::spinOnce();
		loop_rate.sleep();
	}

}