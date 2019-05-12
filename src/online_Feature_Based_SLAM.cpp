
#include <vector>
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>


#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>


// GLOBAL VARIABLES
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map (new pcl::PointCloud<pcl::PointXYZRGB>);

sensor_msgs::PointCloud2 ROS_Map;
// ROS_Map.Header.frame_id = "World"

int count = 0;
bool publish = false;

// This defines the transform between frame_0 and frame_i
Eigen::Matrix4f map_transform = Eigen::Matrix4f::Identity(); 

tf::TransformBroadcaster *br;


void DownsampleAndFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_proc (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Remove NaN values
	std::vector<int> indices_1;
	pcl::removeNaNFromPointCloud(*input, *input_proc, indices_1);

	pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
	sor1.setInputCloud(input_proc);
	sor1.setLeafSize(0.05f, 0.05f, 0.05f); // TODO: Other filtering methods? // 0.05 works well
	sor1.filter(*input_proc);

	input = input_proc;
	// TODO: Some sort of filtering, if needed
}

// REF: github.com/ccny-ros-pkg/ccny_rgbd_tools/blob/f501bc04cb491b33016fbf076e6af5fc23a80466/ccny_rgbd/src/util.cpp
tf::Transform tfFromEigen(Eigen::Matrix4f t)
{
  tf::Transform tf;
  
  tf::Matrix3x3 m;
  m.setValue(t(0,0),t(0,1),t(0,2),
             t(1,0),t(1,1),t(1,2),
             t(2,0),t(2,1),t(2,2));
  tf.setBasis(m);
  
  tf.setOrigin(tf::Vector3(t(0,3),t(1,3),t(2,3)));
  
  return tf;
}


void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{	
	if (count == 0) { // The first time you loop through, you need to save cloud_0
		pcl::fromROSMsg(*msg, *cloud_0);
		DownsampleAndFilter(cloud_0);
	}

	else {
		// std::cout << "Reading in Message... " << std::endl;
		ROS_INFO("Reading in Message... ");

		pcl::fromROSMsg(*msg, *cloud_1); // Save most recent message as cloud_

		// std::cout << "Downsampling & Filtering... " << std::endl;
		ROS_INFO("Downsampling & Filtering... ");

		DownsampleAndFilter(cloud_1);

		// (1) RUN ICP ON CLOUD_0 and CLOUD_1
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		icp.setInputTarget(cloud_0);
		icp.setInputSource(cloud_1);

		// std::cout << "Running ICP... " << std::endl;
		ROS_INFO("Running ICP... ");

		pcl::PointCloud<pcl::PointXYZRGB> Final;
		icp.align(Final);


		// (2) Transform Clouds
		std::cout << "Transforming Clouds... " << std::endl;
		Eigen::Matrix4f iter_transform = icp.getFinalTransformation();
		map_transform = map_transform * iter_transform; // Transforming all clouds into cloud_0 frame

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_trans (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*cloud_1, *cloud_1_trans, map_transform);

		std::cout << "Adding to Map and looping... " << std::endl << std::endl;
		*Map = *Map + *cloud_1_trans;
		std::cout << "W,H of New Map is: " << Map->width <<  "," << Map->height << std::endl;

		// (3) MAKE CLOUD_0 = CLOUD_1
		*cloud_0 = *cloud_1; // set cloud_0 (previous) = cloud_1 (current)

		// (4) Publish motion of camera to ROS
		tf::Transform transform = tfFromEigen(map_transform); // I don't think this is supposed to be .inverse()

		br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "icp_odom"));


		publish = true; // this flag tells my publisher when new information is added to map

	}

	count += 1;
}




int main(int argc, char** argv)
{
	ros::init(argc, argv, "ICP_SLAM_node");
	ros::NodeHandle nh;
	br = new tf::TransformBroadcaster(); //define br in main


	// /camera/depth/points:=/front/depth/points --> from the kinect
	// /front/depth/points:=/camera/depth/points --> from the kinect

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, callback);
	// ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("/clouds", 10);
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/Map", 10);

	// Loop at 30 Hz
	ros::Rate loop_rate(30);

	while (ros::ok()) {
		//ROS_INFO("Publishing: [%f], [%f]", pub_msg.linear.z, pub_msg.linear.x);

		if (publish) { // I only want to publish when new information has been added
			// DownsampleAndFilter(Map);

			// pcl::VoxelGrid<pcl::PointXYZRGB> sor_map;
			// sor_map.setInputCloud(Map);
			// sor_map.setLeafSize(0.005f, 0.005f, 0.005f);
			// sor_map.filter(*Map);

			pcl::toROSMsg(*Map, ROS_Map);
			ROS_Map.header.frame_id = "map";	

			pub.publish(ROS_Map);
			publish = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

}
