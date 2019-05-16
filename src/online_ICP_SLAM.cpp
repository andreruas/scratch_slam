
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

// This is the map we will publish through ROS as a PointCloud2 message
sensor_msgs::PointCloud2 ROS_Map;

// Define ICP globally
pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;


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

// TODO: Use Whitcomb's recommended method (with Mutexes), this is bad form.
// These are the links he sent me about threads:
	// https://answers.ros.org/question/217960/how-to-write-a-thread-in-ros/
	// https://theboostcpplibraries.com/boost.thread-synchronization

void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{	
	if (count == 0) { // The first time you loop through, you need to save cloud_0
		pcl::fromROSMsg(*msg, *cloud_0);
		DownsampleAndFilter(cloud_0);
	}

	else { // Registration between cloud_0 and cloud_1

		// TODO: Add Framerate Tracker

		std::cout << "Reading in Message... " << std::endl;

		pcl::fromROSMsg(*msg, *cloud_1); // Save most recent message as cloud_

		std::cout << "Downsampling & Filtering... " << std::endl;

		DownsampleAndFilter(cloud_1);

		// (1) RUN ICP ON CLOUD_0 and CLOUD_1
		icp.setInputTarget(cloud_0);
		icp.setInputSource(cloud_1);

		std::cout << "Running ICP... " << std::endl;

		pcl::PointCloud<pcl::PointXYZRGB> Final;
		icp.align(Final);

		std::cout << "Trans Epsilon: " << icp.getFitnessScore() << std::endl;

		std::cout << "Transforming Clouds... " << std::endl;
		Eigen::Matrix4f iter_transform = icp.getFinalTransformation();
		map_transform = map_transform * iter_transform; // Transforming all clouds into cloud_0 frame

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_trans (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*cloud_1, *cloud_1_trans, map_transform);

		std::cout << "Adding to Map and looping... " << std::endl;
		*Map = *Map + *cloud_1_trans;
		std::cout << "W,H of New Map is: " << Map->width <<  "," << Map->height << std::endl  << std::endl;

		// (2) MAKE CLOUD_0 = CLOUD_1
		*cloud_0 = *cloud_1; // set cloud_0 (previous) = cloud_1 (current)

		/* Deprecated, but interesting ways of transforming, could use this later
		// Publish World-->icp_odom Transform
		// tf::Transform transform;
		// transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0) );
		// tf::Quaternion q;
		// q.setRPY(0, 0, 0);
		// transform.setRotation(q);

		// Eigen::Matrix4d map_transform_double(map_transform.cast<double>());
		// Eigen::Affine3d affine(map_transform_double);
		// tf::Transform transform;
		// tf::transformEigenToTF(affine, transform);
		*/ 

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

	// Subscribe to Pointclouds (from arbitrary source, but typically I use a Kinect)
	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, callback);
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/Map", 10);

	//// SET PARAMETERS FOR ICP | This can be tricky to get right
		// ICP has several termination criteria (ICP will finish if any of the three are satisfied)

	// Set the max correspondence distance to 10 cm (correspondences with higher distances will be ignored)
	// This helps ICP ignore noise, but can destroy maps if clouds are too far away from one another (but aligned well)
	// Requires clouds to be very closely aligned initially
	// TODO: Add this parameter in after odometry guessing is added (and clouds are pre-transformed)
	// icp.setMaxCorrespondenceDistance (0.05); // 0.05 is a good value

	// Set the maximum number of iterations (first criterion)
	icp.setMaximumIterations (40);
	
	// Set the transformation epsilon (second criterion)
	icp.setTransformationEpsilon (1e-7);

	// Set the euclidean distance difference epsilon (third criterion)
	// No longer using this criterion
	// icp.setEuclideanFitnessEpsilon (1);

	// Loop at 30 Hz
	ros::Rate loop_rate(30);

	while (ros::ok()) { // To Do: Improve this, loop rate isn't the best way to do this

		if (publish) { // I only want to publish when new information has been added

			// DownsampleAndFilter(Map); // No longer downsampling global map at each iteration 

			pcl::toROSMsg(*Map, ROS_Map);
			ROS_Map.header.frame_id = "map";	

			pub.publish(ROS_Map);
			publish = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}