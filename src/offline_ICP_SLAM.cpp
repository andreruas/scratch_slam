#include <iostream>
#include <ctime>
#include <stdlib.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/ply_io.h>
//#include <pcl/filters/uniform_sampling.h>

/*
./my_ICP_v3 20 130 2 --> This works well
*/

int main (int argc, char** argv) {
	int start_time = clock();
//------ INITIALIZING VARIABLES ------//
	int start_frame = atoi(argv[1]);
	int end_frame = atoi(argv[2]);
	int step = atoi(argv[3]); // TODO: Add warning if three arguments aren't used
	// TODO: Add fourth argument to choose path

	// This defines the transform between frame_0 and frame_i
	Eigen::Matrix4f map_transform = Eigen::Matrix4f::Identity(); 

	// std::string path = "/home/andre/scratchSLAM/src/image_pipeline/stereo_image_proc/Kinect_Pointclouds";
	std::string path = "/home/andre/RSP_Pointclouds/Kinect_Pointclouds_room2";
	std::string filename_0 = path + "/EuRoC_Pointcloud_" + std::to_string(start_frame) + ".pcd";

	// This defines a 'previous' cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename_0, *cloud_0);

	// Removing NaN Values from 'previous' cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0_proc (new pcl::PointCloud<pcl::PointXYZRGB>);
	std::vector<int> indices_0;
	pcl::removeNaNFromPointCloud(*cloud_0, *cloud_0_proc, indices_0);

	// Downsampling 'previous' cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0_f (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::VoxelGrid<pcl::PointXYZRGB> sor0;
	sor0.setInputCloud(cloud_0_proc);
	sor0.setLeafSize(0.01f, 0.01f, 0.01f);
	sor0.filter(*cloud_0_f);

	// This defines our global map cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map (new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = start_frame + step; i < end_frame; i+=step) {

	//------ LOADING NEW CLOUD ------//
		int load_start_time = clock();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>);
		std::string filename_1 = path + "/EuRoC_Pointcloud_" + std::to_string(i) + ".pcd";
		pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename_1, *cloud_1);

		// std::cout << "Loaded " << cloud_1->width * cloud_1->height << " data points from " << filename_1 << std::endl;
		std::cout << "Loaded " << filename_1 << std::endl;

		int load_end_time = clock();

	//------ REMOVING NAN VALUES ------//
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_proc (new pcl::PointCloud<pcl::PointXYZRGB>);

		std::vector<int> indices_1;
		pcl::removeNaNFromPointCloud(*cloud_1, *cloud_1_proc, indices_1);

	//------ DOWNSAMPLING CLOUDS FOR SPEED REASONS ------//
		int filter_start_time = clock();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_f (new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
		sor1.setInputCloud(cloud_1_proc);
		sor1.setLeafSize(0.01f, 0.01f, 0.01f);
		sor1.filter(*cloud_1_f);

	//------ REMOVING OUTLIERS ------//

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_fo (new pcl::PointCloud<pcl::PointXYZRGB>);
		// // cloud_1_fo = cloud_1_f;

		// pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> f;
		// f.setInputCloud(cloud_1_f);
		// f.setMeanK(50);
		// f.setStddevMulThresh(1.0);
		// f.filter(*cloud_1_f);
		// int filter_end_time = clock();

		// pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	 //    outrem.setInputCloud(cloud_1_f);
	 //    outrem.setRadiusSearch(0.5);
	 //    outrem.setMinNeighborsInRadius(75);
	 //    outrem.filter (*cloud_1_fo);

		cloud_1_fo = cloud_1_f;


	//------ RUNNING ICP TO CREATE ALIGNED POINTCLOUD (FROM 0 --> 1) ------//
		int icp_start_time = clock();
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		// icp.setInputSource(cloud_0_f);
		// icp.setInputTarget(cloud_1_f);
		icp.setInputTarget(cloud_0_f);
		icp.setInputSource(cloud_1_fo);

		// std::cout << "Starting IterativeClosestPoint (0-->1)..." << std::endl;
		pcl::PointCloud<pcl::PointXYZRGB> Final;
		icp.align(Final);
		if (icp.getFitnessScore() > .1) {
			std::cout << "WARNING: Poor Fitness Score!" << std::endl;
			// *Map = *Map_prev; // try to recover by using older map
			// map_transform = map_transform_prev;
			// *Map -= *cloud_1_trans;
			continue; // Skip matches with poor scores (usually due to noise far from camera)
		}
		std::cout << "ICP converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

		std::cout << "" << std::endl;

		// std::cout << icp.getFinalTransformation() << std::endl;
		int icp_end_time = clock();

		// map_transform_prev = map_transform;

		// Need to cast to <double> for more precision (double is 2x precision of float)
		// Eigen::Matrix4d iter_transform = icp.getFinalTransformation().cast<double>();
		Eigen::Matrix4f iter_transform = icp.getFinalTransformation();
		map_transform = map_transform * iter_transform; // Transforming all clouds into cloud_0 frame

	//------ CREATING MAP_CLOUD BY CONCATENATING (F)BLUE and (2)GREEN ------//
		// Need to make Final pointcloud a Ptr object in order to pass it to viewer
		// pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
		// Final_ptr = Final.makeShared();

		// Transform cloud_1 into cloud_0 coordinates
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_trans (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*cloud_1_fo, *cloud_1_trans, map_transform);

		pcl::VoxelGrid<pcl::PointXYZRGB> sor2;
		sor2.setInputCloud(Map);
		sor2.setLeafSize(0.001f, 0.001f, 0.001f);
		sor2.filter(*Map);

		*Map = *Map + *cloud_1_trans;
		std::cout << "W,H of New Map is: " << Map->width <<  "," << Map->height << std::endl;
		//TODO: Need to downsample global map as I go to avoid slowdown

	//------ PREPARING FOR NEXT LOOP ------//
		*cloud_0_f = *cloud_1_fo; // set cloud_0 (previous) = cloud_1 (current)

	} // end for loop

	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_f (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::io::savePCDFileASCII ("ICP_output_0.pcd", *Map);
	pcl::io::savePLYFileBinary("ICP_output_0.ply", *Map);
	std::cout << "W,H of Downsampled Map is: " << Map->width <<  "," << Map->height << std::endl;


	pcl::visualization::PCLVisualizer viewer;

	std::cout << "W,H of Final Map is: " << Map->width <<  "," << Map->height << std::endl;
	int v2(0);
	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb4(Map, 255, 255, 255); //This will display the point cloud in green (R,G,B)
	viewer.addPointCloud<pcl::PointXYZRGB> (Map, rgb4, "Map", v2);

	viewer.resetCamera();
	viewer.spin();

	return 0;
}
