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
//#include <pcl/filters/uniform_sampling.h>

/*
Combining Chunks
WORKING --> 55/87/1 (not much motion)
WORKING --> 85/95/3
(SORTOF) WORKING --> 26/105/5 
0/100/1 --> 35,47,50,51,52,53

*/


int main (int argc, char** argv) {
	int start_time = clock();
//------ INITIALIZING VARIABLES ------//
	int start_frame = atoi(argv[1]);
	int end_frame = atoi(argv[2]);
	int step = atoi(argv[3]); // TODO: Add warning if three arguments aren't used

	// This defines the transform between frame_0 and frame_i
	// Eigen::Matrix4d map_transform = Eigen::Matrix4d::Identity(); 
	// Eigen::Matrix4d map_transform_prev = Eigen::Matrix4d::Identity(); 
	// Eigen::Matrix4d map_transform;
	Eigen::Matrix4f map_transform;

	// std::string path = "/home/andre/scratchSLAM/src/image_pipeline/stereo_image_proc/Kinect_Pointclouds";
	std::string path = "/home/andre/Kinect_Subset";
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
	sor0.setLeafSize(0.1f, 0.1f, 0.1f);
	sor0.filter(*cloud_0_f);

	// This defines our global map cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map (new pcl::PointCloud<pcl::PointXYZRGB>);

	// TODO: Fix this --> This is a hacky attempt to skip clouds with high noise
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map_prev (new pcl::PointCloud<pcl::PointXYZRGB>);

	// TODO: Fix this --> Hacky solution to skip specific clouds that may be messed up
	// std::vector<int> v = {33,34,35,46,47,49,50,51,52};

	for (int i = start_frame + step; i < end_frame; i+=step) {
		// if (std::find(v.begin(), v.end(), i) != v.end()) {
		// 	std::cout << "Skipping Index: " << i;
		// 	continue;
		// }

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
		sor1.setLeafSize(0.1f, 0.1f, 0.1f);
		sor1.filter(*cloud_1_f);

	//------ REMOVING OUTLIERS ------//

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_fo (new pcl::PointCloud<pcl::PointXYZRGB>);
		// cloud_1_fo = cloud_1_f;

		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> f;
		f.setInputCloud(cloud_1_f);
		f.setMeanK(50);
		f.setStddevMulThresh(1.0);
		f.filter(*cloud_1_f);
		int filter_end_time = clock();

		pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
	    outrem.setInputCloud(cloud_1_f);
	    outrem.setRadiusSearch(0.5);
	    outrem.setMinNeighborsInRadius(75);
	    outrem.filter (*cloud_1_fo);

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
		// map_transform = map_transform * iter_transform; // Transforming all clouds into cloud_0 frame

	//------ CREATING MAP_CLOUD BY CONCATENATING (F)BLUE and (2)GREEN ------//
		// Need to make Final pointcloud a Ptr object in order to pass it to viewer
		// pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
		// Final_ptr = Final.makeShared();

		// Transform cloud_1 into cloud_0 coordinates
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_trans (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*cloud_1_fo, *cloud_1_trans, map_transform);

		// Creating final map cloud which is a combination of cloud_0 and cloud_1
		// *Map += *cloud_0_f + *cloud_1_trans;
		// *Map += *Final_ptr;
		// *Map_prev = *Map;
		// std::cout << "W,H of Old Map is: " << Map->width <<  "," << Map->height << std::endl;
		// std::cout << "W,H of cloud_1_trans is: " << cloud_1_trans->width <<  "," << cloud_1_trans->height << std::endl;

		*Map = *Map + *cloud_1_trans;
		std::cout << "W,H of New Map is: " << Map->width <<  "," << Map->height << std::endl;

		// std::cout << "W,H of cloud_0_f is: " << cloud_0_f->width <<  "," << cloud_0_f->height << std::endl;
		// std::cout << "W,H of cloud_1_f is: " << cloud_1_f->width <<  "," << cloud_1_f->height << std::endl;
		// std::cout << "W,H of cloud_1_trans is: " << cloud_1_trans->width <<  "," << cloud_1_trans->height << std::endl;

	//------ VISUALIZING 3 POINTCLOUDS ------//

		// NOT ENOUGH MOTION --> 0/1
		// JUST RIGHT --> 2/3 (.0023), 3/4 (.0187)
		// NO MATCH --> 20/21

		// 1 - RED   - Original (first) pointcloud (the PointCloud to begin from)
		// 2 - GREEN - Transformed (second) pointcloud (the PointCloud which we want cloud_0 to look like.)
		// F - BLUE  - ICP output cloud 

		// (F)BLUE is just (1)RED mapped into (2)GREEN coordinates
		// Therefore I should be combining (F)BLUE and (2)GREEN without a transformation
		// std::cout << "Load   Time: " << (double) (load_end_time-load_start_time)/CLOCKS_PER_SEC << " seconds to run." << std::endl;	
		// std::cout << "Filter Time: " << (double) (filter_end_time-filter_start_time)/CLOCKS_PER_SEC << " seconds to run." << std::endl;	
		// std::cout << "ICP    Time: " << (double) (icp_end_time-icp_start_time)/CLOCKS_PER_SEC << " seconds to run." << std::endl;	
		// std::cout << "Total  Time: " << (double) (clock()-start_time)/CLOCKS_PER_SEC << " seconds to run." << std::endl;

	//------ PREPARING FOR NEXT LOOP ------//
		*cloud_0_f = *cloud_1_fo; // set cloud_0 (previous) = cloud_1 (current)

		// do { // Waiting for Keypress
		// 	cout << '\n' << "Press Enter to continue...";
		// } while (cin.get() != '\n');

	} // end for loop

	pcl::io::savePCDFileASCII ("ICP_output_0.pcd", *Map);

	pcl::visualization::PCLVisualizer viewer;
	// int v1(0);
	// viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb1(cloud_0_f, 255, 0, 0); //This will display the point cloud in red (R,G,B)
	// viewer.addPointCloud<pcl::PointXYZRGB> (cloud_0_f, rgb1, "cloud_0_f", v1);
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb2(cloud_1_f, 0, 255, 0); //This will display the point cloud in green (R,G,B)
	// viewer.addPointCloud<pcl::PointXYZRGB> (cloud_1_f, rgb2, "cloud_1_f", v1);
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb3(cloud_1_trans, 0, 0, 255); //This will display the point cloud in green (R,G,B)
	// viewer.addPointCloud<pcl::PointXYZRGB> (cloud_1_trans, rgb3, "Final", v1);

	std::cout << "W,H of Final Map is: " << Map->width <<  "," << Map->height << std::endl;
	int v2(0);
	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v2);
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb4(Map, 255, 255, 255); //This will display the point cloud in green (R,G,B)
	// viewer.addPointCloud<pcl::PointXYZRGB> (Map, rgb4, "Map", v2);
	
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb4(cloud_0_f, 255, 255, 255); //This will display the point cloud in green (R,G,B)
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud_0_f, rgb4, "Map", v2);
	
	
	viewer.resetCamera();
	// viewer.spinOnce(2500);
	viewer.spin();

	return 0;
}
