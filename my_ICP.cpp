#include <iostream>
#include <ctime>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>


//#include <pcl/filters/uniform_sampling.h>


int main (int argc, char** argv) {
	int start_time = clock();
//------ INITIALIZING VARIABLES ------//
	Eigen::Matrix4f map_transform; // This defines the transform between frame_i and frame_0


//------ LOADING FILES ------//
	int load_start_time = clock();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>);

	std::string path = "/home/andre/scratchSLAM/src/image_pipeline/stereo_image_proc/EuRoC_Pointclouds";
	std::string filename_0 = path + "/EuRoC_Pointcloud_" + argv[1] + ".pcd";
	std::string filename_1 = path + "/EuRoC_Pointcloud_" + argv[2] + ".pcd";

	pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename_0, *cloud_0);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename_1, *cloud_1);

	// std::cout << "Loaded " << cloud_0->width * cloud_0->height << " data points from " << filename_0 << std::endl;
	// std::cout << "Loaded " << cloud_1->width * cloud_1->height << " data points from " << filename_1 << std::endl;
	int load_end_time = clock();

//------ REMOVING NAN VALUES ------//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0_proc (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_proc (new pcl::PointCloud<pcl::PointXYZRGB>);

	std::vector<int> indices_0;
	pcl::removeNaNFromPointCloud(*cloud_0, *cloud_0_proc, indices_0);

	std::vector<int> indices_1;
	pcl::removeNaNFromPointCloud(*cloud_1, *cloud_1_proc, indices_1);

//------ DOWNSAMPLING CLOUDS FOR SPEED REASONS ------//
	int filter_start_time = clock();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0_f (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_f (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::VoxelGrid<pcl::PointXYZRGB> sor0;
	sor0.setInputCloud(cloud_0_proc);
	sor0.setLeafSize(0.1f, 0.1f, 0.1f);
	sor0.filter(*cloud_0_f);

	pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
	sor1.setInputCloud(cloud_1_proc);
	sor1.setLeafSize(0.1f, 0.1f, 0.1f);
	sor1.filter(*cloud_1_f);

//------ REMOVING OUTLIERS ------//
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_0_fo (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_fo (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> f0;
	f0.setInputCloud(cloud_0_f);
	f0.setMeanK(50);
	f0.setStddevMulThresh(0.3);
	f0.filter(*cloud_0_f);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> f1;
	f1.setInputCloud(cloud_1_f);
	f1.setMeanK(50);
	f1.setStddevMulThresh(0.3);
	f1.filter(*cloud_1_f);

	pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(cloud_0_f);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius(75);
    outrem.filter (*cloud_0_fo);

    outrem.setInputCloud(cloud_1_f);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius(75);
    outrem.filter (*cloud_1_fo);

	int filter_end_time = clock();


//------ RUNNING ICP TO CREATE ALIGNED POINTCLOUD ------//
	int icp_start_time = clock();
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(cloud_0_fo);
	icp.setInputTarget(cloud_1_fo);

	std::cout << "Starting IterativeClosestPoint..." << std::endl;
	pcl::PointCloud<pcl::PointXYZRGB> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	int icp_end_time = clock();

	map_transform = map_transform * icp.getFinalTransformation(); // Note that we have transformed

//------ CREATING MAP_CLOUD BY CONCATENATING (F)BLUE and (2)GREEN ------//
	// Need to make Final pointcloud a Ptr object in order to pass it to viewer
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	Final_ptr = Final.makeShared();

	// Creating final map cloud which is a combination of cloud_0 and Final
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map (new pcl::PointCloud<pcl::PointXYZRGB>);
	*Map = *cloud_1_fo + *Final_ptr;

	// std::cout << "W,H of cloud_0_f is: " << cloud_0_f->width <<  "," << cloud_0_f->height << std::endl;
	// std::cout << "W,H of cloud_1_f is: " << cloud_1_f->width <<  "," << cloud_1_f->height << std::endl;
	// std::cout << "W,H of Final is: " << Final_ptr->width <<  "," << Final_ptr->height << std::endl;
	// std::cout << "W,H of Map is: " << Map->width <<  "," << Map->height << std::endl;

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

	pcl::visualization::PCLVisualizer viewer;
	int v1(0);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb1(cloud_0_fo, 255, 0, 0); //This will display the point cloud in red (R,G,B)
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud_0_fo, rgb1, "cloud_0_fo", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb2(cloud_1_fo, 0, 255, 0); //This will display the point cloud in green (R,G,B)
	viewer.addPointCloud<pcl::PointXYZRGB> (cloud_1_fo, rgb2, "cloud_1_fo", v1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb3(Final_ptr, 0, 0, 255); //This will display the point cloud in green (R,G,B)
	viewer.addPointCloud<pcl::PointXYZRGB> (Final_ptr, rgb3, "Final", v1);

	int v2(0);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb4(Map, 255, 255, 255); //This will display the point cloud in green (R,G,B)
	viewer.addPointCloud<pcl::PointXYZRGB> (Map, rgb4, "Map", v2);

	viewer.resetCamera();
	viewer.spin();

	return 0;
}
