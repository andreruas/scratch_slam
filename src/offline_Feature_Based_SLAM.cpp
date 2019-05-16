#include "../include/functions.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
./offline_Feature_Based_SLAM "/home/andre/RSP_Pointclouds/sim_test_1" 20 130 2 --> This works well
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv) {
	int start_time = clock();

	//------ INITIALIZING VARIABLES ------//	
	// TODO: Add warning if three arguments aren't used

	std::string path = argv[1];
	int start_frame = atoi(argv[2]);
	int end_frame = atoi(argv[3]);
	int step = atoi(argv[4]);
	// TODO: Add fourth argument to choose path

	float leafSize = 0.005f;
	float rej_thresh = 0.15;

	// This defines the transform between frame_0 and frame_i
	Eigen::Matrix4f map_transform = Eigen::Matrix4f::Identity(); 

	// Note: I named all of my files are called EuRoC_Pointcloud, even though they are not all from EuRoC (don't ask me why)
	// std::string path = "/home/andre/RSP_Pointclouds/sim_test_1"; 
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
	sor0.setLeafSize(leafSize, leafSize, leafSize);
	sor0.filter(*cloud_0_f);

	// This defines our global map cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Map (new pcl::PointCloud<pcl::PointXYZRGB>);



	//------ ACCEPTING NEW CLOUDS, DOING PAIRWISE REGISTRATION WITH FEATURES ------//	
	for (int i = start_frame + step; i < end_frame; i+=step) {

	//------ LOADING NEW CLOUD ------//
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>);
		std::string filename_1 = path + "/EuRoC_Pointcloud_" + std::to_string(i) + ".pcd";
		pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename_1, *cloud_1);

		std::cout << "Loaded " << filename_1 << std::endl;

	//------ REMOVING NAN VALUES ------//
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_proc (new pcl::PointCloud<pcl::PointXYZRGB>);

		std::vector<int> indices_1;
		pcl::removeNaNFromPointCloud(*cloud_1, *cloud_1_proc, indices_1);

	//------ DOWNSAMPLING CLOUDS FOR SPEED REASONS ------//
		int filter_start_time = clock();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_f (new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
		sor1.setInputCloud(cloud_1_proc);
		sor1.setLeafSize(leafSize, leafSize, leafSize);
		sor1.filter(*cloud_1_f);

	//------ REMOVING OUTLIERS ------//
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_fo (new pcl::PointCloud<pcl::PointXYZRGB>);
		
		// I've removed all of my outlier removal code, because it adds unreliability to the model
		// This code was mainly used for stereo, and I need to tune it again for the Kinect
		// TODO: Add in outlier removal methods
		
		/*
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
		*/

		cloud_1_fo = cloud_1_f;

	//------ RUNNING FEATURE-BASED ALIGNMENT TO CREATE ALIGNED POINTCLOUD (FROM 0 --> 1) ------//
	boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector;


    // (1) EXTRACT KEYPOINTS
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>;
    sift3D->setScales (0.01f, 3, 2); // Good Values --> (0.01f, 3, 2) --> (0.01f, 2, 1)
    sift3D->setMinimumContrast(0.2); // Default 0.0, raise to extract fewer, distinct features
    keypoint_detector.reset (sift3D);

    // (2) CALCULATE DESCRIPTORS
    pcl::Feature<pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr feature_extractor (new pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>);
    feature_extractor->setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    feature_extractor->setRadiusSearch (0.05);

    // (3) GET TRANSFORM FROM FEATURE MATCHER CODE (CODE IN FEATURE_MATCHER.H)
    boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstruction; //TODO: remove this
	MyFeatureMatcher<pcl::FPFHSignature33> FeatureMatcher (keypoint_detector, feature_extractor, surface_reconstruction, cloud_1_fo, cloud_0_f, rej_thresh);

	Eigen::Matrix4f iter_transform = FeatureMatcher.initial_transformation_matrix_; 
	map_transform = map_transform * iter_transform; // Transforming all clouds into cloud_0 frame


	//------ CREATING MAP_CLOUD BY CONCATENATING cloud_0 and cloud_1 ------//
	// Transform cloud_1 into cloud_0 coordinates
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_trans (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*cloud_1_fo, *cloud_1_trans, map_transform);

	// DOWNSAMPLE THE MAP_CLOUD DURING THE LOOP SO IT DOESN"T GET TOO BIG, DELETE REDUNDANT POINTS
	// pcl::VoxelGrid<pcl::PointXYZRGB> sor2;
	// sor2.setInputCloud(Map);
	// sor2.setLeafSize(0.005f, 0.005f, 0.005f);
	// sor2.filter(*Map);

	*Map = *Map + *cloud_1_trans;
	std::cout << "W,H of New Map is: " << Map->width <<  "," << Map->height << std::endl;
	//TODO: Need to downsample global map as I go to avoid slowdown

	//------ PREPARING FOR NEXT LOOP ------//
		*cloud_0_f = *cloud_1_fo; // set cloud_0 (previous) = cloud_1 (current)

	} // end for loop

	// SAVING FILES FOR VISUALIZATION WITH PCL_VIEWER
	pcl::io::savePCDFileASCII ("Feature_Based_output_0.pcd", *Map);
	pcl::io::savePLYFileBinary("Feature_Based_output_0.ply", *Map);
	std::cout << "W,H of Downsampled Map is: " << Map->width <<  "," << Map->height << std::endl;

/*
	// THIS VISUALIZER SHOWS THE POINT CLOUD AT THE END OF THE LOOP
	// REMOVED BECAUSE IT IS BETTER TO JUST USE 'PCL_VIEWER' FOR LARGE CLOUD VISUALIZATIONS
	pcl::visualization::PCLVisualizer viewer;

	std::cout << "W,H of Final Map is: " << Map->width <<  "," << Map->height << std::endl;
	int v2(0);
	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rgb4(Map, 255, 255, 255); //This will display the point cloud in green (R,G,B)
	viewer.addPointCloud<pcl::PointXYZRGB> (Map, rgb4, "Map", v2);

	viewer.resetCamera();
	viewer.spin();
*/

	return 0;
}
