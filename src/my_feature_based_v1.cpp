#include "../include/feature_based_functions.h"

/*
./my_feature_based_v1 20 130 2 --> This works well
*/


// GLOBAL VARIABLES
boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector_;

template<typename FeatureType>
typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor_;

template<typename FeatureType>
class MyFeatureMatcher
{
  public:
    MyFeatureMatcher (
                  boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstructor,
                  typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source,
                  typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target);


    // TODO: This all really shouldn't be public, it's sloppy
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_;
    typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source_;
    typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target_;
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_segmented_;
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_segmented_;
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_transformed_; // ADD THIS TO CLOUD AT END
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_registered_;
    typename pcl::PointCloud<FeatureType>::Ptr source_features_;
    typename pcl::PointCloud<FeatureType>::Ptr target_features_;
    std::vector<int> source2target_;
    std::vector<int> target2source_;
    pcl::CorrespondencesPtr correspondences_;
    Eigen::Matrix4f initial_transformation_matrix_; // IMPORTANT
    Eigen::Matrix4f transformation_matrix_; 
    bool show_source2target_;
    bool show_target2source_;
    bool show_correspondences;

    /**
     * @brief starts the event loop for the visualizer
     */
    void run ();
  protected:

    void findCorrespondences (typename pcl::PointCloud<FeatureType>::Ptr source, typename pcl::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const;

    void filterCorrespondences ();

    void determineInitialTransformation ();
};



template<typename FeatureType>
MyFeatureMatcher<FeatureType>::MyFeatureMatcher(
                                        boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstructor,
                                        typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source,
                                        typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target)
: source_keypoints_ (new pcl::PointCloud<pcl::PointXYZI> ())
, target_keypoints_ (new pcl::PointCloud<pcl::PointXYZI> ())
, source_ (source)
, target_ (target)
, source_segmented_ (new pcl::PointCloud<pcl::PointXYZRGB>)
, target_segmented_ (new pcl::PointCloud<pcl::PointXYZRGB>)
, source_transformed_ (new pcl::PointCloud<pcl::PointXYZRGB>)
, source_registered_ (new pcl::PointCloud<pcl::PointXYZRGB>)
, source_features_ (new pcl::PointCloud<FeatureType>)
, target_features_ (new pcl::PointCloud<FeatureType>)
, correspondences_ (new pcl::Correspondences)
, show_source2target_ (false)
, show_target2source_ (false)
, show_correspondences (false)
{

  *source_segmented_ = *source_;
  *target_segmented_ = *target_;
 
  // // Remove NaNs from pointcloud //already removed NaNs
  // std::vector<int> indices_0;
  // std::vector<int> indices_1;
  // pcl::removeNaNFromPointCloud(*source_segmented_, *source_segmented_, indices_0);
  // pcl::removeNaNFromPointCloud(*target_segmented_, *target_segmented_, indices_1);

  // // Downsampling pointcloud for speed reasons //already downsampling
  // pcl::VoxelGrid<pcl::PointXYZRGB> sor0;
  // sor0.setInputCloud(source_segmented_);
  // sor0.setLeafSize(0.005f, 0.005f, 0.005f);
  // sor0.filter(*source_segmented_);

  // pcl::VoxelGrid<pcl::PointXYZRGB> sor1;
  // sor0.setInputCloud(target_segmented_);
  // sor0.setLeafSize(0.005f, 0.005f, 0.005f);
  // sor0.filter(*target_segmented_);

  detectKeypoints (keypoint_detector_, source_segmented_, source_keypoints_);
  detectKeypoints (keypoint_detector_, target_segmented_, target_keypoints_);

  extractDescriptors (feature_extractor_, source_segmented_, source_keypoints_, source_features_);
  extractDescriptors (feature_extractor_, target_segmented_, target_keypoints_, target_features_);

  findCorrespondences (source_features_, target_features_, source2target_);
  findCorrespondences (target_features_, source_features_, target2source_);

  filterCorrespondences ();

  determineInitialTransformation ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename FeatureType>
void MyFeatureMatcher<FeatureType>::findCorrespondences (typename pcl::PointCloud<FeatureType>::Ptr source, typename pcl::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const
{
  cout << "correspondence assignment..." << std::flush;
  correspondences.resize (source->size());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<FeatureType> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target);

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (int i = 0; i < static_cast<int> (source->size ()); ++i)
  {
    descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
    correspondences[i] = k_indices[0];
  }
  cout << "OK" << endl;
}

template<typename FeatureType>
void MyFeatureMatcher<FeatureType>::filterCorrespondences ()
{
  // First check if the correspondences are calculated as the same index in both clouds
  cout << "correspondence rejection..." << std::flush;
  std::vector<std::pair<unsigned, unsigned> > correspondences;
  for (size_t cIdx = 0; cIdx < source2target_.size (); ++cIdx)
    if (target2source_[source2target_[cIdx]] == static_cast<int> (cIdx))
      correspondences.push_back(std::make_pair(cIdx, source2target_[cIdx]));

  correspondences_->resize (correspondences.size());
  for (size_t cIdx = 0; cIdx < correspondences.size(); ++cIdx)
  {
    (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
    (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
  }

  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
  rejector.setInputSource (source_keypoints_);
  rejector.setInputTarget (target_keypoints_);
  rejector.setInputCorrespondences(correspondences_);

  rejector.setInlierThreshold(0.2); // Default is 0.05
  rejector.getCorrespondences(*correspondences_);

  cout << "OK. Correspondences found: " << correspondences_->size() << endl;
}

template<typename FeatureType>
void MyFeatureMatcher<FeatureType>::determineInitialTransformation ()
{
  cout << "initial alignment..." << std::flush;
  pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);

  transformation_estimation->estimateRigidTransformation (*source_keypoints_, *target_keypoints_, *correspondences_, initial_transformation_matrix_);

  pcl::transformPointCloud(*source_segmented_, *source_transformed_, initial_transformation_matrix_);
  pcl::io::savePCDFileASCII ("Features_Only_Registration_Guess.pcd", *source_transformed_ + *target_segmented_);
  cout << "OK" << endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv) {
	int start_time = clock();
//------ INITIALIZING VARIABLES ------//
	int start_frame = atoi(argv[1]);
	int end_frame = atoi(argv[2]);
	int step = atoi(argv[3]); // TODO: Add warning if three arguments aren't used

	// This defines the transform between frame_0 and frame_i
	Eigen::Matrix4f map_transform = Eigen::Matrix4f::Identity(); 

	// std::string path = "/home/andre/scratchSLAM/src/image_pipeline/stereo_image_proc/Kinect_Pointclouds";
	std::string path = "/home/andre/sim_test_1";
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
		sor1.setLeafSize(0.01f, 0.01f, 0.01f);
		sor1.filter(*cloud_1_f);

	//------ REMOVING OUTLIERS ------//

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_fo (new pcl::PointCloud<pcl::PointXYZRGB>);

		cloud_1_fo = cloud_1_f;

	//------ RUNNING FEATURE-BASED ALIGNMENT TO CREATE ALIGNED POINTCLOUD (FROM 0 --> 1) ------//

    // (1) EXTRACT KEYPOINTS
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>;
    sift3D->setScales (0.01f, 3, 2);
    sift3D->setMinimumContrast (0.1);
    keypoint_detector_.reset (sift3D);

    // (2) CALCULATE DESCRIPTORS
    pcl::Feature<pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr feature_extractor (new pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>);
    feature_extractor->setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    feature_extractor->setRadiusSearch (0.05);
    feature_extractor_ = feature_extractor;

    // (2) ACTUALLY RUN EVERYTHING
    boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstruction; //TODO: remove this
		MyFeatureMatcher<pcl::FPFHSignature33> FeatureMatcher (surface_reconstruction, cloud_1_fo, cloud_0_f);


		Eigen::Matrix4f iter_transform = FeatureMatcher.initial_transformation_matrix_;
		map_transform = map_transform * iter_transform; // Transforming all clouds into cloud_0 frame




	//------ CREATING MAP_CLOUD BY CONCATENATING (F)BLUE and (2)GREEN ------//
		// Transform cloud_1 into cloud_0 coordinates
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_trans (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*cloud_1_fo, *cloud_1_trans, map_transform);
		// cloud_1_trans = FeatureMatcher.source_transformed_;

		// pcl::VoxelGrid<pcl::PointXYZRGB> sor2; //TODO: Somehow this is deleting the map?
		// sor2.setInputCloud(Map);
		// sor2.setLeafSize(0.005f, 0.005f, 0.005f);
		// sor2.filter(*Map);

		*Map = *Map + *cloud_1_trans;
		std::cout << "W,H of New Map is: " << Map->width <<  "," << Map->height << std::endl;
		//TODO: Need to downsample global map as I go to avoid slowdown

	//------ PREPARING FOR NEXT LOOP ------//
		*cloud_0_f = *cloud_1_fo; // set cloud_0 (previous) = cloud_1 (current)

	} // end for loop


	pcl::io::savePCDFileASCII ("ICP_output_0.pcd", *Map);
	pcl::io::savePLYFileBinary("ICP_output_0.ply", *Map);
	std::cout << "W,H of Downsampled Map is: " << Map->width <<  "," << Map->height << std::endl;
/*
  // TODO: Fix this viewer, so it shows in color
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
