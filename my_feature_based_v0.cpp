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

// #include <my_feature_based_methods.cpp> // TODO: Separate this into a functions file

/*
./my_feature_based_v0 20 130 2 --> This works well
*/

#include <vector>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>


template<typename FeatureType>
class ICCVTutorial
{
  public:
    ICCVTutorial (boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector,
                  typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor,
                  boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstructor,
                  typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source,
                  typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target);


    // TODO: This all really shouldn't be public
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_;
    boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector_;
    typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor_;
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
    /**
     * @brief Detects key points in the input point cloud
     * @param input the input point cloud
     * @param keypoints the resulting key points. Note that they are not necessarily a subset of the input cloud
     */
    void detectKeypoints (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const;

    /**
     * @brief extract descriptors for given key points
     * @param input point cloud to be used for descriptor extraction
     * @param keypoints locations where descriptors are to be extracted
     * @param features resulting descriptors
     */
    void extractDescriptors (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, typename pcl::PointCloud<FeatureType>::Ptr features);

    /**
     * @brief find corresponding features based on some metric
     * @param source source feature descriptors
     * @param target target feature descriptors
     * @param correspondences indices out of the target descriptors that correspond (nearest neighbor) to the source descriptors
     */
    void findCorrespondences (typename pcl::PointCloud<FeatureType>::Ptr source, typename pcl::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const;

    /**
     * @brief  remove non-consistent correspondences
     */
    void filterCorrespondences ();

    /**
     * @brief calculate the initial rigid transformation from filtered corresponding keypoints
     */
    void determineInitialTransformation ();
};



template<typename FeatureType>
ICCVTutorial<FeatureType>::ICCVTutorial(boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> >keypoint_detector,
                                        typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor,
                                        boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstructor,
                                        typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source,
                                        typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target)
: source_keypoints_ (new pcl::PointCloud<pcl::PointXYZI> ())
, target_keypoints_ (new pcl::PointCloud<pcl::PointXYZI> ())
, keypoint_detector_ (keypoint_detector)
, feature_extractor_ (feature_extractor)
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

  // segmentation (source_, source_segmented_);
  // segmentation (target_, target_segmented_);

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


  detectKeypoints (source_segmented_, source_keypoints_);
  detectKeypoints (target_segmented_, target_keypoints_);

  extractDescriptors (source_segmented_, source_keypoints_, source_features_);
  extractDescriptors (target_segmented_, target_keypoints_, target_features_);

  findCorrespondences (source_features_, target_features_, source2target_);
  findCorrespondences (target_features_, source_features_, target2source_);

  filterCorrespondences ();

  determineInitialTransformation ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename FeatureType>
void ICCVTutorial<FeatureType>::detectKeypoints (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const
{
  cout << "keypoint detection..." << std::flush;
  keypoint_detector_->setInputCloud(input);
  keypoint_detector_->compute(*keypoints);
  cout << "OK. keypoints found: " << keypoints->points.size() << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::extractDescriptors (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, typename pcl::PointCloud<FeatureType>::Ptr features)
{
  typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGB>);
  kpts->points.resize(keypoints->points.size());

  pcl::copyPointCloud(*keypoints, *kpts);

  typename pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, FeatureType>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, FeatureType> > (feature_extractor_);

  feature_extractor_->setSearchSurface(input);
  feature_extractor_->setInputCloud(kpts);

  if (feature_from_normals)
  //if (boost::dynamic_pointer_cast<typename pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, FeatureType> > (feature_extractor_))
  {
    cout << "normal estimation..." << std::flush;
    typename pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    normal_estimation.setRadiusSearch (0.01);
    normal_estimation.setInputCloud (input);
    normal_estimation.compute (*normals);
    feature_from_normals->setInputNormals(normals);
    cout << "OK" << endl;
  }

  cout << "descriptor extraction..." << std::flush;
  feature_extractor_->compute (*features);
  cout << "OK" << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::findCorrespondences (typename pcl::PointCloud<FeatureType>::Ptr source, typename pcl::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const
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
void ICCVTutorial<FeatureType>::filterCorrespondences ()
{
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
  rejector.getCorrespondences(*correspondences_);

// typedef boost::shared_ptr<std::vector<pcl::registration::Correspondence> > CorrespondencesPtr (type of correspondences_)

  cout << "OK. Correspondences found: " << correspondences_->size() << endl;
}

template<typename FeatureType>
void ICCVTutorial<FeatureType>::determineInitialTransformation ()
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
		boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector;

 		// Extract Keypoints
		pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>* sift3D = new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>;
		sift3D->setScales (0.01f, 3, 2);
		// sift3D->setScales (0.02f, 3, 2);
		sift3D->setMinimumContrast (0.0);
		keypoint_detector.reset (sift3D);

		pcl::Feature<pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr feature_extractor (new pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>);
		feature_extractor->setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
		feature_extractor->setRadiusSearch (0.05);

		boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstruction; //TODO: remove this

		ICCVTutorial<pcl::FPFHSignature33> tutorial (keypoint_detector, feature_extractor, surface_reconstruction, cloud_1_fo, cloud_0_f);



		Eigen::Matrix4f iter_transform = tutorial.initial_transformation_matrix_;
		map_transform = map_transform * iter_transform; // Transforming all clouds into cloud_0 frame




	//------ CREATING MAP_CLOUD BY CONCATENATING (F)BLUE and (2)GREEN ------//
		// Transform cloud_1 into cloud_0 coordinates
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_trans (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::transformPointCloud(*cloud_1_fo, *cloud_1_trans, map_transform);
		// cloud_1_trans = tutorial.source_transformed_;

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
