#include <iostream>
#include <ctime>
#include <stdlib.h>
#include <vector>
#include <string>
#include <sstream>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/ply_io.h>
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

// #include <pcl/visualization/cloud_viewer.h>
//#include <pcl/filters/uniform_sampling.h>

/*
Code adapted from PCL github:
github.com/PointCloudLibrary/pcl/blob/master/apps/src/feature_matching.cpp
*/

using namespace std;


// ADJUSTABLE PARAMETERS
float FeatureRadiusSearch = 0.01; // 0.01 is a good value


template<typename FeatureType>
class MyFeatureMatcher
{
  public:
    MyFeatureMatcher (boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector,
                  typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor,
                  boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstructor,
                  typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source,
                  typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target,
                  float rej_thresh);


    // TODO: This all really shouldn't be public, it's sloppy
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
    float rej_thresh_; 

    /**
     * @brief starts the event loop for the visualizer
     */
    void run ();
  protected:
    void detectKeypoints (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const;

    void extractDescriptors (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, typename pcl::PointCloud<FeatureType>::Ptr features);

    void findCorrespondences (typename pcl::PointCloud<FeatureType>::Ptr source, typename pcl::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const;

    void filterCorrespondences ();

    void calculateTransform ();
};

template<typename FeatureType>
MyFeatureMatcher<FeatureType>::MyFeatureMatcher(boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> >keypoint_detector,
                                        typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor,
                                        boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstructor,
                                        typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source,
                                        typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target,
                                        float rej_thresh)
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
, rej_thresh_ (rej_thresh)
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

  detectKeypoints (source_segmented_, source_keypoints_);
  detectKeypoints (target_segmented_, target_keypoints_);

  extractDescriptors (source_segmented_, source_keypoints_, source_features_);
  extractDescriptors (target_segmented_, target_keypoints_, target_features_);

  findCorrespondences (source_features_, target_features_, source2target_);
  findCorrespondences (target_features_, source_features_, target2source_);

  filterCorrespondences ();

  calculateTransform ();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename FeatureType>
void MyFeatureMatcher<FeatureType>::detectKeypoints (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const
{
  cout << "keypoint detection..." << std::flush;
  keypoint_detector_->setInputCloud(input);
  keypoint_detector_->compute(*keypoints);
  cout << "OK. keypoints found: " << keypoints->points.size() << endl;
}




template<typename FeatureType>
void MyFeatureMatcher<FeatureType>::extractDescriptors (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, typename pcl::PointCloud<FeatureType>::Ptr features)
{
  typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGB>);
  kpts->points.resize(keypoints->points.size());

  pcl::copyPointCloud(*keypoints, *kpts);

  typename pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, FeatureType>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, FeatureType> > (feature_extractor_);

  feature_extractor_->setSearchSurface(input);
  feature_extractor_->setInputCloud(kpts);

  if (feature_from_normals)
  {
    cout << "normal estimation..." << std::flush;
    typename pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    normal_estimation.setRadiusSearch(FeatureRadiusSearch);
    normal_estimation.setInputCloud(input);
    normal_estimation.compute(*normals);
    feature_from_normals->setInputNormals(normals);
    cout << "OK" << endl;
  }

  cout << "descriptor extraction..." << std::flush;
  feature_extractor_->compute (*features);
  cout << "OK" << endl;
}





template<typename FeatureType>
void MyFeatureMatcher<FeatureType>::findCorrespondences (typename pcl::PointCloud<FeatureType>::Ptr source, typename pcl::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const
{
  cout << "correspondence assignment..." << std::flush;
  correspondences.resize (source->size());

  // Using a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<FeatureType> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target);

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (int i = 0; i < static_cast<int> (source->size ()); ++i)
  {
    descriptor_kdtree.nearestKSearch(*source, i, k, k_indices, k_squared_distances);
    correspondences[i] = k_indices[0];
  }
  cout << "OK" << endl;
}


template<typename FeatureType>
void MyFeatureMatcher<FeatureType>::filterCorrespondences ()
{
  // First check if the correspondences are calculated as the same index in both clouds
  // THIS MAY BE TOO AGGRESSIVE

  /*
  std::vector<int> source2target_;
  std::vector<int> target2source_;
  */

  cout << "correspondence rejection..." << std::flush;
  std::vector<std::pair<unsigned, unsigned> > correspondences;
  for (size_t cIdx = 0; cIdx < source2target_.size(); ++cIdx)
    if (target2source_[source2target_[cIdx]] == static_cast<int> (cIdx))
      correspondences.push_back(std::make_pair(cIdx, source2target_[cIdx]));

  correspondences_->resize (correspondences.size());
  for (size_t cIdx = 0; cIdx < correspondences.size(); ++cIdx)
  {
    (*correspondences_)[cIdx].index_query = correspondences[cIdx].first;
    (*correspondences_)[cIdx].index_match = correspondences[cIdx].second;
  }

  cout << "OK. Correspondences found (1): " << correspondences.size() << endl;


  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
  rejector.setInputSource (source_keypoints_);
  rejector.setInputTarget (target_keypoints_);
  rejector.setInputCorrespondences(correspondences_);


  // Set the maximum distance between corresponding points. (Reduce this number to decrease number of matches)
  // rejector.setInlierThreshold(0.15); // Default is 0.05, but I've used 0.2 as well
  rejector.setInlierThreshold(rej_thresh_); // Default is 0.05, but I've used 0.2 as well

  rejector.getCorrespondences(*correspondences_);

  cout << "OK. Correspondences found (2): " << correspondences_->size() << endl;
}




template<typename FeatureType>
void MyFeatureMatcher<FeatureType>::calculateTransform()
{
  cout << "Aligning Pointclouds..." << std::flush;
  pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);

  transformation_estimation->estimateRigidTransformation (*source_keypoints_, *target_keypoints_, *correspondences_, initial_transformation_matrix_);

  pcl::transformPointCloud(*source_segmented_, *source_transformed_, initial_transformation_matrix_);
  // pcl::io::savePCDFileASCII ("Features_Only_Registration_Output.pcd", *source_transformed_ + *target_segmented_);
  cout << "OK" << endl;
}