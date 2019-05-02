#include "../include/functions.h"
/*

This is all commented out because I've defined everything in functions.h
https://ubuntuforums.org/showthread.php?t=1388438 --> Logic here (because its templated basically)



template<typename FeatureType>
MyFeatureMatcher<FeatureType>::MyFeatureMatcher(boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> >keypoint_detector,
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
*/