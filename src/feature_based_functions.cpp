#include "../include/feature_based_functions.h"


void detectKeypoints (boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector_,
	                  typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, 
	                  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints)
{
  cout << "keypoint detection..." << std::flush;
  keypoint_detector_->setInputCloud(input);
  keypoint_detector_->compute(*keypoints);
  cout << "OK. keypoints found: " << keypoints->points.size() << endl;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



template<typename FeatureType>
void extractDescriptors (typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor_,
						 typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, 
						 typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, 
						 typename pcl::PointCloud<FeatureType>::Ptr features)
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