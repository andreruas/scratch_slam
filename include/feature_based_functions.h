#include <iostream>
#include <ctime>
#include <stdlib.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/ply_io.h>
//#include <pcl/filters/uniform_sampling.h>

// #include <my_feature_based_methods.cpp> // TODO: Separate this into a functions file

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

using namespace std;

/*
detectKeypoints
extractDescriptors
findCorrespondences
FilterCorrespondences
determineInitialTransform
*/


void detectKeypoints (boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector_,
					  typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, 
					  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints);


template<typename FeatureType>
void extractDescriptors (typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor_,
						 typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, 
						 typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, 
						 typename pcl::PointCloud<FeatureType>::Ptr features);


