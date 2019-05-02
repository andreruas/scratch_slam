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
class MyFeatureMatcher
{
  public:
    MyFeatureMatcher (boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector,
                  typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor,
                  boost::shared_ptr<pcl::PCLSurfaceBase<pcl::PointXYZRGBNormal> > surface_reconstructor,
                  typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source,
                  typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target);


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
    bool show_source2target_;
    bool show_target2source_;
    bool show_correspondences;

    /**
     * @brief starts the event loop for the visualizer
     */
    void run ();
  protected:
    void detectKeypoints (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const;

    void extractDescriptors (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, typename pcl::PointCloud<FeatureType>::Ptr features);

    void findCorrespondences (typename pcl::PointCloud<FeatureType>::Ptr source, typename pcl::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const;

    void filterCorrespondences ();

    void determineInitialTransformation ();
};
