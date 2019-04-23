/*
40 - 0
43   1
47   2
50 - 3

60   4
70 - 5

80 - 6
85 - 7
90 - 8

95 - 135  - (every 5) - [9-17]

./icp_slam_v3 "/home/andre/Kinect_Subset/" 0 17 1
./icp_slam_v3 0 17 1
pcl_viewer EuRoC_Pointcloud_70.pcd EuRoC_Pointcloud_80.pcd 
pcl_viewer [1-8].pcd
pcl_viewer 0.pcd 1.pcd 2.pcd 3.pcd 4.pcd 5.pcd 6.pcd 7.pcd 8.pcd 9.pcd 10.pcd 11.pcd 12.pcd 13.pcd 14.pcd 15.pcd 16.pcd 17.pcd 

./icp_slam Subset2 0-4 works pretty well, see image 'align1' on Desktop

./icp_slam_v3 "/home/andre/Kinect_Pointclouds_room2/" 30 60 2 --> Works pretty well
./icp_slam_v3 "/home/andre/Kinect_Pointclouds_room2/" 20 130 2 --> Works pretty well
./icp_slam_v3 "/home/andre/Kinect_Pointclouds_room2/" 20 130 10 --> Doesn't work at all


./icp_slam_v3 "/home/andre/sim_test_0/" 80 120 3 -->???
*/


#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <iostream>
#include <ctime>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

// This is a tutorial so we can afford having global variables 
//our visualizer
pcl::visualization::PCLVisualizer *p;
//its left and right viewports
int vp_1, vp_2;

//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{

  std::string path = argv[1]; // TODO: Add warning if three arguments aren't used
  int start_frame = atoi(argv[2]);
  int end_frame = atoi(argv[3]);
  int step = atoi(argv[4]);

  // int start_frame = atoi(argv[1]);
  // int end_frame = atoi(argv[2]);
  // int step = atoi(argv[3]);

  for (int i = start_frame; i <= end_frame; i+=step)
  {
  	std::string filename = path + "EuRoC_Pointcloud_" + std::to_string(i) + ".pcd";
  	// std::string filename = "/home/andre/Kinect_Pointclouds/EuRoC_Pointcloud_" + std::to_string(i) + ".pcd";

  	std::cout << "--> " << filename << std::endl; 

    // Load the cloud and saves it into the global list of models
    PCD m;
    m.f_name = filename;
    pcl::io::loadPCDFile (filename, *m.cloud);
    //remove NAN points from the cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);
    models.push_back(m);
  }
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    // grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setLeafSize (0.1, 0.1, 0.1);

    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  // // Outlier removal
  // pcl::StatisticalOutlierRemoval<PointT> f;
  // f.setMeanK(50);
  // f.setStddevMulThresh(1.0);

  // f.setInputCloud(src);
  // f.filter(*src);

  // f.setInputCloud(tgt);
  // f.filter(*tgt);

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);

  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

    //accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

    //if the difference between this transformation and the previous one
    //is smaller than the threshold, refine the process by reducing
    //the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

    prev = reg.getLastIncrementalTransformation ();

    // visualize current state
    // showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  //add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;
 }


/* ---[ */
int main (int argc, char** argv)
{
  int start_time = clock();

  // Load data
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  loadData (argc, argv, data);

  PCL_INFO ("Loaded %d datasets. \n", (int)data.size ());

  // Create a PCLVisualizer object
  // p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  // p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  // p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
  PointCloud::Ptr CombinedMap (new PointCloud);

  PointCloud::Ptr result (new PointCloud), source, target;
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

  for (size_t i = 1; i < data.size (); ++i)
  {
    source = data[i-1].cloud;
    target = data[i].cloud;

    // Add visualization data
    // showCloudsLeft(source, target);

    PointCloud::Ptr temp (new PointCloud);
    PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
    pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform
    pcl::transformPointCloud (*temp, *result, GlobalTransform);

    //update the global transform
    GlobalTransform = GlobalTransform * pairTransform;

	//save aligned pair, transformed into the first cloud's frame
    // std::stringstream ss;
    // ss << i << ".pcd";
    // pcl::io::savePCDFile (ss.str (), *result, true);
    *CombinedMap = *CombinedMap + *result;

  }

  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize (0.01, 0.01, 0.01);
  grid.setInputCloud (CombinedMap);
  grid.filter (*CombinedMap);

  pcl::io::savePCDFile("CombinedMap.pcd", *CombinedMap, true);
  std::cout << "This script took " << (clock()-start_time)/CLOCKS_PER_SEC << " seconds to run." << std::endl;
}
/* ]--- */
