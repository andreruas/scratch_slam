# scratch_slam

Andre Ruas - Scratch SLAM

This project was started to build a Dense, 3D SLAM solution from "scratch" using the Point Cloud Library (PCL), OpenCV, and Eigen as a learning exercise.

# Offline Methods
Offline methods read pointclouds from a folder of .pcd files, and incrementally register them to create a global map. These scripts use the same methods as the online methods, but run more slowly, and generate higher-quality, better looking maps. These offline methods also save the final map as .pcd and .ply files, which can be visualized using '$pcl_viewer my_map.pcd'. 

**offline_ICP_SLAM**
Uses Iterative Closest Point algorithm to estimate visual odometry from pairs of clouds and incrementally register them to create a global map. Requires that pointclouds are already 
Run this using: '$./offline_ICP_SLAM <full_path_to_folder> <start_pcd> <start_pcd> <step>' for example '$./offline_ICP_SLAM /home/user/clouds 20 130 2'


**offline_Feature_Based_SLAM**
Uses Feature-Based methods to estimate visual odometry from pairs of clouds and incrementally register them to create a global map.
Run this using: '$./offline_Feature_Based_SLAM <full_path_to_folder> <start_pcd> <start_pcd> <step>' for example '$./offline_Feature_Based_SLAM /home/user/clouds 20 130 2'



# Online Methods
Online methods build a map 'online' using pointclouds from a RGBD camera in real-time, by subscribing to a pointcloud on the "/camera/depth_registered/points" topic and publishing a global 3D pointcloud map to the "/Map" topic.

**online_ICP_SLAM**
Uses Iterative Closest Point algorithm to estimate visual odometry from pairs of clouds and incrementally register them to create a global map.

**online_Feature_Based_SLAM**
Uses Feature-Based methods to estimate visual odometry from pairs of clouds and incrementally register them to create a global map.


# To be added
- Add Instructions for generating a folder of '.pcd' files from a rosbag or Microsoft Kinect (for offline processing)
- Working version of online_Feature_Based_SLAM [DONE]
- Add initial guess for ICP / Feature Based methods using motion model
- Add more noise removal