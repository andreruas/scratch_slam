# scratch_slam

Andre Ruas - Scratch SLAM

This project was started to build a Dense, 3D SLAM solution from "scratch" using the Point Cloud Library (PCL), OpenCV, and Eigen as a learning exercise.

# Offline Methods
Offline methods read pointclouds from a folder of .pcd files, and incrementally register them to create a global map. These scripts use the same methods as the online methods, but run more slowly, and generate higher-quality, better looking maps. These offline methods also save the final map as .pcd and .ply files, which can be visualized using '$pcl_viewer my_map.pcd'. 

**my_ICP_v3**
Uses Iterative Closest Point algorithm to estimate visual odometry from pairs of clouds and incrementally register them to create a global map. Requires that pointclouds are already 
Run this using: '$./my_ICP_v3 <start_pcd> <start_pcd> <step>' for example '$./my_ICP_v3 20 130 2'




# Online Methods
Online methods build a map 'online' using pointclouds from a RGBD camera in real-time, by subscribing to a pointcloud on the "/camera/depth_registered/points" topic and publishing a global 3D pointcloud map to the "/Map" topic.

**online_ICP_slam_v0**
Uses Iterative Closest Point algorithm to estimate visual odometry from pairs of clouds and incrementally register them to create a global map.

**online_feature_based_slam_v0**
Uses Feature-Based methods to estimate visual odometry from pairs of clouds and incrementally register them to create a global map.


# To be added
- Instructions for generating a folder of '.pcd' files from a rosbag or Microsoft Kinect (for offline processing)
- Working version of online_feature_based_slam_v0
- In general clean up repository and write documentation