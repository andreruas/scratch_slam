cmake_minimum_required(VERSION 2.8.3)
project(scratch_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs std_msgs tf roscpp rospy)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL 1.7.2 REQUIRED COMPONENTS)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS sensor_msgs std_msgs tf roscpp rospy
   #DEPENDS eigen opencv
   )


## Add executable for ICP
add_executable (my_feature_based_v0 src/my_feature_based_v0.cpp)
add_library(functions src/functions.cpp)
target_link_libraries (my_feature_based_v0 functions ${PCL_LIBRARIES})



# ############################### OFFLINE CODE ###############################

# ## Add executable for ICP
# add_executable (my_ICP_v3 src/my_ICP_v3.cpp)
# target_link_libraries (my_ICP_v3 ${PCL_LIBRARIES})

# ## Add executable
# add_executable (my_feature_based_v0 src/my_feature_based_v0.cpp)
# target_link_libraries (my_feature_based_v0 ${PCL_LIBRARIES})


# ############################### ONLINE SLAM ###############################

# ## Add executable
# add_executable (online_ICP_slam_v0 src/online_ICP_slam_v0.cpp)
# target_link_libraries (online_ICP_slam_v0 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# ## Add executable
# add_executable (online_feature_based_slam_v0 src/online_feature_based_slam_v0.cpp)
# target_link_libraries (online_feature_based_slam_v0 ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# ############################### UTILS ###############################

# ## Add executable for pointcloud_proc
# add_executable(pointcloud_proc utils/pointcloud_proc.cpp)
# target_link_libraries(pointcloud_proc ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# ############################### DEPRECATED CODE ###############################

# ## To reduce build time, I don't build executables for deprecated code. 
