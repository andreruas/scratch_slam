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

# Add library for my functions
add_library(functions src/functions.cpp)


# ############################### OFFLINE CODE ###############################

## Add executable for ICP
add_executable (offline_ICP_SLAM src/offline_ICP_SLAM.cpp)
target_link_libraries (offline_ICP_SLAM ${PCL_LIBRARIES})

# ## Add executable
add_executable (offline_Feature_Based_SLAM src/offline_Feature_Based_SLAM.cpp)
target_link_libraries (offline_Feature_Based_SLAM functions ${PCL_LIBRARIES})


# ############################### ONLINE SLAM ###############################

# ## Add executable
add_executable (online_ICP_SLAM src/online_ICP_SLAM.cpp)
target_link_libraries (online_ICP_SLAM ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Add executable
add_executable (online_Feature_Based_SLAM src/online_Feature_Based_SLAM.cpp)
target_link_libraries (online_Feature_Based_SLAM functions ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# ############################### UTILS ###############################

# ## Add executable for pointcloud_proc
add_executable(pointcloud_proc utils/pointcloud_proc.cpp)
target_link_libraries(pointcloud_proc ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# ############################### DEPRECATED CODE ###############################

# ## To reduce build time, I don't build executables for deprecated code. 
