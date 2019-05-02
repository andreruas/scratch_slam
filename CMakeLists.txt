cmake_minimum_required(VERSION 2.8.3)
project(scratch_slam)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS sensor_msgs std_msgs tf roscpp rospy)
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



############################### OFFLINE CODE ###############################

## Add executable
add_executable (my_feature_based_v1 src/my_feature_based_v1.cpp)
add_library(feature_based_functions src/feature_based_functions.cpp)
target_link_libraries (my_feature_based_v1 feature_based_functions ${PCL_LIBRARIES})