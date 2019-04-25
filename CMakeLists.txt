cmake_minimum_required(VERSION 2.8)
project(stereo_image_proc)

find_package(catkin REQUIRED cv_bridge dynamic_reconfigure image_geometry image_proc image_transport message_filters nodelet sensor_msgs stereo_msgs roscpp rospy std_msgs)
find_package(Boost REQUIRED COMPONENTS thread)
find_package( PCL 1.7.2 REQUIRED )

if(cv_bridge_VERSION VERSION_GREATER "1.12.0")
  add_compile_options(-std=c++11)
endif()

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/Disparity.cfg)

catkin_package(
  CATKIN_DEPENDS image_geometry image_proc sensor_msgs stereo_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include)

find_package(OpenCV REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# See note in image_proc/CMakeLists.txt
add_definitions(-DOPENCV_TRAITS_ENABLE_DEPRECATED)
add_definitions(${PCL_DEFINITIONS})


# Nodelet library
add_library(${PROJECT_NAME} src/libstereo_image_proc/processor.cpp src/nodelets/disparity.cpp src/nodelets/point_cloud2.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
)

link_directories(${PCL_LIBRARY_DIRS})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Standalone node
add_executable(stereoimageproc_exe src/nodes/stereo_image_proc.cpp)
target_link_libraries(stereoimageproc_exe stereo_image_proc)
SET_TARGET_PROPERTIES(stereoimageproc_exe PROPERTIES OUTPUT_NAME stereo_image_proc)
install(TARGETS stereoimageproc_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install the launch file
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

# install the include directory
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Add executable for pointcloud_proc
add_executable(pointcloud_proc src/nodes/pointcloud_proc.cpp)
target_link_libraries(pointcloud_proc ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Add executable for ICP
add_executable (pairwise_incremental_registration pairwise_incremental_registration.cpp)
target_link_libraries (pairwise_incremental_registration ${PCL_LIBRARIES})

## Add executable for ICP
add_executable (my_ICP my_ICP.cpp)
target_link_libraries (my_ICP ${PCL_LIBRARIES})

## Add executable for ICP
add_executable (my_ICP_v2 my_ICP_v2.cpp)
target_link_libraries (my_ICP_v2 ${PCL_LIBRARIES})

## Add executable for ICP
add_executable (my_ICP_v3 my_ICP_v3.cpp)
target_link_libraries (my_ICP_v3 ${PCL_LIBRARIES})

## Add executable for ICP
add_executable (icp_slam icp_slam.cpp)
target_link_libraries (icp_slam ${PCL_LIBRARIES})

## Add executable for ICP
add_executable (icp_slam_v2 icp_slam_v2.cpp)
target_link_libraries (icp_slam_v2 ${PCL_LIBRARIES})

## Add executable for ICP
add_executable (icp_slam_v3 icp_slam_v3.cpp)
target_link_libraries (icp_slam_v3 ${PCL_LIBRARIES})

## Add executable for ICP
add_executable (feature_based_slam_v0 feature_based_slam_v0.cpp)
target_link_libraries (feature_based_slam_v0 ${PCL_LIBRARIES})

## Add executable for ICP
add_executable (feature_based_slam_v1 feature_based_slam_v1.cpp)
target_link_libraries (feature_based_slam_v1 ${PCL_LIBRARIES})

## Add executable
add_executable (feature_matching feature_matching.cpp)
target_link_libraries (feature_matching ${PCL_LIBRARIES})

## Add executable
add_executable (my_feature_based_v0 my_feature_based_v0.cpp)
target_link_libraries (my_feature_based_v0 ${PCL_LIBRARIES})

## Add executable
add_executable (online_ICP_slam_v0 online_ICP_slam_v0.cpp)
target_link_libraries (online_ICP_slam_v0 ${catkin_LIBRARIES} ${PCL_LIBRARIES})