cmake_minimum_required(VERSION 2.8.3)
project(octomap_buggy_demo)

find_package(catkin REQUIRED
	geometry_msgs
  roscpp
  sensor_msgs
  laser_geometry
  tf
  pcl_ros)

find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)

include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${EIGEN_LIBRARY_DIRS})

catkin_package()

add_executable(scan_to_point_cloud_tf src/scan_to_point_cloud_tf.cpp)

target_link_libraries(scan_to_point_cloud_tf
  ${catkin_LIBRARIES}
)
