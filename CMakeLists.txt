cmake_minimum_required(VERSION 3.0.2)
project(pcl_handler)


if(${CMAKE_VERSION} VERSION_LESS "3.1")
  add_compile_options(-std=c++14)
else()
  set(CMAKE_CXX_STANDARD 14)
endif()  
  
# find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(PCL REQUIRED COMPONENTS core io feature filters segmentation surface)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  velodyne_pcl
)

include_directories(
  SYSTEM
  # ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}


)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_handler
#  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_msgs roscpp sensor_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

add_executable(pointcloud_to_pcd src/pointcloud_to_pcd.cpp)
target_link_libraries(pointcloud_to_pcd ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pointcloud_concatenater src/pointcloud_concatenater.cpp)
target_link_libraries(pointcloud_concatenater ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(velodyne_filter src/velodyne_filter.cpp)
target_link_libraries(velodyne_filter ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})