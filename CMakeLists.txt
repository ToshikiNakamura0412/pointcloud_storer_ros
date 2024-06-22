cmake_minimum_required(VERSION 3.8)
project(pointcloud_storer_ros)

add_compile_options(-std=c++17 -O2 -g)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf2_eigen
  tf2_geometry_msgs
  icp_matching_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_storer_node src/pointcloud_storer.cpp)
target_link_libraries(pointcloud_storer_node ${catkin_LIBRARIES})

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()
