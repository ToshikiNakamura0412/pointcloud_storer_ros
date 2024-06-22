/**
 * @file pointcloud_storer.h
 * @author Toshiki Nakamura
 * @brief Store pointclouds and poses
 * @copyright Copyright (c) 2024
 */

#ifndef POINTCLOUD_STORER_POINTCLOUD_STORER_H
#define POINTCLOUD_STORER_POINTCLOUD_STORER_H

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <optional>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utility>
#include <vector>

#include "icp_matching/icp_matching.h"

/**
 * @class PointCloudStorer
 * @brief Class for storing pointclouds and poses
 */
class PointCloudStorer
{
public:
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef geometry_msgs::PoseWithCovarianceStamped PoseT;

  /**
   * @brief Construct a new Point Cloud Storer object
   */
  PointCloudStorer(void);

private:
  /**
   * @brief Callback function for pose
   * @param msg Pose message
   */
  void pose_callback(const PoseT::ConstPtr &msg);

  /**
   * @brief Callback function for pointcloud
   * @param msg Pointcloud message
   */
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);

  /**
   * @brief Store pose and pointcloud
   * @param cloud Pointcloud
   */
  void store_pose_cloud(const PointCloudT::Ptr &cloud);

  /**
   * @brief Publish stored pointcloud
   * @param header Header of message
   */
  void publish_stored_cloud(const std_msgs::Header &header);

  /**
   * @brief Publish graph node and edge
   * @param header Header of message
   */
  void publish_graph(const std_msgs::Header &header);

  int store_num_;
  float interval_;
  float leaf_size_;
  ICPMatching icp_matching_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher graph_edge_pub_;
  ros::Publisher graph_node_pub_;
  ros::Publisher stored_cloud_pub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber pose_sub_;

  std::optional<PoseT> pose_;
  std::vector<std::pair<PoseT, PointCloudT::Ptr>> pose_cloud_queue_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

#endif  // POINTCLOUD_STORER_POINTCLOUD_STORER_H
