/**
 * @file pointcloud_storer.cpp
 * @author Toshiki Nakamura
 * @brief Store pointclouds and poses
 * @copyright Copyright (c) 2024
 */

#include <utility>

#include "pointcloud_storer/pointcloud_storer.h"

PointCloudStorer::PointCloudStorer(void) : private_nh_("~"), tf_listener_(tf_buffer_)
{
  private_nh_.param<int>("store_num", store_num_, 3);
  private_nh_.param<float>("interval", interval_, 0.5);
  private_nh_.param<float>("leaf_size", leaf_size_, 0.05);

  graph_edge_pub_ = nh_.advertise<nav_msgs::Path>("graph/edge", 1);
  graph_node_pub_ = nh_.advertise<geometry_msgs::PoseArray>("graph/node", 1);
  stored_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("stored_cloud", 1);
  cloud_sub_ = nh_.subscribe("/cloud", 1, &PointCloudStorer::cloud_callback, this, ros::TransportHints().tcpNoDelay());
  pose_sub_ =
      nh_.subscribe("/robot_pose", 1, &PointCloudStorer::pose_callback, this, ros::TransportHints().tcpNoDelay());

  ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
  ROS_INFO_STREAM("store_num: " << store_num_);
  ROS_INFO_STREAM("interval: " << interval_);
  ROS_INFO_STREAM("leaf_size: " << leaf_size_);
}

void PointCloudStorer::pose_callback(const PoseT::ConstPtr &msg) { pose_ = *msg; }

void PointCloudStorer::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (!pose_.has_value())
  {
    ROS_WARN_THROTTLE(1, "pose is not received yet");
    return;
  }

  // transform pointcloud to pose frame
  geometry_msgs::TransformStamped transform_stamped;
  while (ros::ok())
  {
    try
    {
      transform_stamped = tf_buffer_.lookupTransform(pose_.value().header.frame_id, msg->header.frame_id, ros::Time(0));
      break;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(0.5).sleep();
    }
  }
  sensor_msgs::PointCloud2 cloud_transformed;
  Eigen::Matrix4f transform = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(transform, *msg, cloud_transformed);

  // voxel grid filter
  PointCloudT::Ptr pcl_cloud_transformed(new PointCloudT);
  pcl::fromROSMsg(cloud_transformed, *pcl_cloud_transformed);
  icp_matching_.voxel_grid_filter(pcl_cloud_transformed, leaf_size_, pcl_cloud_transformed);

  // store pose & cloud
  store_pose_cloud(pcl_cloud_transformed);

  // publish
  std_msgs::Header header = pose_.value().header;
  header.stamp = ros::Time::now();
  publish_stored_cloud(header);
  publish_graph(header);
}

void PointCloudStorer::store_pose_cloud(const PointCloudT::Ptr &cloud)
{
  // store initial pose
  if (pose_.has_value() && pose_cloud_queue_.empty())
  {
    pose_cloud_queue_.push_back(std::make_pair(pose_.value(), cloud));
    return;
  }

  // check interval
  const float dx = pose_.value().pose.pose.position.x - pose_cloud_queue_.back().first.pose.pose.position.x;
  const float dy = pose_.value().pose.pose.position.y - pose_cloud_queue_.back().first.pose.pose.position.y;
  if (hypot(dx, dy) < interval_)
    return;

  // update queue
  pose_cloud_queue_.push_back(std::make_pair(pose_.value(), cloud));
  if (pose_cloud_queue_.size() > store_num_)
    pose_cloud_queue_.erase(pose_cloud_queue_.begin());

  // Registoration
  for (int i = pose_cloud_queue_.size() - 2; 0 < i; i--)
  {
    icp_matching_.align(
        pose_cloud_queue_[i - 1].second, pose_cloud_queue_[i].second, *pose_cloud_queue_[i - 1].second, 50,
        leaf_size_ * 2.0);
    const Eigen::Affine3d affine(icp_matching_.get_transformation().cast<double>());
    tf2::doTransform(pose_cloud_queue_[i - 1].first, pose_cloud_queue_[i - 1].first, tf2::eigenToTransform(affine));
  }
}

void PointCloudStorer::publish_stored_cloud(const std_msgs::Header &header)
{
  // concatenate pointclouds
  PointCloudT::Ptr pcl_cloud_stored(new PointCloudT);
  for (const auto &pose_cloud : pose_cloud_queue_)
    *pcl_cloud_stored += *pose_cloud.second;

  // convert to sensor_msgs::PointCloud2
  sensor_msgs::PointCloud2 cloud_stored;
  pcl::toROSMsg(*pcl_cloud_stored, cloud_stored);
  cloud_stored.header = header;
  stored_cloud_pub_.publish(cloud_stored);
}

void PointCloudStorer::publish_graph(const std_msgs::Header &header)
{
  geometry_msgs::PoseArray node;
  nav_msgs::Path edge;

  for (const auto &pose_cloud : pose_cloud_queue_)
  {
    // node
    geometry_msgs::Pose pose = pose_cloud.first.pose.pose;
    node.poses.push_back(pose);
    // edge
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header = header;
    edge.poses.push_back(pose_stamped);
  }

  node.header = header;
  edge.header = header;

  graph_node_pub_.publish(node);
  graph_edge_pub_.publish(edge);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_storer");
  PointCloudStorer pointcloud_storer;
  ros::spin();

  return 0;
}
