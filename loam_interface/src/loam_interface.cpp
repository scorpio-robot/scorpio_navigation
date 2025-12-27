// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "loam_interface/loam_interface.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace loam_interface
{

LoamInterfaceNode::LoamInterfaceNode(const rclcpp::NodeOptions & options)
: Node("loam_interface", options)
{
  this->declare_parameter<std::string>("state_estimation_topic", "pslam/imu_odom");
  this->declare_parameter<std::string>("registered_scan_topic", "pslam/aligned_scan_cloud");
  this->declare_parameter<std::string>("map_cloud_topic", "pslam/lio_map_cloud");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("lidar_frame", "mid360");
  this->declare_parameter<std::string>("robot_base_frame", "base_link");

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("map_cloud_topic", map_cloud_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  base_frame_to_lidar_initialized_ = false;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
  sensor_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 5);
  map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 5);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    registered_scan_topic_, 5,
    std::bind(&LoamInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
  map_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    map_cloud_topic_, rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&LoamInterfaceNode::mapCloudCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    state_estimation_topic_, 5,
    std::bind(&LoamInterfaceNode::odometryCallback, this, std::placeholders::_1));
}

void LoamInterfaceNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Get transform at point cloud timestamp for time synchronization
  tf2::Transform tf_odom_to_lidar_odom_sync;
  tf2::Transform tf_lidar_odom_to_lidar_sync;

  if (!getTransformAtTime(
        msg->header.stamp, tf_odom_to_lidar_odom_sync, tf_lidar_odom_to_lidar_sync)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "No transform available for point cloud timestamp, using latest");
    tf_odom_to_lidar_odom_sync = tf_odom_to_lidar_odom_;
    tf_lidar_odom_to_lidar_sync = tf_lidar_odom_to_lidar_;
  }

  // Transform to odom_frame for registered_scan
  auto registered_scan = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_lidar_odom_sync, *msg, *registered_scan);
  pcd_pub_->publish(*registered_scan);

  // Transform to lidar_frame for sensor_scan
  auto sensor_scan = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl_ros::transformPointCloud(
    lidar_frame_, tf_lidar_odom_to_lidar_sync.inverse(), *msg, *sensor_scan);
  sensor_scan_pub_->publish(*sensor_scan);
}

void LoamInterfaceNode::mapCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Transform map cloud from input frame to odom_frame
  auto map_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_lidar_odom_, *msg, *map_cloud);

  map_cloud_pub_->publish(*map_cloud);
}

void LoamInterfaceNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // NOTE: Input odometry message is based on the lidar's odometry frame
  // Here we transform it to the odom frame

  // Initialize the transformation from base_frame to lidar_frame
  if (!base_frame_to_lidar_initialized_) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
      tf2::Transform tf_base_frame_to_lidar;
      tf2::fromMsg(tf_stamped.transform, tf_base_frame_to_lidar);

      // Keep only yaw, zero out roll and pitch
      double roll, pitch, yaw;
      tf2::Matrix3x3(tf_base_frame_to_lidar.getRotation()).getRPY(roll, pitch, yaw);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      tf_odom_to_lidar_odom_.setOrigin(tf_base_frame_to_lidar.getOrigin());
      tf_odom_to_lidar_odom_.setRotation(q);

      base_frame_to_lidar_initialized_ = true;
      RCLCPP_INFO(
        this->get_logger(), "Successfully initialized base_frame to lidar_frame transform");
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      return;
    }
  }

  // Transform the odometry_msg (based on lidar_odom) to the odom frame
  tf2::fromMsg(msg->pose.pose, tf_lidar_odom_to_lidar_);
  tf2::Transform tf_odom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar_;

  // Cache transform for time-synchronized point cloud processing
  {
    std::lock_guard<std::mutex> lock(transform_mutex_);
    TransformStamped ts;
    ts.stamp = msg->header.stamp;
    ts.tf_odom_to_lidar_odom = tf_odom_to_lidar_odom_;
    ts.tf_lidar_odom_to_lidar = tf_lidar_odom_to_lidar_;
    transform_history_.push_back(ts);

    // Keep history size limited
    if (transform_history_.size() > MAX_HISTORY_SIZE) {
      transform_history_.pop_front();
    }
  }

  // Publish transformed odometry
  nav_msgs::msg::Odometry out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = lidar_frame_;

  const auto & origin = tf_odom_to_lidar.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(tf_odom_to_lidar.getRotation());

  odom_pub_->publish(out);

  // Publish TF: odom_frame -> base_frame
  // Calculate transform from odom to base_frame
  tf2::Transform tf_lidar_to_base = getTransform(lidar_frame_, base_frame_, msg->header.stamp);
  tf2::Transform tf_odom_to_base = tf_odom_to_lidar * tf_lidar_to_base;

  publishTransform(tf_odom_to_base, odom_frame_, base_frame_, msg->header.stamp);

  // auto print_tf = [this](const std::string & name, const tf2::Transform & tf) {
  //   double roll, pitch, yaw;
  //   tf2::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);
  //   const auto & o = tf.getOrigin();
  //   RCLCPP_INFO(
  //     this->get_logger(), "%s: x=%.2f y=%.2f z=%.2f roll=%.2f pitch=%.2f yaw=%.2f", name.c_str(),
  //     o.x(), o.y(), o.z(), roll, pitch, yaw);
  // };

  // print_tf("tf_odom_to_base", tf_odom_to_base);
  // print_tf("tf_odom_to_lidar", tf_odom_to_lidar);
  // print_tf("tf_lidar_to_base", tf_lidar_to_base);
  // print_tf("tf_odom_to_lidar_odom_", tf_odom_to_lidar_odom_);
  // print_tf("tf_lidar_odom_to_lidar_", tf_lidar_odom_to_lidar_);
  // RCLCPP_INFO(this->get_logger(), "--------");
}

tf2::Transform LoamInterfaceNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.5));
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    return transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Returning identity.", ex.what());
    return tf2::Transform::getIdentity();
  }
}

void LoamInterfaceNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  tf_broadcaster_->sendTransform(transform_msg);
}

bool LoamInterfaceNode::getTransformAtTime(
  const rclcpp::Time & target_time, tf2::Transform & tf_odom_to_lidar_odom_out,
  tf2::Transform & tf_lidar_odom_to_lidar_out)
{
  std::lock_guard<std::mutex> lock(transform_mutex_);

  if (transform_history_.empty()) {
    return false;
  }

  // Find closest transform by timestamp
  auto closest_it = transform_history_.begin();
  double min_time_diff = std::abs((target_time - closest_it->stamp).seconds());

  for (auto it = transform_history_.begin(); it != transform_history_.end(); ++it) {
    double time_diff = std::abs((target_time - it->stamp).seconds());
    if (time_diff < min_time_diff) {
      min_time_diff = time_diff;
      closest_it = it;
    }
  }

  // Check if time difference is acceptable
  if (min_time_diff > MAX_TIME_DIFF_SEC) {
    RCLCPP_WARN(
      this->get_logger(), "Time difference too large: %.3f sec (threshold: %.3f sec)",
      min_time_diff, MAX_TIME_DIFF_SEC);
    return false;
  }

  tf_odom_to_lidar_odom_out = closest_it->tf_odom_to_lidar_odom;
  tf_lidar_odom_to_lidar_out = closest_it->tf_lidar_odom_to_lidar;
  return true;
}

}  // namespace loam_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(loam_interface::LoamInterfaceNode)
