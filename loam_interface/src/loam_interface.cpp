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
  this->declare_parameter<std::string>("loam_odometry_topic", "pslam/imu_odom");
  this->declare_parameter<std::string>("registered_scan_topic", "pslam/aligned_scan_cloud");
  this->declare_parameter<std::string>("map_cloud_topic", "pslam/lio_map_cloud");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("lidar_frame", "mid360");
  this->declare_parameter<std::string>("robot_base_frame", "base_link");

  this->get_parameter("loam_odometry_topic", loam_odometry_topic_);
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
  base_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("state_estimation", 5);
  lidar_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    registered_scan_topic_, 5,
    std::bind(&LoamInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
  map_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    map_cloud_topic_, rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&LoamInterfaceNode::mapCloudCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    loam_odometry_topic_, 5,
    std::bind(&LoamInterfaceNode::odometryCallback, this, std::placeholders::_1));
}

void LoamInterfaceNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Transform to odom_frame for registered_scan
  auto registered_scan = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_lidar_odom_, *msg, *registered_scan);
  pcd_pub_->publish(*registered_scan);

  // Transform to lidar_frame for sensor_scan
  auto sensor_scan = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl_ros::transformPointCloud(lidar_frame_, tf_lidar_odom_to_lidar_.inverse(), *msg, *sensor_scan);
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
  publishOdometry(
    tf_odom_to_lidar, odom_frame_, lidar_frame_, msg->header.stamp, lidar_odometry_pub_);

  // Calculate transform from odom to base_frame
  tf2::Transform tf_lidar_to_base = getTransform(lidar_frame_, base_frame_, msg->header.stamp);
  tf2::Transform tf_odom_to_base = tf_odom_to_lidar * tf_lidar_to_base;

  publishTransform(tf_odom_to_base, odom_frame_, base_frame_, msg->header.stamp);
  publishOdometry(
    tf_odom_to_base, odom_frame_, robot_base_frame_, msg->header.stamp, base_odometry_pub_);
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

void LoamInterfaceNode::publishOdometry(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp,
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  static tf2::Transform previous_transform;
  static auto previous_time = std::chrono::steady_clock::now();
  const auto current_time = std::chrono::steady_clock::now();

  const double dt =
    std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - previous_time).count() *
    1e-9;

  if (dt > 0) {
    const auto linear_velocity = (transform.getOrigin() - previous_transform.getOrigin()) / dt;

    const tf2::Quaternion q_diff =
      transform.getRotation() * previous_transform.getRotation().inverse();
    const auto angular_velocity = q_diff.getAxis() * q_diff.getAngle() / dt;

    out.twist.twist.linear.x = linear_velocity.x();
    out.twist.twist.linear.y = linear_velocity.y();
    out.twist.twist.linear.z = linear_velocity.z();
    out.twist.twist.angular.x = angular_velocity.x();
    out.twist.twist.angular.y = angular_velocity.y();
    out.twist.twist.angular.z = angular_velocity.z();
  }

  previous_transform = transform;
  previous_time = current_time;

  pub->publish(out);
}

}  // namespace loam_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(loam_interface::LoamInterfaceNode)
