# loam_interface

- [loam\_interface](#loam_interface)
  - [Overview](#overview)
  - [LoamInterfaceNode](#loaminterfacenode)
    - [Subscriptions](#subscriptions)
    - [Publications](#publications)
    - [TF Broadcasts](#tf-broadcasts)
    - [Parameters](#parameters)
  - [Coordinate Frame Transformation](#coordinate-frame-transformation)
    - [pointCloudCallback](#pointcloudcallback)
    - [odometryCallback](#odometrycallback)
  - [Usage](#usage)

## Overview

A ROS2 interface node that bridges LiDAR-based SLAM (e.g., Point-LIO, LOAM) output to the standard robot navigation coordinate frame system. It transforms point cloud and odometry data from the SLAM algorithm's local frame to the global odometry frame, and publishes the TF transformation from `odom` to `base_frame`.

## LoamInterfaceNode

The main node that handles coordinate frame transformations for LiDAR SLAM integration.

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `state_estimation_topic` | `nav_msgs/msg/Odometry` | Odometry output from SLAM algorithm (default: `/pslam/imu_odom`) |
| `registered_scan_topic` | `sensor_msgs/msg/PointCloud2` | Registered point cloud from SLAM (default: `/pslam/deskewed_scan_cloud`) |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `registered_scan` | `sensor_msgs/msg/PointCloud2` | Transformed point cloud in the global `odom_frame` (for mapping and navigation) |
| `sensor_scan` | `sensor_msgs/msg/PointCloud2` | Transformed point cloud in the `lidar_frame` (for local perception and obstacle avoidance) |
| `lidar_odometry` | `nav_msgs/msg/Odometry` | Transformed odometry in the global `odom_frame` |

### TF Broadcasts

| Parent Frame | Child Frame | Description |
|--------------|-------------|-------------|
| `odom_frame` | `base_frame` | Robot base pose in the odometry frame |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `registered_scan_topic` | string | `/pslam/deskewed_scan_cloud` | Input point cloud topic from SLAM |
| `state_estimation_topic` | string | `/pslam/imu_odom` | Input odometry topic from SLAM |
| `odom_frame` | string | `odom` | Global odometry frame ID |
| `base_frame` | string | `base_footprint` | Robot base frame for TF broadcast |
| `lidar_frame` | string | `mid360` | LiDAR sensor frame ID |
| `robot_base_frame` | string | `base_link` | Robot base link frame ID |

## Coordinate Frame Transformation

### pointCloudCallback

to both the global `odom_frame` and `lidar_frame`, supporting various input coordinate frames.

**Supported Input Frames:**

- SLAM's local odometry frame (e.g., `lidar_odom`)
- Global odometry frame (`odom`)
- LiDAR sensor frame (`lidar_frame`)
- IMU frame or other sensor frames

**Transformation Logic:**

```txt
                    Input Point Cloud (any frame)
                              │
                ┌─────────────┴─────────────┐
                ▼                           ▼
        ┌──────────────┐            ┌──────────────┐
        │ Transform to │            │ Transform to │
        │  odom_frame  │            │ lidar_frame  │
        └──────────────┘            └──────────────┘
                │                           │
                ▼                           ▼
      registered_scan (odom)        sensor_scan (lidar)
```

**Optimization:**

- If input is already in target frame → direct copy (no transformation)
- If input is in `lidar_odom` frame → use cached `tf_odom_to_lidar_odom_`
- Otherwise → dynamically query TF tree for transformationput Point Cloud (odom_frame)

### odometryCallback

Performs the following operations:

1. **Initialize Transform**: On first callback, looks up the static transform from `base_frame` to `lidar_frame`, keeping only the yaw rotation (zeroing roll and pitch) to establish `tf_odom_to_lidar_odom_`.

2. **Transform Odometry**: Converts the SLAM odometry from `lidar_odom` frame to the global `odom` frame.

3. **Publish TF**: Broadcasts the transform from `odom_frame` → `base_frame` based on the transformed odometry and the static LiDAR-to-base transform.

```txt

┌──────────────────────────────────────────────────────────┐
│                    Frame Relationship                    │
├──────────────────────────────────────────────────────────┤
│                                                          │
│   lidar_odom ─────► lidar_frame                          │
│       │                 │                                │
│       │ tf_odom_to_     │ tf_lidar_to_base               │
│       │ lidar_odom_     │ (from TF tree)                 │
│       ▼                 ▼                                │
│     odom ───────────► base_frame                         │
│            tf_odom_to_base                               │
│            (published)                                   │
│                                                          │
└──────────────────────────────────────────────────────────┘

```

## Usage

```bash
ros2 launch loam_interface loam_interface_launch.py
```

Or with custom parameters:

```bash
ros2 launch loam_interface loam_interface_launch.py \
    state_estimation_topic:=/your_slam/odom \
    registered_scan_topic:=/your_slam/cloud \
    odom_frame:=odom \
    base_frame:=base_footprint
```
