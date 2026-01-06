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
    - [mapCloudCallback](#mapcloudcallback)
    - [odometryCallback](#odometrycallback)
  - [Usage](#usage)

## Overview

A ROS2 interface node that bridges LiDAR-based SLAM (e.g., Point-LIO, LOAM) output to the standard robot navigation coordinate frame system. It transforms point cloud and odometry data from the SLAM algorithm's local frame to the global odometry frame, and publishes the TF transformation from `odom` to `base_frame`.

## LoamInterfaceNode

The main node that handles coordinate frame transformations for LiDAR SLAM integration.

### Subscriptions

| Topic | Type | QoS | Description |
| ----- | ---- | --- | ----------- |
| `loam_odometry_topic` | `nav_msgs/msg/Odometry` | Default | Odometry output from SLAM algorithm (default: `pslam/imu_odom`) |
| `registered_scan_topic` | `sensor_msgs/msg/PointCloud2` | Default | Registered/aligned point cloud from SLAM (default: `pslam/aligned_scan_cloud`) |
| `map_cloud_topic` | `sensor_msgs/msg/PointCloud2` | Transient Local, Reliable | Map point cloud from SLAM algorithm (default: `pslam/lio_map_cloud`, latched topic) |

### Publications

| Topic | Type | Description |
| ----- | ---- | ----------- |
| `registered_scan` | `sensor_msgs/msg/PointCloud2` | Transformed point cloud in the global `odom_frame` (for mapping and navigation) |
| `sensor_scan` | `sensor_msgs/msg/PointCloud2` | Transformed point cloud in the `lidar_frame` (for local perception and obstacle avoidance) |
| `map_cloud` | `sensor_msgs/msg/PointCloud2` | Transformed map point cloud in the global `odom_frame` |
| `state_estimation` | `nav_msgs/msg/Odometry` | Transformed odometry from `odom_frame` to `base_frame` |
| `lidar_odometry` | `nav_msgs/msg/Odometry` | Transformed odometry from `odom_frame` to `lidar_frame` |

### TF Broadcasts

| Parent Frame | Child Frame | Description |
| ------------ | ----------- | ----------- |
| `odom_frame` | `base_frame` | Robot base pose in the odometry frame |

### Parameters

| Parameter | Type | Default | Description |
| --------- | ---- | ------- | ----------- |
| `registered_scan_topic` | string | `pslam/aligned_scan_cloud` | Input point cloud topic from SLAM |
| `loam_odometry_topic` | string | `pslam/imu_odom` | Input odometry topic from SLAM |
| `map_cloud_topic` | string | `pslam/lio_map_cloud` | Input map point cloud topic from SLAM |
| `odom_frame` | string | `odom` | Global odometry frame ID |
| `base_frame` | string | `base_footprint` | Robot base frame for TF broadcast |
| `lidar_frame` | string | `mid360` | LiDAR sensor frame ID |
| `robot_base_frame` | string | `base_link` | Robot base link frame ID |

## Coordinate Frame Transformation

### pointCloudCallback

Transforms input point cloud to both the global `odom_frame` and `lidar_frame` using **time-synchronized** cached transformations.

**Time Synchronization:**

- Point cloud timestamp is matched with the closest odometry transform from history queue
- Uses transform history queue with 100 entries
- Maximum acceptable time difference: 0.1 seconds
- Falls back to latest transform if no matching transform is found

**Input Assumption:**

- Input point cloud is expected to be in SLAM's local odometry frame (e.g., `lidar_odom`)
- The transformation is retrieved from the transform history queue based on point cloud timestamp

**Transformation Logic:**

```txt
   Input Point Cloud (lidar_odom frame, t=T)
              │
              ├─────────────────────────────┐
              ▼                             ▼
    ┌──────────────────┐        ┌──────────────────┐
    │  Find transform  │        │  Find transform  │
    │  at time T from  │        │  at time T from  │
    │  history queue   │        │  history queue   │
    └──────────────────┘        └──────────────────┘
              │                             │
              ▼                             ▼
    ┌──────────────────┐        ┌──────────────────┐
    │  Transform with  │        │  Transform with  │
    │ tf_odom_to_      │        │ tf_lidar_odom_   │
    │ lidar_odom_(T)   │        │ to_lidar_(T)^-1  │
    └──────────────────┘        └──────────────────┘
              │                             │
              ▼                             ▼
    registered_scan (odom)       sensor_scan (lidar)
```

**Performance:**

- Uses cached transformations from odometry callback (no TF lookups)
- Efficient for high-frequency point cloud processing
- Ensures temporal consistency between odometry and point cloud transformations
- Thread-safe with mutex protection

### mapCloudCallback

Transforms the SLAM-generated map point cloud to the global `odom_frame`.

**Input:**

- Subscribes to `map_cloud_topic` (default: `pslam/lio_map_cloud`) with **Transient Local** QoS (latched topic)

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
    loam_odometry_topic:=/your_slam/odom \
    registered_scan_topic:=/your_slam/cloud \
    odom_frame:=odom \
    base_frame:=base_footprint
```
