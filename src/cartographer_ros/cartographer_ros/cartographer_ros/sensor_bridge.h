/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H

#include <memory>

#include "absl/types/optional.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros_msgs/LandmarkList.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"

namespace cartographer_ros {

// SensorBridge 是“ROS 传感器消息 -> Cartographer 内部数据”的转换器。
// Node 的每条 trajectory 都会拥有一个 SensorBridge。
//
// 它主要做三件事：
// 1. 解析 ROS 消息：LaserScan、PointCloud2、Imu、Odometry、NavSatFix 等。
// 2. 查询 TF：把消息 header.frame_id 或 odom child_frame_id 转到 tracking_frame。
// 3. 调用 trajectory_builder_->AddSensorData()，把标准化后的数据送进
//    Cartographer local SLAM。
//
// 在本项目当前 2D 主线中，最常见输入是 /scan：
// Node::HandleLaserScanMessage() -> SensorBridge::HandleLaserScanMessage()
// -> HandleLaserScan() -> HandleRangefinder() -> AddSensorData(RANGE)。
class SensorBridge {
 public:
  explicit SensorBridge(
      int num_subdivisions_per_laser_scan, bool ignore_out_of_order_messages,
      const std::string& tracking_frame, double lookup_transform_timeout_sec,
      tf2_ros::Buffer* tf_buffer,
      ::cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder);

  SensorBridge(const SensorBridge&) = delete;
  SensorBridge& operator=(const SensorBridge&) = delete;

  // 把 ROS nav_msgs/Odometry 转成 Cartographer OdometryData。
  // 输出位姿会被转换到 tracking_frame，供位姿外推和可选里程计约束使用。
  std::unique_ptr<::cartographer::sensor::OdometryData> ToOdometryData(
      const nav_msgs::Odometry::ConstPtr& msg);
  void HandleOdometryMessage(const std::string& sensor_id,
                             const nav_msgs::Odometry::ConstPtr& msg);
  void HandleNavSatFixMessage(const std::string& sensor_id,
                              const sensor_msgs::NavSatFix::ConstPtr& msg);
  void HandleLandmarkMessage(
      const std::string& sensor_id,
      const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);

  // 把 ROS sensor_msgs/Imu 转成 Cartographer ImuData。
  // Cartographer 要求 IMU 坐标系和 tracking_frame 同位置，只允许旋转不同。
  std::unique_ptr<::cartographer::sensor::ImuData> ToImuData(
      const sensor_msgs::Imu::ConstPtr& msg);
  void HandleImuMessage(const std::string& sensor_id,
                        const sensor_msgs::Imu::ConstPtr& msg);
  void HandleLaserScanMessage(const std::string& sensor_id,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      const std::string& sensor_id,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  void HandlePointCloud2Message(const std::string& sensor_id,
                                const sensor_msgs::PointCloud2::ConstPtr& msg);

  // 暴露 TfBridge 给 MapBuilderBridge，用于发布 TF 时查询 published_frame
  // 到 tracking_frame 的静态/动态关系。
  const TfBridge& tf_bridge() const;
  // 可选的乱序消息过滤。仿真或 rosbag 回放中如果时间戳倒退，Cartographer
  // 的 Collator 可能被旧数据卡住，因此这里可以直接丢弃旧消息。
  bool IgnoreMessage(const std::string& sensor_id,
                     ::cartographer::common::Time sensor_time);

 private:
  // LaserScan/MultiEchoLaserScan 会先被转换成带时间偏移的点云。
  // num_subdivisions_per_laser_scan_>1 时，一帧扫描会按时间切成多段，
  // 让 Cartographer 更接近“扫描期间机器人持续运动”的真实情况。
  void HandleLaserScan(
      const std::string& sensor_id, ::cartographer::common::Time start_time,
      const std::string& frame_id,
      const ::cartographer::sensor::PointCloudWithIntensities& points);
  // RANGE 数据的最终入口：查询 frame_id->tracking_frame 的 TF，把点云转换到
  // tracking_frame 后送入 TrajectoryBuilder。对于本项目，/scan 最终走到这里。
  void HandleRangefinder(const std::string& sensor_id,
                         ::cartographer::common::Time time,
                         const std::string& frame_id,
                         const ::cartographer::sensor::TimedPointCloud& ranges);

  const int num_subdivisions_per_laser_scan_;
  const bool ignore_out_of_order_messages_;
  // 记录每个 laser scan 子段的时间，防止同一传感器时间戳不递增。
  std::map<std::string, cartographer::common::Time>
      sensor_to_previous_subdivision_time_;
  // 记录每个 sensor_id 最新处理时间，用于丢弃乱序消息。
  std::map<std::string, cartographer::common::Time> latest_sensor_time_;
  // 负责所有 TF 查询。SensorBridge 不直接操作 tf2 Buffer。
  const TfBridge tf_bridge_;
  // 指向 Cartographer 核心的轨迹构建器。SensorBridge 不拥有它，只负责喂数据。
  ::cartographer::mapping::TrajectoryBuilderInterface* const
      trajectory_builder_;

  // GPS/NavSatFix 使用的本地 ENU 近似坐标系。当前项目主线不依赖 GPS。
  absl::optional<::cartographer::transform::Rigid3d> ecef_to_local_frame_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
