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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TF_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TF_BRIDGE_H

#include <memory>

#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/time_conversion.h"
#include "tf2_ros/buffer.h"

namespace cartographer_ros {

// TfBridge 是 Cartographer ROS 层对 tf2_ros::Buffer 的轻量封装。
// SensorBridge 只关心“某个传感器 frame 在某个时间到 tracking_frame 的变换”，
// 不直接处理 tf2 异常、超时和时间转换，这些细节集中放在这里。
//
// 在本项目中，/scan 的 header.frame_id 通常来自雷达坐标系；TfBridge 会查询
// laser_frame -> base_link/tracking_frame，使激光点能进入车体中心坐标系。
class TfBridge {
 public:
  TfBridge(const std::string& tracking_frame,
           double lookup_transform_timeout_sec, const tf2_ros::Buffer* buffer);
  ~TfBridge() {}

  TfBridge(const TfBridge&) = delete;
  TfBridge& operator=(const TfBridge&) = delete;

  // 返回指定 time 下 frame_id -> tracking_frame_ 的变换。
  // 返回 nullptr 表示 TF 不可用；调用方会跳过该帧传感器数据。
  std::unique_ptr<::cartographer::transform::Rigid3d> LookupToTracking(
      ::cartographer::common::Time time, const std::string& frame_id) const;

 private:
  // tracking_frame 是 Cartographer 跟踪机器人运动的核心坐标系，2D 小车项目中
  // 通常配置为 base_link 或 imu_link。
  const std::string tracking_frame_;
  // 每次 TF 查询最多等待的秒数，避免传感器回调永久阻塞。
  const double lookup_transform_timeout_sec_;
  // 不拥有 Buffer，只借用 node_main.cc 中创建并由 TransformListener 持续填充的
  // TF 缓存。
  const tf2_ros::Buffer* const buffer_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TF_BRIDGE_H
