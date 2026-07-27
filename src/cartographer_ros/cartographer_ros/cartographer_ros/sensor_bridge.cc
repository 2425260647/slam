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

#include "cartographer_ros/sensor_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

// ROS TF2 规范里 frame_id 不应该以 '/' 开头。这里提前检查，是为了避免
// 后续 lookupTransform 使用不同命名规则导致“明明有 TF 却查不到”的问题。
const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace

SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,
    const bool ignore_out_of_order_messages, const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, tf2_ros::Buffer* const tf_buffer,
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      ignore_out_of_order_messages_(ignore_out_of_order_messages),
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      trajectory_builder_(trajectory_builder) {}

std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(
    const nav_msgs::Odometry::ConstPtr& msg) {
  // Odometry 的 pose 表示 header.frame_id 下 child_frame_id 的位姿。
  // Cartographer 内部希望使用 tracking_frame 的位姿，所以这里先查
  // child_frame_id -> tracking_frame，再把 odom pose 换算为 tracking_frame pose。
  const carto::common::Time time = FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  return absl::make_unique<carto::sensor::OdometryData>(
      carto::sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse()});
}

bool SensorBridge::IgnoreMessage(const std::string& sensor_id,
                                 cartographer::common::Time sensor_time) {
  // Cartographer 要求同一个 sensor_id 的数据时间单调递增。
  // 如果配置允许忽略乱序消息，这里用 latest_sensor_time_ 做轻量过滤。
  if (!ignore_out_of_order_messages_) {
    return false;
  }
  auto it = latest_sensor_time_.find(sensor_id);
  if (it == latest_sensor_time_.end()) {
    return false;
  }
  return sensor_time <= it->second;
}

void SensorBridge::HandleOdometryMessage(
    const std::string& sensor_id, const nav_msgs::Odometry::ConstPtr& msg) {
  // 里程计不是本项目 Cartographer 2D 的必需输入，但如果 Lua 中 use_odometry=true，
  // 它会作为辅助位姿先验进入 trajectory_builder。
  std::unique_ptr<carto::sensor::OdometryData> odometry_data =
      ToOdometryData(msg);
  if (odometry_data != nullptr) {
    if (IgnoreMessage(sensor_id, odometry_data->time)) {
      LOG(WARNING) << "Ignored odometry message from sensor " << sensor_id
                   << " because sensor time " << odometry_data->time
                   << " is not before last odometry message time "
                   << latest_sensor_time_[sensor_id];
      return;
    }
    latest_sensor_time_[sensor_id] = odometry_data->time;
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
  }
}

void SensorBridge::HandleNavSatFixMessage(
    const std::string& sensor_id, const sensor_msgs::NavSatFix::ConstPtr& msg) {
  // GPS 数据会被转换为 FixedFramePoseData。当前 Scout Mini 仿真主线不使用它，
  // 但接口保留用于多传感器或室外实车扩展。
  const carto::common::Time time = FromRos(msg->header.stamp);
  if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::FixedFramePoseData{time, absl::optional<Rigid3d>()});
    return;
  }

  if (!ecef_to_local_frame_.has_value()) {
    ecef_to_local_frame_ =
        ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude);
    LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
              << msg->latitude << ", long = " << msg->longitude << ".";
  }

  trajectory_builder_->AddSensorData(
      sensor_id, carto::sensor::FixedFramePoseData{
                     time, absl::optional<Rigid3d>(Rigid3d::Translation(
                               ecef_to_local_frame_.value() *
                               LatLongAltToEcef(msg->latitude, msg->longitude,
                                                msg->altitude)))});
}

void SensorBridge::HandleLandmarkMessage(
    const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  // Landmark 观测通常来自 AprilTag、反光柱等外部定位特征。
  // ROS 消息中的观测先转换到 tracking_frame，再交给后端作为约束。
  auto landmark_data = ToLandmarkData(*msg);

  auto tracking_from_landmark_sensor = tf_bridge_.LookupToTracking(
      landmark_data.time, CheckNoLeadingSlash(msg->header.frame_id));
  if (tracking_from_landmark_sensor != nullptr) {
    for (auto& observation : landmark_data.landmark_observations) {
      observation.landmark_to_tracking_transform =
          *tracking_from_landmark_sensor *
          observation.landmark_to_tracking_transform;
    }
  }
  trajectory_builder_->AddSensorData(sensor_id, landmark_data);
}

std::unique_ptr<carto::sensor::ImuData> SensorBridge::ToImuData(
    const sensor_msgs::Imu::ConstPtr& msg) {
  // IMU 对 3D SLAM 必需；对 2D SLAM 是否使用由 Lua 中 use_imu_data 决定。
  // 当前项目主输入是 /scan，若 2D 配置关闭 IMU，则 Node 根本不会订阅 /imu。
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

  const carto::common::Time time = FromRos(msg->header.stamp);
  // 只旋转加速度和角速度，不平移 IMU 数据；因此要求 IMU 与 tracking_frame
  // 共点，否则线加速度旋转到车体中心会引入误差。
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
      << "The IMU frame must be colocated with the tracking frame. "
         "Transforming linear acceleration into the tracking frame will "
         "otherwise be imprecise.";
  return absl::make_unique<carto::sensor::ImuData>(carto::sensor::ImuData{
      time, sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
      sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
}

void SensorBridge::HandleImuMessage(const std::string& sensor_id,
                                    const sensor_msgs::Imu::ConstPtr& msg) {
  // IMU 数据除送入 Cartographer 外，也会在 Node 层加入 PoseExtrapolator，
  // 用于提高 TF 发布频率和短时间位姿外推稳定性。
  std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
  if (imu_data != nullptr) {
    if (IgnoreMessage(sensor_id, imu_data->time)) {
      LOG(WARNING) << "Ignored IMU message from sensor " << sensor_id
                   << " because sensor time " << imu_data->time
                   << " is not before last IMU message time "
                   << latest_sensor_time_[sensor_id];
      return;
    }
    latest_sensor_time_[sensor_id] = imu_data->time;
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                               imu_data->angular_velocity});
  }
}

void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::LaserScan::ConstPtr& msg) {
  // /scan 是本项目 Cartographer 2D 的核心输入。
  // 这里先把 LaserScan 的 ranges/intensities 转为带时间偏移的点云，再统一走
  // HandleLaserScan/HandleRangefinder，后续和 PointCloud2 的处理路径接近。
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  // MultiEchoLaserScan 适合多回波激光；转换后也进入同一套 RANGE 数据入口。
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandlePointCloud2Message(
    const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  // 如果 Lua 配置订阅点云而不是 /scan，PointCloud2 会在这里转换为
  // TimedPointCloud。当前项目先由 pointcloud_to_laserscan 投影为 /scan，
  // 因此主线通常不走这个入口。
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleRangefinder(sensor_id, time, msg->header.frame_id, point_cloud.points);
}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::PointCloudWithIntensities& points) {
  if (points.points.empty()) {
    return;
  }
  CHECK_LE(points.points.back().time, 0.f);
  // TODO(gaschler): Use per-point time instead of subdivisions.
  // 一帧 LaserScan 覆盖一个扫描周期，机器人在扫描期间可能已经移动。
  // 将一帧扫描切成若干 subdivision，可以让 Collator 按更细的时间片排序，
  // 减少运动畸变对 2D 扫描匹配的影响。
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
    const size_t start_index =
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    carto::sensor::TimedPointCloud subdivision(
        points.points.begin() + start_index, points.points.begin() + end_index);
    if (start_index == end_index) {
      continue;
    }
    const double time_to_subdivision_end = subdivision.back().time;
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.
    // subdivision_time 取该子段结束时刻。这样同一时间段内的 IMU/odom 会先被
    // Collator 送入，再处理这段激光，时间因果关系更合理。
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
    if (it != sensor_to_previous_subdivision_time_.end() &&
        it->second >= subdivision_time) {
      LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;
      continue;
    }
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    // Cartographer 的 TimedPointCloud 约定最后一个点时间为 0，其他点是相对
    // 最后一个点的负偏移；这里重新归一化每个 subdivision 的点时间。
    for (auto& point : subdivision) {
      point.time -= time_to_subdivision_end;
    }
    CHECK_EQ(subdivision.back().time, 0.f);
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
  }
}

void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) {
  if (!ranges.empty()) {
    CHECK_LE(ranges.back().time, 0.f);
  }
  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));
  if (sensor_to_tracking != nullptr) {
    // 只有查到 frame_id -> tracking_frame 的 TF，激光点才能被放进
    // Cartographer 的统一坐标系。若 TF 缺失，本帧会被跳过并在 TfBridge 中告警。
    if (IgnoreMessage(sensor_id, time)) {
      LOG(WARNING) << "Ignored Rangefinder message from sensor " << sensor_id
                   << " because sensor time " << time
                   << " is not before last Rangefinder message time "
                   << latest_sensor_time_[sensor_id];
      return;
    }
    latest_sensor_time_[sensor_id] = time;
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::TimedPointCloudData{
                       // origin 是传感器在 tracking_frame 中的位置；points 是同一
                       // 坐标系下的点云。2D 扫描匹配正是消费这份 RANGE 数据。
                       time, sensor_to_tracking->translation().cast<float>(),
                       carto::sensor::TransformTimedPointCloud(
                           ranges, sensor_to_tracking->cast<float>())});
  }
}

}  // namespace cartographer_ros
