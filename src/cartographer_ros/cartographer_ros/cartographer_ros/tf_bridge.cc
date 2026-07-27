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

#include "cartographer_ros/tf_bridge.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/msg_conversion.h"

namespace cartographer_ros {

TfBridge::TfBridge(const std::string& tracking_frame,
                   const double lookup_transform_timeout_sec,
                   const tf2_ros::Buffer* buffer)
    : tracking_frame_(tracking_frame),
      lookup_transform_timeout_sec_(lookup_transform_timeout_sec),
      buffer_(buffer) {}

std::unique_ptr<::cartographer::transform::Rigid3d> TfBridge::LookupToTracking(
    const ::cartographer::common::Time time,
    const std::string& frame_id) const {
  // 该函数是所有传感器坐标变换的关键入口。例如 /scan 到来时，会查询
  // scan.header.frame_id -> tracking_frame_，再把激光点变换到 tracking_frame。
  ::ros::Duration timeout(lookup_transform_timeout_sec_);
  std::unique_ptr<::cartographer::transform::Rigid3d> frame_id_to_tracking;
  try {
    // 先查一次最新 TF 的时间。如果缓存中已经有比请求时间更新的 TF，就不再等待；
    // 这避免了请求旧时间戳时仍然阻塞满 timeout。
    const ::ros::Time latest_tf_time =
        buffer_
            ->lookupTransform(tracking_frame_, frame_id, ::ros::Time(0.),
                              timeout)
            .header.stamp;
    const ::ros::Time requested_time = ToRos(time);
    if (latest_tf_time >= requested_time) {
      // We already have newer data, so we do not wait. Otherwise, we would wait
      // for the full 'timeout' even if we ask for data that is too old.
      timeout = ::ros::Duration(0.);
    }
    // tf2 返回 geometry_msgs::TransformStamped；ToRigid3d 转成 Cartographer
    // 使用的刚体变换类型，供点云、IMU、里程计统一处理。
    return absl::make_unique<::cartographer::transform::Rigid3d>(
        ToRigid3d(buffer_->lookupTransform(tracking_frame_, frame_id,
                                           requested_time, timeout)));
  } catch (const tf2::TransformException& ex) {
    // TF 缺失、外推失败、frame 名写错都会走到这里。对本项目而言，如果 /scan
    // 没有 laser_frame 到 base_link 的 TF，Cartographer 就无法消费该帧扫描。
    LOG(WARNING) << ex.what();
  }
  return nullptr;
}

}  // namespace cartographer_ros
