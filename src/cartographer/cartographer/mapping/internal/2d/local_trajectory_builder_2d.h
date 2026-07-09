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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_

#include <chrono>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/proto/local_trajectory_builder_options_2d.pb.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
//
// 中文导读：
// LocalTrajectoryBuilder2D 是 Cartographer 2D 的“前端”。在 ROS 侧，
// /scan 最终会被 cartographer_ros 转成 TimedPointCloudData 传进
// AddRangeData()。本类不负责后端回环优化，只负责把一帧或多帧激光数据：
//   1. 按时间同步、累积，并用 PoseExtrapolator 做每个点的运动补偿；
//   2. 对齐到重力水平面，裁剪高度和体素滤波；
//   3. 用当前 active submap 做 scan-to-map 匹配，得到局部坐标系下位姿；
//   4. 把匹配后的点云插入 active submaps，形成局部实时地图；
//   5. 输出 MatchingResult，交给 PoseGraph2D 作为一个 trajectory node。
// TODO(gaschler): Add test for this class similar to the 3D test.
class LocalTrajectoryBuilder2D {
 public:
  struct InsertionResult {
    // 后端需要的“节点常量数据”：时间、重力对齐、滤波点云、局部位姿。
    // 这些内容在后端优化中不会被重新估计。
    std::shared_ptr<const TrajectoryNode::Data> constant_data;
    // 本次 scan 被插入的 submap 列表。通常是两个：老 submap 用于稳定匹配，
    // 新 submap 用于平滑接替；刚启动时可能只有一个。
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps;
  };
  struct MatchingResult {
    common::Time time;
    // 前端 scan matching 后得到的局部 SLAM 位姿，属于 local map frame，
    // 还不是经过 pose graph 回环优化后的全局位姿。
    transform::Rigid3d local_pose;
    // 已经变换到 local frame 的原始量测，用于可视化、后端约束和 submap 插入。
    sensor::RangeData range_data_in_local;
    // 'nullptr' if dropped by the motion filter.
    // 运动太小会被 MotionFilter 丢弃：此时有局部位姿估计，但不新建后端节点，
    // 也不重复更新 submap，避免地图被静止噪声反复刷黑/刷白。
    std::unique_ptr<const InsertionResult> insertion_result;
  };

  explicit LocalTrajectoryBuilder2D(
      const proto::LocalTrajectoryBuilderOptions2D& options,
      const std::vector<std::string>& expected_range_sensor_ids);
  ~LocalTrajectoryBuilder2D();

  LocalTrajectoryBuilder2D(const LocalTrajectoryBuilder2D&) = delete;
  LocalTrajectoryBuilder2D& operator=(const LocalTrajectoryBuilder2D&) = delete;

  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'. Range data must be approximately horizontal
  // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  //
  // 对 /scan 使用者的直观理解：每次激光扫描到来后，不一定立刻输出一个节点；
  // 只有累计到 num_accumulated_range_data 后，才会执行滤波、scan matching、
  // submap 插入，并返回 MatchingResult。
  std::unique_ptr<MatchingResult> AddRangeData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& range_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
      common::Time time, const sensor::RangeData& gravity_aligned_range_data,
      const transform::Rigid3d& gravity_alignment,
      const absl::optional<common::Duration>& sensor_duration);
  sensor::RangeData TransformToGravityAlignedFrameAndFilter(
      const transform::Rigid3f& transform_to_gravity_aligned_frame,
      const sensor::RangeData& range_data) const;
  std::unique_ptr<InsertionResult> InsertIntoSubmap(
      common::Time time, const sensor::RangeData& range_data_in_local,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
      const transform::Rigid3d& pose_estimate,
      const Eigen::Quaterniond& gravity_alignment);

  // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
  // observed pose, or nullptr on failure.
  // 输入 pose_prediction 来自 IMU/里程计/恒速外推；输出 observed pose 是
  // 被当前 submap 修正后的 2D 位姿，是“前端定位”的核心结果。
  std::unique_ptr<transform::Rigid2d> ScanMatch(
      common::Time time, const transform::Rigid2d& pose_prediction,
      const sensor::PointCloud& filtered_gravity_aligned_point_cloud);

  // Lazily constructs a PoseExtrapolator.
  void InitializeExtrapolator(common::Time time);

  const proto::LocalTrajectoryBuilderOptions2D options_;
  ActiveSubmaps2D active_submaps_;

  // MotionFilter 控制“是否值得把这一帧真的插入地图”。平移/旋转/时间变化都很小
  // 时跳过插入，降低计算量并避免同一位置的噪声被重复积分。
  MotionFilter motion_filter_;
  // 粗匹配：在有限窗口内暴力搜索，让 Ceres 的初值更接近真实解。
  scan_matching::RealTimeCorrelativeScanMatcher2D
      real_time_correlative_scan_matcher_;
  // 精匹配：以概率栅格/TSDF 为代价函数做连续优化，输出最终局部位姿。
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;

  // 根据历史位姿、IMU、里程计外推当前传感器位姿。没有 IMU 时本文件会用恒速模型
  // 懒初始化，保证纯 /scan 建图也能跑起来。
  std::unique_ptr<PoseExtrapolator> extrapolator_;

  // 多帧激光累计缓存。Cartographer 可以把若干个 /scan 合成一次前端匹配，
  // 以提高点数和稳定性。
  int num_accumulated_ = 0;
  sensor::RangeData accumulated_range_data_;

  absl::optional<std::chrono::steady_clock::time_point> last_wall_time_;
  absl::optional<double> last_thread_cpu_time_seconds_;
  absl::optional<common::Time> last_sensor_time_;

  RangeDataCollator range_data_collator_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
