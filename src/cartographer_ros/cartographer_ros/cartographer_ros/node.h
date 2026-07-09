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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/GetTrajectoryStates.h"
#include "cartographer_ros_msgs/ReadMetrics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/WriteState.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"

namespace cartographer_ros {

// Node 是 Cartographer ROS 接口层的中心调度类，作用是“把 ROS 世界接到
// Cartographer 核心算法”：
// - 输入：/scan、/points2、/imu、/odom、/fix、/landmark 等 ROS 话题。
// - 桥接：根据 trajectory_id 找到对应 SensorBridge，把 ROS 消息转换为
//   Cartographer 内部 SensorData。
// - 输出：发布子图列表、轨迹可视化、约束可视化、scan_matched_points2、
//   tracked_pose，并按配置发布 map/odom/base_link 相关 TF。
// - 管理：通过 ROS service 创建、结束、查询、保存和加载 trajectory。
//
// 在本项目的 Cartographer 2D 主线中，最重要的路径是：
// /scan -> Node::HandleLaserScanMessage() -> SensorBridge ->
// MapBuilder/TrajectoryBuilder -> Node 定时发布 TF 与子图列表 ->
// occupancy_grid_node_main.cc 拼接 /map。
class Node {
 public:
  Node(const NodeOptions& node_options,
       std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
       tf2_ros::Buffer* tf_buffer, bool collect_metrics);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  // 结束所有仍处于 ACTIVE 状态的轨迹。退出节点、保存状态或最终优化前都会用到。
  void FinishAllTrajectories();
  // 结束指定轨迹。返回 false 表示 trajectory_id 不存在或已经不是 ACTIVE。
  bool FinishTrajectory(int trajectory_id);

  // 触发后端 pose graph 最终优化。调用时所有轨迹都应已结束，否则会先尝试结束。
  void RunFinalOptimization();

  // 按 launch/Lua 中的默认话题启动第一条在线建图轨迹。
  // 对本项目通常意味着订阅 /scan，开始生成 Cartographer 2D 子图。
  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

  // Returns unique SensorIds for multiple input bag files based on
  // their TrajectoryOptions.
  // 'SensorId::id' is the expected ROS topic name.
  std::vector<
      std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
  ComputeDefaultSensorIdsForMultipleBags(
      const std::vector<TrajectoryOptions>& bags_options) const;

  // 添加离线轨迹，只登记期望传感器，不启动 ROS subscriber；rosbag 离线处理会用。
  int AddOfflineTrajectory(
      const std::set<
          cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& options);

  // 以下 Handle*Message 是 ROS subscriber 的入口。它们先按采样率过滤，
  // 再转交给对应 trajectory 的 SensorBridge。sensor_id 在在线模式下通常就是
  // 话题名，例如 /scan、/odom；Cartographer 用它区分不同传感器数据流。
  void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                             const nav_msgs::Odometry::ConstPtr& msg);
  void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::NavSatFix::ConstPtr& msg);
  void HandleLandmarkMessage(
      int trajectory_id, const std::string& sensor_id,
      const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);
  void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                        const sensor_msgs::Imu::ConstPtr& msg);
  void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      int trajectory_id, const std::string& sensor_id,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::PointCloud2::ConstPtr& msg);

  // 将完整 SLAM 状态写成 .pbstream，包含 pose graph、子图等。
  void SerializeState(const std::string& filename,
                      const bool include_unfinished_submaps);

  // 从 .pbstream 加载已有 SLAM 状态，可用于基于已有地图继续定位/建图。
  void LoadState(const std::string& state_filename, bool load_frozen_state);

  ::ros::NodeHandle* node_handle();

 private:
  struct Subscriber {
    ::ros::Subscriber subscriber;

    // ROS 解析命名空间后，Subscriber::getTopic() 不一定等于构造时传入的字符串。
    // 这里主动保存原始 topic，是因为 Cartographer 把 topic 名当作 sensor_id，
    // 后续结束轨迹、检查话题缺失、避免重复订阅都依赖这个稳定标识。
    std::string topic;
  };

  bool HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleTrajectoryQuery(
      ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
      ::cartographer_ros_msgs::TrajectoryQuery::Response& response);
  bool HandleStartTrajectory(
      cartographer_ros_msgs::StartTrajectory::Request& request,
      cartographer_ros_msgs::StartTrajectory::Response& response);
  bool HandleFinishTrajectory(
      cartographer_ros_msgs::FinishTrajectory::Request& request,
      cartographer_ros_msgs::FinishTrajectory::Response& response);
  bool HandleWriteState(cartographer_ros_msgs::WriteState::Request& request,
                        cartographer_ros_msgs::WriteState::Response& response);
  bool HandleGetTrajectoryStates(
      ::cartographer_ros_msgs::GetTrajectoryStates::Request& request,
      ::cartographer_ros_msgs::GetTrajectoryStates::Response& response);
  bool HandleReadMetrics(
      cartographer_ros_msgs::ReadMetrics::Request& request,
      cartographer_ros_msgs::ReadMetrics::Response& response);

  // 根据 TrajectoryOptions 推导一条轨迹应该接收哪些传感器。
  // 'SensorId::id' 是期望 ROS 话题名，例如本项目 2D 链路里的 /scan。
  std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
  ComputeExpectedSensorIds(const TrajectoryOptions& options) const;
  // AddTrajectory 是在线建图的核心创建步骤：创建 Cartographer trajectory、
  // 建立外推器和采样器、启动 ROS 订阅，并记录已使用的话题。
  int AddTrajectory(const TrajectoryOptions& options);
  // 按 TrajectoryOptions 真正订阅 ROS 话题。num_laser_scans=1 时订阅 /scan；
  // num_point_clouds>0 时订阅 /points2；IMU/odom/GPS/landmark 按开关订阅。
  void LaunchSubscribers(const TrajectoryOptions& options, int trajectory_id);
  // 周期发布 submap_list。occupancy_grid_node 订阅它后，再通过 submap_query
  // 服务取每个子图纹理，最后拼成 ROS /map。
  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  // PoseExtrapolator 用最近的 local SLAM 位姿、IMU、odom 来外推当前时刻位姿，
  // 从而以较高频率发布平滑 TF，而不必等每帧扫描匹配完成。
  void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
  // 为每类传感器创建固定比例采样器，用于降低高频数据压力。
  void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);
  // 定时发布当前轨迹位姿、TF 和 scan_matched_point_cloud。
  // 这里决定最终是 map->odom->base_link，还是 map->base_link 这样的 TF 结构。
  void PublishLocalTrajectoryData(const ::ros::TimerEvent& timer_event);
  void PublishTrajectoryNodeList(const ::ros::WallTimerEvent& timer_event);
  void PublishLandmarkPosesList(const ::ros::WallTimerEvent& timer_event);
  void PublishConstraintList(const ::ros::WallTimerEvent& timer_event);
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicNames(const TrajectoryOptions& options);
  cartographer_ros_msgs::StatusResponse FinishTrajectoryUnderLock(
      int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void MaybeWarnAboutTopicMismatch(const ::ros::WallTimerEvent&);

  // 服务处理函数共用的状态检查工具，避免对不存在、已结束、被冻结轨迹误操作。
  cartographer_ros_msgs::StatusResponse TrajectoryStateToStatus(
      int trajectory_id,
      const std::set<
          cartographer::mapping::PoseGraphInterface::TrajectoryState>&
          valid_states);
  const NodeOptions node_options_;

  // 发布 Cartographer 估计出的 TF。2D 主线中常见配置是：
  // map_frame=map，odom_frame=odom，published_frame=base_link。
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  absl::Mutex mutex_;
  std::unique_ptr<cartographer_ros::metrics::FamilyFactory> metrics_registry_;
  // MapBuilderBridge 持有 Cartographer 核心 MapBuilder，并为每条轨迹维护
  // SensorBridge。Node 自己不直接做扫描匹配。
  MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

  ::ros::NodeHandle node_handle_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::Publisher trajectory_node_list_publisher_;
  ::ros::Publisher landmark_poses_list_publisher_;
  ::ros::Publisher constraint_list_publisher_;
  ::ros::Publisher tracked_pose_publisher_;
  // [Innovation 1] Publishes directional degeneracy diagnostics.
  ::ros::Publisher degeneracy_metric_publisher_;
  ::ros::Publisher degeneracy_direction_publisher_;
  // 这些 ServiceServer 必须和 Node 同生命周期，否则服务会自动下线。
  std::vector<::ros::ServiceServer> service_servers_;
  // 发布扫描匹配后的点云，主要给 RViz 调试局部匹配结果；不是 /map 的来源。
  ::ros::Publisher scan_matched_point_cloud_publisher_;

  struct TrajectorySensorSamplers {
    TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                             const double odometry_sampling_ratio,
                             const double fixed_frame_pose_sampling_ratio,
                             const double imu_sampling_ratio,
                             const double landmark_sampling_ratio)
        : rangefinder_sampler(rangefinder_sampling_ratio),
          odometry_sampler(odometry_sampling_ratio),
          fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
          imu_sampler(imu_sampling_ratio),
          landmark_sampler(landmark_sampling_ratio) {}

    ::cartographer::common::FixedRatioSampler rangefinder_sampler;
    ::cartographer::common::FixedRatioSampler odometry_sampler;
    ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
    ::cartographer::common::FixedRatioSampler imu_sampler;
    ::cartographer::common::FixedRatioSampler landmark_sampler;
  };

  // 以下容器均以 trajectory_id 为键。一个 cartographer_node 可以同时管理多条
  // 轨迹；本项目在线 2D 建图通常只有一条 ACTIVE 轨迹。
  std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
  std::map<int, ::ros::Time> last_published_tf_stamps_;
  std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
  std::unordered_map<int, std::vector<Subscriber>> subscribers_;
  std::unordered_set<std::string> subscribed_topics_;
  std::unordered_set<int> trajectories_scheduled_for_finish_;

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;

  // The timer for publishing local trajectory data (i.e. pose transforms and
  // range data point clouds) is a regular timer which is not triggered when
  // simulation time is standing still. This prevents overflowing the transform
  // listener buffer by publishing the same transforms over and over again.
  ::ros::Timer publish_local_trajectory_data_timer_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
