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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "absl/synchronization/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectoryQuery.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"

// Abseil unfortunately pulls in winnt.h, which #defines DELETE.
// Clean up to unbreak visualization_msgs::Marker::DELETE.
#ifdef DELETE
#undef DELETE
#endif
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

// MapBuilderBridge 位于 ROS Node 与 Cartographer MapBuilderInterface 之间。
// Node 负责 ROS 订阅/发布，MapBuilder 负责 SLAM 算法；本类负责把二者组织起来：
// - AddTrajectory() 创建 Cartographer trajectory，并为它创建 SensorBridge。
// - OnLocalSlamResult() 接收 local SLAM 回调，缓存最新局部位姿和匹配点云。
// - GetSubmapList()/HandleSubmapQuery() 向 ROS 可视化和 /map 生成节点暴露子图。
// - GetLocalTrajectoryData() 给 Node 发布 TF、tracked_pose 和 scan_matched_points2。
//
// 在本项目的 2D 链路中，/map 并不是这里直接发布的；这里提供 submap 数据，
// occupancy_grid_node_main.cc 再把所有子图绘制成 nav_msgs/OccupancyGrid。
class MapBuilderBridge {
 public:
  struct LocalTrajectoryData {
    // LocalSlamData 是 local SLAM 每次处理完一帧累计 range data 后给 ROS 层的
    // 最新结果：
    // - time：该结果对应的传感器时间。
    // - local_pose：tracking_frame 在 local SLAM 局部坐标系中的位姿。
    // - range_data_in_local：扫描匹配后位于 local 坐标系的点云，用于 RViz 调试。
    struct LocalSlamData {
      ::cartographer::common::Time time;
      ::cartographer::transform::Rigid3d local_pose;
      ::cartographer::sensor::RangeData range_data_in_local;
    };
    std::shared_ptr<const LocalSlamData> local_slam_data;
    // local_to_map 来自后端 pose graph，表示局部 SLAM 坐标系到全局 map_frame
    // 的校正关系。回环和后端优化会改变它。
    cartographer::transform::Rigid3d local_to_map;
    // published_frame 到 tracking_frame 的变换。若 published_frame=base_link 且
    // tracking_frame=base_link，它通常接近单位变换；若 tracking_frame=imu_link，
    // 则用于发布 base_link 的 TF。
    std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
    TrajectoryOptions trajectory_options;
  };

  MapBuilderBridge(
      const NodeOptions& node_options,
      std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
      tf2_ros::Buffer* tf_buffer);

  MapBuilderBridge(const MapBuilderBridge&) = delete;
  MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

  // 加载 .pbstream 到 MapBuilder。load_frozen_state=true 时，加载轨迹不再接收
  // 新传感器数据，常用于纯定位或多轨迹对齐。
  void LoadState(const std::string& state_filename, bool load_frozen_state);
  // 创建一条轨迹，并登记这条轨迹预期接收的 sensor_id 集合。
  int AddTrajectory(
      const std::set<
          ::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& trajectory_options);
  // 通知 Cartographer 不再接收该轨迹的新数据，并释放对应 SensorBridge。
  void FinishTrajectory(int trajectory_id);
  // 后端最终优化，一般在所有轨迹结束后执行。
  void RunFinalOptimization();
  // 序列化 pose graph 和 submap 状态，生成 .pbstream。
  bool SerializeState(const std::string& filename,
                      const bool include_unfinished_submaps);

  // submap_query 服务的实际实现：按 trajectory_id/submap_index 返回子图纹理。
  // occupancy_grid_node 使用它把子图拼成 /map。
  void HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  // trajectory_query 服务的实际实现：返回轨迹节点在 map_frame 下的位姿序列。
  void HandleTrajectoryQuery(
      cartographer_ros_msgs::TrajectoryQuery::Request& request,
      cartographer_ros_msgs::TrajectoryQuery::Response& response);

  // 当前所有轨迹状态，供 Node 的服务校验和退出收尾使用。
  std::map<int /* trajectory_id */,
           ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
  GetTrajectoryStates();
  // 返回所有子图的 ID、版本、位姿和冻结状态；发布到 submap_list 话题。
  cartographer_ros_msgs::SubmapList GetSubmapList();
  // 返回每条 active trajectory 的最新 local SLAM 数据和全局校正，用于 TF 发布。
  std::unordered_map<int, LocalTrajectoryData> GetLocalTrajectoryData()
      LOCKS_EXCLUDED(mutex_);
  visualization_msgs::MarkerArray GetTrajectoryNodeList();
  visualization_msgs::MarkerArray GetLandmarkPosesList();
  visualization_msgs::MarkerArray GetConstraintList();

  // 取指定轨迹的 SensorBridge，Node 的传感器回调通过它喂数据。
  SensorBridge* sensor_bridge(int trajectory_id);

 private:
  // Cartographer local SLAM 每处理出一次局部匹配结果，就通过 AddTrajectoryBuilder
  // 注册的回调进入这里。这里只缓存最新结果，真正发布由 Node 定时完成。
  void OnLocalSlamResult(const int trajectory_id,
                         const ::cartographer::common::Time time,
                         const ::cartographer::transform::Rigid3d local_pose,
                         ::cartographer::sensor::RangeData range_data_in_local)
      LOCKS_EXCLUDED(mutex_);

  absl::Mutex mutex_;
  const NodeOptions node_options_;
  // 每条轨迹最近一次 local SLAM 结果。用 shared_ptr 是为了在加锁复制指针后，
  // 发布线程可以无锁读取不可变数据。
  std::unordered_map<int,
                     std::shared_ptr<const LocalTrajectoryData::LocalSlamData>>
      local_slam_data_ GUARDED_BY(mutex_);
  // Cartographer 核心对象，拥有 TrajectoryBuilder、PoseGraph、Submap 等算法状态。
  std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;
  // 传给每个 SensorBridge 的 TF 缓存，不由本类拥有。
  tf2_ros::Buffer* const tf_buffer_;

  std::unordered_map<std::string /* landmark ID */, int> landmark_to_index_;

  // 以下容器均以 trajectory_id 为键；本项目在线 2D 建图通常只使用一条轨迹。
  std::unordered_map<int, TrajectoryOptions> trajectory_options_;
  std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
  std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
