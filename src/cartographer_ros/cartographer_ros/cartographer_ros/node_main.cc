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

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
// 下面这些 gflags 参数通常由 launch 文件传入。
// 在本项目的 2D 建图链路中，configuration_directory 和
// configuration_basename 指向 Cartographer Lua 配置；Lua 配置决定订阅
// /scan 还是点云、tracking_frame/base_frame/odom_frame/map_frame 如何命名、
// 是否发布 map->odom 或 map->base_link 等关键行为。
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

namespace cartographer_ros {
namespace {

void Run() {
  // tf_buffer 是 Cartographer ROS 层查询 TF 的统一入口。
  // SensorBridge 会用它把 /scan、/imu、/odom 等消息的 frame_id 转到
  // tracking_frame；Node 发布位姿时也依赖同一套 frame 配置。
  // 这里缓存 10 秒 TF，足够覆盖传感器消息轻微延迟和仿真时间抖动。
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);

  // LoadOptions 读取 Lua 配置，拆成两层：
  // - NodeOptions：ROS 接口层配置，例如 map_frame、发布频率、是否发布 TF。
  // - TrajectoryOptions：单条轨迹的传感器配置，例如 num_laser_scans=1
  //   时默认订阅 /scan，这正是本项目 pointcloud_to_laserscan 后的主输入。
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  // MapBuilder 是 Cartographer 核心算法对象，负责子图、扫描匹配和后端优化。
  // Node 是 ROS 外壳，负责订阅话题、发布 TF/可视化信息、提供服务。
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);

  // 如果从 .pbstream 恢复，已有子图和轨迹会先加载进 pose graph。
  // load_frozen_state=true 表示历史轨迹只作为固定地图/约束使用，不继续插入新点。
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  // 在线建图最常见路径：启动后立刻创建第一条 trajectory，并按 Lua 配置订阅
  // 默认话题。在本项目中通常就是 /scan 进入 Cartographer 2D。
  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  // ros::spin() 进入 ROS 回调循环：传感器回调、服务回调、定时发布 TF/子图列表
  // 都在这里持续运行。Gazebo 和人工速度控制只负责让 /scan 随车运动而变化。
  ::ros::spin();

  // 节点退出前收尾所有轨迹并做一次最终优化，保证保存状态或离线分析时 pose graph
  // 已经处理完当前可用数据。
  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // 没有 Lua 配置就无法知道 ROS 话题、坐标系和 2D/3D 建图参数，因此直接失败。
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  // 该可执行文件在 ROS 图中显示为 cartographer_node。
  // 它本身不直接发布 /map；/map 由 occupancy_grid_node_main.cc 中的
  // cartographer_occupancy_grid_node 根据本节点发布的 submap_list 和
  // submap_query 服务拼接生成。
  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}
