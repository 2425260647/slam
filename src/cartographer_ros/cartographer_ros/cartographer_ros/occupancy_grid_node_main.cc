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

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/synchronization/mutex.h"
#include "cairo/cairo.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "gflags/gflags.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
// occupancy_grid_node 是把 Cartographer 子图转换成 ROS /map 的独立节点。
// cartographer_node 本身发布 submap_list 并提供 submap_query 服务；
// 本节点订阅/调用这些接口，周期性拼接 nav_msgs/OccupancyGrid。
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");
DEFINE_string(occupancy_grid_topic, cartographer_ros::kOccupancyGridTopic,
              "Name of the topic on which the occupancy grid is published.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node {
 public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  // 收到 cartographer_node 发布的 submap_list 后，判断哪些子图需要新增、更新或删除。
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
  // 定时把当前缓存的所有子图纹理绘制成一张 OccupancyGrid，并发布到 /map。
  void DrawAndPublish(const ::ros::WallTimerEvent& timer_event);

  ::ros::NodeHandle node_handle_;
  // /map 栅格分辨率，默认 0.05 m/cell。它不改变 Cartographer 内部子图分辨率，
  // 只是控制最终发布给 ROS 的 OccupancyGrid 采样尺度。
  const double resolution_;

  absl::Mutex mutex_;
  // submap_query 服务客户端，用于按需向 cartographer_node 拉取子图纹理。
  ::ros::ServiceClient client_ GUARDED_BY(mutex_);
  // 订阅 submap_list 元数据。这个话题告诉本节点“有哪些子图、版本是多少、
  // 在 map_frame 下位姿是什么”。
  ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
  // 发布最终 /map。latched=true 表示新订阅者一连接就能拿到最近一张地图。
  ::ros::Publisher occupancy_grid_publisher_ GUARDED_BY(mutex_);
  // 缓存每个子图的纹理、位姿和版本。只有版本变化时才重新拉取纹理。
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  ::ros::WallTimer occupancy_grid_publisher_timer_;
  // 记录最近 submap_list 的 frame_id 和时间戳，通常 frame_id 是 map。
  std::string last_frame_id_;
  ros::Time last_timestamp_;
};

Node::Node(const double resolution, const double publish_period_sec)
    : resolution_(resolution),
      // 连接 cartographer_node 提供的 submap_query 服务。
      client_(node_handle_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
          kSubmapQueryServiceName)),
      // submap_list 是轻量元数据；真正栅格纹理在 HandleSubmapList 中按需查询。
      submap_list_subscriber_(node_handle_.subscribe(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize,
          boost::function<void(
              const cartographer_ros_msgs::SubmapList::ConstPtr&)>(
              [this](const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
                HandleSubmapList(msg);
              }))),
      occupancy_grid_publisher_(
          node_handle_.advertise<::nav_msgs::OccupancyGrid>(
              FLAGS_occupancy_grid_topic, kLatestOnlyPublisherQueueSize,
              true /* latched */)),
      // 周期性重绘 /map。WallTimer 不依赖仿真时间，即使 /clock 暂停也按墙钟触发。
      occupancy_grid_publisher_timer_(
          node_handle_.createWallTimer(::ros::WallDuration(publish_period_sec),
                                       &Node::DrawAndPublish, this)) {}

void Node::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  absl::MutexLock locker(&mutex_);

  // We do not do any work if nobody listens.
  // 如果没有 RViz 或其他节点订阅 /map，就不拉取子图纹理，节省服务调用和绘图开销。
  if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
    return;
  }

  // Keep track of submap IDs that don't appear in the message anymore.
  // 先假设缓存中的子图都要删除，随后在最新 submap_list 中出现的再移除删除集合。
  std::set<SubmapId> submap_ids_to_delete;
  for (const auto& pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }

  for (const auto& submap_msg : msg->submap) {
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    submap_ids_to_delete.erase(id);
    // frozen/unfrozen 过滤用于控制是否把加载的旧地图或当前活动地图画进 /map。
    if ((submap_msg.is_frozen && !FLAGS_include_frozen_submaps) ||
        (!submap_msg.is_frozen && !FLAGS_include_unfrozen_submaps)) {
      continue;
    }
    SubmapSlice& submap_slice = submap_slices_[id];
    // submap pose 是该子图在 map_frame 下的位置；绘制全局 /map 时靠它把各子图
    // 放到同一张图上。
    submap_slice.pose = ToRigid3d(submap_msg.pose);
    submap_slice.metadata_version = submap_msg.submap_version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_msg.submap_version) {
      // 版本没变说明纹理没更新，复用缓存即可。
      continue;
    }

    // 子图版本变化或首次出现时，通过服务拉取纹理数据。
    auto fetched_textures =
        ::cartographer_ros::FetchSubmapTextures(id, &client_);
    if (fetched_textures == nullptr) {
      continue;
    }
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    // Cartographer 子图纹理可能有多个分辨率。ROS /map 选第一张最高分辨率纹理，
    // 再按 FLAGS_resolution 绘制到最终 OccupancyGrid。
    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  }

  // Delete all submaps that didn't appear in the message.
  // pose graph 删除或轨迹状态变化后，旧子图如果不再出现在 submap_list，就从缓存移除。
  for (const auto& id : submap_ids_to_delete) {
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

void Node::DrawAndPublish(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock locker(&mutex_);
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    // 尚未收到任何子图或 frame_id 时无法生成 /map。
    return;
  }
  // PaintSubmapSlices 把所有子图按各自 pose 投影到同一张画布上；
  // CreateOccupancyGridMsg 再把画布转换为 nav_msgs/OccupancyGrid。
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  occupancy_grid_publisher_.publish(*msg_ptr);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(FLAGS_include_frozen_submaps || FLAGS_include_unfrozen_submaps)
      << "Ignoring both frozen and unfrozen submaps makes no sense.";

  // 该节点名通常显示为 cartographer_occupancy_grid_node，发布的话题默认是 /map。
  // RViz 的 Map display 和本项目 local_grid 旁路显示都可以同时存在：
  // /map 是全局 Cartographer 地图，/local_occupancy_grid 是小车中心局部地图。
  ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Node node(FLAGS_resolution, FLAGS_publish_period_sec);

  ::ros::spin();
  ::ros::shutdown();
}
