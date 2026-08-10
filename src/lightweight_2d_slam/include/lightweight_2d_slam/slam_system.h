#pragma once

#include <atomic>
#include <cstddef>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "lightweight_2d_slam/occupancy_grid.h"
#include "lightweight_2d_slam/loop_closure_gate.h"
#include "lightweight_2d_slam/pose_graph.h"
#include "lightweight_2d_slam/scan_matcher.h"
#include "lightweight_2d_slam/submap_2d.h"
#include "lightweight_2d_slam/types.h"

namespace lightweight_2d_slam {

struct SlamOptions {
  std::string scan_topic = "/scan";
  std::string odom_topic = "/scout_mini_velocity_controller/odom";
  std::string map_topic = "/map";
  std::string local_grid_topic = "/local_occupancy_grid";
  std::string tracked_pose_topic = "/tracked_pose";
  std::string path_topic = "/lightweight_slam/path";
  std::string local_path_topic = "/lightweight_slam/local_path";
  std::string submap_path_topic = "/lightweight_slam/submap_path";
  std::string loop_edges_topic = "/lightweight_slam/loop_edges";
  std::string diagnostics_topic = "/slam_diagnostics";
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";

  bool publish_odom_tf = true;
  double transform_timeout = 0.10;
  double min_range = 0.30;
  double max_range = 25.0;
  int point_stride = 3;

  double grid_resolution = 0.05;
  double active_grid_size = 30.0;
  int active_keyframes = 60;
  bool active_grid_insert_free_space = true;
  bool submap_insert_free_space = false;
  int submap_keyframes = 50;
  // When enabled, submaps overlap and become explicit pose-graph variables.
  // Keyframes are inserted into both active submaps and constrained to each
  // submap in its immutable local coordinate system.
  bool use_rigid_submap_pose_graph = false;
  int submap_overlap_keyframes = 25;
  double node_submap_translation_weight = 20.0;
  double node_submap_rotation_weight = 30.0;
  double keyframe_translation = 0.10;
  double keyframe_rotation = 0.035;
  double keyframe_time = 0.75;
  double keyframe_min_translation_for_time = 0.02;
  double keyframe_min_rotation_for_time = 0.01;
  double min_match_score = 0.45;
  double max_match_rms = 0.25;

  int optimize_every_n_keyframes = 0;
  double scan_constraint_translation_weight = 20.0;
  double scan_constraint_rotation_weight = 30.0;
  bool quality_weighted_constraints = false;
  double min_constraint_quality = 0.25;
  bool robust_scan_constraints = false;
  // The residuals are already multiplied by the information weights.  A
  // scale below one would robustify sub-centimetre errors for the default
  // weights (20/30), so keep the optional robust kernel at a useful scale.
  double scan_constraint_huber_scale = 1.0;
  double odom_constraint_translation_weight = 0.0;
  double odom_constraint_rotation_weight = 0.0;
  bool robust_odom_constraints = false;
  double odom_constraint_huber_scale = 1.0;

  bool enable_loop_closure = true;
  int loop_min_separation = 35;
  int loop_min_submap_separation = 2;
  int loop_search_every_n_keyframes = 5;
  int loop_top_k_candidates = 5;
  int loop_candidates_per_submap = 1;
  int loop_descriptor_angular_bins = 60;
  int loop_descriptor_radial_scales = 1;
  double loop_descriptor_min_similarity = 0.68;
  double loop_min_score_margin = 0.03;
  double loop_linear_search_window = 0.80;
  double loop_angular_search_window = 0.70;
  double loop_linear_step = 0.10;
  double loop_angular_step = 0.05;
  int loop_max_coarse_points = 120;
  bool loop_use_multiresolution_search = false;
  double loop_global_linear_search_window = 3.0;
  double loop_global_angular_search_window = 0.70;
  double loop_global_linear_step = 0.30;
  double loop_global_angular_step = 0.10;
  int loop_global_max_coarse_points = 80;
  double loop_fine_linear_search_window = 0.40;
  double loop_fine_angular_search_window = 0.20;
  double loop_min_match_score = 0.75;
  double loop_max_match_rms = 0.08;
  double loop_min_translation_observability = 0.08;
  double loop_unobservable_max_correction = 0.50;
  double loop_max_correction_translation = 30.0;
  double loop_max_correction_rotation = 1.20;
  bool loop_reject_low_observability_large_correction = false;
  int loop_min_confirmations = 2;
  double loop_confirmation_translation_tolerance = 0.15;
  double loop_confirmation_rotation_tolerance = 0.12;
  int loop_min_keyframes_between_accepts = 30;
  double loop_translation_weight = 25.0;
  double loop_rotation_weight = 35.0;
  bool loop_use_switchable_constraints = false;
  double loop_switch_prior_weight = 1.0;
  // Keep both translation components in a loop residual by default.  A
  // weak-direction loop must not hide a large global correction from the
  // switchable-constraint residual.
  bool loop_use_anisotropic_constraints = false;
  int loop_sequence_keyframes = 3;
  int loop_sequence_stride = 5;
  int loop_min_sequence_support = 0;
  double loop_sequence_linear_window = 0.35;
  double loop_sequence_angular_window = 0.25;
  double loop_sequence_correction_tolerance = 0.35;
  double loop_sequence_yaw_tolerance = 0.12;
  bool loop_require_se2_consensus = false;
  int loop_consensus_min_submaps = 2;
  double loop_consensus_translation_tolerance = 0.60;
  double loop_consensus_rotation_tolerance = 0.15;
  bool loop_log_candidate_evaluations = false;

  // Interpolate optimized graph poses before publishing the map and TF. A
  // zero duration preserves immediate pose-graph updates.
  double global_correction_blend_duration = 0.0;

  double map_publish_period = 1.0;
  int max_global_grid_cells = 4000000;
  int global_map_point_stride = 2;
  double global_map_hit_connection_max_distance = 0.0;
  // Preserve the historical grid model unless an experiment explicitly
  // selects a calibrated global-map-only inverse sensor model.
  double global_map_hit_probability =
      1.0 / (1.0 + 0.4274149319487267);  // sigmoid(0.85)
  double global_map_miss_probability =
      1.0 / (1.0 + 1.4918246976412703);  // sigmoid(-0.40)
  double local_grid_size = 5.0;
  double local_grid_max_range = 5.0;

  ScanMatcherOptions scan_matcher;
};

class SlamSystem {
 public:
  SlamSystem(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
  ~SlamSystem();

 private:
  struct TimedPose {
    ros::Time stamp;
    Pose2D pose;
  };

  struct MatchDiagnostics {
    bool initialized = false;
    bool tracking_ok = false;
    double coarse_score = 0.0;
    double residual_rms = 0.0;
    double score_margin = 0.0;
    double translation_observability = 0.0;
    bool oad_triggered = false;
    bool oad_expanded = false;
    double oad_search_window = 0.0;
    double scan_quality = 0.0;
    int correspondences = 0;
    double processing_ms = 0.0;
    std::size_t dropped_scans = 0;
  };

  struct LoopClosureRequest {
    std::size_t current_id = 0;
    std::size_t current_submap = 0;
    Pose2D current_local_pose;
    PointCloud2D current_points;
    std::vector<float> current_descriptor;
    std::vector<Keyframe> sequence_keyframes;
    std::vector<std::shared_ptr<const Submap2D>> frozen_submaps;
  };

  struct MapBuildRequest {
    std::vector<Keyframe> keyframes;
    std::vector<Pose2D> optimized_poses;
    std::vector<Pose2D> submap_local_poses;
    std::vector<Pose2D> optimized_submap_poses;
    std::vector<std::shared_ptr<const SubmapTexture>> submap_textures;
    ros::Time stamp;
    std::size_t keyframe_count = 0;
    std::size_t loop_count = 0;
    std::size_t pose_version = 0;
    double final_cost = -1.0;
  };

  void LoadOptions();
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& message);
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& message);
  void ProcessPendingScans();
  void ProcessScan(const sensor_msgs::LaserScan::ConstPtr& message,
                   const Pose2D& odom_pose);
  bool LookupOdomPose(const ros::Time& stamp, Pose2D* pose) const;
  bool ConvertScan(const sensor_msgs::LaserScan& scan,
                   PointCloud2D* tracking_points,
                   PointCloud2D* mapping_points) const;

  void Initialize(const ros::Time& stamp, const Pose2D& odom_pose,
                  const PointCloud2D& tracking_points,
                  const PointCloud2D& mapping_points);
  bool ShouldCreateKeyframe(const ros::Time& stamp,
                            const Pose2D& pose,
                            const Pose2D& odom_pose) const;
  void AddKeyframe(const ros::Time& stamp, const Pose2D& local_pose,
                   const Pose2D& odom_pose,
                   const PointCloud2D& tracking_points,
                   const PointCloud2D& mapping_points, double scan_quality);
  void RebuildActiveGrid(const Pose2D& center);
  void AddToActiveSubmaps(const Keyframe& keyframe,
                          const PointCloud2D& mapping_points,
                          const Pose2D& graph_initial_pose,
                          const Pose2D& published_initial_pose);
  bool ScheduleLoopClosure(std::size_t current_id);
  void LoopClosureWorker();
  void EvaluateLoopClosure(LoopClosureRequest request);
  void MapWorkerLoop();
  void RequestMapRebuild(const PoseGraphStatus& graph);
  void UpdatePublishedGraphPoses(const ros::Time& stamp);

  Pose2D CurrentMapPose() const;
  void PublishPoseAndTf(const ros::Time& stamp, const Pose2D& odom_pose);
  void PublishLocalGrid(const ros::Time& stamp,
                        const PointCloud2D& points);
  void PublishDiagnostics(const ros::Time& stamp);
  void MapTimerCallback(const ros::TimerEvent& event);
  bool BuildGlobalMap(const std::vector<Keyframe>& keyframes,
                      const std::vector<Pose2D>& optimized_poses,
                      const std::vector<Pose2D>& submap_local_poses,
                      const std::vector<Pose2D>& optimized_submap_poses,
                      const std::vector<std::shared_ptr<const SubmapTexture>>&
                          submap_textures,
                      const ros::Time& stamp,
                      nav_msgs::OccupancyGrid* message) const;
  nav_msgs::Path BuildPath(const std::vector<Keyframe>& keyframes,
                           const std::vector<Pose2D>& optimized_poses,
                           const ros::Time& stamp) const;
  nav_msgs::Path BuildSubmapPath(
      const std::vector<Keyframe>& keyframes,
      const std::vector<Pose2D>& optimized_poses,
      const std::vector<Pose2D>& optimized_submap_poses,
      const ros::Time& stamp) const;
  bool GetMapService(nav_msgs::GetMap::Request& request,
                     nav_msgs::GetMap::Response& response);

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;
  SlamOptions options_;
  std::unique_ptr<ScanMatcher> scan_matcher_;
  std::unique_ptr<ScanMatcher> loop_scan_matcher_;
  std::unique_ptr<ScanMatcher> coarse_loop_scan_matcher_;
  PoseGraph pose_graph_;
  LoopClosureGate loop_closure_gate_;

  std::mutex loop_mutex_;
  std::condition_variable loop_condition_;
  LoopClosureRequest loop_request_;
  bool loop_request_ready_ = false;
  bool stop_loop_worker_ = false;
  std::thread loop_worker_;
  std::atomic<bool> loop_worker_busy_{false};
  std::atomic<std::size_t> loop_searches_completed_{0};
  std::atomic<std::size_t> loop_searches_skipped_{0};
  std::atomic<std::size_t> loop_candidates_rejected_{0};
  std::atomic<std::size_t> loop_candidates_evaluated_{0};
  std::atomic<std::size_t> loop_no_candidate_rejections_{0};
  std::atomic<std::size_t> loop_match_rejections_{0};
  std::atomic<std::size_t> loop_score_rejections_{0};
  std::atomic<std::size_t> loop_rms_rejections_{0};
  std::atomic<std::size_t> loop_anisotropic_candidates_{0};
  std::atomic<std::size_t> loop_anisotropic_constraints_{0};
  std::atomic<std::size_t> loop_low_observability_rejections_{0};
  std::atomic<std::size_t> loop_sequence_rejections_{0};
  std::atomic<std::size_t> loop_correction_rejections_{0};
  std::atomic<std::size_t> loop_ambiguity_rejections_{0};
  std::atomic<std::size_t> loop_confirmation_rejections_{0};
  std::atomic<std::size_t> loop_consensus_rejections_{0};
  std::atomic<std::size_t> loop_consensus_accepts_{0};
  std::atomic<int> loop_last_consensus_support_{0};

  std::mutex map_mutex_;
  std::condition_variable map_condition_;
  MapBuildRequest map_request_;
  bool map_request_ready_ = false;
  bool stop_map_worker_ = false;
  std::thread map_worker_;
  std::atomic<bool> map_worker_busy_{false};
  std::atomic<std::size_t> map_rebuild_failures_{0};
  std::atomic<double> last_map_build_seconds_{0.0};
  ros::WallTime map_retry_not_before_;

  ros::Subscriber scan_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Publisher map_publisher_;
  ros::Publisher local_grid_publisher_;
  ros::Publisher pose_publisher_;
  ros::Publisher path_publisher_;
  ros::Publisher local_path_publisher_;
  ros::Publisher submap_path_publisher_;
  ros::Publisher loop_edges_publisher_;
  ros::Publisher diagnostics_publisher_;
  ros::ServiceServer map_service_;
  ros::Timer map_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  mutable std::mutex odom_mutex_;
  std::deque<TimedPose> odom_buffer_;
  std::deque<sensor_msgs::LaserScan::ConstPtr> pending_scans_;
  mutable std::mutex keyframes_mutex_;
  mutable std::mutex published_poses_mutex_;

  bool initialized_ = false;
  Pose2D current_local_pose_;
  Pose2D last_scan_odom_pose_;
  std::vector<Keyframe> keyframes_;
  std::vector<Pose2D> published_graph_poses_;
  std::vector<Pose2D> correction_blend_start_poses_;
  std::vector<Pose2D> correction_blend_target_poses_;
  std::vector<Pose2D> published_submap_poses_;
  std::vector<Pose2D> correction_blend_start_submap_poses_;
  std::vector<Pose2D> correction_blend_target_submap_poses_;
  std::size_t blended_pose_graph_version_ = 0;
  double correction_blend_start_time_ = 0.0;
  bool global_correction_blending_ = false;
  std::atomic<std::size_t> published_pose_revision_{0};
  std::atomic<double> global_correction_blend_progress_{1.0};
  ProbabilityGrid active_grid_;
  std::vector<std::unique_ptr<Submap2D>> active_submaps_;
  std::vector<std::shared_ptr<const Submap2D>> frozen_submaps_;
  std::vector<Pose2D> submap_local_poses_;
  std::vector<std::pair<std::size_t, std::size_t>> accepted_loop_edges_;
  nav_msgs::OccupancyGrid latest_map_;
  nav_msgs::Path latest_path_;
  nav_msgs::Path latest_local_path_;
  nav_msgs::Path latest_submap_path_;
  MatchDiagnostics diagnostics_;
  std::size_t last_map_request_keyframe_count_ = 0;
  std::size_t last_map_request_loop_count_ = 0;
  std::size_t last_map_request_pose_version_ = 0;
  double last_map_request_final_cost_ = -1.0;
};

}  // namespace lightweight_2d_slam
