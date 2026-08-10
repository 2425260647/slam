#include "lightweight_2d_slam/slam_system.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace lightweight_2d_slam {
namespace {

geometry_msgs::Quaternion YawQuaternion(double yaw) {
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

geometry_msgs::Pose PoseMessage(const Pose2D& pose) {
  geometry_msgs::Pose message;
  message.position.x = pose.x;
  message.position.y = pose.y;
  message.orientation = YawQuaternion(pose.yaw);
  return message;
}

geometry_msgs::TransformStamped TransformMessage(
    const ros::Time& stamp, const std::string& parent,
    const std::string& child, const Pose2D& pose) {
  geometry_msgs::TransformStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = parent;
  message.child_frame_id = child;
  message.transform.translation.x = pose.x;
  message.transform.translation.y = pose.y;
  message.transform.rotation = YawQuaternion(pose.yaw);
  return message;
}

diagnostic_msgs::KeyValue DiagnosticValue(const std::string& key,
                                          const std::string& value) {
  diagnostic_msgs::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

template <typename T>
std::string ToString(const T& value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

}  // namespace

SlamSystem::SlamSystem(ros::NodeHandle node_handle,
                       ros::NodeHandle private_node_handle)
    : node_handle_(std::move(node_handle)),
      private_node_handle_(std::move(private_node_handle)),
      loop_closure_gate_(LoopClosureGateOptions{}),
      tf_listener_(tf_buffer_) {
  LoadOptions();
  scan_matcher_.reset(new ScanMatcher(options_.scan_matcher));
  ScanMatcherOptions loop_matcher_options = options_.scan_matcher;
  loop_matcher_options.linear_step = options_.loop_linear_step;
  loop_matcher_options.angular_step = options_.loop_angular_step;
  loop_matcher_options.max_coarse_points = options_.loop_max_coarse_points;
  loop_matcher_options.use_likelihood_field = true;
  loop_matcher_options.enable_fine_correlative = true;
  loop_matcher_options.fine_candidate_count = 3;
  loop_matcher_options.fine_linear_step =
      std::min(0.05, 0.5 * options_.loop_linear_step);
  loop_matcher_options.fine_angular_window = options_.loop_angular_step;
  loop_matcher_options.fine_angular_step =
      std::min(0.025, 0.5 * options_.loop_angular_step);
  loop_matcher_options.use_likelihood_refinement = true;
  loop_matcher_options.prior_translation_weight = 0.25;
  loop_matcher_options.prior_rotation_weight = 1.0;
  loop_scan_matcher_.reset(new ScanMatcher(loop_matcher_options));
  ScanMatcherOptions coarse_loop_matcher_options = loop_matcher_options;
  coarse_loop_matcher_options.linear_step =
      options_.loop_global_linear_step;
  coarse_loop_matcher_options.angular_step =
      options_.loop_global_angular_step;
  coarse_loop_matcher_options.max_coarse_points =
      options_.loop_global_max_coarse_points;
  coarse_loop_matcher_options.enable_fine_correlative = false;
  coarse_loop_matcher_options.max_refinement_iterations = 0;
  coarse_loop_matcher_options.use_likelihood_refinement = false;
  coarse_loop_matcher_options.use_hybrid_refinement = false;
  coarse_loop_matcher_options.correlative_translation_cost_weight = 0.0;
  coarse_loop_matcher_options.correlative_rotation_cost_weight = 0.0;
  coarse_loop_matcher_options.prior_translation_weight = 0.0;
  coarse_loop_matcher_options.prior_rotation_weight = 0.0;
  coarse_loop_scan_matcher_.reset(
      new ScanMatcher(coarse_loop_matcher_options));
  loop_closure_gate_ = LoopClosureGate({
      options_.loop_min_confirmations,
      options_.loop_confirmation_translation_tolerance,
      options_.loop_confirmation_rotation_tolerance,
      options_.loop_min_keyframes_between_accepts});
  loop_worker_ = std::thread(&SlamSystem::LoopClosureWorker, this);
  map_worker_ = std::thread(&SlamSystem::MapWorkerLoop, this);

  map_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(
      options_.map_topic, 1, true);
  local_grid_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(
      options_.local_grid_topic, 1, false);
  pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(
      options_.tracked_pose_topic, 10, false);
  path_publisher_ = node_handle_.advertise<nav_msgs::Path>(
      options_.path_topic, 1, true);
  local_path_publisher_ = node_handle_.advertise<nav_msgs::Path>(
      options_.local_path_topic, 1, true);
  submap_path_publisher_ = node_handle_.advertise<nav_msgs::Path>(
      options_.submap_path_topic, 1, true);
  loop_edges_publisher_ = node_handle_.advertise<geometry_msgs::PoseArray>(
      options_.loop_edges_topic, 1, true);
  diagnostics_publisher_ = node_handle_.advertise<diagnostic_msgs::DiagnosticArray>(
      options_.diagnostics_topic, 10, false);
  map_service_ = node_handle_.advertiseService(
      "dynamic_map", &SlamSystem::GetMapService, this);

  odom_subscriber_ = node_handle_.subscribe(
      options_.odom_topic, 200, &SlamSystem::OdomCallback, this);
  scan_subscriber_ = node_handle_.subscribe(
      options_.scan_topic, 10, &SlamSystem::ScanCallback, this);
  map_timer_ = node_handle_.createTimer(
      ros::Duration(options_.map_publish_period),
      &SlamSystem::MapTimerCallback, this);

  latest_path_.header.frame_id = options_.map_frame;
  latest_local_path_.header.frame_id = options_.map_frame;
  latest_submap_path_.header.frame_id = options_.map_frame;
  ROS_INFO_STREAM("lightweight_2d_slam started: scan=" << options_.scan_topic
                  << " odom=" << options_.odom_topic
                  << " map=" << options_.map_topic);
}

SlamSystem::~SlamSystem() {
  {
    std::lock_guard<std::mutex> lock(loop_mutex_);
    stop_loop_worker_ = true;
  }
  loop_condition_.notify_all();
  if (loop_worker_.joinable()) loop_worker_.join();
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    stop_map_worker_ = true;
  }
  map_condition_.notify_all();
  if (map_worker_.joinable()) map_worker_.join();
}

void SlamSystem::LoadOptions() {
  private_node_handle_.param("scan_topic", options_.scan_topic,
                             options_.scan_topic);
  private_node_handle_.param("odom_topic", options_.odom_topic,
                             options_.odom_topic);
  private_node_handle_.param("map_topic", options_.map_topic,
                             options_.map_topic);
  private_node_handle_.param("local_grid_topic", options_.local_grid_topic,
                             options_.local_grid_topic);
  private_node_handle_.param("tracked_pose_topic", options_.tracked_pose_topic,
                             options_.tracked_pose_topic);
  private_node_handle_.param("path_topic", options_.path_topic,
                             options_.path_topic);
  private_node_handle_.param("local_path_topic", options_.local_path_topic,
                             options_.local_path_topic);
  private_node_handle_.param("submap_path_topic", options_.submap_path_topic,
                             options_.submap_path_topic);
  private_node_handle_.param("loop_edges_topic", options_.loop_edges_topic,
                             options_.loop_edges_topic);
  private_node_handle_.param("diagnostics_topic", options_.diagnostics_topic,
                             options_.diagnostics_topic);
  private_node_handle_.param("map_frame", options_.map_frame,
                             options_.map_frame);
  private_node_handle_.param("odom_frame", options_.odom_frame,
                             options_.odom_frame);
  private_node_handle_.param("base_frame", options_.base_frame,
                             options_.base_frame);
  private_node_handle_.param("publish_odom_tf", options_.publish_odom_tf,
                             options_.publish_odom_tf);
  private_node_handle_.param("transform_timeout", options_.transform_timeout,
                             options_.transform_timeout);
  private_node_handle_.param("min_range", options_.min_range,
                             options_.min_range);
  private_node_handle_.param("max_range", options_.max_range,
                             options_.max_range);
  private_node_handle_.param("point_stride", options_.point_stride,
                             options_.point_stride);
  private_node_handle_.param("grid_resolution", options_.grid_resolution,
                             options_.grid_resolution);
  private_node_handle_.param("active_grid_size", options_.active_grid_size,
                             options_.active_grid_size);
  private_node_handle_.param("active_keyframes", options_.active_keyframes,
                             options_.active_keyframes);
  private_node_handle_.param("active_grid_insert_free_space",
                             options_.active_grid_insert_free_space,
                             options_.active_grid_insert_free_space);
  private_node_handle_.param("submap_insert_free_space",
                             options_.submap_insert_free_space,
                             options_.submap_insert_free_space);
  private_node_handle_.param("submap_keyframes", options_.submap_keyframes,
                             options_.submap_keyframes);
  private_node_handle_.param("use_rigid_submap_pose_graph",
                             options_.use_rigid_submap_pose_graph,
                             options_.use_rigid_submap_pose_graph);
  private_node_handle_.param("submap_overlap_keyframes",
                             options_.submap_overlap_keyframes,
                             options_.submap_overlap_keyframes);
  private_node_handle_.param("node_submap_translation_weight",
                             options_.node_submap_translation_weight,
                             options_.node_submap_translation_weight);
  private_node_handle_.param("node_submap_rotation_weight",
                             options_.node_submap_rotation_weight,
                             options_.node_submap_rotation_weight);
  private_node_handle_.param("keyframe_translation",
                             options_.keyframe_translation,
                             options_.keyframe_translation);
  private_node_handle_.param("keyframe_rotation", options_.keyframe_rotation,
                             options_.keyframe_rotation);
  private_node_handle_.param("keyframe_time", options_.keyframe_time,
                             options_.keyframe_time);
  private_node_handle_.param("keyframe_min_translation_for_time",
                             options_.keyframe_min_translation_for_time,
                             options_.keyframe_min_translation_for_time);
  private_node_handle_.param("keyframe_min_rotation_for_time",
                             options_.keyframe_min_rotation_for_time,
                             options_.keyframe_min_rotation_for_time);
  private_node_handle_.param("min_match_score", options_.min_match_score,
                             options_.min_match_score);
  private_node_handle_.param("max_match_rms", options_.max_match_rms,
                             options_.max_match_rms);
  private_node_handle_.param("optimize_every_n_keyframes",
                             options_.optimize_every_n_keyframes,
                             options_.optimize_every_n_keyframes);
  private_node_handle_.param("scan_constraint_translation_weight",
                             options_.scan_constraint_translation_weight,
                             options_.scan_constraint_translation_weight);
  private_node_handle_.param("scan_constraint_rotation_weight",
                             options_.scan_constraint_rotation_weight,
                             options_.scan_constraint_rotation_weight);
  private_node_handle_.param("quality_weighted_constraints",
                             options_.quality_weighted_constraints,
                             options_.quality_weighted_constraints);
  private_node_handle_.param("min_constraint_quality",
                             options_.min_constraint_quality,
                             options_.min_constraint_quality);
  private_node_handle_.param("robust_scan_constraints",
                             options_.robust_scan_constraints,
                             options_.robust_scan_constraints);
  private_node_handle_.param("scan_constraint_huber_scale",
                             options_.scan_constraint_huber_scale,
                             options_.scan_constraint_huber_scale);
  private_node_handle_.param("odom_constraint_translation_weight",
                             options_.odom_constraint_translation_weight,
                             options_.odom_constraint_translation_weight);
  private_node_handle_.param("odom_constraint_rotation_weight",
                             options_.odom_constraint_rotation_weight,
                             options_.odom_constraint_rotation_weight);
  private_node_handle_.param("robust_odom_constraints",
                             options_.robust_odom_constraints,
                             options_.robust_odom_constraints);
  private_node_handle_.param("odom_constraint_huber_scale",
                             options_.odom_constraint_huber_scale,
                             options_.odom_constraint_huber_scale);
  private_node_handle_.param("enable_loop_closure",
                             options_.enable_loop_closure,
                             options_.enable_loop_closure);
  private_node_handle_.param("loop_min_separation",
                             options_.loop_min_separation,
                             options_.loop_min_separation);
  private_node_handle_.param("loop_min_submap_separation",
                             options_.loop_min_submap_separation,
                             options_.loop_min_submap_separation);
  private_node_handle_.param("loop_search_every_n_keyframes",
                             options_.loop_search_every_n_keyframes,
                             options_.loop_search_every_n_keyframes);
  private_node_handle_.param("loop_top_k_candidates",
                             options_.loop_top_k_candidates,
                             options_.loop_top_k_candidates);
  private_node_handle_.param("loop_candidates_per_submap",
                             options_.loop_candidates_per_submap,
                             options_.loop_candidates_per_submap);
  private_node_handle_.param("loop_descriptor_angular_bins",
                             options_.loop_descriptor_angular_bins,
                             options_.loop_descriptor_angular_bins);
  private_node_handle_.param("loop_descriptor_radial_scales",
                             options_.loop_descriptor_radial_scales,
                             options_.loop_descriptor_radial_scales);
  private_node_handle_.param("loop_descriptor_min_similarity",
                             options_.loop_descriptor_min_similarity,
                             options_.loop_descriptor_min_similarity);
  private_node_handle_.param("loop_min_score_margin",
                             options_.loop_min_score_margin,
                             options_.loop_min_score_margin);
  private_node_handle_.param("loop_linear_search_window",
                             options_.loop_linear_search_window,
                             options_.loop_linear_search_window);
  private_node_handle_.param("loop_angular_search_window",
                             options_.loop_angular_search_window,
                             options_.loop_angular_search_window);
  private_node_handle_.param("loop_linear_step", options_.loop_linear_step,
                             options_.loop_linear_step);
  private_node_handle_.param("loop_angular_step", options_.loop_angular_step,
                             options_.loop_angular_step);
  private_node_handle_.param("loop_max_coarse_points",
                             options_.loop_max_coarse_points,
                             options_.loop_max_coarse_points);
  private_node_handle_.param("loop_use_multiresolution_search",
                             options_.loop_use_multiresolution_search,
                             options_.loop_use_multiresolution_search);
  private_node_handle_.param("loop_global_linear_search_window",
                             options_.loop_global_linear_search_window,
                             options_.loop_global_linear_search_window);
  private_node_handle_.param("loop_global_angular_search_window",
                             options_.loop_global_angular_search_window,
                             options_.loop_global_angular_search_window);
  private_node_handle_.param("loop_global_linear_step",
                             options_.loop_global_linear_step,
                             options_.loop_global_linear_step);
  private_node_handle_.param("loop_global_angular_step",
                             options_.loop_global_angular_step,
                             options_.loop_global_angular_step);
  private_node_handle_.param("loop_global_max_coarse_points",
                             options_.loop_global_max_coarse_points,
                             options_.loop_global_max_coarse_points);
  private_node_handle_.param("loop_fine_linear_search_window",
                             options_.loop_fine_linear_search_window,
                             options_.loop_fine_linear_search_window);
  private_node_handle_.param("loop_fine_angular_search_window",
                             options_.loop_fine_angular_search_window,
                             options_.loop_fine_angular_search_window);
  private_node_handle_.param("loop_min_match_score",
                             options_.loop_min_match_score,
                             options_.loop_min_match_score);
  private_node_handle_.param("loop_max_match_rms",
                             options_.loop_max_match_rms,
                             options_.loop_max_match_rms);
  private_node_handle_.param("loop_min_translation_observability",
                             options_.loop_min_translation_observability,
                             options_.loop_min_translation_observability);
  private_node_handle_.param("loop_unobservable_max_correction",
                             options_.loop_unobservable_max_correction,
                             options_.loop_unobservable_max_correction);
  private_node_handle_.param("loop_max_correction_translation",
                             options_.loop_max_correction_translation,
                             options_.loop_max_correction_translation);
  private_node_handle_.param("loop_max_correction_rotation",
                             options_.loop_max_correction_rotation,
                             options_.loop_max_correction_rotation);
  private_node_handle_.param(
      "loop_reject_low_observability_large_correction",
      options_.loop_reject_low_observability_large_correction,
      options_.loop_reject_low_observability_large_correction);
  private_node_handle_.param("loop_min_confirmations",
                             options_.loop_min_confirmations,
                             options_.loop_min_confirmations);
  private_node_handle_.param("loop_confirmation_translation_tolerance",
                             options_.loop_confirmation_translation_tolerance,
                             options_.loop_confirmation_translation_tolerance);
  private_node_handle_.param("loop_confirmation_rotation_tolerance",
                             options_.loop_confirmation_rotation_tolerance,
                             options_.loop_confirmation_rotation_tolerance);
  private_node_handle_.param("loop_min_keyframes_between_accepts",
                             options_.loop_min_keyframes_between_accepts,
                             options_.loop_min_keyframes_between_accepts);
  private_node_handle_.param("loop_translation_weight",
                             options_.loop_translation_weight,
                             options_.loop_translation_weight);
  private_node_handle_.param("loop_rotation_weight",
                             options_.loop_rotation_weight,
                             options_.loop_rotation_weight);
  private_node_handle_.param("loop_use_switchable_constraints",
                             options_.loop_use_switchable_constraints,
                             options_.loop_use_switchable_constraints);
  private_node_handle_.param("loop_switch_prior_weight",
                             options_.loop_switch_prior_weight,
                             options_.loop_switch_prior_weight);
  private_node_handle_.param("loop_use_anisotropic_constraints",
                             options_.loop_use_anisotropic_constraints,
                             options_.loop_use_anisotropic_constraints);
  private_node_handle_.param("loop_sequence_keyframes",
                             options_.loop_sequence_keyframes,
                             options_.loop_sequence_keyframes);
  private_node_handle_.param("loop_sequence_stride",
                             options_.loop_sequence_stride,
                             options_.loop_sequence_stride);
  private_node_handle_.param("loop_min_sequence_support",
                             options_.loop_min_sequence_support,
                             options_.loop_min_sequence_support);
  private_node_handle_.param("loop_sequence_linear_window",
                             options_.loop_sequence_linear_window,
                             options_.loop_sequence_linear_window);
  private_node_handle_.param("loop_sequence_angular_window",
                             options_.loop_sequence_angular_window,
                             options_.loop_sequence_angular_window);
  private_node_handle_.param("loop_sequence_correction_tolerance",
                             options_.loop_sequence_correction_tolerance,
                             options_.loop_sequence_correction_tolerance);
  private_node_handle_.param("loop_sequence_yaw_tolerance",
                             options_.loop_sequence_yaw_tolerance,
                             options_.loop_sequence_yaw_tolerance);
  private_node_handle_.param("loop_require_se2_consensus",
                             options_.loop_require_se2_consensus,
                             options_.loop_require_se2_consensus);
  private_node_handle_.param("loop_consensus_min_submaps",
                             options_.loop_consensus_min_submaps,
                             options_.loop_consensus_min_submaps);
  private_node_handle_.param("loop_consensus_translation_tolerance",
                             options_.loop_consensus_translation_tolerance,
                             options_.loop_consensus_translation_tolerance);
  private_node_handle_.param("loop_consensus_rotation_tolerance",
                             options_.loop_consensus_rotation_tolerance,
                             options_.loop_consensus_rotation_tolerance);
  private_node_handle_.param("loop_log_candidate_evaluations",
                             options_.loop_log_candidate_evaluations,
                             options_.loop_log_candidate_evaluations);
  private_node_handle_.param("global_correction_blend_duration",
                             options_.global_correction_blend_duration,
                             options_.global_correction_blend_duration);
  private_node_handle_.param("map_publish_period", options_.map_publish_period,
                             options_.map_publish_period);
  private_node_handle_.param("max_global_grid_cells",
                             options_.max_global_grid_cells,
                             options_.max_global_grid_cells);
  private_node_handle_.param("global_map_point_stride",
                             options_.global_map_point_stride,
                             options_.global_map_point_stride);
  private_node_handle_.param(
      "global_map_hit_connection_max_distance",
      options_.global_map_hit_connection_max_distance,
      options_.global_map_hit_connection_max_distance);
  private_node_handle_.param("global_map_hit_probability",
                             options_.global_map_hit_probability,
                             options_.global_map_hit_probability);
  private_node_handle_.param("global_map_miss_probability",
                             options_.global_map_miss_probability,
                             options_.global_map_miss_probability);
  private_node_handle_.param("local_grid_size", options_.local_grid_size,
                             options_.local_grid_size);
  private_node_handle_.param("local_grid_max_range",
                             options_.local_grid_max_range,
                             options_.local_grid_max_range);

  private_node_handle_.param("scan_matcher/linear_search_window",
      options_.scan_matcher.linear_search_window,
      options_.scan_matcher.linear_search_window);
  private_node_handle_.param("scan_matcher/angular_search_window",
      options_.scan_matcher.angular_search_window,
      options_.scan_matcher.angular_search_window);
  private_node_handle_.param("scan_matcher/linear_step",
      options_.scan_matcher.linear_step, options_.scan_matcher.linear_step);
  private_node_handle_.param("scan_matcher/angular_step",
      options_.scan_matcher.angular_step, options_.scan_matcher.angular_step);
  private_node_handle_.param("scan_matcher/use_likelihood_field",
      options_.scan_matcher.use_likelihood_field,
      options_.scan_matcher.use_likelihood_field);
  private_node_handle_.param("scan_matcher/enable_fine_correlative",
      options_.scan_matcher.enable_fine_correlative,
      options_.scan_matcher.enable_fine_correlative);
  private_node_handle_.param("scan_matcher/fine_candidate_count",
      options_.scan_matcher.fine_candidate_count,
      options_.scan_matcher.fine_candidate_count);
  private_node_handle_.param("scan_matcher/fine_linear_step",
      options_.scan_matcher.fine_linear_step,
      options_.scan_matcher.fine_linear_step);
  private_node_handle_.param("scan_matcher/fine_angular_window",
      options_.scan_matcher.fine_angular_window,
      options_.scan_matcher.fine_angular_window);
  private_node_handle_.param("scan_matcher/fine_angular_step",
      options_.scan_matcher.fine_angular_step,
      options_.scan_matcher.fine_angular_step);
  private_node_handle_.param("scan_matcher/endpoint_neighborhood",
      options_.scan_matcher.endpoint_neighborhood,
      options_.scan_matcher.endpoint_neighborhood);
  private_node_handle_.param("scan_matcher/max_coarse_points",
      options_.scan_matcher.max_coarse_points,
      options_.scan_matcher.max_coarse_points);
  private_node_handle_.param("scan_matcher/correlative_translation_cost_weight",
      options_.scan_matcher.correlative_translation_cost_weight,
      options_.scan_matcher.correlative_translation_cost_weight);
  private_node_handle_.param("scan_matcher/correlative_rotation_cost_weight",
      options_.scan_matcher.correlative_rotation_cost_weight,
      options_.scan_matcher.correlative_rotation_cost_weight);
  private_node_handle_.param("scan_matcher/enable_oad_csm",
      options_.scan_matcher.enable_oad_csm,
      options_.scan_matcher.enable_oad_csm);
  private_node_handle_.param("scan_matcher/oad_candidate_count",
      options_.scan_matcher.oad_candidate_count,
      options_.scan_matcher.oad_candidate_count);
  private_node_handle_.param("scan_matcher/oad_min_score_margin",
      options_.scan_matcher.oad_min_score_margin,
      options_.scan_matcher.oad_min_score_margin);
  private_node_handle_.param("scan_matcher/oad_min_translation_observability",
      options_.scan_matcher.oad_min_translation_observability,
      options_.scan_matcher.oad_min_translation_observability);
  private_node_handle_.param("scan_matcher/oad_search_expansion_factor",
      options_.scan_matcher.oad_search_expansion_factor,
      options_.scan_matcher.oad_search_expansion_factor);
  private_node_handle_.param("scan_matcher/oad_max_linear_search_window",
      options_.scan_matcher.oad_max_linear_search_window,
      options_.scan_matcher.oad_max_linear_search_window);
  private_node_handle_.param("scan_matcher/oad_max_angular_search_window",
      options_.scan_matcher.oad_max_angular_search_window,
      options_.scan_matcher.oad_max_angular_search_window);
  private_node_handle_.param("scan_matcher/oad_min_score_gain",
      options_.scan_matcher.oad_min_score_gain,
      options_.scan_matcher.oad_min_score_gain);
  private_node_handle_.param("scan_matcher/line_search_radius_cells",
      options_.scan_matcher.line_search_radius_cells,
      options_.scan_matcher.line_search_radius_cells);
  private_node_handle_.param("scan_matcher/line_min_neighbors",
      options_.scan_matcher.line_min_neighbors,
      options_.scan_matcher.line_min_neighbors);
  private_node_handle_.param("scan_matcher/max_refinement_iterations",
      options_.scan_matcher.max_refinement_iterations,
      options_.scan_matcher.max_refinement_iterations);
  private_node_handle_.param("scan_matcher/use_likelihood_refinement",
      options_.scan_matcher.use_likelihood_refinement,
      options_.scan_matcher.use_likelihood_refinement);
  private_node_handle_.param("scan_matcher/use_hybrid_refinement",
      options_.scan_matcher.use_hybrid_refinement,
      options_.scan_matcher.use_hybrid_refinement);
  private_node_handle_.param("scan_matcher/max_likelihood_refinement_points",
      options_.scan_matcher.max_likelihood_refinement_points,
      options_.scan_matcher.max_likelihood_refinement_points);
  private_node_handle_.param("scan_matcher/likelihood_refinement_weight",
      options_.scan_matcher.likelihood_refinement_weight,
      options_.scan_matcher.likelihood_refinement_weight);
  private_node_handle_.param("scan_matcher/point_line_refinement_weight",
      options_.scan_matcher.point_line_refinement_weight,
      options_.scan_matcher.point_line_refinement_weight);
  private_node_handle_.param("scan_matcher/refinement_huber_scale",
      options_.scan_matcher.refinement_huber_scale,
      options_.scan_matcher.refinement_huber_scale);
  private_node_handle_.param("scan_matcher/min_correspondences",
      options_.scan_matcher.min_correspondences,
      options_.scan_matcher.min_correspondences);
  private_node_handle_.param("scan_matcher/prior_translation_weight",
      options_.scan_matcher.prior_translation_weight,
      options_.scan_matcher.prior_translation_weight);
  private_node_handle_.param("scan_matcher/prior_rotation_weight",
      options_.scan_matcher.prior_rotation_weight,
      options_.scan_matcher.prior_rotation_weight);
  private_node_handle_.param(
      "scan_matcher/use_prediction_translation_prior",
      options_.scan_matcher.use_prediction_translation_prior,
      options_.scan_matcher.use_prediction_translation_prior);
  private_node_handle_.param("scan_matcher/line_max_eigenvalue_ratio",
      options_.scan_matcher.line_max_eigenvalue_ratio,
      options_.scan_matcher.line_max_eigenvalue_ratio);
  private_node_handle_.param("scan_matcher/line_min_eigenvalue",
      options_.scan_matcher.line_min_eigenvalue,
      options_.scan_matcher.line_min_eigenvalue);

  options_.point_stride = std::max(1, options_.point_stride);
  options_.active_keyframes = std::max(2, options_.active_keyframes);
  options_.submap_keyframes = std::max(2, options_.submap_keyframes);
  options_.submap_overlap_keyframes = std::max(
      1, std::min(options_.submap_keyframes - 1,
                  options_.submap_overlap_keyframes));
  options_.node_submap_translation_weight =
      std::max(1e-3, options_.node_submap_translation_weight);
  options_.node_submap_rotation_weight =
      std::max(1e-3, options_.node_submap_rotation_weight);
  options_.loop_min_submap_separation =
      std::max(1, options_.loop_min_submap_separation);
  options_.loop_min_confirmations = std::max(1, options_.loop_min_confirmations);
  options_.loop_top_k_candidates =
      std::max(1, std::min(20, options_.loop_top_k_candidates));
  options_.loop_candidates_per_submap =
      std::max(1, std::min(8, options_.loop_candidates_per_submap));
  options_.loop_descriptor_angular_bins =
      std::max(8, std::min(180, options_.loop_descriptor_angular_bins));
  options_.loop_descriptor_radial_scales =
      std::max(1, std::min(8, options_.loop_descriptor_radial_scales));
  options_.loop_min_score_margin =
      std::max(0.0, options_.loop_min_score_margin);
  options_.loop_min_translation_observability = std::max(
      0.0, std::min(1.0, options_.loop_min_translation_observability));
  options_.loop_unobservable_max_correction =
      std::max(0.0, options_.loop_unobservable_max_correction);
  options_.loop_min_keyframes_between_accepts =
      std::max(0, options_.loop_min_keyframes_between_accepts);
  options_.loop_switch_prior_weight =
      std::max(1e-3, options_.loop_switch_prior_weight);
  options_.loop_sequence_keyframes =
      std::max(0, std::min(8, options_.loop_sequence_keyframes));
  options_.loop_sequence_stride = std::max(1, options_.loop_sequence_stride);
  options_.loop_min_sequence_support = std::max(
      0, std::min(options_.loop_sequence_keyframes,
                  options_.loop_min_sequence_support));
  options_.loop_sequence_linear_window =
      std::max(0.05, options_.loop_sequence_linear_window);
  options_.loop_sequence_angular_window =
      std::max(0.05, options_.loop_sequence_angular_window);
  options_.loop_sequence_correction_tolerance =
      std::max(0.05, options_.loop_sequence_correction_tolerance);
  options_.loop_sequence_yaw_tolerance =
      std::max(0.02, options_.loop_sequence_yaw_tolerance);
  options_.loop_max_coarse_points = std::max(20, options_.loop_max_coarse_points);
  options_.loop_linear_step = std::max(0.01, options_.loop_linear_step);
  options_.loop_angular_step = std::max(0.005, options_.loop_angular_step);
  options_.loop_global_linear_search_window =
      std::max(0.05, options_.loop_global_linear_search_window);
  options_.loop_global_angular_search_window =
      std::max(0.05, options_.loop_global_angular_search_window);
  options_.loop_global_linear_step =
      std::max(0.02, std::min(options_.loop_global_linear_search_window,
                             options_.loop_global_linear_step));
  options_.loop_global_angular_step =
      std::max(0.005, std::min(options_.loop_global_angular_search_window,
                              options_.loop_global_angular_step));
  options_.loop_global_max_coarse_points =
      std::max(20, options_.loop_global_max_coarse_points);
  options_.loop_fine_linear_search_window =
      std::max(0.02, options_.loop_fine_linear_search_window);
  options_.loop_fine_angular_search_window =
      std::max(0.01, options_.loop_fine_angular_search_window);
  options_.loop_consensus_min_submaps =
      std::max(1, std::min(8, options_.loop_consensus_min_submaps));
  options_.loop_consensus_translation_tolerance =
      std::max(0.02, options_.loop_consensus_translation_tolerance);
  options_.loop_consensus_rotation_tolerance =
      std::max(0.01, options_.loop_consensus_rotation_tolerance);
  options_.global_correction_blend_duration =
      std::max(0.0, options_.global_correction_blend_duration);
  options_.global_map_point_stride =
      std::max(1, options_.global_map_point_stride);
  options_.global_map_hit_connection_max_distance =
      std::max(0.0, options_.global_map_hit_connection_max_distance);
  if (!(options_.global_map_hit_probability > 0.5 &&
        options_.global_map_hit_probability < 1.0)) {
    ROS_WARN_STREAM("Invalid global_map_hit_probability="
                    << options_.global_map_hit_probability
                    << "; restoring historical value");
    options_.global_map_hit_probability =
        1.0 / (1.0 + std::exp(-0.85));
  }
  if (!(options_.global_map_miss_probability > 0.0 &&
        options_.global_map_miss_probability < 0.5)) {
    ROS_WARN_STREAM("Invalid global_map_miss_probability="
                    << options_.global_map_miss_probability
                    << "; restoring historical value");
    options_.global_map_miss_probability =
        1.0 / (1.0 + std::exp(0.40));
  }
  options_.map_publish_period = std::max(0.2, options_.map_publish_period);
  options_.scan_matcher.linear_search_window =
      std::max(0.0, options_.scan_matcher.linear_search_window);
  options_.scan_matcher.angular_search_window =
      std::max(0.0, options_.scan_matcher.angular_search_window);
  options_.scan_matcher.linear_step =
      std::max(0.001, options_.scan_matcher.linear_step);
  options_.scan_matcher.angular_step =
      std::max(0.0005, options_.scan_matcher.angular_step);
  options_.scan_matcher.fine_linear_step = std::min(
      options_.scan_matcher.linear_step,
      std::max(0.001, options_.scan_matcher.fine_linear_step));
  options_.scan_matcher.fine_candidate_count = std::max(
      1, std::min(8, options_.scan_matcher.fine_candidate_count));
  options_.scan_matcher.fine_angular_window = std::max(
      0.0, std::min(options_.scan_matcher.angular_search_window,
                    options_.scan_matcher.fine_angular_window));
  options_.scan_matcher.fine_angular_step = std::min(
      options_.scan_matcher.angular_step,
      std::max(0.0005, options_.scan_matcher.fine_angular_step));
  options_.scan_matcher.correlative_translation_cost_weight = std::max(
      0.0, options_.scan_matcher.correlative_translation_cost_weight);
  options_.scan_matcher.correlative_rotation_cost_weight = std::max(
      0.0, options_.scan_matcher.correlative_rotation_cost_weight);
  options_.scan_matcher.oad_candidate_count = std::max(
      2, std::min(12, options_.scan_matcher.oad_candidate_count));
  options_.scan_matcher.oad_min_score_margin = std::max(
      0.0, options_.scan_matcher.oad_min_score_margin);
  options_.scan_matcher.oad_min_translation_observability = std::max(
      0.0, std::min(1.0,
                    options_.scan_matcher.oad_min_translation_observability));
  options_.scan_matcher.oad_search_expansion_factor = std::max(
      1.0, options_.scan_matcher.oad_search_expansion_factor);
  options_.scan_matcher.oad_max_linear_search_window = std::max(
      options_.scan_matcher.linear_search_window,
      options_.scan_matcher.oad_max_linear_search_window);
  options_.scan_matcher.oad_max_angular_search_window = std::max(
      options_.scan_matcher.angular_search_window,
      options_.scan_matcher.oad_max_angular_search_window);
  options_.scan_matcher.oad_min_score_gain = std::max(
      0.0, options_.scan_matcher.oad_min_score_gain);
  options_.scan_matcher.max_likelihood_refinement_points = std::max(
      20, options_.scan_matcher.max_likelihood_refinement_points);
  options_.scan_matcher.likelihood_refinement_weight = std::max(
      1e-3, options_.scan_matcher.likelihood_refinement_weight);
  options_.scan_matcher.point_line_refinement_weight = std::max(
      0.0, options_.scan_matcher.point_line_refinement_weight);
  options_.scan_matcher.refinement_huber_scale = std::max(
      1e-3, options_.scan_matcher.refinement_huber_scale);
  options_.min_constraint_quality = std::max(
      0.05, std::min(1.0, options_.min_constraint_quality));
  options_.scan_constraint_huber_scale = std::max(
      1e-3, options_.scan_constraint_huber_scale);
  options_.odom_constraint_huber_scale = std::max(
      1e-3, options_.odom_constraint_huber_scale);
  options_.scan_matcher.line_max_eigenvalue_ratio = std::max(
      0.01, std::min(0.99, options_.scan_matcher.line_max_eigenvalue_ratio));
  options_.scan_matcher.line_min_eigenvalue = std::max(
      1e-12, options_.scan_matcher.line_min_eigenvalue);
}

void SlamSystem::OdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
  const auto& position = message->pose.pose.position;
  const Pose2D pose{position.x, position.y,
                    tf2::getYaw(message->pose.pose.orientation)};
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_buffer_.push_back({message->header.stamp, pose});
    while (odom_buffer_.size() > 2000) odom_buffer_.pop_front();
    while (odom_buffer_.size() > 2 &&
           (odom_buffer_.back().stamp - odom_buffer_.front().stamp).toSec() >
               20.0) {
      odom_buffer_.pop_front();
    }
  }
  ProcessPendingScans();
}

bool SlamSystem::LookupOdomPose(const ros::Time& stamp, Pose2D* pose) const {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (odom_buffer_.empty()) return false;
  if (stamp <= odom_buffer_.front().stamp) {
    if ((odom_buffer_.front().stamp - stamp).toSec() > 0.25) return false;
    *pose = odom_buffer_.front().pose;
    return true;
  }
  if (stamp >= odom_buffer_.back().stamp) {
    if ((stamp - odom_buffer_.back().stamp).toSec() > 0.25) return false;
    *pose = odom_buffer_.back().pose;
    return true;
  }
  const auto upper = std::lower_bound(
      odom_buffer_.begin(), odom_buffer_.end(), stamp,
      [](const TimedPose& value, const ros::Time& target) {
        return value.stamp < target;
      });
  if (upper == odom_buffer_.begin() || upper == odom_buffer_.end()) return false;
  const auto lower = std::prev(upper);
  const double duration = (upper->stamp - lower->stamp).toSec();
  if (duration <= 1e-6) {
    *pose = upper->pose;
    return true;
  }
  const double ratio = (stamp - lower->stamp).toSec() / duration;
  pose->x = lower->pose.x + ratio * (upper->pose.x - lower->pose.x);
  pose->y = lower->pose.y + ratio * (upper->pose.y - lower->pose.y);
  pose->yaw = NormalizeAngle(
      lower->pose.yaw + ratio * NormalizeAngle(upper->pose.yaw - lower->pose.yaw));
  return true;
}

bool SlamSystem::ConvertScan(const sensor_msgs::LaserScan& scan,
                             PointCloud2D* tracking_points,
                             PointCloud2D* mapping_points) const {
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
        options_.base_frame, scan.header.frame_id, scan.header.stamp,
        ros::Duration(options_.transform_timeout));
  } catch (const tf2::TransformException& exception) {
    ROS_WARN_THROTTLE(2.0, "Scan transform unavailable: %s", exception.what());
    return false;
  }
  const double yaw = tf2::getYaw(transform.transform.rotation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double tx = transform.transform.translation.x;
  const double ty = transform.transform.translation.y;

  tracking_points->clear();
  mapping_points->clear();
  tracking_points->reserve(
      scan.ranges.size() /
          static_cast<std::size_t>(options_.point_stride) +
      1);
  mapping_points->reserve(scan.ranges.size());
  const double configured_min = std::max(options_.min_range,
                                         static_cast<double>(scan.range_min));
  const double configured_max = std::min(options_.max_range,
                                         static_cast<double>(scan.range_max));
  const std::size_t tracking_stride =
      static_cast<std::size_t>(options_.point_stride);
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = scan.ranges[index];
    if (!std::isfinite(range) || range < configured_min ||
        range > configured_max) {
      continue;
    }
    const double angle = scan.angle_min + index * scan.angle_increment;
    const double laser_x = range * std::cos(angle);
    const double laser_y = range * std::sin(angle);
    const Point2D point{tx + c * laser_x - s * laser_y,
                        ty + s * laser_x + c * laser_y};
    mapping_points->push_back(point);
    if (index % tracking_stride == 0) {
      tracking_points->push_back(point);
    }
  }
  return static_cast<int>(tracking_points->size()) >=
         options_.scan_matcher.min_correspondences;
}

void SlamSystem::Initialize(const ros::Time& stamp, const Pose2D& odom_pose,
                            const PointCloud2D& tracking_points,
                            const PointCloud2D& mapping_points) {
  initialized_ = true;
  current_local_pose_ = Pose2D{};
  last_scan_odom_pose_ = odom_pose;
  AddKeyframe(stamp, current_local_pose_, odom_pose, tracking_points,
              mapping_points, 1.0);
  diagnostics_.initialized = true;
  diagnostics_.tracking_ok = true;
  ROS_INFO("lightweight_2d_slam initialized from the first synchronized scan");
}

bool SlamSystem::ShouldCreateKeyframe(const ros::Time& stamp,
                                      const Pose2D& pose,
                                      const Pose2D& odom_pose) const {
  if (keyframes_.empty()) return true;
  const Keyframe& last = keyframes_.back();
  const double translation = TranslationDistance(last.local_pose, pose);
  const double rotation =
      std::abs(NormalizeAngle(pose.yaw - last.local_pose.yaw));
  const bool enough_motion = translation >= options_.keyframe_translation ||
                             rotation >= options_.keyframe_rotation;
  const bool timed_slow_motion =
      stamp.toSec() - last.stamp >= options_.keyframe_time &&
      (translation >= options_.keyframe_min_translation_for_time ||
       rotation >= options_.keyframe_min_rotation_for_time);
  const double odom_translation =
      TranslationDistance(last.odom_pose, odom_pose);
  const double odom_rotation = std::abs(
      NormalizeAngle(odom_pose.yaw - last.odom_pose.yaw));
  const bool odom_confirms_motion =
      odom_translation >= options_.keyframe_min_translation_for_time ||
      odom_rotation >= options_.keyframe_min_rotation_for_time;
  return odom_confirms_motion && (enough_motion || timed_slow_motion);
}

void SlamSystem::AddKeyframe(const ros::Time& stamp,
                             const Pose2D& local_pose,
                             const Pose2D& odom_pose,
                             const PointCloud2D& tracking_points,
                             const PointCloud2D& mapping_points,
                             double scan_quality) {
  const std::size_t submap_start_count = static_cast<std::size_t>(
      options_.submap_keyframes - options_.submap_overlap_keyframes);
  const bool start_new_submap =
      active_submaps_.empty() ||
      (options_.use_rigid_submap_pose_graph &&
       active_submaps_.size() == 1 &&
       active_submaps_.front()->keyframe_count() >= submap_start_count);
  Keyframe keyframe;
  keyframe.id = keyframes_.size();
  keyframe.submap_id = start_new_submap
                            ? submap_local_poses_.size()
                            : active_submaps_.back()->id();
  keyframe.stamp = stamp.toSec();
  keyframe.local_pose = local_pose;
  keyframe.odom_pose = odom_pose;
  keyframe.points = tracking_points;
  keyframe.descriptor = BuildMultiScalePolarDescriptor(
      tracking_points, options_.max_range,
      options_.loop_descriptor_angular_bins,
      options_.loop_descriptor_radial_scales);
  keyframe.scan_quality = std::max(0.25, std::min(1.0, scan_quality));

  Pose2D graph_initial_pose = local_pose;
  if (!keyframes_.empty()) {
    const Keyframe& previous = keyframes_.back();
    Pose2D previous_optimized_pose = previous.local_pose;
    pose_graph_.GetOptimizedPose(previous.id, &previous_optimized_pose);
    graph_initial_pose = PropagateOptimizedPose(
        previous.local_pose, previous_optimized_pose, local_pose);
  }
  const std::size_t graph_id = pose_graph_.AddNode(graph_initial_pose);
  if (graph_id != keyframe.id) {
    ROS_ERROR_STREAM("Pose graph/keyframe index mismatch: " << graph_id
                     << " vs " << keyframe.id);
    return;
  }
  Pose2D published_initial_pose = graph_initial_pose;
  {
    std::lock_guard<std::mutex> lock(published_poses_mutex_);
    if (published_graph_poses_.size() == graph_id) {
      if (!keyframes_.empty()) {
        const Keyframe& previous = keyframes_.back();
        const bool can_extend_active_blend =
            global_correction_blending_ &&
            correction_blend_start_poses_.size() == graph_id &&
            correction_blend_target_poses_.size() == graph_id &&
            previous.id < correction_blend_start_poses_.size() &&
            previous.id < correction_blend_target_poses_.size();
        if (can_extend_active_blend) {
          const PropagatedPoseBlend extension = PropagatePoseBlend(
              previous.local_pose, local_pose,
              correction_blend_start_poses_[previous.id],
              correction_blend_target_poses_[previous.id],
              global_correction_blend_progress_.load());
          correction_blend_start_poses_.push_back(extension.start);
          correction_blend_target_poses_.push_back(extension.target);
          published_initial_pose = extension.published;
        } else if (previous.id < published_graph_poses_.size()) {
          published_initial_pose = PropagateOptimizedPose(
              previous.local_pose, published_graph_poses_[previous.id],
              local_pose);
        }
      }
      published_graph_poses_.push_back(published_initial_pose);
      ++published_pose_revision_;
    }
  }

  if (!keyframes_.empty() && !options_.use_rigid_submap_pose_graph) {
    const Keyframe& previous = keyframes_.back();
    const double quality = options_.quality_weighted_constraints
                               ? std::max(options_.min_constraint_quality,
                                          keyframe.scan_quality)
                               : 1.0;
    const double information_scale = std::sqrt(quality);
    PoseGraphConstraint scan_constraint{
        previous.id,
        keyframe.id,
        Between(previous.local_pose, keyframe.local_pose),
        options_.scan_constraint_translation_weight * information_scale,
        options_.scan_constraint_rotation_weight * information_scale,
        ConstraintType::kScan};
    scan_constraint.robust = options_.robust_scan_constraints;
    scan_constraint.robust_loss_scale = options_.scan_constraint_huber_scale;
    pose_graph_.AddConstraint(scan_constraint);
  }
  if (!keyframes_.empty() &&
      (options_.odom_constraint_translation_weight > 0.0 ||
       options_.odom_constraint_rotation_weight > 0.0)) {
    const Keyframe& previous = keyframes_.back();
    PoseGraphConstraint odom_constraint{
        previous.id,
        keyframe.id,
        Between(previous.odom_pose, keyframe.odom_pose),
        // Keep odometry as an independent weak motion prior. A poor scan
        // should reduce the scan edge, not simultaneously erase this prior.
        options_.odom_constraint_translation_weight,
        options_.odom_constraint_rotation_weight,
        ConstraintType::kOdometry};
    odom_constraint.robust = options_.robust_odom_constraints;
    odom_constraint.robust_loss_scale = options_.odom_constraint_huber_scale;
    pose_graph_.AddConstraint(odom_constraint);
  }

  {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    keyframes_.push_back(std::move(keyframe));
  }
  AddToActiveSubmaps(keyframes_.back(), mapping_points, graph_initial_pose,
                     published_initial_pose);
  RebuildActiveGrid(local_pose);

  if (options_.enable_loop_closure &&
      keyframes_.size() %
              static_cast<std::size_t>(
                  std::max(1, options_.loop_search_every_n_keyframes)) ==
          0) {
    ScheduleLoopClosure(keyframes_.size() - 1);
  }
  if (options_.optimize_every_n_keyframes > 0 &&
      keyframes_.size() % static_cast<std::size_t>(
                              options_.optimize_every_n_keyframes) == 0) {
    pose_graph_.RequestOptimization();
  }
}

void SlamSystem::AddToActiveSubmaps(const Keyframe& keyframe,
                                    const PointCloud2D& mapping_points,
                                    const Pose2D& graph_initial_pose,
                                    const Pose2D& published_initial_pose) {
  if (active_submaps_.empty() ||
      active_submaps_.back()->id() != keyframe.submap_id) {
    if (keyframe.submap_id != submap_local_poses_.size()) {
      ROS_ERROR_STREAM("Unexpected submap id " << keyframe.submap_id
                       << "; expected " << submap_local_poses_.size());
      return;
    }
    std::unique_ptr<Submap2D> submap(new Submap2D(
        keyframe.submap_id, options_.grid_resolution,
        options_.active_grid_size, options_.submap_insert_free_space,
        options_.global_map_hit_probability,
        options_.global_map_miss_probability,
        options_.global_map_hit_connection_max_distance));
    submap_local_poses_.push_back(keyframe.local_pose);
    if (options_.use_rigid_submap_pose_graph) {
      const std::size_t graph_submap_id =
          pose_graph_.AddSubmap(graph_initial_pose);
      if (graph_submap_id != keyframe.submap_id) {
        ROS_ERROR_STREAM("Pose graph/submap index mismatch: "
                         << graph_submap_id << " vs " << keyframe.submap_id);
        return;
      }
      std::lock_guard<std::mutex> lock(published_poses_mutex_);
      published_submap_poses_.push_back(published_initial_pose);
      if (global_correction_blending_) {
        correction_blend_start_submap_poses_.push_back(
            published_initial_pose);
        correction_blend_target_submap_poses_.push_back(graph_initial_pose);
      }
    }
    active_submaps_.push_back(std::move(submap));
    ROS_INFO_STREAM("Started submap " << keyframe.submap_id
                    << " at keyframe " << keyframe.id);
  }

  const double quality = options_.quality_weighted_constraints
                             ? std::max(options_.min_constraint_quality,
                                        keyframe.scan_quality)
                             : 1.0;
  const double information_scale = std::sqrt(quality);
  for (auto& submap : active_submaps_) {
    const bool owns_texture = submap->id() == keyframe.submap_id;
    const PointCloud2D* texture_points = owns_texture ? &mapping_points
                                                      : nullptr;
    if (!submap->AddKeyframe(keyframe, owns_texture, texture_points)) {
      ROS_ERROR_STREAM("Failed to insert keyframe " << keyframe.id
                       << " into active submap " << submap->id());
      continue;
    }
    if (options_.use_rigid_submap_pose_graph) {
      NodeSubmapConstraint insertion{
          submap->id(), keyframe.id,
          Between(submap->anchor_local_pose(), keyframe.local_pose),
          options_.node_submap_translation_weight * information_scale,
          options_.node_submap_rotation_weight * information_scale,
          ConstraintType::kScan};
      insertion.robust = options_.robust_scan_constraints;
      insertion.robust_loss_scale = options_.scan_constraint_huber_scale;
      pose_graph_.AddNodeToSubmapConstraint(insertion);
    }
  }

  while (!active_submaps_.empty() &&
         active_submaps_.front()->keyframe_count() >=
             static_cast<std::size_t>(options_.submap_keyframes)) {
    if (!active_submaps_.front()->Freeze()) {
      ROS_ERROR_STREAM("Failed to freeze submap "
                       << active_submaps_.front()->id());
      return;
    }
    const std::size_t frozen_id = active_submaps_.front()->id();
    const std::size_t first_id =
        active_submaps_.front()->first_keyframe_id();
    const std::size_t last_id = active_submaps_.front()->last_keyframe_id();
    std::unique_ptr<Submap2D> frozen =
        std::move(active_submaps_.front());
    active_submaps_.erase(active_submaps_.begin());
    frozen_submaps_.push_back(
        std::shared_ptr<const Submap2D>(frozen.release()));
    ROS_INFO_STREAM("Frozen submap " << frozen_id << " keyframes="
                    << first_id << ".." << last_id);
  }
}

void SlamSystem::RebuildActiveGrid(const Pose2D& center) {
  const int cells = static_cast<int>(
      std::ceil(options_.active_grid_size / options_.grid_resolution));
  active_grid_.Reset(options_.grid_resolution, cells, cells,
                     center.x - options_.active_grid_size * 0.5,
                     center.y - options_.active_grid_size * 0.5);
  const std::size_t begin = keyframes_.size() >
                                    static_cast<std::size_t>(options_.active_keyframes)
                                ? keyframes_.size() -
                                      static_cast<std::size_t>(options_.active_keyframes)
                                : 0;
  for (std::size_t index = begin; index < keyframes_.size(); ++index) {
    active_grid_.InsertScan(keyframes_[index].local_pose,
                            keyframes_[index].points,
                            options_.active_grid_insert_free_space);
  }
}

bool SlamSystem::ScheduleLoopClosure(std::size_t current_id) {
  if (current_id < static_cast<std::size_t>(options_.loop_min_separation)) {
    return false;
  }
  const Keyframe& current = keyframes_.at(current_id);
  if (frozen_submaps_.empty()) return false;

  LoopClosureRequest request;
  request.current_id = current_id;
  request.current_submap = current.submap_id;
  request.current_local_pose = current.local_pose;
  request.current_points = current.points;
  request.current_descriptor = current.descriptor;
  for (int sequence_index = 1;
       sequence_index <= options_.loop_sequence_keyframes;
       ++sequence_index) {
    const std::size_t offset = static_cast<std::size_t>(sequence_index) *
                               static_cast<std::size_t>(
                                   options_.loop_sequence_stride);
    if (offset > current_id) break;
    request.sequence_keyframes.push_back(keyframes_.at(current_id - offset));
  }
  request.frozen_submaps = frozen_submaps_;
  {
    std::lock_guard<std::mutex> lock(loop_mutex_);
    if (loop_request_ready_ || loop_worker_busy_.load()) {
      ++loop_searches_skipped_;
      return false;
    }
    loop_request_ = std::move(request);
    loop_request_ready_ = true;
  }
  loop_condition_.notify_one();
  return true;
}

void SlamSystem::LoopClosureWorker() {
  while (true) {
    LoopClosureRequest request;
    {
      std::unique_lock<std::mutex> lock(loop_mutex_);
      loop_condition_.wait(lock, [this]() {
        return stop_loop_worker_ || loop_request_ready_;
      });
      if (stop_loop_worker_) return;
      request = std::move(loop_request_);
      loop_request_ready_ = false;
      loop_worker_busy_.store(true);
    }
    EvaluateLoopClosure(std::move(request));
    loop_worker_busy_.store(false);
  }
}

void SlamSystem::EvaluateLoopClosure(LoopClosureRequest request) {
  ++loop_searches_completed_;
  std::vector<SubmapCandidate> candidates = RetrieveTopKSubmaps(
      request.current_descriptor, request.frozen_submaps,
      request.current_submap,
      static_cast<std::size_t>(options_.loop_min_submap_separation),
      request.frozen_submaps.size(),
      options_.loop_descriptor_min_similarity,
      static_cast<std::size_t>(options_.loop_candidates_per_submap),
      options_.loop_descriptor_angular_bins, request.current_id,
      static_cast<std::size_t>(options_.loop_min_separation));
  {
    std::lock_guard<std::mutex> lock(loop_mutex_);
    candidates.erase(
        std::remove_if(
            candidates.begin(), candidates.end(),
            [this, &request](const SubmapCandidate& candidate) {
              return loop_closure_gate_.IsPairAccepted(
                  candidate.submap->id(), request.current_submap);
            }),
        candidates.end());
  }
  if (candidates.size() >
      static_cast<std::size_t>(options_.loop_top_k_candidates)) {
    candidates.resize(static_cast<std::size_t>(
        options_.loop_top_k_candidates));
  }
  if (candidates.empty()) {
    ++loop_no_candidate_rejections_;
    ++loop_candidates_rejected_;
    return;
  }

  struct ValidatedCandidate {
    SubmapCandidate candidate;
    ScanMatchResult match;
    Pose2D correction;
    bool anisotropic = false;
    double strong_direction_local_yaw = 0.0;
    int sequence_support = 0;
  };
  std::vector<ValidatedCandidate> valid;
  valid.reserve(candidates.size());
  for (const auto& candidate : candidates) {
    ++loop_candidates_evaluated_;
    Pose2D initial_pose = candidate.reference_relative_pose;
    initial_pose.yaw = NormalizeAngle(
        initial_pose.yaw + candidate.yaw_offset);
    ScanMatchResult match;
    if (options_.loop_use_multiresolution_search) {
      const ScanMatchResult coarse_match =
          coarse_loop_scan_matcher_->MatchWithWindows(
              candidate.submap->grid(), request.current_points, initial_pose,
              options_.loop_global_linear_search_window,
              options_.loop_global_angular_search_window);
      if (!coarse_match.success) {
        ++loop_match_rejections_;
        ++loop_candidates_rejected_;
        continue;
      }
      match = loop_scan_matcher_->MatchWithWindows(
          candidate.submap->grid(), request.current_points, coarse_match.pose,
          options_.loop_fine_linear_search_window,
          options_.loop_fine_angular_search_window);
    } else {
      match = loop_scan_matcher_->MatchWithWindows(
          candidate.submap->grid(), request.current_points, initial_pose,
          options_.loop_linear_search_window,
          options_.loop_angular_search_window);
    }
    if (!match.success) {
      ++loop_match_rejections_;
      ++loop_candidates_rejected_;
      continue;
    }
    const Pose2D matched_local_pose = Compose(
        candidate.submap->anchor_local_pose(), match.pose);
    const Pose2D correction = Compose(
        matched_local_pose, Inverse(request.current_local_pose));
    if (options_.loop_log_candidate_evaluations) {
      ROS_INFO_STREAM("Loop candidate evaluation candidate="
                      << candidate.reference_keyframe_id << " current="
                      << request.current_id << " submaps="
                      << candidate.submap->id() << "->"
                      << request.current_submap << " descriptor="
                      << candidate.descriptor_similarity << " score="
                      << match.coarse_score << " rms=" << match.residual_rms
                      << " correction=" << correction.x << ","
                      << correction.y << "," << correction.yaw);
    }
    if (match.coarse_score < options_.loop_min_match_score) {
      ++loop_score_rejections_;
      ++loop_candidates_rejected_;
      continue;
    }
    if (match.residual_rms > options_.loop_max_match_rms) {
      ++loop_rms_rejections_;
      ++loop_candidates_rejected_;
      continue;
    }
    const double correction_translation =
        std::hypot(correction.x, correction.y);
    const bool anisotropic =
        correction_translation > options_.loop_unobservable_max_correction &&
        match.translation_observability <
            options_.loop_min_translation_observability;
    if (anisotropic) ++loop_anisotropic_candidates_;
    if (anisotropic &&
        options_.loop_reject_low_observability_large_correction) {
      ++loop_low_observability_rejections_;
      ++loop_candidates_rejected_;
      continue;
    }
    if (correction_translation >
            options_.loop_max_correction_translation ||
        std::abs(correction.yaw) > options_.loop_max_correction_rotation) {
      ++loop_correction_rejections_;
      ++loop_candidates_rejected_;
      continue;
    }
    int sequence_support = 0;
    if (options_.loop_min_sequence_support > 0) {
      for (const auto& sequence_keyframe : request.sequence_keyframes) {
        const Pose2D expected_submap_pose = Compose(
            Inverse(candidate.submap->anchor_local_pose()),
            Compose(correction, sequence_keyframe.local_pose));
        const ScanMatchResult sequence_match =
            loop_scan_matcher_->MatchWithWindows(
                candidate.submap->grid(), sequence_keyframe.points,
                expected_submap_pose,
                options_.loop_sequence_linear_window,
                options_.loop_sequence_angular_window);
        if (!sequence_match.success ||
            sequence_match.coarse_score < options_.loop_min_match_score ||
            sequence_match.residual_rms > options_.loop_max_match_rms) {
          continue;
        }
        const Pose2D sequence_matched_local = Compose(
            candidate.submap->anchor_local_pose(), sequence_match.pose);
        const Pose2D sequence_correction = Compose(
            sequence_matched_local, Inverse(sequence_keyframe.local_pose));
        const Pose2D correction_error = Between(correction, sequence_correction);
        if (TranslationDistance(Pose2D{}, correction_error) <=
                options_.loop_sequence_correction_tolerance &&
            std::abs(correction_error.yaw) <=
                options_.loop_sequence_yaw_tolerance) {
          ++sequence_support;
        }
      }
      if (sequence_support < options_.loop_min_sequence_support) {
        ++loop_sequence_rejections_;
        ++loop_candidates_rejected_;
        continue;
      }
    }
    valid.push_back(
        {candidate, match, correction, anisotropic,
         NormalizeAngle(candidate.submap->anchor_local_pose().yaw +
                        match.translation_strong_direction_yaw),
         sequence_support});
  }
  if (valid.empty()) return;

  std::sort(valid.begin(), valid.end(),
            [](const ValidatedCandidate& lhs,
               const ValidatedCandidate& rhs) {
              if (lhs.match.coarse_score != rhs.match.coarse_score) {
                return lhs.match.coarse_score > rhs.match.coarse_score;
              }
              return lhs.match.residual_rms < rhs.match.residual_rms;
            });
  if (!options_.loop_require_se2_consensus && valid.size() > 1 &&
      valid[0].match.coarse_score - valid[1].match.coarse_score <
          options_.loop_min_score_margin) {
    ++loop_ambiguity_rejections_;
    ++loop_candidates_rejected_;
    return;
  }

  std::size_t best_index = 0;
  if (options_.loop_require_se2_consensus) {
    std::vector<LoopCorrectionHypothesis> hypotheses;
    hypotheses.reserve(valid.size());
    for (std::size_t index = 0; index < valid.size(); ++index) {
      hypotheses.push_back(
          {index, valid[index].candidate.submap->id(), valid[index].correction,
           valid[index].match.coarse_score, valid[index].match.residual_rms});
    }
    const LoopCorrectionConsensus consensus = SelectLoopCorrectionConsensus(
        hypotheses, options_.loop_consensus_min_submaps,
        options_.loop_consensus_translation_tolerance,
        options_.loop_consensus_rotation_tolerance);
    loop_last_consensus_support_.store(consensus.supporting_submaps);
    if (!consensus.valid) {
      ++loop_consensus_rejections_;
      ++loop_candidates_rejected_;
      return;
    }
    best_index = consensus.representative_index;
    ++loop_consensus_accepts_;
  }
  const ValidatedCandidate& best = valid.at(best_index);

  Pose2D gate_correction = best.correction;
  if (best.anisotropic && options_.loop_use_anisotropic_constraints) {
    const double cosine = std::cos(best.strong_direction_local_yaw);
    const double sine = std::sin(best.strong_direction_local_yaw);
    const double strong_component =
        cosine * gate_correction.x + sine * gate_correction.y;
    gate_correction.x = strong_component * cosine;
    gate_correction.y = strong_component * sine;
  }
  const LoopClosureProposal proposal{
      best.candidate.reference_keyframe_id, request.current_id,
      best.candidate.submap->id(), request.current_submap,
      gate_correction};
  ROS_INFO_STREAM("Loop proposal candidate="
                  << best.candidate.reference_keyframe_id << " current="
                  << request.current_id << " submaps="
                  << best.candidate.submap->id() << "->"
                  << request.current_submap << " descriptor="
                  << best.candidate.descriptor_similarity << " score="
                  << best.match.coarse_score << " rms="
                  << best.match.residual_rms << " sequence_support="
                  << best.sequence_support << " correction="
                  << gate_correction.x << "," << gate_correction.y << ","
                  << gate_correction.yaw);
  {
    std::lock_guard<std::mutex> lock(loop_mutex_);
    if (!loop_closure_gate_.Consider(proposal)) {
      ++loop_confirmation_rejections_;
      ++loop_candidates_rejected_;
      return;
    }
  }

  if (options_.use_rigid_submap_pose_graph) {
    NodeSubmapConstraint constraint{
        best.candidate.submap->id(), request.current_id, best.match.pose,
        options_.loop_translation_weight, options_.loop_rotation_weight,
        ConstraintType::kLoopClosure};
    constraint.switchable = options_.loop_use_switchable_constraints;
    constraint.switch_prior_weight = options_.loop_switch_prior_weight;
    if (best.anisotropic && options_.loop_use_anisotropic_constraints) {
      constraint.anisotropic_translation = true;
      constraint.translation_direction_yaw =
          best.match.translation_strong_direction_yaw;
      constraint.orthogonal_translation_weight = 0.0;
      ++loop_anisotropic_constraints_;
    }
    pose_graph_.AddNodeToSubmapConstraint(constraint);
  } else {
    PoseGraphConstraint constraint{
        best.candidate.reference_keyframe_id, request.current_id,
        Between(best.candidate.reference_relative_pose, best.match.pose),
        options_.loop_translation_weight, options_.loop_rotation_weight,
        ConstraintType::kLoopClosure};
    constraint.switchable = options_.loop_use_switchable_constraints;
    constraint.switch_prior_weight = options_.loop_switch_prior_weight;
    if (best.anisotropic && options_.loop_use_anisotropic_constraints) {
      constraint.anisotropic_translation = true;
      constraint.translation_direction_yaw = NormalizeAngle(
          best.match.translation_strong_direction_yaw -
          best.candidate.reference_relative_pose.yaw);
      constraint.orthogonal_translation_weight = 0.0;
      ++loop_anisotropic_constraints_;
    }
    pose_graph_.AddConstraint(constraint);
  }
  {
    std::lock_guard<std::mutex> lock(loop_mutex_);
    accepted_loop_edges_.emplace_back(
        best.candidate.reference_keyframe_id, request.current_id);
  }
  pose_graph_.RequestOptimization();
  ROS_INFO_STREAM("Accepted confirmed loop closure "
                  << best.candidate.reference_keyframe_id
                  << " -> " << request.current_id
                  << " submaps=" << best.candidate.submap->id()
                  << "->" << request.current_submap
                  << " descriptor="
                  << best.candidate.descriptor_similarity
                  << " yaw_seed=" << best.candidate.yaw_offset
                  << " score=" << best.match.coarse_score
                  << " rms=" << best.match.residual_rms
                  << " observability="
                  << best.match.translation_observability
                  << " sequence_support=" << best.sequence_support
                  << " consensus_support="
                  << loop_last_consensus_support_.load()
                  << " anisotropic="
                  << (best.anisotropic ? "true" : "false")
                  << " correction=" << best.correction.x << ","
                  << best.correction.y << "," << best.correction.yaw);
}

void SlamSystem::RequestMapRebuild(const PoseGraphStatus& graph) {
  MapBuildRequest request;
  {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    request.keyframes = keyframes_;
    request.submap_local_poses = submap_local_poses_;
    if (options_.use_rigid_submap_pose_graph) {
      request.submap_textures.reserve(frozen_submaps_.size() +
                                      active_submaps_.size());
      for (const auto& submap : frozen_submaps_) {
        request.submap_textures.push_back(submap->GetTextureSnapshot());
      }
      for (const auto& submap : active_submaps_) {
        request.submap_textures.push_back(submap->GetTextureSnapshot());
      }
    }
  }
  if (request.keyframes.empty()) return;
  if (options_.global_correction_blend_duration > 0.0) {
    std::lock_guard<std::mutex> lock(published_poses_mutex_);
    request.optimized_poses = published_graph_poses_;
    if (options_.use_rigid_submap_pose_graph) {
      request.optimized_submap_poses = published_submap_poses_;
    }
  } else {
    request.optimized_poses = pose_graph_.GetOptimizedPoses();
    if (options_.use_rigid_submap_pose_graph) {
      request.optimized_submap_poses =
          pose_graph_.GetOptimizedSubmapPoses();
    }
  }
  request.stamp = ros::Time::now();
  request.keyframe_count = request.keyframes.size();
  request.loop_count = graph.loop_constraints;
  request.pose_version = options_.global_correction_blend_duration > 0.0
                             ? published_pose_revision_.load()
                             : graph.pose_version;
  request.final_cost = graph.last_final_cost;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_request_ = std::move(request);
    map_request_ready_ = true;
    last_map_request_keyframe_count_ = map_request_.keyframe_count;
    last_map_request_loop_count_ = map_request_.loop_count;
    last_map_request_pose_version_ = map_request_.pose_version;
    last_map_request_final_cost_ = map_request_.final_cost;
  }
  map_condition_.notify_one();
}

void SlamSystem::UpdatePublishedGraphPoses(const ros::Time& stamp) {
  if (options_.global_correction_blend_duration <= 0.0) return;
  const PoseGraphStatus graph = pose_graph_.GetStatus();
  const std::vector<Pose2D> optimized = pose_graph_.GetOptimizedPoses();
  const std::vector<Pose2D> optimized_submaps =
      options_.use_rigid_submap_pose_graph
          ? pose_graph_.GetOptimizedSubmapPoses()
          : std::vector<Pose2D>{};
  std::lock_guard<std::mutex> lock(published_poses_mutex_);
  while (published_graph_poses_.size() < optimized.size()) {
    published_graph_poses_.push_back(
        optimized[published_graph_poses_.size()]);
  }
  while (published_submap_poses_.size() < optimized_submaps.size()) {
    published_submap_poses_.push_back(
        optimized_submaps[published_submap_poses_.size()]);
  }
  if (graph.pose_version != blended_pose_graph_version_) {
    correction_blend_start_poses_ = published_graph_poses_;
    correction_blend_target_poses_ = optimized;
    correction_blend_start_submap_poses_ = published_submap_poses_;
    correction_blend_target_submap_poses_ = optimized_submaps;
    correction_blend_start_time_ = stamp.toSec();
    blended_pose_graph_version_ = graph.pose_version;
    global_correction_blending_ = true;
    global_correction_blend_progress_.store(0.0);
  }
  if (!global_correction_blending_) return;

  const double elapsed = std::max(0.0, stamp.toSec() -
                                           correction_blend_start_time_);
  const double alpha = std::min(
      1.0, elapsed / options_.global_correction_blend_duration);
  const std::size_t count = std::min(
      correction_blend_start_poses_.size(),
      correction_blend_target_poses_.size());
  for (std::size_t index = 0; index < count; ++index) {
    const Pose2D& from = correction_blend_start_poses_[index];
    const Pose2D& to = correction_blend_target_poses_[index];
    published_graph_poses_[index] = InterpolatePose(from, to, alpha);
  }
  const std::size_t submap_count = std::min(
      correction_blend_start_submap_poses_.size(),
      correction_blend_target_submap_poses_.size());
  for (std::size_t index = 0; index < submap_count; ++index) {
    published_submap_poses_[index] = InterpolatePose(
        correction_blend_start_submap_poses_[index],
        correction_blend_target_submap_poses_[index], alpha);
  }
  global_correction_blend_progress_.store(alpha);
  ++published_pose_revision_;
  if (alpha >= 1.0) {
    global_correction_blending_ = false;
    correction_blend_start_poses_.clear();
    correction_blend_target_poses_.clear();
    correction_blend_start_submap_poses_.clear();
    correction_blend_target_submap_poses_.clear();
  }
}

void SlamSystem::MapWorkerLoop() {
  while (true) {
    MapBuildRequest request;
    {
      std::unique_lock<std::mutex> lock(map_mutex_);
      map_condition_.wait(lock, [this]() {
        return stop_map_worker_ || map_request_ready_;
      });
      if (stop_map_worker_) return;
      request = std::move(map_request_);
      map_request_ready_ = false;
      map_worker_busy_.store(true);
    }

    const auto started = std::chrono::steady_clock::now();
    nav_msgs::OccupancyGrid map;
    const bool success = BuildGlobalMap(
        request.keyframes, request.optimized_poses,
        request.submap_local_poses, request.optimized_submap_poses,
        request.submap_textures,
        request.stamp, &map);
    const nav_msgs::Path path = BuildPath(
        request.keyframes, request.optimized_poses, request.stamp);
    const nav_msgs::Path local_path = BuildPath(
        request.keyframes, {}, request.stamp);
    const nav_msgs::Path submap_path = BuildSubmapPath(
        request.keyframes, request.optimized_poses,
        request.optimized_submap_poses, request.stamp);
    const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();

    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_worker_busy_.store(false);
      last_map_build_seconds_ = elapsed;
      if (success) {
        latest_map_ = std::move(map);
        latest_path_ = path;
        latest_local_path_ = local_path;
        latest_submap_path_ = submap_path;
        map_rebuild_failures_.store(0);
        map_retry_not_before_ = ros::WallTime();
      } else {
        const std::size_t failures = map_rebuild_failures_.fetch_add(1) + 1;
        const double retry_seconds = std::min(
            10.0, std::pow(2.0, static_cast<double>(std::min<std::size_t>(
                failures, 4))));
        map_retry_not_before_ = ros::WallTime::now() +
                                ros::WallDuration(retry_seconds);
        ROS_WARN_STREAM_THROTTLE(
            5.0, "Global map rebuild failed; keeping the last valid map "
                     "(failures=" << failures << ")");
      }
    }
  }
}

void SlamSystem::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& message) {
  const auto insertion = std::upper_bound(
      pending_scans_.begin(), pending_scans_.end(), message->header.stamp,
      [](const ros::Time& stamp,
         const sensor_msgs::LaserScan::ConstPtr& queued) {
        return stamp < queued->header.stamp;
      });
  pending_scans_.insert(insertion, message);
  constexpr std::size_t kMaximumPendingScans = 50;
  while (pending_scans_.size() > kMaximumPendingScans) {
    const auto dropped = pending_scans_.front();
    pending_scans_.pop_front();
    ++diagnostics_.dropped_scans;
    diagnostics_.tracking_ok = false;
    PublishDiagnostics(dropped->header.stamp);
    ROS_ERROR_THROTTLE(2.0, "Pending laser scan queue overflow");
  }
  ProcessPendingScans();
}

void SlamSystem::ProcessPendingScans() {
  while (!pending_scans_.empty()) {
    const sensor_msgs::LaserScan::ConstPtr message = pending_scans_.front();
    bool odometry_ready = false;
    bool scan_is_too_old = false;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (odom_buffer_.empty()) return;
      scan_is_too_old =
          (odom_buffer_.front().stamp - message->header.stamp).toSec() > 0.25;
      odometry_ready = odom_buffer_.back().stamp >= message->header.stamp;
    }
    if (!odometry_ready && !scan_is_too_old) return;

    Pose2D odom_pose;
    if (scan_is_too_old ||
        !LookupOdomPose(message->header.stamp, &odom_pose)) {
      pending_scans_.pop_front();
      ++diagnostics_.dropped_scans;
      diagnostics_.tracking_ok = false;
      PublishDiagnostics(message->header.stamp);
      ROS_WARN_THROTTLE(2.0, "No synchronized odometry for laser scan");
      continue;
    }
    pending_scans_.pop_front();
    ProcessScan(message, odom_pose);
  }
}

void SlamSystem::ProcessScan(
    const sensor_msgs::LaserScan::ConstPtr& message,
    const Pose2D& odom_pose) {
  const auto started = std::chrono::steady_clock::now();
  PointCloud2D tracking_points;
  PointCloud2D mapping_points;
  if (!ConvertScan(*message, &tracking_points, &mapping_points)) {
    ++diagnostics_.dropped_scans;
    diagnostics_.tracking_ok = false;
    PublishDiagnostics(message->header.stamp);
    return;
  }

  if (!initialized_) {
    Initialize(message->header.stamp, odom_pose, tracking_points,
               mapping_points);
    PublishLocalGrid(message->header.stamp, tracking_points);
    PublishPoseAndTf(message->header.stamp, odom_pose);
    PublishDiagnostics(message->header.stamp);
    return;
  }

  const Pose2D predicted_pose = Compose(
      current_local_pose_, Between(last_scan_odom_pose_, odom_pose));
  const ScanMatchResult match =
      scan_matcher_->Match(active_grid_, tracking_points, predicted_pose);
  const bool tracking_ok = match.success &&
                           match.coarse_score >= options_.min_match_score &&
                           match.residual_rms <= options_.max_match_rms;
  current_local_pose_ = tracking_ok ? match.pose : predicted_pose;
  last_scan_odom_pose_ = odom_pose;

  diagnostics_.tracking_ok = tracking_ok;
  diagnostics_.coarse_score = match.coarse_score;
  diagnostics_.residual_rms = match.residual_rms;
  diagnostics_.score_margin = match.score_margin;
  diagnostics_.translation_observability =
      match.translation_observability;
  diagnostics_.oad_triggered = match.oad_triggered;
  diagnostics_.oad_expanded = match.oad_expanded;
  diagnostics_.oad_search_window = match.oad_search_window;
  diagnostics_.correspondences = match.correspondences;
  const double score_quality = std::max(
      0.0, std::min(1.0, (match.coarse_score - options_.min_match_score) /
                              std::max(1e-6, 1.0 - options_.min_match_score)));
  const double rms_quality = std::isfinite(match.residual_rms)
                                 ? std::max(0.0, std::min(
                                       1.0, 1.0 - match.residual_rms /
                                                   std::max(1e-6,
                                                            options_.max_match_rms)))
                                 : 0.0;
  const double legacy_scan_quality = std::sqrt(score_quality * rms_quality);
  const double scan_quality = match.match_quality > 0.0
                                  ? match.match_quality
                                  : legacy_scan_quality;
  diagnostics_.scan_quality = scan_quality;
  if (tracking_ok &&
      ShouldCreateKeyframe(message->header.stamp, current_local_pose_,
                           odom_pose)) {
    AddKeyframe(message->header.stamp, current_local_pose_, odom_pose,
                tracking_points, mapping_points, scan_quality);
  }

  PublishLocalGrid(message->header.stamp, tracking_points);
  PublishPoseAndTf(message->header.stamp, odom_pose);
  diagnostics_.processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  PublishDiagnostics(message->header.stamp);
}

Pose2D SlamSystem::CurrentMapPose() const {
  if (keyframes_.empty()) return current_local_pose_;
  const Keyframe& anchor = keyframes_.back();
  Pose2D anchor_map_pose = anchor.local_pose;
  if (options_.global_correction_blend_duration > 0.0) {
    std::lock_guard<std::mutex> lock(published_poses_mutex_);
    if (anchor.id < published_graph_poses_.size()) {
      anchor_map_pose = published_graph_poses_[anchor.id];
    }
  } else {
    pose_graph_.GetOptimizedPose(anchor.id, &anchor_map_pose);
  }
  return Compose(anchor_map_pose,
                 Between(anchor.local_pose, current_local_pose_));
}

void SlamSystem::PublishPoseAndTf(const ros::Time& stamp,
                                  const Pose2D& odom_pose) {
  UpdatePublishedGraphPoses(stamp);
  const Pose2D map_pose = CurrentMapPose();
  const Pose2D map_to_odom = Compose(map_pose, Inverse(odom_pose));

  geometry_msgs::PoseStamped pose_message;
  pose_message.header.stamp = stamp;
  pose_message.header.frame_id = options_.map_frame;
  pose_message.pose = PoseMessage(map_pose);
  pose_publisher_.publish(pose_message);

  tf_broadcaster_.sendTransform(TransformMessage(
      stamp, options_.map_frame, options_.odom_frame, map_to_odom));
  if (options_.publish_odom_tf) {
    tf_broadcaster_.sendTransform(TransformMessage(
        stamp, options_.odom_frame, options_.base_frame, odom_pose));
  }
}

void SlamSystem::PublishLocalGrid(const ros::Time& stamp,
                                  const PointCloud2D& points) {
  const int cells = static_cast<int>(
      std::ceil(options_.local_grid_size / options_.grid_resolution));
  ProbabilityGrid grid(options_.grid_resolution, cells, cells,
                       -0.5 * options_.local_grid_size,
                       -0.5 * options_.local_grid_size);
  PointCloud2D local_points;
  local_points.reserve(points.size());
  for (const auto& point : points) {
    if (std::hypot(point.x, point.y) <= options_.local_grid_max_range) {
      local_points.push_back(point);
    }
  }
  grid.InsertScan(Pose2D{}, local_points);
  local_grid_publisher_.publish(grid.ToMessage(options_.base_frame, stamp));
}

void SlamSystem::PublishDiagnostics(const ros::Time& stamp) {
  const PoseGraphStatus graph = pose_graph_.GetStatus();
  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::DiagnosticStatus status;
  status.name = "lightweight_2d_slam/tracking";
  status.hardware_id = "2d_lidar_wheel_odom";
  if (!diagnostics_.initialized) {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = "waiting_for_synchronized_input";
  } else if (!diagnostics_.tracking_ok) {
    status.level = diagnostic_msgs::DiagnosticStatus::WARN;
    status.message = "odometry_fallback";
  } else {
    status.level = diagnostic_msgs::DiagnosticStatus::OK;
    status.message = "tracking";
  }
  status.values.push_back(DiagnosticValue(
      "coarse_score", ToString(diagnostics_.coarse_score)));
  status.values.push_back(DiagnosticValue(
      "residual_rms_m", ToString(diagnostics_.residual_rms)));
  status.values.push_back(DiagnosticValue(
      "score_margin", ToString(diagnostics_.score_margin)));
  status.values.push_back(DiagnosticValue(
      "translation_observability",
      ToString(diagnostics_.translation_observability)));
  status.values.push_back(DiagnosticValue(
      "oad_triggered", diagnostics_.oad_triggered ? "true" : "false"));
  status.values.push_back(DiagnosticValue(
      "oad_expanded", diagnostics_.oad_expanded ? "true" : "false"));
  status.values.push_back(DiagnosticValue(
      "oad_search_window_m", ToString(diagnostics_.oad_search_window)));
  status.values.push_back(DiagnosticValue(
      "scan_quality", ToString(diagnostics_.scan_quality)));
  status.values.push_back(DiagnosticValue(
      "correspondences", ToString(diagnostics_.correspondences)));
  status.values.push_back(DiagnosticValue(
      "processing_ms", ToString(diagnostics_.processing_ms)));
  status.values.push_back(DiagnosticValue(
      "dropped_scans", ToString(diagnostics_.dropped_scans)));
  status.values.push_back(DiagnosticValue(
      "keyframes", ToString(keyframes_.size())));
  status.values.push_back(DiagnosticValue(
      "submaps", ToString(frozen_submaps_.size() +
                           active_submaps_.size())));
  status.values.push_back(DiagnosticValue(
      "frozen_submaps", ToString(frozen_submaps_.size())));
  status.values.push_back(DiagnosticValue(
      "pose_graph_constraints", ToString(graph.constraints)));
  status.values.push_back(DiagnosticValue(
      "pose_graph_submaps", ToString(graph.submaps)));
  status.values.push_back(DiagnosticValue(
      "node_submap_constraints",
      ToString(graph.node_submap_constraints)));
  status.values.push_back(DiagnosticValue(
      "loop_constraints", ToString(graph.loop_constraints)));
  status.values.push_back(DiagnosticValue(
      "loop_worker_busy", loop_worker_busy_.load() ? "true" : "false"));
  status.values.push_back(DiagnosticValue(
      "loop_searches_completed", ToString(loop_searches_completed_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_searches_skipped", ToString(loop_searches_skipped_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_candidates_rejected", ToString(loop_candidates_rejected_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_candidates_evaluated", ToString(loop_candidates_evaluated_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_reject_no_candidate", ToString(loop_no_candidate_rejections_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_reject_match", ToString(loop_match_rejections_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_reject_score", ToString(loop_score_rejections_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_reject_rms", ToString(loop_rms_rejections_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_anisotropic_candidates",
      ToString(loop_anisotropic_candidates_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_anisotropic_constraints",
      ToString(loop_anisotropic_constraints_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_low_observability_rejections",
      ToString(loop_low_observability_rejections_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_sequence_rejections",
      ToString(loop_sequence_rejections_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_reject_correction", ToString(loop_correction_rejections_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_reject_ambiguity", ToString(loop_ambiguity_rejections_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_reject_confirmation", ToString(loop_confirmation_rejections_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_reject_consensus", ToString(loop_consensus_rejections_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_consensus_accepts", ToString(loop_consensus_accepts_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_last_consensus_support",
      ToString(loop_last_consensus_support_.load())));
  status.values.push_back(DiagnosticValue(
      "loop_suppressed_constraints",
      ToString(graph.suppressed_loop_constraints)));
  status.values.push_back(DiagnosticValue(
      "loop_minimum_switch", ToString(graph.minimum_loop_switch)));
  status.values.push_back(DiagnosticValue(
      "optimizer_running", graph.optimizing ? "true" : "false"));
  status.values.push_back(DiagnosticValue(
      "optimizer_last_seconds", ToString(graph.last_solve_seconds)));
  status.values.push_back(DiagnosticValue(
      "optimizer_pose_version", ToString(graph.pose_version)));
  status.values.push_back(DiagnosticValue(
      "global_correction_blending",
      global_correction_blend_progress_.load() < 1.0 ? "true" : "false"));
  status.values.push_back(DiagnosticValue(
      "global_correction_blend_progress",
      ToString(global_correction_blend_progress_.load())));
  status.values.push_back(DiagnosticValue(
      "map_worker_busy", map_worker_busy_.load() ? "true" : "false"));
  status.values.push_back(DiagnosticValue(
      "map_rebuild_failures", ToString(map_rebuild_failures_.load())));
  status.values.push_back(DiagnosticValue(
      "map_build_seconds", ToString(last_map_build_seconds_)));
  array.status.push_back(status);
  diagnostics_publisher_.publish(array);
}

bool SlamSystem::BuildGlobalMap(
    const std::vector<Keyframe>& keyframes,
    const std::vector<Pose2D>& optimized,
    const std::vector<Pose2D>& submap_local_poses,
    const std::vector<Pose2D>& optimized_submap_poses,
    const std::vector<std::shared_ptr<const SubmapTexture>>& submap_textures,
    const ros::Time& stamp,
    nav_msgs::OccupancyGrid* message) const {
  if (keyframes.empty()) return false;
  if (options_.use_rigid_submap_pose_graph) {
    std::vector<Pose2D> texture_poses = optimized_submap_poses;
    if (texture_poses.size() < submap_local_poses.size()) {
      texture_poses.resize(submap_local_poses.size());
      for (std::size_t index = optimized_submap_poses.size();
           index < submap_local_poses.size(); ++index) {
        texture_poses[index] = submap_local_poses[index];
      }
    }
    if (!ComposeSubmapTextures(
            submap_textures, texture_poses, options_.grid_resolution,
            options_.max_global_grid_cells, options_.map_frame, stamp,
            message)) {
      ROS_ERROR_STREAM_THROTTLE(
          5.0, "Failed to compose rigid submap probability textures; "
               "textures=" << submap_textures.size()
               << " poses=" << texture_poses.size());
      return false;
    }
    return true;
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  const auto map_pose = [&](const Keyframe& keyframe) {
    return keyframe.id < optimized.size() ? optimized[keyframe.id]
                                          : keyframe.local_pose;
  };
  for (const auto& keyframe : keyframes) {
    const Pose2D pose = map_pose(keyframe);
    min_x = std::min(min_x, pose.x);
    min_y = std::min(min_y, pose.y);
    max_x = std::max(max_x, pose.x);
    max_y = std::max(max_y, pose.y);
    for (std::size_t index = 0; index < keyframe.points.size();
         index += static_cast<std::size_t>(options_.global_map_point_stride)) {
      const Point2D point = TransformPoint(pose, keyframe.points[index]);
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
    }
  }
  constexpr double kMargin = 1.0;
  min_x -= kMargin;
  min_y -= kMargin;
  max_x += kMargin;
  max_y += kMargin;
  const int width = std::max(1, static_cast<int>(
      std::ceil((max_x - min_x) / options_.grid_resolution)));
  const int height = std::max(1, static_cast<int>(
      std::ceil((max_y - min_y) / options_.grid_resolution)));
  const long long cells = static_cast<long long>(width) * height;
  if (cells > options_.max_global_grid_cells) {
    ROS_ERROR_STREAM_THROTTLE(
        5.0, "Refusing global grid rebuild with " << cells
        << " cells; limit=" << options_.max_global_grid_cells);
    return false;
  }

  ProbabilityGrid grid(options_.grid_resolution, width, height, min_x, min_y);
  if (!grid.SetUpdateProbabilities(options_.global_map_hit_probability,
                                   options_.global_map_miss_probability)) {
    ROS_ERROR_STREAM("Rejected global map inverse sensor model: hit="
                     << options_.global_map_hit_probability << " miss="
                     << options_.global_map_miss_probability);
    return false;
  }
  for (const auto& keyframe : keyframes) {
    const Pose2D pose = map_pose(keyframe);
    PointCloud2D sampled;
    sampled.reserve(keyframe.points.size() /
                    static_cast<std::size_t>(options_.global_map_point_stride) + 1);
    for (std::size_t index = 0; index < keyframe.points.size();
         index += static_cast<std::size_t>(options_.global_map_point_stride)) {
      sampled.push_back(keyframe.points[index]);
    }
    grid.InsertScan(pose, sampled, true,
                    options_.global_map_hit_connection_max_distance);
  }
  *message = grid.ToMessage(options_.map_frame, stamp);
  return true;
}

nav_msgs::Path SlamSystem::BuildPath(
    const std::vector<Keyframe>& keyframes,
    const std::vector<Pose2D>& optimized,
    const ros::Time& stamp) const {
  nav_msgs::Path path;
  path.header.frame_id = options_.map_frame;
  path.header.stamp = stamp;
  path.poses.reserve(keyframes.size());
  for (const auto& keyframe : keyframes) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose = PoseMessage(keyframe.id < optimized.size()
                                ? optimized[keyframe.id]
                                : keyframe.local_pose);
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::Path SlamSystem::BuildSubmapPath(
    const std::vector<Keyframe>& keyframes,
    const std::vector<Pose2D>& optimized,
    const std::vector<Pose2D>& optimized_submap_poses,
    const ros::Time& stamp) const {
  nav_msgs::Path path;
  path.header.frame_id = options_.map_frame;
  path.header.stamp = stamp;
  if (options_.use_rigid_submap_pose_graph &&
      !optimized_submap_poses.empty()) {
    path.poses.reserve(optimized_submap_poses.size());
    for (const auto& submap_pose : optimized_submap_poses) {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose = PoseMessage(submap_pose);
      path.poses.push_back(pose);
    }
    return path;
  }
  std::size_t previous_submap = std::numeric_limits<std::size_t>::max();
  for (const auto& keyframe : keyframes) {
    if (keyframe.submap_id == previous_submap) continue;
    previous_submap = keyframe.submap_id;
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose = PoseMessage(keyframe.id < optimized.size()
                                ? optimized[keyframe.id]
                                : keyframe.local_pose);
    path.poses.push_back(pose);
  }
  return path;
}

void SlamSystem::MapTimerCallback(const ros::TimerEvent&) {
  if (!initialized_) return;
  const PoseGraphStatus graph = pose_graph_.GetStatus();
  const std::size_t map_pose_version =
      options_.global_correction_blend_duration > 0.0
          ? published_pose_revision_.load()
          : graph.pose_version;
  const bool graph_changed =
      keyframes_.size() != last_map_request_keyframe_count_ ||
      graph.loop_constraints != last_map_request_loop_count_ ||
      map_pose_version != last_map_request_pose_version_ ||
      std::abs(graph.last_final_cost - last_map_request_final_cost_) > 1e-12;
  bool retry_due = false;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    retry_due = map_rebuild_failures_.load() > 0 &&
                !map_request_ready_ && !map_worker_busy_.load() &&
                ros::WallTime::now() >= map_retry_not_before_;
  }
  if (graph_changed || retry_due) {
    RequestMapRebuild(graph);
  }
  nav_msgs::OccupancyGrid map;
  nav_msgs::Path path;
  nav_msgs::Path local_path;
  nav_msgs::Path submap_path;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = latest_map_;
    path = latest_path_;
    local_path = latest_local_path_;
    submap_path = latest_submap_path_;
  }
  if (!map.data.empty()) map_publisher_.publish(map);
  if (!path.poses.empty()) path_publisher_.publish(path);
  if (!local_path.poses.empty()) local_path_publisher_.publish(local_path);
  if (!submap_path.poses.empty()) submap_path_publisher_.publish(submap_path);

  geometry_msgs::PoseArray loop_edges;
  loop_edges.header.frame_id = options_.map_frame;
  loop_edges.header.stamp = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(loop_mutex_);
    loop_edges.poses.reserve(2 * accepted_loop_edges_.size());
    for (const auto& edge : accepted_loop_edges_) {
      if (edge.first >= path.poses.size() || edge.second >= path.poses.size()) {
        continue;
      }
      loop_edges.poses.push_back(path.poses[edge.first].pose);
      loop_edges.poses.push_back(path.poses[edge.second].pose);
    }
  }
  if (!loop_edges.poses.empty()) loop_edges_publisher_.publish(loop_edges);
}

bool SlamSystem::GetMapService(nav_msgs::GetMap::Request&,
                               nav_msgs::GetMap::Response& response) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (latest_map_.data.empty()) return false;
  response.map = latest_map_;
  return true;
}

}  // namespace lightweight_2d_slam
