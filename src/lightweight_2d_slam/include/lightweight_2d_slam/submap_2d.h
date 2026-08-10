#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "lightweight_2d_slam/occupancy_grid.h"
#include "lightweight_2d_slam/types.h"

namespace lightweight_2d_slam {

struct CircularDescriptorMatch {
  double similarity = 0.0;
  int shift_bins = 0;
  double yaw_offset = 0.0;
  bool valid = false;
};

struct SubmapDescriptorEntry {
  std::size_t keyframe_id = 0;
  Pose2D relative_pose;
  std::vector<float> values;
};

struct SubmapTexture {
  std::size_t submap_id = 0;
  ProbabilityGridTexture grid;
};

std::vector<float> BuildPolarDescriptor(const PointCloud2D& points,
                                        double max_range,
                                        int bins = 60);

// Builds one clipped range profile per radial scale. Values are laid out as
// [scale][angular_bin], allowing yaw alignment to rotate every scale by the
// same sector offset.
std::vector<float> BuildMultiScalePolarDescriptor(
    const PointCloud2D& points, double max_range, int angular_bins = 60,
    int radial_scales = 3);

CircularDescriptorMatch MatchCircularDescriptor(
    const std::vector<float>& query,
    const std::vector<float>& reference);

CircularDescriptorMatch MatchMultiScaleCircularDescriptor(
    const std::vector<float>& query, const std::vector<float>& reference,
    int angular_bins);

class Submap2D {
 public:
  Submap2D(std::size_t id, double resolution, double grid_size,
           bool insert_free_space,
           double texture_hit_probability = 0.55,
           double texture_miss_probability = 0.49,
           double texture_hit_connection_max_distance = 0.0);

  bool AddKeyframe(const Keyframe& keyframe,
                   bool insert_into_texture = true,
                   const PointCloud2D* texture_points = nullptr);
  bool Freeze();
  std::shared_ptr<const SubmapTexture> GetTextureSnapshot() const;

  std::size_t id() const { return id_; }
  std::size_t first_keyframe_id() const { return first_keyframe_id_; }
  std::size_t last_keyframe_id() const { return last_keyframe_id_; }
  std::size_t keyframe_count() const { return keyframe_count_; }
  std::size_t anchor_keyframe_id() const { return anchor_keyframe_id_; }
  const Pose2D& anchor_local_pose() const { return anchor_local_pose_; }
  const ProbabilityGrid& grid() const { return grid_; }
  const std::vector<SubmapDescriptorEntry>& descriptors() const {
    return descriptors_;
  }
  bool frozen() const { return frozen_; }

 private:
  std::size_t id_ = 0;
  double resolution_ = 0.05;
  double grid_size_ = 30.0;
  bool insert_free_space_ = true;
  double texture_hit_probability_ = 0.55;
  double texture_miss_probability_ = 0.49;
  double texture_hit_connection_max_distance_ = 0.0;
  bool initialized_ = false;
  bool frozen_ = false;
  std::size_t first_keyframe_id_ = 0;
  std::size_t last_keyframe_id_ = 0;
  std::size_t keyframe_count_ = 0;
  std::size_t anchor_keyframe_id_ = 0;
  Pose2D anchor_local_pose_;
  ProbabilityGrid grid_;
  ProbabilityGrid texture_grid_;
  std::shared_ptr<const SubmapTexture> frozen_texture_;
  std::vector<SubmapDescriptorEntry> descriptors_;
};

// Composes immutable local probability textures at optimized rigid submap
// poses. Log odds are averaged with bounded observation confidence, avoiding
// duplicate overconfidence where neighboring textures overlap.
bool ComposeSubmapTextures(
    const std::vector<std::shared_ptr<const SubmapTexture>>& textures,
    const std::vector<Pose2D>& submap_poses, double resolution,
    long long max_grid_cells, const std::string& frame_id,
    const ros::Time& stamp, nav_msgs::OccupancyGrid* message);

struct SubmapCandidate {
  std::shared_ptr<const Submap2D> submap;
  std::size_t reference_keyframe_id = 0;
  Pose2D reference_relative_pose;
  double descriptor_similarity = 0.0;
  double yaw_offset = 0.0;
};

struct LoopCorrectionHypothesis {
  std::size_t candidate_index = 0;
  std::size_t submap_id = 0;
  Pose2D correction;
  double score = 0.0;
  double residual_rms = 0.0;
};

struct LoopCorrectionConsensus {
  bool valid = false;
  std::size_t representative_index = 0;
  int supporting_submaps = 0;
};

std::vector<SubmapCandidate> RetrieveTopKSubmaps(
    const std::vector<float>& query,
    const std::vector<std::shared_ptr<const Submap2D>>& submaps,
    std::size_t current_submap_id,
    std::size_t minimum_submap_separation,
    std::size_t maximum_candidates,
    double minimum_similarity,
    std::size_t maximum_candidates_per_submap = 1,
    int descriptor_angular_bins = 0,
    std::size_t current_keyframe_id = 0,
    std::size_t minimum_keyframe_separation = 0);

// Selects a correction only when geometrically consistent hypotheses come
// from enough independent submaps. Multiple keyframes from one submap count
// as one vote, preventing a repeated local structure from dominating.
LoopCorrectionConsensus SelectLoopCorrectionConsensus(
    const std::vector<LoopCorrectionHypothesis>& hypotheses,
    int minimum_supporting_submaps, double translation_tolerance,
    double rotation_tolerance);

}  // namespace lightweight_2d_slam
