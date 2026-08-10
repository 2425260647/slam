#include "lightweight_2d_slam/submap_2d.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace lightweight_2d_slam {

std::vector<float> BuildPolarDescriptor(const PointCloud2D& points,
                                        double max_range, int bins) {
  bins = std::max(1, bins);
  max_range = std::max(1.0, max_range);
  std::vector<float> descriptor(static_cast<std::size_t>(bins), 1.0f);
  for (const auto& point : points) {
    const double angle = std::atan2(point.y, point.x);
    const double normalized = (NormalizeAngle(angle) + kPi) / (2.0 * kPi);
    int bin = static_cast<int>(std::floor(normalized * bins));
    bin = std::max(0, std::min(bins - 1, bin));
    const float range = static_cast<float>(
        std::min(max_range, std::hypot(point.x, point.y)) / max_range);
    float& value = descriptor[static_cast<std::size_t>(bin)];
    value = std::min(value, range);
  }
  return descriptor;
}

std::vector<float> BuildMultiScalePolarDescriptor(
    const PointCloud2D& points, double max_range, int angular_bins,
    int radial_scales) {
  angular_bins = std::max(1, angular_bins);
  radial_scales = std::max(1, radial_scales);
  max_range = std::max(1.0, max_range);
  std::vector<float> descriptor(
      static_cast<std::size_t>(angular_bins * radial_scales), 1.0f);
  for (const auto& point : points) {
    const double angle = std::atan2(point.y, point.x);
    const double normalized = (NormalizeAngle(angle) + kPi) / (2.0 * kPi);
    int bin = static_cast<int>(std::floor(normalized * angular_bins));
    bin = std::max(0, std::min(angular_bins - 1, bin));
    const double range = std::hypot(point.x, point.y);
    for (int scale = 0; scale < radial_scales; ++scale) {
      const double scale_range =
          max_range * static_cast<double>(scale + 1) /
          static_cast<double>(radial_scales);
      const float value = static_cast<float>(
          std::min(scale_range, range) / scale_range);
      float& stored = descriptor[
          static_cast<std::size_t>(scale * angular_bins + bin)];
      stored = std::min(stored, value);
    }
  }
  return descriptor;
}

CircularDescriptorMatch MatchCircularDescriptor(
    const std::vector<float>& query,
    const std::vector<float>& reference) {
  CircularDescriptorMatch result;
  if (query.empty() || query.size() != reference.size()) return result;

  double best_similarity = -std::numeric_limits<double>::infinity();
  std::size_t best_shift = 0;
  for (std::size_t shift = 0; shift < query.size(); ++shift) {
    double difference = 0.0;
    for (std::size_t index = 0; index < query.size(); ++index) {
      difference +=
          std::abs(query[index] - reference[(index + shift) % query.size()]);
    }
    const double similarity =
        1.0 - difference / static_cast<double>(query.size());
    if (similarity > best_similarity) {
      best_similarity = similarity;
      best_shift = shift;
    }
  }

  int signed_shift = static_cast<int>(best_shift);
  const int bin_count = static_cast<int>(query.size());
  if (signed_shift > bin_count / 2) signed_shift -= bin_count;
  result.similarity = best_similarity;
  result.shift_bins = signed_shift;
  result.yaw_offset = NormalizeAngle(
      2.0 * kPi * static_cast<double>(signed_shift) /
      static_cast<double>(bin_count));
  result.valid = true;
  return result;
}

CircularDescriptorMatch MatchMultiScaleCircularDescriptor(
    const std::vector<float>& query, const std::vector<float>& reference,
    int angular_bins) {
  CircularDescriptorMatch result;
  if (angular_bins <= 0 || query.empty() || query.size() != reference.size() ||
      query.size() % static_cast<std::size_t>(angular_bins) != 0) {
    return result;
  }
  const std::size_t bins = static_cast<std::size_t>(angular_bins);
  const std::size_t scales = query.size() / bins;
  double best_similarity = -std::numeric_limits<double>::infinity();
  std::size_t best_shift = 0;
  for (std::size_t shift = 0; shift < bins; ++shift) {
    double difference = 0.0;
    for (std::size_t scale = 0; scale < scales; ++scale) {
      const std::size_t offset = scale * bins;
      for (std::size_t index = 0; index < bins; ++index) {
        difference += std::abs(
            query[offset + index] -
            reference[offset + (index + shift) % bins]);
      }
    }
    const double similarity =
        1.0 - difference / static_cast<double>(query.size());
    if (similarity > best_similarity) {
      best_similarity = similarity;
      best_shift = shift;
    }
  }

  int signed_shift = static_cast<int>(best_shift);
  if (signed_shift > angular_bins / 2) signed_shift -= angular_bins;
  result.similarity = best_similarity;
  result.shift_bins = signed_shift;
  result.yaw_offset = NormalizeAngle(
      2.0 * kPi * static_cast<double>(signed_shift) /
      static_cast<double>(angular_bins));
  result.valid = true;
  return result;
}

Submap2D::Submap2D(std::size_t id, double resolution, double grid_size,
                   bool insert_free_space,
                   double texture_hit_probability,
                   double texture_miss_probability,
                   double texture_hit_connection_max_distance)
    : id_(id),
      resolution_(std::max(0.001, resolution)),
      grid_size_(std::max(resolution_, grid_size)),
      insert_free_space_(insert_free_space),
      texture_hit_probability_(texture_hit_probability),
      texture_miss_probability_(texture_miss_probability),
      texture_hit_connection_max_distance_(
          std::max(0.0, texture_hit_connection_max_distance)) {}

bool Submap2D::AddKeyframe(const Keyframe& keyframe,
                           bool insert_into_texture,
                           const PointCloud2D* texture_points) {
  // Membership is controlled by the active-submap manager. A keyframe can
  // belong to two overlapping submaps while submap_id records only its
  // primary submap for rigid global-map rendering.
  if (frozen_) return false;
  if (!initialized_) {
    initialized_ = true;
    first_keyframe_id_ = keyframe.id;
    anchor_keyframe_id_ = keyframe.id;
    anchor_local_pose_ = keyframe.local_pose;
    const int cells = std::max(
        1, static_cast<int>(std::ceil(grid_size_ / resolution_)));
    grid_.Reset(resolution_, cells, cells, -0.5 * grid_size_,
                -0.5 * grid_size_);
    texture_grid_.Reset(resolution_, cells, cells, -0.5 * grid_size_,
                        -0.5 * grid_size_);
    texture_grid_.SetUpdateProbabilities(texture_hit_probability_,
                                         texture_miss_probability_);
  }

  const Pose2D relative_pose =
      Between(anchor_local_pose_, keyframe.local_pose);
  grid_.InsertScan(relative_pose, keyframe.points, insert_free_space_);
  if (insert_into_texture) {
    const PointCloud2D& points =
        texture_points == nullptr ? keyframe.points : *texture_points;
    texture_grid_.InsertScan(relative_pose, points, true,
                             texture_hit_connection_max_distance_);
  }
  descriptors_.push_back(
      {keyframe.id, relative_pose, keyframe.descriptor});
  last_keyframe_id_ = keyframe.id;
  ++keyframe_count_;
  return true;
}

bool Submap2D::Freeze() {
  if (frozen_ || !initialized_ || keyframe_count_ == 0) return false;
  std::shared_ptr<SubmapTexture> texture(new SubmapTexture);
  texture->submap_id = id_;
  texture->grid = texture_grid_.TextureSnapshot();
  frozen_texture_ = texture;
  frozen_ = true;
  return true;
}

std::shared_ptr<const SubmapTexture> Submap2D::GetTextureSnapshot() const {
  if (frozen_texture_) return frozen_texture_;
  std::shared_ptr<SubmapTexture> texture(new SubmapTexture);
  texture->submap_id = id_;
  texture->grid = texture_grid_.TextureSnapshot();
  return texture;
}

bool ComposeSubmapTextures(
    const std::vector<std::shared_ptr<const SubmapTexture>>& textures,
    const std::vector<Pose2D>& submap_poses, double resolution,
    long long max_grid_cells, const std::string& frame_id,
    const ros::Time& stamp, nav_msgs::OccupancyGrid* message) {
  if (message == nullptr || textures.empty() || resolution <= 0.0) {
    return false;
  }

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  std::size_t valid_textures = 0;
  for (const auto& texture : textures) {
    if (!texture || texture->grid.empty() ||
        texture->submap_id >= submap_poses.size()) {
      continue;
    }
    const ProbabilityGridTexture& grid = texture->grid;
    const Pose2D& pose = submap_poses[texture->submap_id];
    const double upper_x = grid.origin_x + grid.width * grid.resolution;
    const double upper_y = grid.origin_y + grid.height * grid.resolution;
    const Point2D corners[] = {
        {grid.origin_x, grid.origin_y},
        {upper_x, grid.origin_y},
        {grid.origin_x, upper_y},
        {upper_x, upper_y}};
    for (const Point2D& corner : corners) {
      const Point2D world = TransformPoint(pose, corner);
      min_x = std::min(min_x, world.x);
      min_y = std::min(min_y, world.y);
      max_x = std::max(max_x, world.x);
      max_y = std::max(max_y, world.y);
    }
    ++valid_textures;
  }
  if (valid_textures == 0) return false;

  constexpr double kMargin = 1.0;
  min_x -= kMargin;
  min_y -= kMargin;
  max_x += kMargin;
  max_y += kMargin;
  const int width = std::max(
      1, static_cast<int>(std::ceil((max_x - min_x) / resolution)));
  const int height = std::max(
      1, static_cast<int>(std::ceil((max_y - min_y) / resolution)));
  const long long cells = static_cast<long long>(width) * height;
  if (cells <= 0 || cells > max_grid_cells) return false;

  std::vector<double> weighted_log_odds(static_cast<std::size_t>(cells), 0.0);
  std::vector<double> total_weight(static_cast<std::size_t>(cells), 0.0);
  const auto splat = [&](int x, int y, double weight, double log_odds,
                         double confidence) {
    if (x < 0 || x >= width || y < 0 || y >= height || weight <= 0.0) {
      return;
    }
    const std::size_t index = static_cast<std::size_t>(y * width + x);
    const double combined_weight = weight * confidence;
    weighted_log_odds[index] += combined_weight * log_odds;
    total_weight[index] += combined_weight;
  };

  for (const auto& texture : textures) {
    if (!texture || texture->grid.empty() ||
        texture->submap_id >= submap_poses.size()) {
      continue;
    }
    const ProbabilityGridTexture& grid = texture->grid;
    const Pose2D& pose = submap_poses[texture->submap_id];
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const std::size_t source = static_cast<std::size_t>(
            y * grid.width + x);
        const double log_odds = grid.log_odds[source];
        if (!std::isfinite(log_odds) ||
            grid.observation_count[source] == 0u) {
          continue;
        }
        const Point2D local{
            grid.origin_x + (static_cast<double>(x) + 0.5) * grid.resolution,
            grid.origin_y + (static_cast<double>(y) + 0.5) * grid.resolution};
        const Point2D world = TransformPoint(pose, local);
        const double grid_x = (world.x - min_x) / resolution - 0.5;
        const double grid_y = (world.y - min_y) / resolution - 0.5;
        const int x0 = static_cast<int>(std::floor(grid_x));
        const int y0 = static_cast<int>(std::floor(grid_y));
        const double tx = grid_x - x0;
        const double ty = grid_y - y0;
        const double confidence = std::min(
            20.0, static_cast<double>(grid.observation_count[source]));
        splat(x0, y0, (1.0 - tx) * (1.0 - ty), log_odds, confidence);
        splat(x0 + 1, y0, tx * (1.0 - ty), log_odds, confidence);
        splat(x0, y0 + 1, (1.0 - tx) * ty, log_odds, confidence);
        splat(x0 + 1, y0 + 1, tx * ty, log_odds, confidence);
      }
    }
  }

  nav_msgs::OccupancyGrid result;
  result.header.frame_id = frame_id;
  result.header.stamp = stamp;
  result.info.resolution = resolution;
  result.info.width = static_cast<uint32_t>(width);
  result.info.height = static_cast<uint32_t>(height);
  result.info.origin.position.x = min_x;
  result.info.origin.position.y = min_y;
  result.info.origin.orientation.w = 1.0;
  result.data.assign(static_cast<std::size_t>(cells), -1);
  for (std::size_t index = 0; index < result.data.size(); ++index) {
    if (total_weight[index] <= 0.0) continue;
    const double log_odds = std::max(
        -4.0, std::min(4.0, weighted_log_odds[index] / total_weight[index]));
    const double probability = 1.0 / (1.0 + std::exp(-log_odds));
    result.data[index] = static_cast<int8_t>(
        std::round(100.0 * probability));
  }
  *message = std::move(result);
  return true;
}

std::vector<SubmapCandidate> RetrieveTopKSubmaps(
    const std::vector<float>& query,
    const std::vector<std::shared_ptr<const Submap2D>>& submaps,
    std::size_t current_submap_id,
    std::size_t minimum_submap_separation,
    std::size_t maximum_candidates,
    double minimum_similarity,
    std::size_t maximum_candidates_per_submap,
    int descriptor_angular_bins,
    std::size_t current_keyframe_id,
    std::size_t minimum_keyframe_separation) {
  std::vector<SubmapCandidate> candidates;
  if (query.empty() || maximum_candidates == 0 ||
      maximum_candidates_per_submap == 0) {
    return candidates;
  }

  for (const auto& submap : submaps) {
    if (!submap || !submap->frozen() || submap->id() >= current_submap_id ||
        current_submap_id - submap->id() < minimum_submap_separation) {
      continue;
    }

    std::vector<SubmapCandidate> submap_candidates;
    submap_candidates.reserve(submap->descriptors().size());
    for (const auto& descriptor : submap->descriptors()) {
      if (minimum_keyframe_separation > 0 &&
          (descriptor.keyframe_id >= current_keyframe_id ||
           current_keyframe_id - descriptor.keyframe_id <
               minimum_keyframe_separation)) {
        continue;
      }
      const CircularDescriptorMatch match = descriptor_angular_bins > 0
          ? MatchMultiScaleCircularDescriptor(
                query, descriptor.values, descriptor_angular_bins)
          : MatchCircularDescriptor(query, descriptor.values);
      if (!match.valid || match.similarity < minimum_similarity) {
        continue;
      }
      SubmapCandidate candidate;
      candidate.submap = submap;
      candidate.reference_keyframe_id = descriptor.keyframe_id;
      candidate.reference_relative_pose = descriptor.relative_pose;
      candidate.descriptor_similarity = match.similarity;
      candidate.yaw_offset = match.yaw_offset;
      submap_candidates.push_back(std::move(candidate));
    }
    std::sort(submap_candidates.begin(), submap_candidates.end(),
              [](const SubmapCandidate& lhs, const SubmapCandidate& rhs) {
                if (lhs.descriptor_similarity != rhs.descriptor_similarity) {
                  return lhs.descriptor_similarity > rhs.descriptor_similarity;
                }
                return lhs.reference_keyframe_id < rhs.reference_keyframe_id;
              });
    if (submap_candidates.size() > maximum_candidates_per_submap) {
      submap_candidates.resize(maximum_candidates_per_submap);
    }
    candidates.insert(candidates.end(), submap_candidates.begin(),
                      submap_candidates.end());
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const SubmapCandidate& lhs, const SubmapCandidate& rhs) {
              if (lhs.descriptor_similarity != rhs.descriptor_similarity) {
                return lhs.descriptor_similarity > rhs.descriptor_similarity;
              }
              return lhs.submap->id() < rhs.submap->id();
            });
  if (candidates.size() > maximum_candidates) {
    candidates.resize(maximum_candidates);
  }
  return candidates;
}

LoopCorrectionConsensus SelectLoopCorrectionConsensus(
    const std::vector<LoopCorrectionHypothesis>& hypotheses,
    int minimum_supporting_submaps, double translation_tolerance,
    double rotation_tolerance) {
  LoopCorrectionConsensus result;
  if (hypotheses.empty()) return result;
  minimum_supporting_submaps = std::max(1, minimum_supporting_submaps);
  translation_tolerance = std::max(0.0, translation_tolerance);
  rotation_tolerance = std::max(0.0, rotation_tolerance);

  int best_support = 0;
  double best_cluster_score = -std::numeric_limits<double>::infinity();
  std::size_t best_seed = 0;
  for (std::size_t seed = 0; seed < hypotheses.size(); ++seed) {
    std::map<std::size_t, std::size_t> best_per_submap;
    for (std::size_t index = 0; index < hypotheses.size(); ++index) {
      const Pose2D& lhs = hypotheses[seed].correction;
      const Pose2D& rhs = hypotheses[index].correction;
      if (TranslationDistance(lhs, rhs) > translation_tolerance ||
          std::abs(NormalizeAngle(lhs.yaw - rhs.yaw)) > rotation_tolerance) {
        continue;
      }
      const auto existing = best_per_submap.find(hypotheses[index].submap_id);
      if (existing == best_per_submap.end() ||
          hypotheses[index].score > hypotheses[existing->second].score ||
          (hypotheses[index].score == hypotheses[existing->second].score &&
           hypotheses[index].residual_rms <
               hypotheses[existing->second].residual_rms)) {
        best_per_submap[hypotheses[index].submap_id] = index;
      }
    }
    double cluster_score = 0.0;
    for (const auto& entry : best_per_submap) {
      const LoopCorrectionHypothesis& hypothesis = hypotheses[entry.second];
      cluster_score += hypothesis.score - hypothesis.residual_rms;
    }
    const int support = static_cast<int>(best_per_submap.size());
    if (support > best_support ||
        (support == best_support && cluster_score > best_cluster_score)) {
      best_support = support;
      best_cluster_score = cluster_score;
      best_seed = seed;
    }
  }

  result.supporting_submaps = best_support;
  if (best_support < minimum_supporting_submaps) return result;

  std::size_t representative = best_seed;
  for (std::size_t index = 0; index < hypotheses.size(); ++index) {
    if (TranslationDistance(hypotheses[best_seed].correction,
                            hypotheses[index].correction) >
            translation_tolerance ||
        std::abs(NormalizeAngle(hypotheses[best_seed].correction.yaw -
                                hypotheses[index].correction.yaw)) >
            rotation_tolerance) {
      continue;
    }
    if (hypotheses[index].score > hypotheses[representative].score ||
        (hypotheses[index].score == hypotheses[representative].score &&
         hypotheses[index].residual_rms <
             hypotheses[representative].residual_rms)) {
      representative = index;
    }
  }
  result.valid = true;
  result.representative_index = hypotheses[representative].candidate_index;
  return result;
}

}  // namespace lightweight_2d_slam
