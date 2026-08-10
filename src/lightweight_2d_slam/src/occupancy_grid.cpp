#include "lightweight_2d_slam/occupancy_grid.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>

namespace lightweight_2d_slam {
namespace {

constexpr float kMinLogOdds = -4.0f;
constexpr float kMaxLogOdds = 4.0f;
constexpr double kOccupiedProbability = 0.55;

}  // namespace

ProbabilityGrid::ProbabilityGrid(double resolution, int width, int height,
                                 double origin_x, double origin_y) {
  Reset(resolution, width, height, origin_x, origin_y);
}

void ProbabilityGrid::Reset(double resolution, int width, int height,
                            double origin_x, double origin_y) {
  resolution_ = resolution;
  width_ = std::max(1, width);
  height_ = std::max(1, height);
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  log_odds_.assign(static_cast<std::size_t>(width_ * height_), 0.0f);
  observed_.assign(static_cast<std::size_t>(width_ * height_), 0u);
  observation_count_.assign(static_cast<std::size_t>(width_ * height_), 0u);
  miss_update_epochs_.assign(static_cast<std::size_t>(width_ * height_), 0u);
  hit_update_epochs_.assign(static_cast<std::size_t>(width_ * height_), 0u);
  current_update_epoch_ = 0;
  observed_count_ = 0;
  likelihood_field_.clear();
  likelihood_radius_cells_ = -1;
  likelihood_field_dirty_ = true;
}

bool ProbabilityGrid::SetUpdateProbabilities(double hit_probability,
                                             double miss_probability) {
  if (!std::isfinite(hit_probability) ||
      !std::isfinite(miss_probability) || hit_probability <= 0.5 ||
      hit_probability >= 1.0 || miss_probability <= 0.0 ||
      miss_probability >= 0.5) {
    return false;
  }
  hit_log_odds_ = static_cast<float>(
      std::log(hit_probability / (1.0 - hit_probability)));
  miss_log_odds_ = static_cast<float>(
      std::log(miss_probability / (1.0 - miss_probability)));
  return true;
}

bool ProbabilityGrid::WorldToCell(const Point2D& point, int* x, int* y) const {
  *x = static_cast<int>(std::floor((point.x - origin_x_) / resolution_));
  *y = static_cast<int>(std::floor((point.y - origin_y_) / resolution_));
  return *x >= 0 && *x < width_ && *y >= 0 && *y < height_;
}

Point2D ProbabilityGrid::CellCenter(int x, int y) const {
  return {origin_x_ + (static_cast<double>(x) + 0.5) * resolution_,
          origin_y_ + (static_cast<double>(y) + 0.5) * resolution_};
}

void ProbabilityGrid::UpdateCell(int x, int y, float delta) {
  if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
  const int index = Index(x, y);
  if (!observed_[index]) {
    observed_[index] = 1u;
    ++observed_count_;
  }
  log_odds_[index] =
      std::max(kMinLogOdds, std::min(kMaxLogOdds, log_odds_[index] + delta));
  likelihood_field_dirty_ = true;
}

void ProbabilityGrid::UpdateMissCell(int x, int y) {
  if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
  const int index = Index(x, y);
  if (hit_update_epochs_[index] == current_update_epoch_ ||
      miss_update_epochs_[index] == current_update_epoch_) {
    return;
  }
  UpdateCell(x, y, miss_log_odds_);
  if (observation_count_[index] < std::numeric_limits<uint16_t>::max()) {
    ++observation_count_[index];
  }
  miss_update_epochs_[index] = current_update_epoch_;
}

void ProbabilityGrid::UpdateHitCell(int x, int y) {
  if (x < 0 || x >= width_ || y < 0 || y >= height_) return;
  const int index = Index(x, y);
  if (hit_update_epochs_[index] == current_update_epoch_) return;
  if (miss_update_epochs_[index] == current_update_epoch_) {
    UpdateCell(x, y, -miss_log_odds_);
  } else if (observation_count_[index] <
             std::numeric_limits<uint16_t>::max()) {
    ++observation_count_[index];
  }
  UpdateCell(x, y, hit_log_odds_);
  hit_update_epochs_[index] = current_update_epoch_;
}

void ProbabilityGrid::TraceRay(int x0, int y0, int x1, int y1,
                               bool include_end) {
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  int x = x0;
  int y = y0;
  while (true) {
    if ((x != x1 || y != y1) || include_end) UpdateMissCell(x, y);
    if (x == x1 && y == y1) break;
    const int doubled = 2 * error;
    if (doubled >= dy) {
      error += dy;
      x += sx;
    }
    if (doubled <= dx) {
      error += dx;
      y += sy;
    }
  }
}

void ProbabilityGrid::TraceHitSegment(int x0, int y0, int x1, int y1) {
  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  int x = x0;
  int y = y0;
  while (true) {
    UpdateHitCell(x, y);
    if (x == x1 && y == y1) break;
    const int doubled = 2 * error;
    if (doubled >= dy) {
      error += dy;
      x += sx;
    }
    if (doubled <= dx) {
      error += dx;
      y += sy;
    }
  }
}

void ProbabilityGrid::InsertScan(const Pose2D& sensor_pose,
                                 const PointCloud2D& points,
                                 bool insert_free_space,
                                 double max_hit_connection_distance) {
  ++current_update_epoch_;
  if (current_update_epoch_ == 0u) {
    std::fill(miss_update_epochs_.begin(), miss_update_epochs_.end(), 0u);
    std::fill(hit_update_epochs_.begin(), hit_update_epochs_.end(), 0u);
    current_update_epoch_ = 1u;
  }
  int origin_cell_x = 0;
  int origin_cell_y = 0;
  const Point2D sensor_origin{sensor_pose.x, sensor_pose.y};
  if (!WorldToCell(sensor_origin, &origin_cell_x, &origin_cell_y)) return;

  bool previous_endpoint_valid = false;
  Point2D previous_endpoint;
  int previous_endpoint_x = 0;
  int previous_endpoint_y = 0;
  for (const auto& point : points) {
    const Point2D endpoint = TransformPoint(sensor_pose, point);
    int endpoint_x = 0;
    int endpoint_y = 0;
    const bool endpoint_inside =
        WorldToCell(endpoint, &endpoint_x, &endpoint_y);
    if (!endpoint_inside) {
      previous_endpoint_valid = false;
      if (!insert_free_space) continue;

      const double dx = endpoint.x - sensor_origin.x;
      const double dy = endpoint.y - sensor_origin.y;
      double scale = 1.0;
      const double epsilon = std::max(1e-9, resolution_ * 1e-6);
      const double min_x = origin_x_ + epsilon;
      const double min_y = origin_y_ + epsilon;
      const double max_x = origin_x_ + width_ * resolution_ - epsilon;
      const double max_y = origin_y_ + height_ * resolution_ - epsilon;
      if (dx > 0.0) {
        scale = std::min(scale, (max_x - sensor_origin.x) / dx);
      } else if (dx < 0.0) {
        scale = std::min(scale, (min_x - sensor_origin.x) / dx);
      }
      if (dy > 0.0) {
        scale = std::min(scale, (max_y - sensor_origin.y) / dy);
      } else if (dy < 0.0) {
        scale = std::min(scale, (min_y - sensor_origin.y) / dy);
      }
      if (!std::isfinite(scale) || scale <= 0.0) continue;

      const Point2D clipped_endpoint{
          sensor_origin.x + std::min(1.0, scale) * dx,
          sensor_origin.y + std::min(1.0, scale) * dy};
      if (!WorldToCell(clipped_endpoint, &endpoint_x, &endpoint_y)) continue;
      TraceRay(origin_cell_x, origin_cell_y, endpoint_x, endpoint_y, true);
      continue;
    }
    if (insert_free_space) {
      TraceRay(origin_cell_x, origin_cell_y, endpoint_x, endpoint_y, false);
    }
    UpdateHitCell(endpoint_x, endpoint_y);
    if (previous_endpoint_valid && max_hit_connection_distance > 0.0 &&
        std::hypot(endpoint.x - previous_endpoint.x,
                   endpoint.y - previous_endpoint.y) <=
            max_hit_connection_distance) {
      TraceHitSegment(previous_endpoint_x, previous_endpoint_y,
                      endpoint_x, endpoint_y);
    }
    previous_endpoint_valid = true;
    previous_endpoint = endpoint;
    previous_endpoint_x = endpoint_x;
    previous_endpoint_y = endpoint_y;
  }
}

double ProbabilityGrid::Probability(int index) const {
  return 1.0 / (1.0 + std::exp(-static_cast<double>(log_odds_[index])));
}

double ProbabilityGrid::EndpointScore(const Point2D& world_point,
                                      int neighborhood) const {
  int center_x = 0;
  int center_y = 0;
  if (!WorldToCell(world_point, &center_x, &center_y)) return 0.0;
  double best = 0.0;
  for (int dy = -neighborhood; dy <= neighborhood; ++dy) {
    for (int dx = -neighborhood; dx <= neighborhood; ++dx) {
      const int x = center_x + dx;
      const int y = center_y + dy;
      if (x < 0 || x >= width_ || y < 0 || y >= height_) continue;
      const int index = Index(x, y);
      if (!observed_[index]) continue;
      best = std::max(best, Probability(index));
    }
  }
  return best;
}

void ProbabilityGrid::EnsureLikelihoodField(int radius_cells) const {
  radius_cells = std::max(0, radius_cells);
  if (!likelihood_field_dirty_ &&
      likelihood_radius_cells_ == radius_cells) {
    return;
  }

  likelihood_field_.assign(static_cast<std::size_t>(width_ * height_), 0.0f);
  const double sigma_cells = std::max(1.0, 0.75 * radius_cells);
  const double inverse_two_sigma_squared =
      0.5 / (sigma_cells * sigma_cells);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const int source_index = Index(x, y);
      if (!observed_[source_index]) continue;
      const double occupancy = Probability(source_index);
      if (occupancy < kOccupiedProbability) continue;
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
          const int target_x = x + dx;
          const int target_y = y + dy;
          if (target_x < 0 || target_x >= width_ || target_y < 0 ||
              target_y >= height_) {
            continue;
          }
          const double squared_distance = dx * dx + dy * dy;
          const float likelihood = static_cast<float>(
              std::exp(-squared_distance * inverse_two_sigma_squared));
          float& target = likelihood_field_[Index(target_x, target_y)];
          target = std::max(target, likelihood);
        }
      }
    }
  }
  likelihood_radius_cells_ = radius_cells;
  likelihood_field_dirty_ = false;
}

double ProbabilityGrid::LikelihoodAt(int x, int y) const {
  if (x < 0 || x >= width_ || y < 0 || y >= height_) return 0.0;
  return likelihood_field_[Index(x, y)];
}

double ProbabilityGrid::LikelihoodScore(const Point2D& world_point,
                                        int radius_cells) const {
  return LikelihoodScoreAndGradient(world_point, radius_cells, nullptr);
}

double ProbabilityGrid::LikelihoodScoreAndGradient(
    const Point2D& world_point, int radius_cells, Point2D* gradient) const {
  EnsureLikelihoodField(radius_cells);
  const double grid_x = (world_point.x - origin_x_) / resolution_ - 0.5;
  const double grid_y = (world_point.y - origin_y_) / resolution_ - 0.5;
  const int x0 = static_cast<int>(std::floor(grid_x));
  const int y0 = static_cast<int>(std::floor(grid_y));
  const double tx = grid_x - x0;
  const double ty = grid_y - y0;
  const double lower_left = LikelihoodAt(x0, y0);
  const double lower_right = LikelihoodAt(x0 + 1, y0);
  const double upper_left = LikelihoodAt(x0, y0 + 1);
  const double upper_right = LikelihoodAt(x0 + 1, y0 + 1);
  const double lower = (1.0 - tx) * lower_left + tx * lower_right;
  const double upper = (1.0 - tx) * upper_left + tx * upper_right;
  if (gradient != nullptr) {
    gradient->x = ((1.0 - ty) * (lower_right - lower_left) +
                   ty * (upper_right - upper_left)) /
                  resolution_;
    gradient->y = ((1.0 - tx) * (upper_left - lower_left) +
                   tx * (upper_right - lower_right)) /
                  resolution_;
  }
  return (1.0 - ty) * lower + ty * upper;
}

bool ProbabilityGrid::FitLine(const Point2D& world_point, int radius_cells,
                              int min_neighbors, Point2D* centroid,
                              Point2D* normal, double max_eigenvalue_ratio,
                              double min_eigenvalue) const {
  int center_x = 0;
  int center_y = 0;
  if (!WorldToCell(world_point, &center_x, &center_y)) return false;

  std::vector<Eigen::Vector2d> neighbors;
  neighbors.reserve(static_cast<std::size_t>((2 * radius_cells + 1) *
                                             (2 * radius_cells + 1)));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int x = center_x + dx;
      const int y = center_y + dy;
      if (x < 0 || x >= width_ || y < 0 || y >= height_) continue;
      const int index = Index(x, y);
      if (!observed_[index] || Probability(index) < kOccupiedProbability) continue;
      const Point2D point = CellCenter(x, y);
      neighbors.emplace_back(point.x, point.y);
    }
  }
  if (static_cast<int>(neighbors.size()) < min_neighbors) return false;

  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto& point : neighbors) mean += point;
  mean /= static_cast<double>(neighbors.size());

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto& point : neighbors) {
    const Eigen::Vector2d delta = point - mean;
    covariance += delta * delta.transpose();
  }
  covariance /= static_cast<double>(neighbors.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success) return false;
  const auto eigenvalues = solver.eigenvalues();
  if (eigenvalues[1] < min_eigenvalue ||
      eigenvalues[0] / eigenvalues[1] > max_eigenvalue_ratio) {
    return false;
  }
  const Eigen::Vector2d eigen_normal = solver.eigenvectors().col(0);
  *centroid = {mean.x(), mean.y()};
  *normal = {eigen_normal.x(), eigen_normal.y()};
  return true;
}

nav_msgs::OccupancyGrid ProbabilityGrid::ToMessage(
    const std::string& frame_id, const ros::Time& stamp) const {
  nav_msgs::OccupancyGrid message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.info.resolution = resolution_;
  message.info.width = static_cast<uint32_t>(width_);
  message.info.height = static_cast<uint32_t>(height_);
  message.info.origin.position.x = origin_x_;
  message.info.origin.position.y = origin_y_;
  message.info.origin.orientation.w = 1.0;
  message.data.resize(static_cast<std::size_t>(width_ * height_), -1);
  for (int index = 0; index < width_ * height_; ++index) {
    if (!observed_[index]) continue;
    message.data[static_cast<std::size_t>(index)] = static_cast<int8_t>(
        std::round(100.0 * Probability(index)));
  }
  return message;
}

ProbabilityGridTexture ProbabilityGrid::TextureSnapshot() const {
  ProbabilityGridTexture texture;
  texture.resolution = resolution_;
  if (observed_count_ == 0) return texture;

  int min_x = width_;
  int min_y = height_;
  int max_x = -1;
  int max_y = -1;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (!observed_[Index(x, y)]) continue;
      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
    }
  }
  if (max_x < min_x || max_y < min_y) return texture;

  texture.width = max_x - min_x + 1;
  texture.height = max_y - min_y + 1;
  texture.origin_x = origin_x_ + min_x * resolution_;
  texture.origin_y = origin_y_ + min_y * resolution_;
  const std::size_t cell_count = static_cast<std::size_t>(
      texture.width * texture.height);
  texture.log_odds.assign(cell_count,
                          std::numeric_limits<float>::quiet_NaN());
  texture.observation_count.assign(cell_count, 0u);
  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      const int source = Index(x, y);
      if (!observed_[source]) continue;
      const std::size_t target = static_cast<std::size_t>(
          (y - min_y) * texture.width + (x - min_x));
      texture.log_odds[target] = log_odds_[source];
      texture.observation_count[target] = observation_count_[source];
    }
  }
  return texture;
}

}  // namespace lightweight_2d_slam
