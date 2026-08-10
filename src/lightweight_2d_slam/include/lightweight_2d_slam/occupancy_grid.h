#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/time.h>

#include "lightweight_2d_slam/types.h"

namespace lightweight_2d_slam {

struct ProbabilityGridTexture {
  double resolution = 0.05;
  int width = 0;
  int height = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<float> log_odds;
  std::vector<uint16_t> observation_count;

  bool empty() const { return width <= 0 || height <= 0 || log_odds.empty(); }
};

class ProbabilityGrid {
 public:
  ProbabilityGrid() = default;
  ProbabilityGrid(double resolution, int width, int height,
                  double origin_x, double origin_y);

  void Reset(double resolution, int width, int height,
             double origin_x, double origin_y);
  // Probabilities are converted to additive occupancy log odds. The sensor
  // model survives Reset() so callers can configure a grid once.
  bool SetUpdateProbabilities(double hit_probability,
                              double miss_probability);
  void InsertScan(const Pose2D& sensor_pose, const PointCloud2D& points,
                  bool insert_free_space = true,
                  double max_hit_connection_distance = 0.0);

  double EndpointScore(const Point2D& world_point, int neighborhood = 1) const;
  double LikelihoodScore(const Point2D& world_point,
                         int radius_cells = 1) const;
  double LikelihoodScoreAndGradient(const Point2D& world_point,
                                    int radius_cells,
                                    Point2D* gradient) const;
  bool FitLine(const Point2D& world_point, int radius_cells,
               int min_neighbors, Point2D* centroid,
               Point2D* normal, double max_eigenvalue_ratio = 0.35,
               double min_eigenvalue = 1e-8) const;
  nav_msgs::OccupancyGrid ToMessage(const std::string& frame_id,
                                    const ros::Time& stamp) const;
  // Returns a value-type, cropped copy suitable for immutable handoff to a
  // background map-composition thread.
  ProbabilityGridTexture TextureSnapshot() const;

  bool Empty() const { return observed_count_ == 0; }
  double resolution() const { return resolution_; }
  int width() const { return width_; }
  int height() const { return height_; }
  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }

 private:
  bool WorldToCell(const Point2D& point, int* x, int* y) const;
  Point2D CellCenter(int x, int y) const;
  int Index(int x, int y) const { return y * width_ + x; }
  void UpdateCell(int x, int y, float delta);
  void UpdateMissCell(int x, int y);
  void UpdateHitCell(int x, int y);
  void TraceRay(int x0, int y0, int x1, int y1, bool include_end);
  void TraceHitSegment(int x0, int y0, int x1, int y1);
  double Probability(int index) const;
  void EnsureLikelihoodField(int radius_cells) const;
  double LikelihoodAt(int x, int y) const;

  double resolution_ = 0.05;
  int width_ = 0;
  int height_ = 0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  float hit_log_odds_ = 0.85f;
  float miss_log_odds_ = -0.40f;
  std::vector<float> log_odds_;
  std::vector<uint8_t> observed_;
  std::vector<uint16_t> observation_count_;
  std::vector<uint32_t> miss_update_epochs_;
  std::vector<uint32_t> hit_update_epochs_;
  uint32_t current_update_epoch_ = 0;
  std::size_t observed_count_ = 0;
  mutable std::vector<float> likelihood_field_;
  mutable int likelihood_radius_cells_ = -1;
  mutable bool likelihood_field_dirty_ = true;
};

}  // namespace lightweight_2d_slam
