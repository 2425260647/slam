#pragma once

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <thread>
#include <vector>

#include "lightweight_2d_slam/types.h"

namespace lightweight_2d_slam {

enum class ConstraintType {
  kScan,
  kOdometry,
  kLoopClosure,
};

struct PoseGraphConstraint {
  std::size_t from = 0;
  std::size_t to = 0;
  Pose2D relative_pose;
  double translation_weight = 1.0;
  double rotation_weight = 1.0;
  ConstraintType type = ConstraintType::kScan;
  bool anisotropic_translation = false;
  double translation_direction_yaw = 0.0;
  double orthogonal_translation_weight = 0.0;
  // Loop closures can be solved as switchable constraints. Sequential scan
  // and odometry edges can optionally use a robust loss.
  bool switchable = false;
  double switch_prior_weight = 1.0;
  bool robust = false;
  double robust_loss_scale = 1.0;
};

// A node-to-submap measurement expresses the node pose in the fixed local
// coordinate system of a submap. Insertion constraints connect overlapping
// submaps through shared nodes; loop constraints connect a current node to a
// previously frozen submap without deforming either submap internally.
struct NodeSubmapConstraint {
  std::size_t submap = 0;
  std::size_t node = 0;
  Pose2D relative_pose;
  double translation_weight = 1.0;
  double rotation_weight = 1.0;
  ConstraintType type = ConstraintType::kScan;
  bool anisotropic_translation = false;
  double translation_direction_yaw = 0.0;
  double orthogonal_translation_weight = 0.0;
  bool switchable = false;
  double switch_prior_weight = 1.0;
  bool robust = false;
  double robust_loss_scale = 1.0;
};

struct PoseGraphStatus {
  std::size_t nodes = 0;
  std::size_t submaps = 0;
  std::size_t constraints = 0;
  std::size_t node_submap_constraints = 0;
  std::size_t loop_constraints = 0;
  std::size_t suppressed_loop_constraints = 0;
  double minimum_loop_switch = 1.0;
  std::size_t pose_version = 0;
  bool optimizing = false;
  double last_solve_seconds = 0.0;
  double last_final_cost = 0.0;
};

class PoseGraph {
 public:
  PoseGraph();
  ~PoseGraph();

  PoseGraph(const PoseGraph&) = delete;
  PoseGraph& operator=(const PoseGraph&) = delete;

  std::size_t AddNode(const Pose2D& initial_pose);
  std::size_t AddSubmap(const Pose2D& initial_pose);
  void AddConstraint(const PoseGraphConstraint& constraint);
  void AddNodeToSubmapConstraint(const NodeSubmapConstraint& constraint);
  void RequestOptimization();
  bool WaitForIdle(double timeout_seconds);

  bool GetOptimizedPose(std::size_t index, Pose2D* pose) const;
  std::vector<Pose2D> GetOptimizedPoses() const;
  bool GetOptimizedSubmapPose(std::size_t index, Pose2D* pose) const;
  std::vector<Pose2D> GetOptimizedSubmapPoses() const;
  PoseGraphStatus GetStatus() const;

 private:
  void WorkerLoop();
  void OptimizeSnapshot(const std::vector<Pose2D>& poses,
                        const std::vector<Pose2D>& submap_poses,
                        const std::vector<PoseGraphConstraint>& constraints,
                        const std::vector<double>& constraint_switches,
                        const std::vector<NodeSubmapConstraint>&
                            node_submap_constraints,
                        const std::vector<double>&
                            node_submap_constraint_switches);

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::condition_variable idle_condition_;
  std::vector<Pose2D> poses_;
  std::vector<Pose2D> submap_poses_;
  std::vector<PoseGraphConstraint> constraints_;
  // One switch value per constraint. Non-loop constraints remain at one.
  std::vector<double> constraint_switches_;
  std::vector<NodeSubmapConstraint> node_submap_constraints_;
  std::vector<double> node_submap_constraint_switches_;
  std::size_t suppressed_loop_constraints_ = 0;
  double minimum_loop_switch_ = 1.0;
  std::thread worker_;
  bool stop_ = false;
  bool optimization_requested_ = false;
  bool optimizing_ = false;
  std::size_t pose_version_ = 0;
  double last_solve_seconds_ = 0.0;
  double last_final_cost_ = 0.0;
};

}  // namespace lightweight_2d_slam
