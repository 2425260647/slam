#include "lightweight_2d_slam/pose_graph.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include <ceres/ceres.h>

namespace lightweight_2d_slam {
namespace {

struct RelativePoseResidual {
  RelativePoseResidual(const Pose2D& measurement, double translation_weight,
                       double rotation_weight)
      : measurement(measurement),
        translation_weight(translation_weight),
        rotation_weight(rotation_weight) {}

  template <typename T>
  bool operator()(const T* const from, const T* const to,
                  T* residuals) const {
    const T dx = to[0] - from[0];
    const T dy = to[1] - from[1];
    const T c = ceres::cos(from[2]);
    const T s = ceres::sin(from[2]);
    const T predicted_x = c * dx + s * dy;
    const T predicted_y = -s * dx + c * dy;
    residuals[0] = T(translation_weight) *
                   (predicted_x - T(measurement.x));
    residuals[1] = T(translation_weight) *
                   (predicted_y - T(measurement.y));
    residuals[2] = T(rotation_weight) *
                   ceres::sin((to[2] - from[2]) - T(measurement.yaw));
    return true;
  }

  Pose2D measurement;
  double translation_weight;
  double rotation_weight;
};

}  // namespace

PoseGraph::PoseGraph() : worker_(&PoseGraph::WorkerLoop, this) {}

PoseGraph::~PoseGraph() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  condition_.notify_all();
  if (worker_.joinable()) worker_.join();
}

std::size_t PoseGraph::AddNode(const Pose2D& initial_pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  poses_.push_back(initial_pose);
  return poses_.size() - 1;
}

void PoseGraph::AddConstraint(const PoseGraphConstraint& constraint) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (constraint.from >= poses_.size() || constraint.to >= poses_.size() ||
      constraint.from == constraint.to) {
    return;
  }
  constraints_.push_back(constraint);
}

void PoseGraph::RequestOptimization() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (poses_.size() < 2) return;
    optimization_requested_ = true;
  }
  condition_.notify_one();
}

bool PoseGraph::WaitForIdle(double timeout_seconds) {
  std::unique_lock<std::mutex> lock(mutex_);
  return idle_condition_.wait_for(
      lock, std::chrono::duration<double>(timeout_seconds), [this]() {
        return !optimizing_ && !optimization_requested_;
      });
}

std::vector<Pose2D> PoseGraph::GetOptimizedPoses() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return poses_;
}

bool PoseGraph::GetOptimizedPose(std::size_t index, Pose2D* pose) const {
  if (pose == nullptr) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= poses_.size()) return false;
  *pose = poses_[index];
  return true;
}

PoseGraphStatus PoseGraph::GetStatus() const {
  std::lock_guard<std::mutex> lock(mutex_);
  PoseGraphStatus status;
  status.nodes = poses_.size();
  status.constraints = constraints_.size();
  status.optimizing = optimizing_;
  status.pose_version = pose_version_;
  status.last_solve_seconds = last_solve_seconds_;
  status.last_final_cost = last_final_cost_;
  for (const auto& constraint : constraints_) {
    if (constraint.type == ConstraintType::kLoopClosure) {
      ++status.loop_constraints;
    }
  }
  return status;
}

void PoseGraph::WorkerLoop() {
  while (true) {
    std::vector<Pose2D> poses;
    std::vector<PoseGraphConstraint> constraints;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this]() {
        return stop_ || optimization_requested_;
      });
      if (stop_) return;
      optimization_requested_ = false;
      optimizing_ = true;
      poses = poses_;
      constraints = constraints_;
    }

    OptimizeSnapshot(poses, constraints);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      optimizing_ = false;
    }
    idle_condition_.notify_all();
  }
}

void PoseGraph::OptimizeSnapshot(
    const std::vector<Pose2D>& poses,
    const std::vector<PoseGraphConstraint>& constraints) {
  if (poses.size() < 2) return;
  const auto start = std::chrono::steady_clock::now();

  std::vector<std::array<double, 3>> parameters(poses.size());
  for (std::size_t i = 0; i < poses.size(); ++i) {
    parameters[i] = {{poses[i].x, poses[i].y, poses[i].yaw}};
  }

  ceres::Problem problem;
  for (auto& parameter : parameters) problem.AddParameterBlock(parameter.data(), 3);
  problem.SetParameterBlockConstant(parameters.front().data());

  for (const auto& constraint : constraints) {
    if (constraint.from >= parameters.size() ||
        constraint.to >= parameters.size()) {
      continue;
    }
    auto* cost = new ceres::AutoDiffCostFunction<RelativePoseResidual, 3, 3, 3>(
        new RelativePoseResidual(constraint.relative_pose,
                                 constraint.translation_weight,
                                 constraint.rotation_weight));
    ceres::LossFunction* loss = nullptr;
    if (constraint.type == ConstraintType::kLoopClosure) {
      loss = new ceres::HuberLoss(1.0);
    }
    problem.AddResidualBlock(cost, loss, parameters[constraint.from].data(),
                             parameters[constraint.to].data());
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.max_num_iterations = 40;
  options.num_threads = 2;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  const auto end = std::chrono::steady_clock::now();
  const double elapsed = std::chrono::duration<double>(end - start).count();
  std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t apply_count = std::min(poses_.size(), parameters.size());
  if (summary.IsSolutionUsable()) {
    for (std::size_t i = 0; i < apply_count; ++i) {
      poses_[i] = {parameters[i][0], parameters[i][1],
                   NormalizeAngle(parameters[i][2])};
    }
    ++pose_version_;
  }
  last_solve_seconds_ = elapsed;
  last_final_cost_ = summary.final_cost;
}

}  // namespace lightweight_2d_slam
