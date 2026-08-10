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
                       double rotation_weight,
                       bool anisotropic_translation,
                       double translation_direction_yaw,
                       double orthogonal_translation_weight)
      : measurement(measurement),
        translation_weight(translation_weight),
        rotation_weight(rotation_weight),
        anisotropic_translation(anisotropic_translation),
        translation_direction_yaw(translation_direction_yaw),
        orthogonal_translation_weight(orthogonal_translation_weight) {}

  template <typename T>
  bool operator()(const T* const from, const T* const to,
                  T* residuals) const {
    const T dx = to[0] - from[0];
    const T dy = to[1] - from[1];
    const T c = ceres::cos(from[2]);
    const T s = ceres::sin(from[2]);
    const T predicted_x = c * dx + s * dy;
    const T predicted_y = -s * dx + c * dy;
    const T error_x = predicted_x - T(measurement.x);
    const T error_y = predicted_y - T(measurement.y);
    if (anisotropic_translation) {
      const T direction_cosine = T(std::cos(translation_direction_yaw));
      const T direction_sine = T(std::sin(translation_direction_yaw));
      residuals[0] = T(translation_weight) *
                     (direction_cosine * error_x +
                      direction_sine * error_y);
      residuals[1] = T(orthogonal_translation_weight) *
                     (-direction_sine * error_x +
                      direction_cosine * error_y);
    } else {
      residuals[0] = T(translation_weight) * error_x;
      residuals[1] = T(translation_weight) * error_y;
    }
    residuals[2] = T(rotation_weight) *
                   ceres::sin((to[2] - from[2]) - T(measurement.yaw));
    return true;
  }

  Pose2D measurement;
  double translation_weight;
  double rotation_weight;
  bool anisotropic_translation;
  double translation_direction_yaw;
  double orthogonal_translation_weight;
};

// A switchable loop residual lets the optimizer reject a geometrically
// plausible but globally inconsistent loop without removing the edge from
// the graph. The switch prior keeps consistent loops near one.
struct SwitchableRelativePoseResidual {
  SwitchableRelativePoseResidual(
      const Pose2D& measurement, double translation_weight,
      double rotation_weight, bool anisotropic_translation,
      double translation_direction_yaw, double orthogonal_translation_weight)
      : measurement(measurement),
        translation_weight(translation_weight),
        rotation_weight(rotation_weight),
        anisotropic_translation(anisotropic_translation),
        translation_direction_yaw(translation_direction_yaw),
        orthogonal_translation_weight(orthogonal_translation_weight) {}

  template <typename T>
  bool operator()(const T* const from, const T* const to,
                  const T* const switch_value, T* residuals) const {
    const T dx = to[0] - from[0];
    const T dy = to[1] - from[1];
    const T c = ceres::cos(from[2]);
    const T s = ceres::sin(from[2]);
    const T predicted_x = c * dx + s * dy;
    const T predicted_y = -s * dx + c * dy;
    const T error_x = predicted_x - T(measurement.x);
    const T error_y = predicted_y - T(measurement.y);
    if (anisotropic_translation) {
      const T direction_cosine = T(std::cos(translation_direction_yaw));
      const T direction_sine = T(std::sin(translation_direction_yaw));
      residuals[0] = switch_value[0] * T(translation_weight) *
                     (direction_cosine * error_x +
                      direction_sine * error_y);
      residuals[1] = switch_value[0] * T(orthogonal_translation_weight) *
                     (-direction_sine * error_x +
                      direction_cosine * error_y);
    } else {
      residuals[0] = switch_value[0] * T(translation_weight) * error_x;
      residuals[1] = switch_value[0] * T(translation_weight) * error_y;
    }
    residuals[2] = switch_value[0] * T(rotation_weight) *
                   ceres::sin((to[2] - from[2]) - T(measurement.yaw));
    return true;
  }

  Pose2D measurement;
  double translation_weight;
  double rotation_weight;
  bool anisotropic_translation;
  double translation_direction_yaw;
  double orthogonal_translation_weight;
};

struct SwitchPriorResidual {
  explicit SwitchPriorResidual(double prior_weight)
      : prior_weight(prior_weight) {}

  template <typename T>
  bool operator()(const T* const switch_value, T* residuals) const {
    residuals[0] = T(prior_weight) * (T(1.0) - switch_value[0]);
    return true;
  }

  double prior_weight;
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

std::size_t PoseGraph::AddSubmap(const Pose2D& initial_pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  submap_poses_.push_back(initial_pose);
  return submap_poses_.size() - 1;
}

void PoseGraph::AddConstraint(const PoseGraphConstraint& constraint) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (constraint.from >= poses_.size() || constraint.to >= poses_.size() ||
      constraint.from == constraint.to) {
    return;
  }
  constraints_.push_back(constraint);
  constraint_switches_.push_back(1.0);
}

void PoseGraph::AddNodeToSubmapConstraint(
    const NodeSubmapConstraint& constraint) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (constraint.submap >= submap_poses_.size() ||
      constraint.node >= poses_.size()) {
    return;
  }
  node_submap_constraints_.push_back(constraint);
  node_submap_constraint_switches_.push_back(1.0);
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

std::vector<Pose2D> PoseGraph::GetOptimizedSubmapPoses() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return submap_poses_;
}

bool PoseGraph::GetOptimizedSubmapPose(std::size_t index,
                                       Pose2D* pose) const {
  if (pose == nullptr) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= submap_poses_.size()) return false;
  *pose = submap_poses_[index];
  return true;
}

PoseGraphStatus PoseGraph::GetStatus() const {
  std::lock_guard<std::mutex> lock(mutex_);
  PoseGraphStatus status;
  status.nodes = poses_.size();
  status.submaps = submap_poses_.size();
  status.constraints = constraints_.size() + node_submap_constraints_.size();
  status.node_submap_constraints = node_submap_constraints_.size();
  status.optimizing = optimizing_;
  status.suppressed_loop_constraints = suppressed_loop_constraints_;
  status.minimum_loop_switch = minimum_loop_switch_;
  status.pose_version = pose_version_;
  status.last_solve_seconds = last_solve_seconds_;
  status.last_final_cost = last_final_cost_;
  for (const auto& constraint : constraints_) {
    if (constraint.type == ConstraintType::kLoopClosure) {
      ++status.loop_constraints;
    }
  }
  for (const auto& constraint : node_submap_constraints_) {
    if (constraint.type == ConstraintType::kLoopClosure) {
      ++status.loop_constraints;
    }
  }
  return status;
}

void PoseGraph::WorkerLoop() {
  while (true) {
    std::vector<Pose2D> poses;
    std::vector<Pose2D> submap_poses;
    std::vector<PoseGraphConstraint> constraints;
    std::vector<double> constraint_switches;
    std::vector<NodeSubmapConstraint> node_submap_constraints;
    std::vector<double> node_submap_constraint_switches;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(lock, [this]() {
        return stop_ || optimization_requested_;
      });
      if (stop_) return;
      optimization_requested_ = false;
      optimizing_ = true;
      poses = poses_;
      submap_poses = submap_poses_;
      constraints = constraints_;
      constraint_switches = constraint_switches_;
      node_submap_constraints = node_submap_constraints_;
      node_submap_constraint_switches = node_submap_constraint_switches_;
    }

    OptimizeSnapshot(poses, submap_poses, constraints, constraint_switches,
                     node_submap_constraints,
                     node_submap_constraint_switches);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      optimizing_ = false;
    }
    idle_condition_.notify_all();
  }
}

void PoseGraph::OptimizeSnapshot(
    const std::vector<Pose2D>& poses,
    const std::vector<Pose2D>& submap_poses,
    const std::vector<PoseGraphConstraint>& constraints,
    const std::vector<double>& constraint_switches,
    const std::vector<NodeSubmapConstraint>& node_submap_constraints,
    const std::vector<double>& node_submap_constraint_switches) {
  if (poses.empty() ||
      (constraints.empty() && node_submap_constraints.empty())) {
    return;
  }
  const auto start = std::chrono::steady_clock::now();

  std::vector<std::array<double, 3>> parameters(poses.size());
  for (std::size_t i = 0; i < poses.size(); ++i) {
    parameters[i] = {{poses[i].x, poses[i].y, poses[i].yaw}};
  }
  std::vector<std::array<double, 3>> submap_parameters(submap_poses.size());
  for (std::size_t i = 0; i < submap_poses.size(); ++i) {
    submap_parameters[i] = {{submap_poses[i].x, submap_poses[i].y,
                             submap_poses[i].yaw}};
  }

  ceres::Problem problem;
  for (auto& parameter : parameters) problem.AddParameterBlock(parameter.data(), 3);
  for (auto& parameter : submap_parameters) {
    problem.AddParameterBlock(parameter.data(), 3);
  }
  if (!submap_parameters.empty()) {
    problem.SetParameterBlockConstant(submap_parameters.front().data());
  } else {
    problem.SetParameterBlockConstant(parameters.front().data());
  }

  std::vector<std::array<double, 1>> switch_parameters(constraints.size());
  std::vector<double> solved_switches(constraints.size(), 1.0);
  for (std::size_t index = 0; index < constraints.size(); ++index) {
    const double initial = index < constraint_switches.size()
                               ? constraint_switches[index]
                               : 1.0;
    switch_parameters[index] = {{std::max(0.0, std::min(1.0, initial))}};
    solved_switches[index] = switch_parameters[index][0];
  }

  for (std::size_t index = 0; index < constraints.size(); ++index) {
    const auto& constraint = constraints[index];
    if (constraint.from >= parameters.size() ||
        constraint.to >= parameters.size()) {
      continue;
    }
    ceres::CostFunction* cost = nullptr;
    if (constraint.type == ConstraintType::kLoopClosure &&
        constraint.switchable) {
      double* switch_value = switch_parameters[index].data();
      problem.AddParameterBlock(switch_value, 1);
      problem.SetParameterLowerBound(switch_value, 0, 0.0);
      problem.SetParameterUpperBound(switch_value, 0, 1.0);
      cost = new ceres::AutoDiffCostFunction<
          SwitchableRelativePoseResidual, 3, 3, 3, 1>(
          new SwitchableRelativePoseResidual(
              constraint.relative_pose, constraint.translation_weight,
              constraint.rotation_weight, constraint.anisotropic_translation,
              constraint.translation_direction_yaw,
              constraint.orthogonal_translation_weight));
      problem.AddResidualBlock(
          cost, new ceres::HuberLoss(1.0),
          parameters[constraint.from].data(), parameters[constraint.to].data(),
          switch_value);
      auto* prior = new ceres::AutoDiffCostFunction<SwitchPriorResidual, 1, 1>(
          new SwitchPriorResidual(std::sqrt(
              std::max(1e-6, constraint.switch_prior_weight))));
      problem.AddResidualBlock(prior, nullptr, switch_value);
      continue;
    }
    cost = new ceres::AutoDiffCostFunction<RelativePoseResidual, 3, 3, 3>(
        new RelativePoseResidual(
            constraint.relative_pose, constraint.translation_weight,
            constraint.rotation_weight, constraint.anisotropic_translation,
            constraint.translation_direction_yaw,
            constraint.orthogonal_translation_weight));
    ceres::LossFunction* loss = nullptr;
    if (constraint.type == ConstraintType::kLoopClosure) {
      loss = new ceres::HuberLoss(1.0);
    } else if (constraint.robust) {
      loss = new ceres::HuberLoss(
          std::max(1e-6, constraint.robust_loss_scale));
    }
    problem.AddResidualBlock(cost, loss, parameters[constraint.from].data(),
                             parameters[constraint.to].data());
  }

  std::vector<std::array<double, 1>> node_submap_switch_parameters(
      node_submap_constraints.size());
  std::vector<double> solved_node_submap_switches(
      node_submap_constraints.size(), 1.0);
  for (std::size_t index = 0; index < node_submap_constraints.size(); ++index) {
    const double initial = index < node_submap_constraint_switches.size()
                               ? node_submap_constraint_switches[index]
                               : 1.0;
    node_submap_switch_parameters[index] =
        {{std::max(0.0, std::min(1.0, initial))}};
    solved_node_submap_switches[index] =
        node_submap_switch_parameters[index][0];
  }
  for (std::size_t index = 0; index < node_submap_constraints.size(); ++index) {
    const auto& constraint = node_submap_constraints[index];
    if (constraint.submap >= submap_parameters.size() ||
        constraint.node >= parameters.size()) {
      continue;
    }
    ceres::CostFunction* cost = nullptr;
    if (constraint.type == ConstraintType::kLoopClosure &&
        constraint.switchable) {
      double* switch_value = node_submap_switch_parameters[index].data();
      problem.AddParameterBlock(switch_value, 1);
      problem.SetParameterLowerBound(switch_value, 0, 0.0);
      problem.SetParameterUpperBound(switch_value, 0, 1.0);
      cost = new ceres::AutoDiffCostFunction<
          SwitchableRelativePoseResidual, 3, 3, 3, 1>(
          new SwitchableRelativePoseResidual(
              constraint.relative_pose, constraint.translation_weight,
              constraint.rotation_weight,
              constraint.anisotropic_translation,
              constraint.translation_direction_yaw,
              constraint.orthogonal_translation_weight));
      problem.AddResidualBlock(
          cost, new ceres::HuberLoss(1.0),
          submap_parameters[constraint.submap].data(),
          parameters[constraint.node].data(), switch_value);
      auto* prior = new ceres::AutoDiffCostFunction<SwitchPriorResidual, 1, 1>(
          new SwitchPriorResidual(std::sqrt(
              std::max(1e-6, constraint.switch_prior_weight))));
      problem.AddResidualBlock(prior, nullptr, switch_value);
      continue;
    }
    cost = new ceres::AutoDiffCostFunction<RelativePoseResidual, 3, 3, 3>(
        new RelativePoseResidual(
            constraint.relative_pose, constraint.translation_weight,
            constraint.rotation_weight,
            constraint.anisotropic_translation,
            constraint.translation_direction_yaw,
            constraint.orthogonal_translation_weight));
    ceres::LossFunction* loss = nullptr;
    if (constraint.type == ConstraintType::kLoopClosure) {
      loss = new ceres::HuberLoss(1.0);
    } else if (constraint.robust) {
      loss = new ceres::HuberLoss(
          std::max(1e-6, constraint.robust_loss_scale));
    }
    problem.AddResidualBlock(
        cost, loss, submap_parameters[constraint.submap].data(),
        parameters[constraint.node].data());
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
    const std::size_t apply_submap_count =
        std::min(submap_poses_.size(), submap_parameters.size());
    for (std::size_t i = 0; i < apply_submap_count; ++i) {
      submap_poses_[i] = {submap_parameters[i][0], submap_parameters[i][1],
                          NormalizeAngle(submap_parameters[i][2])};
    }
    for (std::size_t index = 0; index < constraints.size(); ++index) {
      solved_switches[index] = std::max(
          0.0, std::min(1.0, switch_parameters[index][0]));
    }
    for (std::size_t index = 0; index < node_submap_constraints.size();
         ++index) {
      solved_node_submap_switches[index] = std::max(
          0.0, std::min(1.0, node_submap_switch_parameters[index][0]));
    }
    const std::size_t switch_count =
        std::min(constraint_switches_.size(), solved_switches.size());
    for (std::size_t index = 0; index < switch_count; ++index) {
      constraint_switches_[index] = solved_switches[index];
    }
    const std::size_t node_submap_switch_count = std::min(
        node_submap_constraint_switches_.size(),
        solved_node_submap_switches.size());
    for (std::size_t index = 0; index < node_submap_switch_count; ++index) {
      node_submap_constraint_switches_[index] =
          solved_node_submap_switches[index];
    }
    suppressed_loop_constraints_ = 0;
    minimum_loop_switch_ = 1.0;
    for (std::size_t index = 0;
         index < constraints_.size() && index < constraint_switches_.size();
         ++index) {
      if (constraints_[index].type != ConstraintType::kLoopClosure) continue;
      minimum_loop_switch_ =
          std::min(minimum_loop_switch_, constraint_switches_[index]);
      if (constraints_[index].switchable &&
          constraint_switches_[index] < 0.5) {
        ++suppressed_loop_constraints_;
      }
    }
    for (std::size_t index = 0;
         index < node_submap_constraints_.size() &&
         index < node_submap_constraint_switches_.size();
         ++index) {
      if (node_submap_constraints_[index].type !=
          ConstraintType::kLoopClosure) {
        continue;
      }
      minimum_loop_switch_ = std::min(
          minimum_loop_switch_, node_submap_constraint_switches_[index]);
      if (node_submap_constraints_[index].switchable &&
          node_submap_constraint_switches_[index] < 0.5) {
        ++suppressed_loop_constraints_;
      }
    }
    ++pose_version_;
  }
  last_solve_seconds_ = elapsed;
  last_final_cost_ = summary.final_cost;
}

}  // namespace lightweight_2d_slam
