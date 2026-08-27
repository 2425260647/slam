#include "nav_driver/my_navigation.h"
#include <cmath>
#include <algorithm>
#include <tf2/utils.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <cstdlib>
#include <ctime>
#include <random>
#include <limits>
#include <boost/bind.hpp>

const std::string NavigationDriver::VERSION = "3.2.1-real-map-frontier";

NavigationDriver::NavigationDriver(ros::NodeHandle& nh)
  : nh_(nh)
  , ac_("move_base", true)
  , tf_listener_(tf_buffer_)
  , map_received_(false)
  , object_found_(false)
  , task_finished_(false)
  , last_object_time_(ros::Time(0))
  , has_last_known_object_pose_(false)
  , last_map_width_(0)
  , last_map_height_(0)
  , last_map_change_time_(ros::Time(0))
  , map_stable_time_(5.0)
  , max_consecutive_failures_(5)
  , has_last_goal_(false)
  , fail_count_(0)
  , last_fail_time_(0)
  , last_goal_sent_time_(ros::Time::now())
  , consecutive_failures_(0)
  , last_goal_switch_time_(0)
  , last_vel_update_time_(0)
  , last_vel_set_(0.20)
  , last_goal_recheck_time_(0)
  , last_progress_check_time_(0)
  , has_last_robot_pose_(false)
  , random_goal_attempts_(0)
  , goal_request_pending_(true)
  , current_goal_is_object_(false)
  , last_planning_attempt_time_(0)
  , no_frontier_rotations_(0)
  , last_no_frontier_time_(0)
  , emergency_stopped_(false)
  , emergency_stop_duration_(1.0)
  , is_smart_rotating_(false)
  , accumulated_angle_(0.0)
  , max_rotate_angle_(M_PI)
  , last_rotate_check_time_(0.0)
  , is_direct_drive_(false)
  , last_direct_replan_time_(0)
  , has_scan_(false)
  , direct_drive_heading_(0.0)
  , current_max_vel_x_(0.20)
{
  ROS_INFO("========================================");
  ROS_INFO(" Navigation Driver Version: %s", VERSION.c_str());
  ROS_INFO("========================================");

  // 读取参数
  nh_.param("camera_offset_x", camera_offset_x_, 0.32);
  nh_.param("stop_distance", stop_distance_, 1.0);
  nh_.param("object_timeout", object_timeout_, 5.0);
  nh_.param("exploration_frequency", exploration_frequency_, 1.0);
  nh_.param("frontier_min_dist", frontier_min_dist_, 0.5);
  nh_.param("fov_horizontal", fov_horizontal_, 70.0);
  nh_.param("fov_range", fov_range_, 1.5);
  nh_.param("max_search_range", max_search_range_, 6.0);
  nh_.param("fallback_range", fallback_range_, 12.0);
  nh_.param("goal_timeout", goal_timeout_, 30.0);
  nh_.param("fail_backoff_time", fail_backoff_time_, 2.0);
  nh_.param("min_map_width", min_map_width_, 5.0);
  nh_.param("min_map_height", min_map_height_, 5.0);
  nh_.param("emergency_stop_dist", emergency_stop_dist_, 0.55);
  nh_.param("rotate_speed", rotate_speed_, 0.4);
  nh_.param("max_rotate_angle_deg", max_rotate_angle_, 180.0);
  nh_.param("start_delay", start_delay_, 10.0);
  nh_.param("force_random_dist", force_random_dist_, 2.0);
  nh_.param("min_goal_switch_interval", min_goal_switch_interval_, 3.0);
  nh_.param("vel_update_interval", vel_update_interval_, 1.0);
  nh_.param("vel_change_threshold", vel_change_threshold_, 0.1);
  nh_.param("goal_recheck_interval", goal_recheck_interval_, 3.0);
  nh_.param("heading_penalty_weight", heading_penalty_weight_, 0.8);
  nh_.param("progress_check_interval", progress_check_interval_, 5.0);
  nh_.param("min_progress_dist", min_progress_dist_, 0.1);
  nh_.param("min_progress_yaw", min_progress_yaw_, 0.15);
  nh_.param("max_random_attempts", max_random_attempts_, 3);
  // 直驱参数
  nh_.param("direct_drive_duration", direct_drive_duration_, 5.0);
  nh_.param("direct_drive_speed", direct_drive_speed_, 0.2);
  nh_.param("direct_drive_turn_speed", direct_drive_turn_speed_, 0.6);
  nh_.param("direct_drive_obstacle_threshold", direct_drive_obstacle_threshold_, 0.5);
  nh_.param("direct_drive_replan_interval", direct_drive_replan_interval_, 2.0);
  // 稳定性参数
  nh_.param("map_stable_time", map_stable_time_, 5.0);
  nh_.param("max_consecutive_failures", max_consecutive_failures_, 5);
  nh_.param("planning_retry_interval", planning_retry_interval_, 2.0);
  nh_.param("frontier_clearance", frontier_clearance_, 0.35);
  nh_.param("frontier_cluster_radius", frontier_cluster_radius_, 0.40);
  nh_.param("rotation_clearance", rotation_clearance_, 0.45);
  nh_.param("max_autonomous_speed", max_autonomous_speed_, 0.30);
  nh_.param("no_frontier_retry_interval", no_frontier_retry_interval_, 5.0);
  nh_.param("max_no_frontier_rotations", max_no_frontier_rotations_, 1);
  nh_.param("free_cell_threshold", free_cell_threshold_, 49);
  nh_.param("occupied_cell_threshold", occupied_cell_threshold_, 65);
  nh_.param("enable_direct_drive", enable_direct_drive_, false);

  free_cell_threshold_ = std::max(0, std::min(free_cell_threshold_, 49));
  occupied_cell_threshold_ = std::max(free_cell_threshold_ + 1,
                                      std::min(occupied_cell_threshold_, 100));
  max_autonomous_speed_ = std::max(0.05, std::min(max_autonomous_speed_, 0.30));
  max_no_frontier_rotations_ = std::max(0, max_no_frontier_rotations_);
  no_frontier_retry_interval_ = std::max(1.0, no_frontier_retry_interval_);

  max_rotate_angle_ = max_rotate_angle_ * M_PI / 180.0;
  node_start_time_ = ros::Time::now();
  last_goal_switch_time_ = node_start_time_;
  last_vel_update_time_ = node_start_time_;
  last_goal_recheck_time_ = node_start_time_;
  last_progress_check_time_ = node_start_time_;
  last_goal_sent_time_ = node_start_time_;
  last_planning_attempt_time_ = node_start_time_;

  tf_buffer_.setUsingDedicatedThread(true);

  object_sub_ = nh_.subscribe("/object_detected", 1, &NavigationDriver::objectCallback, this);
  map_sub_ = nh_.subscribe("/map", 1, &NavigationDriver::mapCallback, this);
  laser_sub_ = nh_.subscribe("/scan", 1, &NavigationDriver::laserCallback, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  ROS_INFO("Waiting for move_base action server...");
  ac_.waitForServer();
  ROS_INFO("Connected to move_base server");

  clear_costmap_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  clear_costmap_client_.waitForExistence();
  ROS_INFO("Connected to clear_costmaps service");

  explore_timer_ = nh_.createTimer(ros::Duration(1.0 / exploration_frequency_),
                                   &NavigationDriver::timerCallback, this, false, true);
  ROS_INFO("Navigation driver initialized (emergency stop at %.2f m, start delay %.1f s).",
           emergency_stop_dist_, start_delay_);
  ROS_INFO("Exploration mode: connected free-space frontiers, direct drive %s.",
           enable_direct_drive_ ? "ENABLED" : "DISABLED");
  ROS_INFO("Real-map thresholds: free <= %d, occupied >= %d, clearance %.2f m, speed cap %.2f m/s.",
           free_cell_threshold_, occupied_cell_threshold_, frontier_clearance_,
           max_autonomous_speed_);
}

NavigationDriver::~NavigationDriver()
{
  stopSmartRotation();
  stopDirectDrive();
  geometry_msgs::Twist stop;
  cmd_vel_pub_.publish(stop);
}

void NavigationDriver::setMaxVelocity(double vel)
{
  if (vel < 0.05) vel = 0.05;
  if (vel > max_autonomous_speed_) vel = max_autonomous_speed_;

  ros::Time now = ros::Time::now();
  if (fabs(vel - last_vel_set_) < vel_change_threshold_ &&
      (now - last_vel_update_time_).toSec() < vel_update_interval_)
    return;

  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  double_param.name = "max_vel_x";
  double_param.value = vel;
  srv_req.config.doubles.push_back(double_param);

  if (ros::service::call("/move_base/TebLocalPlannerROS/set_parameters", srv_req, srv_resp))
  {
    last_vel_set_ = vel;
    last_vel_update_time_ = now;
  }
  else
  {
    ROS_WARN_THROTTLE(5.0, "[DYN] Failed to set TEB max velocity via service.");
  }
}

void NavigationDriver::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_scan_ = *msg;
  has_scan_ = true;

  if (task_finished_) return;

  double min_dist = 9999.0;
  double angle_min = -fov_horizontal_ * M_PI / 180.0 / 2.0;
  double angle_max = +fov_horizontal_ * M_PI / 180.0 / 2.0;

  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    double angle = msg->angle_min + i * msg->angle_increment;
    if (angle < angle_min || angle > angle_max) continue;
    if (msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max) continue;
    if (msg->ranges[i] < min_dist) min_dist = msg->ranges[i];
  }

  if (min_dist < emergency_stop_dist_)
  {
    if (!emergency_stopped_)
    {
      ROS_WARN_THROTTLE(1.0, "[LASER] Obstacle at %.2f m < %.2f m! EMERGENCY STOP.",
                        min_dist, emergency_stop_dist_);
      emergency_stopped_ = true;
      emergency_stop_time_ = ros::Time::now();
      geometry_msgs::Twist stop;
      cmd_vel_pub_.publish(stop);
      ac_.cancelGoal();
      has_last_goal_ = false;
      current_goal_is_object_ = false;
      stopSmartRotation();
      stopDirectDrive();
      setMaxVelocity(0.05);
    }
  }
  else
  {
    double desired_max_vel = 0.0;
    if (min_dist > 3.0)      desired_max_vel = max_autonomous_speed_;
    else if (min_dist > 1.5) desired_max_vel = 0.35;
    else if (min_dist > 0.8) desired_max_vel = 0.2;
    else                     desired_max_vel = 0.1;

    current_max_vel_x_ = 0.7 * current_max_vel_x_ + 0.3 * desired_max_vel;
    setMaxVelocity(current_max_vel_x_);

    if (emergency_stopped_)
    {
      if ((ros::Time::now() - emergency_stop_time_).toSec() > emergency_stop_duration_)
      {
        emergency_stopped_ = false;
        ROS_INFO_THROTTLE(2.0, "[LASER] Path clear. Resuming exploration.");
        setMaxVelocity(current_max_vel_x_);
        if (!has_last_goal_)
          requestPlanning();
      }
    }
  }
}

void NavigationDriver::objectCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  try
  {
    geometry_msgs::PoseStamped pose_map;
    tf_buffer_.transform(*msg, pose_map, "map", ros::Duration(1.0));
    object_pose_map_ = pose_map.pose;
    object_found_ = true;
    last_object_time_ = ros::Time::now();
    last_known_object_pose_ = pose_map.pose;
    has_last_known_object_pose_ = true;
    fail_count_ = 0;
    consecutive_failures_ = 0;
    failed_frontiers_.clear();

    // P0修复1: 不立即清除emergency_stopped_，等待laserCallback确认路径安全
    if (emergency_stopped_) {
      ROS_WARN("[OBJECT] Object detected but emergency stop is active. Waiting for clear path...");
      // 不清除emergency_stopped_，不发送新目标
      return;
    }

    stopSmartRotation();
    stopDirectDrive();
    ROS_INFO("[OBJECT] Detected at (%.2f, %.2f). Approaching.",
             object_pose_map_.position.x, object_pose_map_.position.y);
    ac_.cancelGoal();
    has_last_goal_ = false;
    requestPlanning();
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_THROTTLE(5.0, "[TF] Object transform failed: %s", ex.what());
  }
}

void NavigationDriver::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  const bool geometry_changed = map_received_ &&
      (msg->info.width != current_map_.info.width ||
       msg->info.height != current_map_.info.height ||
       msg->info.resolution != current_map_.info.resolution ||
       msg->info.origin.position.x != current_map_.info.origin.position.x ||
       msg->info.origin.position.y != current_map_.info.origin.position.y);
  current_map_ = *msg;
  if (!map_received_)
  {
    map_received_ = true;
    last_map_width_ = current_map_.info.width;
    last_map_height_ = current_map_.info.height;
    last_map_change_time_ = ros::Time::now();
    ROS_INFO("[MAP] First map received. Size: %d x %d, resolution: %.3f",
             current_map_.info.width, current_map_.info.height,
             current_map_.info.resolution);
    requestPlanning();
  }
  else
  {
    if (static_cast<int>(current_map_.info.width) != last_map_width_ ||
        static_cast<int>(current_map_.info.height) != last_map_height_)
    {
      last_map_width_ = current_map_.info.width;
      last_map_height_ = current_map_.info.height;
      ROS_INFO_THROTTLE(2.0, "[MAP] Map size changed to %d x %d",
                        last_map_width_, last_map_height_);
    }
    if (geometry_changed)
    {
      last_map_width_ = current_map_.info.width;
      last_map_height_ = current_map_.info.height;
      failed_frontiers_.clear();
      ROS_INFO_THROTTLE(2.0,
          "[MAP] Geometry changed; cleared the cell-index blacklist without pausing online planning.");
    }
  }
}

void NavigationDriver::requestPlanning()
{
  goal_request_pending_ = true;
}

bool NavigationDriver::isMapStable()
{
  if (!map_received_) return false;
  return (ros::Time::now() - last_map_change_time_).toSec() > map_stable_time_;
}

bool NavigationDriver::worldToMap(double wx, double wy, int& mx, int& my) const
{
  if (!map_received_ || current_map_.info.resolution <= 0.0) return false;

  mx = static_cast<int>(std::floor(
      (wx - current_map_.info.origin.position.x) / current_map_.info.resolution));
  my = static_cast<int>(std::floor(
      (wy - current_map_.info.origin.position.y) / current_map_.info.resolution));
  return mx >= 0 && my >= 0 &&
         mx < static_cast<int>(current_map_.info.width) &&
         my < static_cast<int>(current_map_.info.height);
}

bool NavigationDriver::isKnownFreeCell(int mx, int my) const
{
  const int width = static_cast<int>(current_map_.info.width);
  const int height = static_cast<int>(current_map_.info.height);
  if (mx < 0 || my < 0 || mx >= width || my >= height) return false;

  const size_t index = static_cast<size_t>(my) * width + mx;
  if (index >= current_map_.data.size()) return false;
  const int value = current_map_.data[index];
  return value >= 0 && value <= free_cell_threshold_;
}

bool NavigationDriver::hasOccupiedNeighbor(int mx, int my, int radius) const
{
  const int width = static_cast<int>(current_map_.info.width);
  const int height = static_cast<int>(current_map_.info.height);
  for (int dy = -radius; dy <= radius; ++dy)
  {
    for (int dx = -radius; dx <= radius; ++dx)
    {
      if (dx * dx + dy * dy > radius * radius) continue;
      const int nx = mx + dx;
      const int ny = my + dy;
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) return true;

      const size_t index = static_cast<size_t>(ny) * width + nx;
      if (index >= current_map_.data.size()) return true;
      if (current_map_.data[index] >= occupied_cell_threshold_) return true;
    }
  }
  return false;
}

bool NavigationDriver::buildReachableMask(
    std::vector<unsigned char>& reachable, int& robot_mx, int& robot_my) const
{
  const int width = static_cast<int>(current_map_.info.width);
  const int height = static_cast<int>(current_map_.info.height);
  if (!map_received_ || width <= 0 || height <= 0 ||
      current_map_.data.size() != static_cast<size_t>(width * height))
  {
    return false;
  }

  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(5.0, "[GRID] TF map->base_link unavailable: %s", ex.what());
    return false;
  }

  if (!worldToMap(transform.transform.translation.x,
                  transform.transform.translation.y, robot_mx, robot_my))
  {
    ROS_WARN_THROTTLE(5.0, "[GRID] Robot pose is outside the occupancy grid.");
    return false;
  }

  if (!isKnownFreeCell(robot_mx, robot_my))
  {
    const int search_radius = std::max(
        1, static_cast<int>(std::ceil(0.5 / current_map_.info.resolution)));
    int best_x = -1;
    int best_y = -1;
    int best_distance_sq = std::numeric_limits<int>::max();
    for (int dy = -search_radius; dy <= search_radius; ++dy)
    {
      for (int dx = -search_radius; dx <= search_radius; ++dx)
      {
        const int distance_sq = dx * dx + dy * dy;
        if (distance_sq >= best_distance_sq) continue;
        if (isKnownFreeCell(robot_mx + dx, robot_my + dy))
        {
          best_x = robot_mx + dx;
          best_y = robot_my + dy;
          best_distance_sq = distance_sq;
        }
      }
    }
    if (best_x < 0)
    {
      ROS_WARN_THROTTLE(5.0, "[GRID] No known-free cell found near the robot.");
      return false;
    }
    robot_mx = best_x;
    robot_my = best_y;
  }

  reachable.assign(static_cast<size_t>(width * height), 0);
  std::vector<int> queue;
  queue.reserve(static_cast<size_t>(width * height));
  const int start_index = robot_my * width + robot_mx;
  reachable[start_index] = 1;
  queue.push_back(start_index);

  static const int kDx[4] = {1, -1, 0, 0};
  static const int kDy[4] = {0, 0, 1, -1};
  for (size_t head = 0; head < queue.size(); ++head)
  {
    const int index = queue[head];
    const int x = index % width;
    const int y = index / width;
    for (int direction = 0; direction < 4; ++direction)
    {
      const int nx = x + kDx[direction];
      const int ny = y + kDy[direction];
      if (!isKnownFreeCell(nx, ny)) continue;
      const int neighbor_index = ny * width + nx;
      if (reachable[neighbor_index]) continue;
      reachable[neighbor_index] = 1;
      queue.push_back(neighbor_index);
    }
  }
  return true;
}

bool NavigationDriver::snapGoalToReachableFreeCell(
    geometry_msgs::PoseStamped& goal, double tolerance) const
{
  std::vector<unsigned char> reachable;
  int robot_mx = 0;
  int robot_my = 0;
  if (!buildReachableMask(reachable, robot_mx, robot_my)) return false;

  int goal_mx = 0;
  int goal_my = 0;
  if (!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
    return false;

  const int width = static_cast<int>(current_map_.info.width);
  const int radius = std::max(
      0, static_cast<int>(std::ceil(tolerance / current_map_.info.resolution)));
  const int clearance_cells = std::max(
      1, static_cast<int>(std::ceil(frontier_clearance_ / current_map_.info.resolution)));
  int best_x = -1;
  int best_y = -1;
  double best_distance_sq = std::numeric_limits<double>::max();

  for (int dy = -radius; dy <= radius; ++dy)
  {
    for (int dx = -radius; dx <= radius; ++dx)
    {
      const int mx = goal_mx + dx;
      const int my = goal_my + dy;
      if (!isKnownFreeCell(mx, my)) continue;
      const size_t index = static_cast<size_t>(my) * width + mx;
      if (index >= reachable.size() || !reachable[index]) continue;
      if (hasOccupiedNeighbor(mx, my, clearance_cells)) continue;

      const double wx = current_map_.info.origin.position.x +
                        (mx + 0.5) * current_map_.info.resolution;
      const double wy = current_map_.info.origin.position.y +
                        (my + 0.5) * current_map_.info.resolution;
      const double dx_world = wx - goal.pose.position.x;
      const double dy_world = wy - goal.pose.position.y;
      const double distance_sq = dx_world * dx_world + dy_world * dy_world;
      if (distance_sq < best_distance_sq)
      {
        best_x = mx;
        best_y = my;
        best_distance_sq = distance_sq;
      }
    }
  }

  if (best_x < 0) return false;
  goal.pose.position.x = current_map_.info.origin.position.x +
                         (best_x + 0.5) * current_map_.info.resolution;
  goal.pose.position.y = current_map_.info.origin.position.y +
                         (best_y + 0.5) * current_map_.info.resolution;
  return true;
}

bool NavigationDriver::hasRotationClearance() const
{
  if (!has_scan_) return false;
  double minimum_range = std::numeric_limits<double>::max();
  bool has_valid_measurement = false;
  for (double raw_range : latest_scan_.ranges)
  {
    double range = raw_range;
    if (std::isinf(range) && range > 0.0) range = latest_scan_.range_max;
    if (!std::isfinite(range) ||
        range < latest_scan_.range_min || range > latest_scan_.range_max)
      continue;
    minimum_range = std::min(minimum_range, range);
    has_valid_measurement = true;
  }
  return has_valid_measurement && minimum_range >= rotation_clearance_;
}

void NavigationDriver::clearCostmaps()
{
  std_srvs::Empty srv;
  if (clear_costmap_client_.call(srv))
    ROS_INFO("[CLEAR] Costmaps cleared.");
  else
    ROS_WARN("[CLEAR] Failed to clear costmaps.");
}

void NavigationDriver::startDirectDrive()
{
  if (is_direct_drive_) return;
  if (!enable_direct_drive_)
  {
    ROS_WARN_THROTTLE(5.0, "[DIRECT] Disabled for real-robot safety; using bounded rotation recovery.");
    startSmartRotation();
    return;
  }
  ROS_WARN("[DIRECT] Starting direct drive mode.");
  is_direct_drive_ = true;
  direct_drive_start_ = ros::Time::now();
  last_direct_replan_time_ = ros::Time::now();
  direct_drive_heading_ = 0.0;
  ac_.cancelGoal();
  has_last_goal_ = false;
  stopSmartRotation();
}

void NavigationDriver::stopDirectDrive()
{
  if (!is_direct_drive_) return;
  is_direct_drive_ = false;
  geometry_msgs::Twist stop;
  cmd_vel_pub_.publish(stop);
  ROS_INFO("[DIRECT] Stopped direct drive mode.");
}

double NavigationDriver::findBestDirection()
{
  if (!has_scan_) return 0.0;

  const int num_sectors = 36;
  double sector_angle_range = 2 * M_PI / num_sectors;
  std::vector<double> sector_min_dist(num_sectors, 0.0);
  std::vector<bool> sector_observed(num_sectors, false);
  std::vector<double> sector_mid_angle(num_sectors);

  for (int i = 0; i < num_sectors; ++i)
  {
    double mid = -M_PI + (i + 0.5) * sector_angle_range;
    sector_mid_angle[i] = mid;
  }

  for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
  {
    double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;
    if (angle > M_PI) angle -= 2*M_PI;
    if (angle < -M_PI) angle += 2*M_PI;

    double range = latest_scan_.ranges[i];
    if (std::isinf(range) && range > 0.0) range = latest_scan_.range_max;
    if (!std::isfinite(range) ||
        range < latest_scan_.range_min || range > latest_scan_.range_max) continue;

    int sector = static_cast<int>((angle + M_PI) / sector_angle_range);
    if (sector >= num_sectors) sector = num_sectors - 1;
    if (sector < 0) sector = 0;

    if (!sector_observed[sector] || range < sector_min_dist[sector])
      sector_min_dist[sector] = range;
    sector_observed[sector] = true;
  }

  double best_angle = 0.0;
  double max_dist = 0.0;
  for (int i = 0; i < num_sectors; ++i)
  {
    if (sector_observed[i] && sector_min_dist[i] > max_dist)
    {
      max_dist = sector_min_dist[i];
      best_angle = sector_mid_angle[i];
    }
  }

  return best_angle;
}

void NavigationDriver::directDriveControl()
{
  if (!is_direct_drive_ || !has_scan_) return;

  if ((ros::Time::now() - direct_drive_start_).toSec() > direct_drive_duration_)
  {
    stopDirectDrive();
    requestPlanning();
    return;
  }

  if ((ros::Time::now() - last_direct_replan_time_).toSec() > direct_drive_replan_interval_)
  {
    last_direct_replan_time_ = ros::Time::now();
    if (isMapStable())
    {
      std::vector<std::pair<geometry_msgs::PoseStamped, double>> candidates;
      if (findFrontierCandidates(candidates, max_search_range_, std::set<std::pair<int,int>>()) ||
          findFrontierCandidates(candidates, fallback_range_, std::set<std::pair<int,int>>()))
      {
        stopDirectDrive();
        ROS_INFO("[DIRECT] Found a connected frontier, exiting direct drive.");
        requestPlanning();
        return;
      }
    }
  }

  double target_heading = findBestDirection();
  double front_clearance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
  {
    const double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;
    if (std::fabs(angle) > 0.35) continue;
    double range = latest_scan_.ranges[i];
    if (std::isinf(range) && range > 0.0) range = latest_scan_.range_max;
    if (!std::isfinite(range) ||
        range < latest_scan_.range_min || range > latest_scan_.range_max) continue;
    front_clearance = std::min(front_clearance, range);
  }
  geometry_msgs::Twist cmd;
  if (fabs(target_heading) < 0.3 &&
      front_clearance >= direct_drive_obstacle_threshold_)
  {
    cmd.linear.x = direct_drive_speed_;
    cmd.angular.z = std::max(-0.5, std::min(0.5, target_heading * 1.0));
  }
  else
  {
    cmd.linear.x = 0.0;
    cmd.angular.z = (target_heading > 0) ? direct_drive_turn_speed_ : -direct_drive_turn_speed_;
  }
  cmd_vel_pub_.publish(cmd);
}

void NavigationDriver::timerCallback(const ros::TimerEvent&)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (task_finished_ || emergency_stopped_) return;
  if (!map_received_) return;

  const ros::Time now = ros::Time::now();

  if (is_direct_drive_)
  {
    directDriveControl();
    return;
  }

  if ((now - node_start_time_).toSec() < start_delay_)
    return;

  if (object_found_ && (now - last_object_time_).toSec() > object_timeout_)
  {
    object_found_ = false;
    ROS_WARN_THROTTLE(5.0, "[TIMER] Object signal lost. Continuing to last known position...");
    if (has_last_known_object_pose_ && !has_last_goal_) requestPlanning();
  }

  actionlib::SimpleClientGoalState current_state = ac_.getState();
  bool has_active_goal = (current_state == actionlib::SimpleClientGoalState::ACTIVE ||
                          current_state == actionlib::SimpleClientGoalState::PENDING);

  if (object_found_)
  {
    try
    {
      const auto transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
      const double distance_to_object = distance(
          transform.transform.translation.x, transform.transform.translation.y,
          object_pose_map_.position.x, object_pose_map_.position.y);
      if (distance_to_object < stop_distance_ + 0.3)
      {
        ROS_INFO("[TIMER] Object is close enough (%.2f m). Task completed.",
                 distance_to_object);
        task_finished_ = true;
        ac_.cancelAllGoals();
        geometry_msgs::Twist stop;
        cmd_vel_pub_.publish(stop);
        return;
      }
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(5.0, "[TIMER] Robot pose unavailable: %s", ex.what());
    }
  }

  if (is_smart_rotating_) return;

  if (!has_active_goal && has_last_goal_ && current_state.isDone())
  {
    // The action callback normally handles this. This fallback prevents a stale local state.
    ROS_WARN_THROTTLE(5.0, "[TIMER] Goal is terminal but completion callback is pending.");
    return;
  }

  if (!has_active_goal && !has_last_goal_)
  {
    requestPlanning();
    if (no_frontier_rotations_ >= max_no_frontier_rotations_ &&
        !last_no_frontier_time_.isZero() &&
        (now - last_no_frontier_time_).toSec() < no_frontier_retry_interval_)
    {
      return;
    }
    if (!isMapStable())
    {
      ROS_INFO_THROTTLE(5.0, "[TIMER] Map geometry is still changing; waiting.");
      return;
    }
    if (goal_request_pending_ &&
        (now - last_planning_attempt_time_).toSec() >= planning_retry_interval_ &&
        (last_fail_time_.isZero() ||
         (now - last_fail_time_).toSec() >= fail_backoff_time_))
    {
      goal_request_pending_ = false;
      sendNextGoal();
    }
    return;
  }

  if (!has_active_goal || !has_last_goal_) return;

  const double goal_age = (now - last_goal_sent_time_).toSec();
  if (goal_age > goal_timeout_)
  {
    ROS_WARN("[TIMER] Goal timed out after %.1f s; canceling and backing off.", goal_age);
    if (!current_goal_is_object_)
    {
      int gx = 0;
      int gy = 0;
      if (worldToMap(last_goal_.pose.position.x, last_goal_.pose.position.y, gx, gy))
        failed_frontiers_.insert(std::make_pair(gx, gy));
    }
    ac_.cancelGoal();
    has_last_goal_ = false;
    current_goal_is_object_ = false;
    last_fail_time_ = now;
    ++consecutive_failures_;
    requestPlanning();
    return;
  }

  if ((now - last_goal_recheck_time_).toSec() >= goal_recheck_interval_)
  {
    last_goal_recheck_time_ = now;
    if (!isCurrentGoalReachable())
    {
      ROS_WARN("[TIMER] Goal left the connected free region; replanning.");
      ac_.cancelGoal();
      has_last_goal_ = false;
      current_goal_is_object_ = false;
      last_fail_time_ = now;
      requestPlanning();
      return;
    }
  }

  if ((now - last_progress_check_time_).toSec() < progress_check_interval_) return;
  last_progress_check_time_ = now;

  geometry_msgs::Pose current_pose;
  try
  {
    const auto transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    current_pose.position.x = transform.transform.translation.x;
    current_pose.position.y = transform.transform.translation.y;
    current_pose.orientation = transform.transform.rotation;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(5.0, "[TIMER] Progress check TF failed: %s", ex.what());
    return;
  }

  if (has_last_robot_pose_)
  {
    const double moved = distance(last_robot_pose_.position.x, last_robot_pose_.position.y,
                                  current_pose.position.x, current_pose.position.y);
    const double yaw_change = std::fabs(std::atan2(
        std::sin(tf2::getYaw(current_pose.orientation) -
                 tf2::getYaw(last_robot_pose_.orientation)),
        std::cos(tf2::getYaw(current_pose.orientation) -
                 tf2::getYaw(last_robot_pose_.orientation))));
    if (moved < min_progress_dist_ && yaw_change < min_progress_yaw_)
    {
      ROS_WARN("[TIMER] No progress (%.2f m, %.1f deg in %.1f s); stopping and replanning.",
               moved, yaw_change * 180.0 / M_PI, progress_check_interval_);
      if (!current_goal_is_object_)
      {
        int gx = 0;
        int gy = 0;
        if (worldToMap(last_goal_.pose.position.x, last_goal_.pose.position.y, gx, gy))
          failed_frontiers_.insert(std::make_pair(gx, gy));
      }
      ac_.cancelGoal();
      geometry_msgs::Twist stop;
      cmd_vel_pub_.publish(stop);
      has_last_goal_ = false;
      current_goal_is_object_ = false;
      last_fail_time_ = now;
      has_last_robot_pose_ = false;
      if (++consecutive_failures_ >= max_consecutive_failures_)
      {
        clearCostmaps();
        consecutive_failures_ = 0;
        startSmartRotation();
      }
      else
      {
        requestPlanning();
      }
      return;
    }
  }
  last_robot_pose_ = current_pose;
  has_last_robot_pose_ = true;
}

void NavigationDriver::doneCallback(const actionlib::SimpleClientGoalState& state,
                                    const move_base_msgs::MoveBaseResultConstPtr&)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (task_finished_ || emergency_stopped_) return;

  const bool goal_was_object = current_goal_is_object_;
  if (state == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    has_last_goal_ = false;
    current_goal_is_object_ = false;
    requestPlanning();
    return;
  }

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    has_last_goal_ = false;
    current_goal_is_object_ = false;
    consecutive_failures_ = 0;
    has_last_robot_pose_ = false;

    if (goal_was_object && object_found_)
    {
      task_finished_ = true;
      ROS_INFO("[DONE] TASK COMPLETED! Robot arrived at object.");
      ac_.cancelAllGoals();
      geometry_msgs::Twist stop;
      cmd_vel_pub_.publish(stop);
      return;
    }

    if (goal_was_object)
    {
      ROS_INFO("[DONE] Last known object position reached without detection; resuming exploration.");
      has_last_known_object_pose_ = false;
    }
    else
    {
      ROS_INFO_THROTTLE(2.0, "[DONE] Exploration goal reached.");
    }
    requestPlanning();
    return;
  }

  if (!goal_was_object && has_last_goal_)
  {
    int gx = 0;
    int gy = 0;
    if (worldToMap(last_goal_.pose.position.x, last_goal_.pose.position.y, gx, gy))
      failed_frontiers_.insert(std::make_pair(gx, gy));
  }

  ++fail_count_;
  ++consecutive_failures_;
  last_fail_time_ = ros::Time::now();
  has_last_goal_ = false;
  current_goal_is_object_ = false;
  has_last_robot_pose_ = false;
  ROS_WARN("[DONE] Goal failed (%s). Fail: %d, consecutive: %d.",
           state.toString().c_str(), fail_count_, consecutive_failures_);

  if (consecutive_failures_ >= max_consecutive_failures_)
  {
    clearCostmaps();
    consecutive_failures_ = 0;
    startSmartRotation();
  }
  else
  {
    requestPlanning();
  }
}

bool NavigationDriver::isCurrentGoalReachable()
{
  if (!has_last_goal_) return true;
  return isGoalReachable(last_goal_);
}

bool NavigationDriver::isSurrounded()
{
  std::vector<std::pair<geometry_msgs::PoseStamped, double>> candidates;
  return !findFrontierCandidates(
      candidates, 2.0, std::set<std::pair<int, int>>());
}

bool NavigationDriver::findBetterGoal(geometry_msgs::PoseStamped& better_goal)
{
  if (!has_last_goal_ || !map_received_) return false;

  std::vector<std::pair<geometry_msgs::PoseStamped, double>> candidates;
  bool found = false;

  if (has_last_known_object_pose_)
    found = searchNearLastObject(candidates);

  if (!found || candidates.empty())
  {
    found = findFrontierCandidates(candidates, max_search_range_, failed_frontiers_);
    if (!found || candidates.empty())
      found = findFrontierCandidates(candidates, fallback_range_, failed_frontiers_);
  }

  if (candidates.empty()) return false;

  double rx, ry;
  try {
    auto tf = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    rx = tf.transform.translation.x;
    ry = tf.transform.translation.y;
  } catch (...) { return false; }

  double current_dist = distance(rx, ry, last_goal_.pose.position.x, last_goal_.pose.position.y);

  for (auto& cand : candidates)
  {
    if (cand.second >= current_dist * 0.8)
      continue;
    better_goal = cand.first;
    return true;
  }
  return false;
}

void NavigationDriver::sendNextGoal()
{
  if (task_finished_ || emergency_stopped_ || is_direct_drive_) return;
  if (!map_received_) { ROS_INFO_THROTTLE(5.0, "[SEND] No map yet."); return; }

  last_planning_attempt_time_ = ros::Time::now();

  if ((ros::Time::now() - node_start_time_).toSec() < start_delay_)
  {
    ROS_INFO_THROTTLE(5.0, "[SEND] Waiting for start delay...");
    return;
  }

  if (!isMapStable())
  {
    ROS_INFO_THROTTLE(5.0, "[SEND] Map not stable yet, waiting...");
    requestPlanning();
    return;
  }

  geometry_msgs::PoseStamped new_goal;
  bool goal_is_object = false;

  if (object_found_)
  {
    if (!computeFinalGoal(object_pose_map_, new_goal))
    {
      ROS_WARN_THROTTLE(5.0, "[SEND] No connected approach pose for detected object.");
      requestPlanning();
      return;
    }
    goal_is_object = true;
  }
  else
  {
    if (has_last_known_object_pose_)
    {
      if (computeFinalGoal(last_known_object_pose_, new_goal))
      {
        goal_is_object = true;
      }
      else
      {
        ROS_WARN("[SEND] Last known object pose is not connected; returning to exploration.");
        has_last_known_object_pose_ = false;
      }
    }

    if (!goal_is_object)
    {
      std::vector<std::pair<geometry_msgs::PoseStamped, double>> candidates;
      bool found = findFrontierCandidates(candidates, max_search_range_, failed_frontiers_);
      if (!found || candidates.empty())
        found = findFrontierCandidates(candidates, fallback_range_, failed_frontiers_);
      if (!found || candidates.empty())
      {
        found = findFrontierCandidates(
            candidates, std::numeric_limits<double>::infinity(), failed_frontiers_);
      }

      if (!found || candidates.empty())
      {
        if (!failed_frontiers_.empty())
        {
          ROS_INFO("[SEND] No untried frontier remains; clearing the temporary blacklist once.");
          failed_frontiers_.clear();
          found = findFrontierCandidates(
              candidates, std::numeric_limits<double>::infinity(), failed_frontiers_);
        }
      }

      if (!found || candidates.empty())
      {
        last_no_frontier_time_ = ros::Time::now();
        requestPlanning();
        if (no_frontier_rotations_ < max_no_frontier_rotations_)
        {
          ++no_frontier_rotations_;
          ROS_WARN("[SEND] No connected frontier; running bounded rotation %d/%d.",
                   no_frontier_rotations_, max_no_frontier_rotations_);
          startSmartRotation();
        }
        else
        {
          ROS_WARN_THROTTLE(5.0,
              "[SEND] No connected frontier after bounded recovery; staying stopped and retrying every %.1f s.",
              no_frontier_retry_interval_);
        }
        return;
      }
      new_goal = candidates.front().first;
    }
  }

  last_goal_ = new_goal;
  no_frontier_rotations_ = 0;
  last_no_frontier_time_ = ros::Time(0);
  has_last_goal_ = true;
  current_goal_is_object_ = goal_is_object;
  goal_request_pending_ = false;
  last_goal_sent_time_ = ros::Time::now();
  last_goal_switch_time_ = ros::Time::now();
  random_goal_attempts_ = 0;

  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose = new_goal;
  ac_.sendGoal(mb_goal, boost::bind(&NavigationDriver::doneCallback, this, _1, _2));

  ROS_INFO("[SEND] %s goal: (%.2f, %.2f, yaw=%.1f deg).",
           goal_is_object ? "Object" : "Frontier",
           new_goal.pose.position.x, new_goal.pose.position.y,
           tf::getYaw(new_goal.pose.orientation) * 180.0 / M_PI);
}

void NavigationDriver::forceRandomGoal()
{
  ROS_WARN_THROTTLE(5.0, "[FORCE] Random goals are disabled; rotating to acquire new map data.");
  startSmartRotation();
}

bool NavigationDriver::isGoalReachable(const geometry_msgs::PoseStamped& goal)
{
  std::vector<unsigned char> reachable;
  int robot_mx = 0;
  int robot_my = 0;
  if (!buildReachableMask(reachable, robot_mx, robot_my)) return false;

  int goal_mx = 0;
  int goal_my = 0;
  if (!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
    return false;
  const int width = static_cast<int>(current_map_.info.width);
  const size_t index = static_cast<size_t>(goal_my) * width + goal_mx;
  return index < reachable.size() && reachable[index] &&
         !hasOccupiedNeighbor(
             goal_mx, goal_my,
             std::max(1, static_cast<int>(std::ceil(
                 frontier_clearance_ / current_map_.info.resolution))));
}

int NavigationDriver::countUnknownNeighbors(int cx, int cy, int radius)
{
  int count = 0;
  int w = current_map_.info.width, h = current_map_.info.height;
  for (int dy = -radius; dy <= radius; ++dy)
    for (int dx = -radius; dx <= radius; ++dx)
    {
      if (dx == 0 && dy == 0) continue;
      int nx = cx + dx, ny = cy + dy;
      if (nx >= 0 && nx < w && ny >= 0 && ny < h)
      {
        if (current_map_.data[ny * w + nx] == -1) ++count;
      }
    }
  return count;
}

bool NavigationDriver::findFrontierCandidates(
    std::vector<std::pair<geometry_msgs::PoseStamped, double>>& candidates,
    double search_range,
    const std::set<std::pair<int,int>>& blacklist)
{
  const ros::WallTime start_time = ros::WallTime::now();
  const size_t max_candidates = 20;
  const int width = static_cast<int>(current_map_.info.width);
  const int height = static_cast<int>(current_map_.info.height);
  const double resolution = current_map_.info.resolution;
  const double origin_x = current_map_.info.origin.position.x;
  const double origin_y = current_map_.info.origin.position.y;
  if (!map_received_ || width <= 0 || height <= 0 || resolution <= 0.0)
    return false;

  std::vector<unsigned char> reachable;
  int robot_mx = 0;
  int robot_my = 0;
  if (!buildReachableMask(reachable, robot_mx, robot_my)) return false;

  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  try
  {
    const geometry_msgs::TransformStamped transform =
        tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    robot_x = transform.transform.translation.x;
    robot_y = transform.transform.translation.y;
    tf2::Quaternion q;
    tf2::fromMsg(transform.transform.rotation, q);
    robot_yaw = tf2::getYaw(q);
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(5.0, "[FRONTIER] Robot pose unavailable: %s", ex.what());
    return false;
  }

  int min_x = 0;
  int max_x = width - 1;
  int min_y = 0;
  int max_y = height - 1;
  if (std::isfinite(search_range))
  {
    const int range_cells = std::max(
        0, static_cast<int>(std::ceil(search_range / resolution)));
    min_x = std::max(0, robot_mx - range_cells);
    max_x = std::min(width - 1, robot_mx + range_cells);
    min_y = std::max(0, robot_my - range_cells);
    max_y = std::min(height - 1, robot_my + range_cells);
  }

  const int clearance_cells = std::max(
      1, static_cast<int>(std::ceil(frontier_clearance_ / resolution)));
  std::vector<std::pair<geometry_msgs::PoseStamped, double>> raw_candidates;
  size_t reachable_cells_checked = 0;
  size_t uncertain_cells = 0;
  size_t adjacent_unknown_cells = 0;
  size_t clearance_rejections = 0;
  size_t blacklist_rejections = 0;
  size_t distance_rejections = 0;
  static const int kNeighborDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kNeighborDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  for (int y = min_y; y <= max_y; ++y)
  {
    for (int x = min_x; x <= max_x; ++x)
    {
      const size_t index = static_cast<size_t>(y) * width + x;
      if (index >= reachable.size()) continue;
      if (!reachable[index])
      {
        if (index < current_map_.data.size() &&
            current_map_.data[index] > free_cell_threshold_ &&
            current_map_.data[index] < occupied_cell_threshold_)
        {
          ++uncertain_cells;
        }
        continue;
      }
      ++reachable_cells_checked;

      bool adjacent_to_unknown = false;
      for (int neighbor = 0; neighbor < 8; ++neighbor)
      {
        const int nx = x + kNeighborDx[neighbor];
        const int ny = y + kNeighborDy[neighbor];
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
        const size_t neighbor_index = static_cast<size_t>(ny) * width + nx;
        if (neighbor_index < current_map_.data.size() &&
            current_map_.data[neighbor_index] == -1)
        {
          adjacent_to_unknown = true;
          break;
        }
      }
      if (!adjacent_to_unknown) continue;
      ++adjacent_unknown_cells;
      if (hasOccupiedNeighbor(x, y, clearance_cells))
      {
        ++clearance_rejections;
        continue;
      }

      bool blacklisted_nearby = false;
      for (const auto& failed : blacklist)
      {
        const double dx = (x - failed.first) * resolution;
        const double dy = (y - failed.second) * resolution;
        if (std::hypot(dx, dy) < frontier_cluster_radius_)
        {
          blacklisted_nearby = true;
          break;
        }
      }
      if (blacklisted_nearby)
      {
        ++blacklist_rejections;
        continue;
      }

      const double world_x = origin_x + (x + 0.5) * resolution;
      const double world_y = origin_y + (y + 0.5) * resolution;
      const double goal_distance = distance(robot_x, robot_y, world_x, world_y);
      if (goal_distance < frontier_min_dist_ ||
          (std::isfinite(search_range) && goal_distance > search_range))
      {
        ++distance_rejections;
        continue;
      }

      const double heading = std::atan2(world_y - robot_y, world_x - robot_x);
      const double heading_difference = std::fabs(
          std::atan2(std::sin(heading - robot_yaw), std::cos(heading - robot_yaw)));
      const int information_gain = countUnknownNeighbors(x, y, 4);
      const double cost = goal_distance +
                          heading_penalty_weight_ * heading_difference -
                          0.05 * information_gain;

      geometry_msgs::PoseStamped candidate;
      candidate.header.frame_id = "map";
      candidate.header.stamp = ros::Time::now();
      candidate.pose.position.x = world_x;
      candidate.pose.position.y = world_y;
      candidate.pose.orientation = tf2::toMsg(
          tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
      raw_candidates.push_back({candidate, cost});
    }
  }

  std::sort(raw_candidates.begin(), raw_candidates.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });
  candidates.clear();
  for (const auto& candidate : raw_candidates)
  {
    bool overlaps_cluster = false;
    for (const auto& selected : candidates)
    {
      if (distance(candidate.first.pose.position.x, candidate.first.pose.position.y,
                   selected.first.pose.position.x, selected.first.pose.position.y) <
          frontier_cluster_radius_)
      {
        overlaps_cluster = true;
        break;
      }
    }
    if (overlaps_cluster) continue;
    candidates.push_back(candidate);
    if (candidates.size() >= max_candidates) break;
  }

  const double elapsed = (ros::WallTime::now() - start_time).toSec();
  ROS_INFO_THROTTLE(2.0,
      "[FRONTIER] Search %.3f s: reachable=%zu uncertain=%zu adjacent=%zu "
      "reject(clearance=%zu blacklist=%zu distance=%zu) raw=%zu clustered=%zu.",
      elapsed, reachable_cells_checked, uncertain_cells, adjacent_unknown_cells,
      clearance_rejections, blacklist_rejections, distance_rejections,
      raw_candidates.size(), candidates.size());

  return !candidates.empty();
}

bool NavigationDriver::searchNearLastObject(
    std::vector<std::pair<geometry_msgs::PoseStamped, double>>& candidates)
{
  candidates.clear();
  if (!has_last_known_object_pose_ || !map_received_) return false;
  geometry_msgs::PoseStamped goal;
  if (!computeFinalGoal(last_known_object_pose_, goal)) return false;
  candidates.push_back({goal, 0.0});
  return true;
}

bool NavigationDriver::computeFinalGoal(const geometry_msgs::Pose& object_map,
                                        geometry_msgs::PoseStamped& goal)
{
  geometry_msgs::TransformStamped tf;
  try
  {
    tf = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_THROTTLE(5.0, "[COMPUTE] TF error: %s", ex.what());
    return false;
  }

  double rx = tf.transform.translation.x, ry = tf.transform.translation.y;
  double obj_x = object_map.position.x, obj_y = object_map.position.y;
  double angle_to_obj = atan2(obj_y - ry, obj_x - rx);

  double cam_x = obj_x - stop_distance_ * cos(angle_to_obj);
  double cam_y = obj_y - stop_distance_ * sin(angle_to_obj);
  double goal_x = cam_x - camera_offset_x_ * cos(angle_to_obj);
  double goal_y = cam_y - camera_offset_x_ * sin(angle_to_obj);

  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = goal_x;
  goal.pose.position.y = goal_y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), angle_to_obj));
  return snapGoalToReachableFreeCell(goal, 0.75);
}

void NavigationDriver::executeEscape()
{
  ROS_INFO("[ESCAPE] Starting escape procedure...");

  if (is_smart_rotating_) stopSmartRotation();
  if (is_direct_drive_) stopDirectDrive();
  ac_.cancelGoal();
  has_last_goal_ = false;
  clearCostmaps();

  geometry_msgs::PoseStamped current;
  try
  {
    geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    current.pose.position.x = tf.transform.translation.x;
    current.pose.position.y = tf.transform.translation.y;
    current.pose.orientation = tf.transform.rotation;
    current.header.frame_id = "map";
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR_THROTTLE(5.0, "[ESCAPE] TF error: %s", ex.what());
    startDirectDrive();
    return;
  }

  double yaw0 = tf2::getYaw(current.pose.orientation);
  std::vector<geometry_msgs::PoseStamped> attempts;

  // 优化：只生成10个高质量的脱困点，不是50+个
  // 优先尝试后退和侧移
  for (double dist : {1.5, 1.0, 0.5}) {
    for (double angle : {M_PI, M_PI*3/4, M_PI*5/4, M_PI/2, -M_PI/2}) {
      geometry_msgs::PoseStamped p = current;
      p.pose.position.x += dist * cos(yaw0 + angle);
      p.pose.position.y += dist * sin(yaw0 + angle);
      p.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), yaw0 + angle));
      attempts.push_back(p);
      if (attempts.size() >= 10) break;
    }
    if (attempts.size() >= 10) break;
  }

  ROS_INFO("[ESCAPE] Generated %zu escape candidates", attempts.size());

  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(attempts.begin(), attempts.end(), g);

  int checked = 0;
  for (const auto& goal : attempts) {
    checked++;
    ROS_INFO("[ESCAPE] Trying escape point %d/%zu...", checked, attempts.size());

    if (isGoalReachable(goal)) {
      ROS_INFO("[ESCAPE] Found escape goal: (%.2f, %.2f)",
               goal.pose.position.x, goal.pose.position.y);
      last_goal_ = goal;
      has_last_goal_ = true;
      current_goal_is_object_ = false;
      goal_request_pending_ = false;
      last_goal_sent_time_ = ros::Time::now();
      last_goal_switch_time_ = ros::Time::now();
      move_base_msgs::MoveBaseGoal mb_goal;
      mb_goal.target_pose = goal;
      ac_.sendGoal(mb_goal, boost::bind(&NavigationDriver::doneCallback, this, _1, _2));
      consecutive_failures_ = 0;
      return;
    }
  }

  ROS_WARN("[ESCAPE] No escape goal reachable after checking %d points. Starting direct drive.", checked);
  startDirectDrive();
}

void NavigationDriver::startSmartRotation()
{
  if (is_smart_rotating_) return;
  if (is_direct_drive_) stopDirectDrive();

  if (!hasRotationClearance())
  {
    ROS_WARN_THROTTLE(5.0,
        "[SMART_ROTATE] Insufficient all-around clearance (need %.2f m); staying stopped.",
        rotation_clearance_);
    requestPlanning();
    return;
  }

  const actionlib::SimpleClientGoalState rotation_action_state = ac_.getState();
  const bool rotation_goal_active =
      rotation_action_state == actionlib::SimpleClientGoalState::ACTIVE ||
      rotation_action_state == actionlib::SimpleClientGoalState::PENDING;
  if (rotation_goal_active)
    ac_.cancelGoal();
  has_last_goal_ = false;
  current_goal_is_object_ = false;

  is_smart_rotating_ = true;
  accumulated_angle_ = 0.0;
  rotate_start_time_ = ros::Time::now();
  last_rotate_check_time_ = 0.0;

  ROS_WARN("[SMART_ROTATE] Starting smart rotation (max %.1f°).",
           max_rotate_angle_ * 180.0 / M_PI);

  rotate_timer_ = nh_.createTimer(ros::Duration(0.05), [this](const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!is_smart_rotating_) return;

    if (emergency_stopped_ || !hasRotationClearance())
    {
      ROS_WARN_THROTTLE(2.0, "[SMART_ROTATE] Clearance lost; stopping rotation.");
      stopSmartRotation();
      requestPlanning();
      return;
    }

    double dt = (ros::Time::now() - rotate_start_time_).toSec();
    rotate_start_time_ = ros::Time::now();

    if (accumulated_angle_ >= max_rotate_angle_)
    {
      stopSmartRotation();
      ROS_INFO_THROTTLE(2.0, "[SMART_ROTATE] Half-turn completed. Re-evaluating...");
      requestPlanning();
      return;
    }

    geometry_msgs::Twist twist;
    twist.angular.z = rotate_speed_;
    cmd_vel_pub_.publish(twist);
    accumulated_angle_ += rotate_speed_ * dt;
  });
}

void NavigationDriver::stopSmartRotation()
{
  if (!is_smart_rotating_) return;
  is_smart_rotating_ = false;
  rotate_timer_.stop();
  geometry_msgs::Twist stop;
  cmd_vel_pub_.publish(stop);
  ROS_INFO_THROTTLE(2.0, "[SMART_ROTATE] Rotation stopped.");
}

double NavigationDriver::distance(double x1, double y1, double x2, double y2)
{
  return hypot(x1-x2, y1-y2);
}

int main(int argc, char** argv)
{
  srand(time(0));
  ros::init(argc, argv, "navigation_driver");
  ros::NodeHandle nh("~");
  NavigationDriver driver(nh);
  ros::spin();
  return 0;
}
