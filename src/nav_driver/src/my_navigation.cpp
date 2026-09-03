#include "nav_driver/my_navigation.h"
#include <cmath>
#include <algorithm>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <cstdlib>
#include <ctime>
#include <random>
#include <limits>
#include <sstream>
#include <queue>
#include <boost/bind.hpp>

const std::string NavigationDriver::VERSION = "3.4.1-autonomous-navigation";

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
  , goal_generation_(0)
  , last_safety_publish_time_(0)
  , safety_stop_atomic_(true)
  , last_scan_received_nsec_(0)
  , emergency_stopped_(false)
  , emergency_stop_duration_(1.0)
  , emergency_stop_candidate_count_(0)
  , emergency_clear_candidate_count_(0)
  , is_smart_rotating_(false)
  , accumulated_angle_(0.0)
  , max_rotate_angle_(M_PI)
  , is_escape_backing_(false)
  , escape_backoff_start_(0)
  , last_rotate_check_time_(0.0)
  , is_direct_drive_(false)
  , last_direct_replan_time_(0)
  , has_scan_(false)
  , direct_drive_heading_(0.0)
  , current_max_vel_x_(0.20)
  , last_diagnostic_log_time_(0)
  , last_scan_received_time_(0)
  , last_scan_stamp_(0)
  , last_map_stamp_(0)
  , last_scan_tf_status_("unavailable")
  , last_scan_tf_x_(0.0)
  , last_scan_tf_y_(0.0)
  , last_scan_tf_yaw_(0.0)
  , last_scan_tf_age_(-1.0)
  , last_scan_angle_min_(0.0)
  , last_scan_angle_max_(0.0)
  , last_scan_angle_increment_(0.0)
  , last_scan_range_min_(0.0)
  , last_scan_range_max_(0.0)
  , last_scan_total_count_(0)
  , last_scan_min_distance_(-1.0)
  , last_scan_min_laser_angle_(0.0)
  , last_scan_min_base_angle_(0.0)
  , last_scan_min_angle_(0.0)
  , last_center_min_distance_(-1.0)
  , last_center_valid_count_(0)
  , last_center_near_count_(0)
  , last_center_floor_count_(0)
  , last_center_cluster_(0)
  , last_scan_valid_count_(0)
  , last_scan_invalid_count_(0)
  , last_cmd_vel_time_(0)
  , last_odom_received_time_(0)
  , last_odom_stamp_(0)
  , has_global_footprint_(false)
  , has_local_footprint_(false)
{
  last_front_sector_min_distance_.fill(-1.0);
  ROS_INFO("========================================");
  ROS_INFO(" Navigation Driver Version: %s", VERSION.c_str());
  ROS_INFO("========================================");

  // 读取参数
  nh_.param("camera_offset_x", camera_offset_x_, 0.32);
  nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
  nh_.param("stop_distance", stop_distance_, 1.0);
  nh_.param("object_timeout", object_timeout_, 5.0);
  nh_.param("exploration_frequency", exploration_frequency_, 1.0);
  nh_.param("frontier_min_dist", frontier_min_dist_, 0.5);
  nh_.param("fov_horizontal", fov_horizontal_, 70.0);
  nh_.param("fov_range", fov_range_, 1.5);
  nh_.param("max_search_range", max_search_range_, 6.0);
  nh_.param("fallback_range", fallback_range_, 12.0);
  nh_.param("goal_timeout", goal_timeout_, 22.0);
  nh_.param("fail_backoff_time", fail_backoff_time_, 2.0);
  nh_.param("min_map_width", min_map_width_, 5.0);
  nh_.param("min_map_height", min_map_height_, 5.0);
  nh_.param("emergency_stop_dist", emergency_stop_dist_, 0.45);
  nh_.param("emergency_stop_hard_dist", emergency_stop_hard_dist_, 0.36);
  nh_.param("emergency_clear_margin", emergency_clear_margin_, 0.08);
  nh_.param("emergency_range_floor_margin", emergency_range_floor_margin_, 0.03);
  nh_.param("emergency_min_near_points", emergency_min_near_points_, 3);
  nh_.param("emergency_min_cluster_points", emergency_min_cluster_points_, 3);
  nh_.param("scan_timeout", scan_timeout_, 0.50);
  nh_.param("scan_tf_max_age", scan_tf_max_age_, 0.25);
  nh_.param("scan_min_valid_count", scan_min_valid_count_, 3);
  nh_.param("emergency_stop_confirm_scans", emergency_stop_confirm_scans_, 2);
  nh_.param("emergency_clear_confirm_scans", emergency_clear_confirm_scans_, 3);
  nh_.param("rotate_speed", rotate_speed_, 0.4);
  nh_.param("max_rotate_angle_deg", max_rotate_angle_, 180.0);
  nh_.param("start_delay", start_delay_, 10.0);
  nh_.param("force_random_dist", force_random_dist_, 2.0);
  nh_.param("min_goal_switch_interval", min_goal_switch_interval_, 3.0);
  nh_.param("vel_update_interval", vel_update_interval_, 1.0);
  nh_.param("vel_change_threshold", vel_change_threshold_, 0.1);
  nh_.param("goal_recheck_interval", goal_recheck_interval_, 3.0);
  nh_.param("goal_recheck_confirmations", goal_recheck_confirmations_, 3);
  nh_.param("heading_penalty_weight", heading_penalty_weight_, 0.45);
  nh_.param("progress_check_interval", progress_check_interval_, 4.0);
  nh_.param("min_progress_dist", min_progress_dist_, 0.1);
  nh_.param("min_progress_yaw", min_progress_yaw_, 0.15);
  nh_.param("progress_grace_period", progress_grace_period_, 10.0);
  nh_.param("progress_confirmations", progress_confirmations_, 2);
  nh_.param("frontier_unknown_weight", frontier_unknown_weight_, 0.08);
  nh_.param("frontier_branch_weight", frontier_branch_weight_, 0.06);
  nh_.param("max_random_attempts", max_random_attempts_, 3);
  // 直驱参数
  nh_.param("direct_drive_duration", direct_drive_duration_, 5.0);
  nh_.param("direct_drive_speed", direct_drive_speed_, 0.2);
  nh_.param("direct_drive_turn_speed", direct_drive_turn_speed_, 0.6);
  nh_.param("direct_drive_obstacle_threshold", direct_drive_obstacle_threshold_, 0.5);
  nh_.param("direct_drive_replan_interval", direct_drive_replan_interval_, 2.0);
  // 稳定性参数
  nh_.param("map_stable_time", map_stable_time_, 1.0);
  nh_.param("max_consecutive_failures", max_consecutive_failures_, 5);
  nh_.param("planning_retry_interval", planning_retry_interval_, 2.0);
  nh_.param("frontier_clearance", frontier_clearance_, 0.25);
  nh_.param("frontier_cluster_radius", frontier_cluster_radius_, 0.45);
  nh_.param("frontier_search_timeout", frontier_search_timeout_, 0.25);
  nh_.param("frontier_max_raw_candidates", frontier_max_raw_candidates_, 4000);
  nh_.param("rotation_clearance", rotation_clearance_, 0.45);
  nh_.param("escape_backoff_duration", escape_backoff_duration_, 1.5);
  nh_.param("max_autonomous_speed", max_autonomous_speed_, 0.25);
  nh_.param("no_frontier_retry_interval", no_frontier_retry_interval_, 5.0);
  nh_.param("max_no_frontier_rotations", max_no_frontier_rotations_, 1);
  nh_.param("free_cell_threshold", free_cell_threshold_, 49);
  nh_.param("occupied_cell_threshold", occupied_cell_threshold_, 65);
  nh_.param("enable_direct_drive", enable_direct_drive_, false);
  nh_.param<std::string>("safety_stop_topic", safety_stop_topic_,
                         "/navigation_driver/safety_stop");

  free_cell_threshold_ = std::max(0, std::min(free_cell_threshold_, 49));
  occupied_cell_threshold_ = std::max(free_cell_threshold_ + 1,
                                      std::min(occupied_cell_threshold_, 100));
  // Keep max_vel_x above the TEB penalty epsilon used by this launch.
  max_autonomous_speed_ = std::max(0.15, std::min(max_autonomous_speed_, 0.30));
  max_no_frontier_rotations_ = std::max(0, max_no_frontier_rotations_);
  no_frontier_retry_interval_ = std::max(1.0, no_frontier_retry_interval_);
  goal_recheck_interval_ = std::max(1.0, goal_recheck_interval_);
  goal_recheck_confirmations_ = std::max(2, goal_recheck_confirmations_);
  goal_unreachable_count_ = 0;
  progress_check_interval_ = std::max(2.0, progress_check_interval_);
  progress_grace_period_ = std::max(progress_check_interval_, progress_grace_period_);
  progress_confirmations_ = std::max(2, progress_confirmations_);
  progress_stall_count_ = 0;
  frontier_unknown_weight_ = std::max(0.0, frontier_unknown_weight_);
  frontier_branch_weight_ = std::max(0.0, frontier_branch_weight_);
  frontier_search_timeout_ = std::max(0.05, frontier_search_timeout_);
  frontier_max_raw_candidates_ = std::max(100, frontier_max_raw_candidates_);
  escape_backoff_duration_ = std::max(0.5, std::min(escape_backoff_duration_, 3.0));
  emergency_stop_hard_dist_ = std::max(0.10, std::min(emergency_stop_hard_dist_, emergency_stop_dist_));
  emergency_clear_margin_ = std::max(0.02, emergency_clear_margin_);
  emergency_range_floor_margin_ = std::max(0.0, std::min(emergency_range_floor_margin_, 0.10));
  emergency_min_near_points_ = std::max(2, emergency_min_near_points_);
  emergency_min_cluster_points_ = std::max(2, emergency_min_cluster_points_);
  scan_timeout_ = std::max(0.10, scan_timeout_);
  scan_tf_max_age_ = std::max(0.05, scan_tf_max_age_);
  scan_min_valid_count_ = std::max(1, scan_min_valid_count_);
  emergency_stop_confirm_scans_ = std::max(1, emergency_stop_confirm_scans_);
  emergency_clear_confirm_scans_ = std::max(1, emergency_clear_confirm_scans_);

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
  odom_sub_ = nh_.subscribe("/odom", 10, &NavigationDriver::odomCallback, this);
  global_footprint_sub_ = nh_.subscribe(
      "/move_base/global_costmap/footprint", 1,
      &NavigationDriver::globalFootprintCallback, this);
  local_footprint_sub_ = nh_.subscribe(
      "/move_base/local_costmap/footprint", 1,
      &NavigationDriver::localFootprintCallback, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
  safety_stop_pub_ = nh_.advertise<std_msgs::Bool>(safety_stop_topic_, 1, true);
  // This callback never takes data_mutex_.  It keeps the arbiter's safety
  // heartbeat alive while planning or a clear-costmaps service call is slow.
  safety_heartbeat_timer_ = nh_.createTimer(
      ros::Duration(0.10), &NavigationDriver::safetyHeartbeatCallback, this);
  // Fail safe until the first usable scan has been transformed and checked.
  publishSafetyStop(true);

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
  ROS_INFO("[CONFIG] Topics: map=/map scan=/scan odom=/odom cmd_vel=%s safety_stop=%s action=/move_base",
           cmd_vel_topic_.c_str(), safety_stop_topic_.c_str());
  ROS_INFO("[CONFIG] Frames: global=map local=odom base=base_link; scan FOV=%.1f deg; object_fov_range=%.2f m.",
           fov_horizontal_, fov_range_);
  ROS_INFO("[CONFIG] Laser emergency FOV is base_link [%.1f,%.1f] deg; laser zero is transformed through base_link<-laser_link. confirm=%d clear=%d hard_stop=%.2f m near_points=%d cluster_points=%d floor_margin=%.3f m.",
           -fov_horizontal_ / 2.0, fov_horizontal_ / 2.0,
           emergency_stop_confirm_scans_, emergency_clear_confirm_scans_, emergency_stop_hard_dist_,
           emergency_min_near_points_, emergency_min_cluster_points_, emergency_range_floor_margin_);
  ROS_INFO("[CONFIG] Runtime footprint topics: global=/move_base/global_costmap/footprint local=/move_base/local_costmap/footprint.");
  ROS_INFO("[CONFIG] Limits: max_vel_x=%.2f m/s max_vel_theta=%.2f rad/s emergency_stop=%.2f m.",
           max_autonomous_speed_, 0.8, emergency_stop_dist_);
  ROS_INFO("[CONFIG] Timing: exploration=%.2f Hz map_stable=%.2f s goal_timeout=%.1f s planning_retry=%.1f s diagnostics=5.0 s.",
           exploration_frequency_, map_stable_time_, goal_timeout_, planning_retry_interval_);
  ROS_INFO("[CONFIG] Costmap footprint expected: [(-0.32,-0.285),(-0.32,0.285),(0.32,0.285),(0.32,-0.285)] with padding 0.01 m.");
  XmlRpc::XmlRpcValue configured_param;
  if (ros::param::get("/move_base/global_costmap/footprint", configured_param))
    ROS_INFO("[CONFIG] global_costmap/footprint type=%d value=%s",
             configured_param.getType(), configured_param.toXml().c_str());
  else
    ROS_WARN("[CONFIG] global_costmap/footprint is not present on the parameter server.");
  if (ros::param::get("/move_base/local_costmap/footprint", configured_param))
    ROS_INFO("[CONFIG] local_costmap/footprint type=%d value=%s",
             configured_param.getType(), configured_param.toXml().c_str());
  else
    ROS_WARN("[CONFIG] local_costmap/footprint is not present on the parameter server.");
  if (ros::param::get("/move_base/global_costmap/plugins", configured_param))
    ROS_INFO("[CONFIG] global_costmap/plugins=%s", configured_param.toXml().c_str());
  if (ros::param::get("/move_base/local_costmap/plugins", configured_param))
    ROS_INFO("[CONFIG] local_costmap/plugins=%s", configured_param.toXml().c_str());
  ROS_INFO("Exploration mode: connected free-space frontiers, direct drive %s.",
           enable_direct_drive_ ? "ENABLED" : "DISABLED");
  ROS_INFO("Real-map thresholds: free <= %d, occupied >= %d, clearance %.2f m, speed cap %.2f m/s.",
           free_cell_threshold_, occupied_cell_threshold_, frontier_clearance_,
           max_autonomous_speed_);
  ROS_INFO("[CONFIG] Frontier score: heading_weight=%.2f unknown_weight=%.3f branch_weight=%.3f cluster_radius=%.2f m.",
           heading_penalty_weight_, frontier_unknown_weight_, frontier_branch_weight_,
           frontier_cluster_radius_);
}

NavigationDriver::~NavigationDriver()
{
  stopEscapeBackoff();
  stopSmartRotation();
  stopDirectDrive();
  geometry_msgs::Twist stop;
  publishCmdVel(stop);
}

void NavigationDriver::setMaxVelocity(double vel)
{
  // TEB's default penalty_epsilon is commonly 0.1 m/s.  Do not send a
  // lower max_vel_x: it creates an invalid negative velocity bound.  An
  // emergency stop is enforced by cancelling the action and publishing a
  // zero Twist, so this is only a normal driving limit.
  if (vel < 0.15) vel = 0.15;
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
  // A timer callback may be performing a full-map operation or waiting on a
  // service.  Do not queue a laser callback behind that lock: fail safe and
  // let the next scan recover normal operation.
  std::unique_lock<std::mutex> lock(data_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
  {
    // A transient callback collision is not evidence of an obstacle.  Emit a
    // one-cycle zero command, but leave the safety latch to the scan watchdog;
    // otherwise a long planning callback can turn harmless scheduling jitter
    // into a persistent arbiter safety stop.
    cmd_vel_pub_.publish(geometry_msgs::Twist());
    ROS_WARN_THROTTLE(2.0,
        "[EVENT][SAFETY_LOCK_CONTENTION] Laser callback could not acquire navigation state lock; dropping this scan and emitting one zero command.");
    return;
  }
  bool update_velocity = false;
  double velocity_to_apply = current_max_vel_x_;
  latest_scan_ = *msg;
  has_scan_ = true;
  last_scan_received_time_ = ros::Time::now();
  last_scan_received_nsec_.store(
      static_cast<int64_t>(last_scan_received_time_.toNSec()),
      std::memory_order_release);
  last_scan_stamp_ = msg->header.stamp;
  last_scan_frame_id_ = msg->header.frame_id;
  last_scan_angle_min_ = msg->angle_min;
  last_scan_angle_max_ = msg->angle_max;
  last_scan_angle_increment_ = msg->angle_increment;
  last_scan_range_min_ = msg->range_min;
  last_scan_range_max_ = msg->range_max;
  last_scan_total_count_ = msg->ranges.size();
  last_scan_tf_status_ = "unavailable";
  last_scan_tf_age_ = -1.0;
  geometry_msgs::TransformStamped scan_tf;
  bool scan_transform_available = false;
  if (!msg->header.frame_id.empty())
  {
    try
    {
      scan_tf = tf_buffer_.lookupTransform(
          "base_link", msg->header.frame_id, msg->header.stamp,
          ros::Duration(0.0));
      const double tf_age = std::fabs(
          (ros::Time::now() - scan_tf.header.stamp).toSec());
      if (tf_age <= scan_tf_max_age_)
      {
        scan_transform_available = true;
        last_scan_tf_status_ = "ok";
        last_scan_tf_x_ = scan_tf.transform.translation.x;
        last_scan_tf_y_ = scan_tf.transform.translation.y;
        last_scan_tf_yaw_ = tf2::getYaw(scan_tf.transform.rotation);
        last_scan_tf_age_ = tf_age;
      }
    }
    catch (const tf2::TransformException&)
    {
      // Fall through to the latest-transform fallback below.
    }
    if (!scan_transform_available)
    {
      // A scan timestamp can be a few milliseconds newer than the latest TF
      // sample.  Use the latest transform in that case, but reject a stale
      // transform below instead of repeatedly false-triggering an emergency
      // stop on harmless clock skew.
      try
      {
        scan_tf = tf_buffer_.lookupTransform(
            "base_link", msg->header.frame_id, ros::Time(0));
        const double tf_age = std::fabs(
            (ros::Time::now() - scan_tf.header.stamp).toSec());
        if (tf_age <= scan_tf_max_age_)
        {
          scan_transform_available = true;
          last_scan_tf_status_ = "latest";
          last_scan_tf_x_ = scan_tf.transform.translation.x;
          last_scan_tf_y_ = scan_tf.transform.translation.y;
          last_scan_tf_yaw_ = tf2::getYaw(scan_tf.transform.rotation);
          last_scan_tf_age_ = tf_age;
        }
      }
      catch (const tf2::TransformException&)
      {
        // The status is emitted in the periodic snapshot; the watchdog below
        // holds zero velocity until a valid transform is available again.
      }
    }
  }

  if (task_finished_) return;

  double min_dist = std::numeric_limits<double>::max();
  double min_laser_angle = 0.0;
  double min_base_angle = 0.0;
  double center_min_dist = std::numeric_limits<double>::max();
  int valid_count = 0;
  int invalid_count = 0;
  int center_valid_count = 0;
  int center_near_count = 0;
  int center_floor_count = 0;
  int center_near_cluster = 0;
  int max_center_near_cluster = 0;
  int near_count = 0;
  int max_near_cluster = 0;
  int current_near_cluster = 0;
  last_front_sector_min_distance_.fill(-1.0);
  const double base_angle_min = -fov_horizontal_ * M_PI / 180.0 / 2.0;
  const double base_angle_max = +fov_horizontal_ * M_PI / 180.0 / 2.0;
  const double sector_width = (base_angle_max - base_angle_min) / 7.0;

  tf2::Matrix3x3 scan_rotation;
  if (scan_transform_available)
  {
    tf2::Quaternion q;
    tf2::fromMsg(scan_tf.transform.rotation, q);
    scan_rotation.setRotation(q);
  }

  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    const double laser_angle = msg->angle_min + i * msg->angle_increment;
    if (!scan_transform_available)
      continue;
    const tf2::Vector3 ray_in_laser(std::cos(laser_angle), std::sin(laser_angle), 0.0);
    const tf2::Vector3 ray_in_base = scan_rotation * ray_in_laser;
    const double base_angle = std::atan2(ray_in_base.y(), ray_in_base.x());
    if (base_angle < base_angle_min || base_angle > base_angle_max) continue;
    double range = msg->ranges[i];
    // pointcloud_to_laserscan publishes +Inf for an observed clear ray when
    // use_inf=true.  Treat it as a valid max-range measurement; otherwise an
    // open corridor appears to have an invalid/failed laser frame.
    if (std::isinf(range) && range > 0.0)
      range = msg->range_max;
    if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max)
    {
      ++invalid_count;
      current_near_cluster = 0;
      continue;
    }
    ++valid_count;
    if (range < min_dist)
    {
      min_dist = range;
      min_laser_angle = laser_angle;
      min_base_angle = base_angle;
    }
    // A single edge return is common with a side wall, chassis reflection or
    // sparse point-cloud projection.  Use the central +/-22 deg sector and a
    // contiguous cluster for the normal stop decision instead of the global
    // minimum alone.
    if (std::fabs(base_angle) <= 22.0 * M_PI / 180.0)
    {
      ++center_valid_count;
      center_min_dist = std::min(center_min_dist, range);
      if (range <= msg->range_min + emergency_range_floor_margin_)
        ++center_floor_count;
      if (range < emergency_stop_dist_)
      {
        ++center_near_count;
        ++center_near_cluster;
        max_center_near_cluster = std::max(max_center_near_cluster,
                                           center_near_cluster);
      }
      else
      {
        center_near_cluster = 0;
      }
    }
    if (range < emergency_stop_dist_)
    {
      ++near_count;
      ++current_near_cluster;
      max_near_cluster = std::max(max_near_cluster, current_near_cluster);
    }
    else
    {
      current_near_cluster = 0;
    }
    int sector = static_cast<int>((base_angle - base_angle_min) / sector_width);
    sector = std::max(0, std::min(6, sector));
    if (last_front_sector_min_distance_[sector] < 0.0 ||
        range < last_front_sector_min_distance_[sector])
      last_front_sector_min_distance_[sector] = range;
  }

  last_scan_min_distance_ = (valid_count > 0) ? min_dist : -1.0;
  last_scan_min_laser_angle_ = min_laser_angle;
  last_scan_min_base_angle_ = min_base_angle;
  last_scan_min_angle_ = min_base_angle;
  last_center_min_distance_ = center_valid_count > 0 ? center_min_dist : -1.0;
  last_center_valid_count_ = center_valid_count;
  last_center_near_count_ = center_near_count;
  last_center_floor_count_ = center_floor_count;
  last_center_cluster_ = max_center_near_cluster;
  last_scan_valid_count_ = valid_count;
  last_scan_invalid_count_ = invalid_count;

  // A missing scan TF is a safety fault, but must not be confused with an
  // obstacle in the laser frame.  Hold zero velocity until a valid transform
  // and a clear scan are available again.
  const bool scan_usable = scan_transform_available &&
      valid_count >= scan_min_valid_count_;
  if (!scan_usable)
  {
    emergency_stop_candidate_count_ = emergency_stop_confirm_scans_;
    emergency_clear_candidate_count_ = 0;
    if (!emergency_stopped_)
    {
      ROS_WARN_THROTTLE(2.0,
          "[EVENT][LASER_STOP] Scan unusable (tf=%s valid=%d/%d); holding zero velocity.",
          scan_transform_available ? last_scan_tf_status_.c_str() : "missing",
          valid_count, static_cast<int>(msg->ranges.size()));
      emergency_stopped_ = true;
      safety_stop_atomic_.store(true, std::memory_order_release);
      emergency_stop_time_ = ros::Time::now();
      geometry_msgs::Twist stop;
      publishCmdVel(stop);
      cancelActiveGoal();
      has_last_goal_ = false;
      current_goal_is_object_ = false;
      stopEscapeBackoff();
      stopSmartRotation();
      stopDirectDrive();
    }
    publishSafetyStop(true);
    return;
  }

  // The laser projection can produce isolated range_min returns from the
  // floor, chassis, or a sparse low obstacle.  Require a contiguous central
  // cluster before stopping.  If every near return is at the sensor floor,
  // require one additional ray because that measurement has the least
  // geometric confidence.
  const bool only_range_floor_returns = center_near_count > 0 &&
      center_floor_count == center_near_count;
  const int required_cluster_points = emergency_min_cluster_points_ +
      (only_range_floor_returns ? 1 : 0);
  const bool obstacle_in_base_fov = center_near_count >= emergency_min_near_points_ &&
      max_center_near_cluster >= required_cluster_points &&
      center_min_dist < emergency_stop_dist_;
  // A hard stop still uses the shorter threshold, but is subject to the same
  // spatial and temporal confirmation so one saturated ray cannot lock the
  // vehicle indefinitely.
  const bool hard_obstacle_in_base_fov = obstacle_in_base_fov &&
      center_min_dist <= emergency_stop_hard_dist_;
  if (obstacle_in_base_fov)
    ++emergency_stop_candidate_count_;
  else
    emergency_stop_candidate_count_ = 0;

  const int hard_confirm_scans = std::max(1, emergency_stop_confirm_scans_ - 1);
  const bool confirm_stop =
      (hard_obstacle_in_base_fov &&
       emergency_stop_candidate_count_ >= hard_confirm_scans) ||
      emergency_stop_candidate_count_ >= emergency_stop_confirm_scans_;
  const bool scan_clear = center_valid_count > 0 &&
      (center_min_dist >= emergency_stop_dist_ + emergency_clear_margin_ ||
       !obstacle_in_base_fov);
  if (confirm_stop)
  {
    if (!emergency_stopped_)
    {
      const actionlib::SimpleClientGoalState state = ac_.getState();
      ROS_WARN("[EVENT][LASER_STOP] now=%.3f scan_stamp=%.3f frame=%s min=%.3f m laser_angle=%.2f deg base_angle=%.2f deg center_min=%.3f center_valid=%d center_near=%d center_floor=%d center_cluster=%d required_cluster=%d near=%d cluster=%d threshold=%.3f hard=%.3f confirm=%d/%d fov_valid=%d fov_invalid=%d total=%d scan_angles=[%.2f,%.2f] deg inc=%.4f deg range=[%.2f,%.2f] m sectors=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] goal=%s action=%s cmd=(%.3f,%.3f)",
               ros::Time::now().toSec(), msg->header.stamp.toSec(),
               msg->header.frame_id.c_str(), min_dist,
               min_laser_angle * 180.0 / M_PI, min_base_angle * 180.0 / M_PI,
               center_min_dist == std::numeric_limits<double>::max() ? -1.0 : center_min_dist,
               center_valid_count, center_near_count, center_floor_count,
               max_center_near_cluster, required_cluster_points, near_count,
               max_near_cluster,
               emergency_stop_dist_, emergency_stop_hard_dist_,
               emergency_stop_candidate_count_, emergency_stop_confirm_scans_, valid_count,
               invalid_count, static_cast<int>(msg->ranges.size()),
               msg->angle_min * 180.0 / M_PI, msg->angle_max * 180.0 / M_PI,
               msg->angle_increment * 180.0 / M_PI, msg->range_min, msg->range_max,
               last_front_sector_min_distance_[0],
               last_front_sector_min_distance_[1], last_front_sector_min_distance_[2],
               last_front_sector_min_distance_[3], last_front_sector_min_distance_[4],
               last_front_sector_min_distance_[5], last_front_sector_min_distance_[6],
               has_last_goal_ ? "active" : "none", state.toString().c_str(),
               last_cmd_vel_.linear.x, last_cmd_vel_.angular.z);
      emergency_stopped_ = true;
      safety_stop_atomic_.store(true, std::memory_order_release);
      emergency_stop_time_ = ros::Time::now();
      geometry_msgs::Twist stop;
      publishCmdVel(stop);
      cancelActiveGoal();
      has_last_goal_ = false;
      current_goal_is_object_ = false;
      stopEscapeBackoff();
      stopSmartRotation();
      stopDirectDrive();
    }
    emergency_clear_candidate_count_ = 0;
  }
  else
  {
    // Side returns should not continuously throttle forward motion.  When
    // the central sector is unavailable, keep a conservative floor and wait
    // for the costmap/TEB obstacle layer to make the final decision.
    const double speed_distance = center_valid_count > 0 ? center_min_dist : 3.0;
    double desired_max_vel = 0.0;
    if (speed_distance > 3.0)      desired_max_vel = max_autonomous_speed_;
    else if (speed_distance > 1.5) desired_max_vel = 0.25;
    else if (speed_distance > 0.8) desired_max_vel = 0.18;
    else                           desired_max_vel = 0.15;

    current_max_vel_x_ = 0.7 * current_max_vel_x_ + 0.3 * desired_max_vel;
    update_velocity = true;
    velocity_to_apply = current_max_vel_x_;

    if (emergency_stopped_ && scan_clear)
    {
      ++emergency_clear_candidate_count_;
      if ((ros::Time::now() - emergency_stop_time_).toSec() > emergency_stop_duration_ &&
          emergency_clear_candidate_count_ >= emergency_clear_confirm_scans_)
      {
        emergency_stopped_ = false;
        safety_stop_atomic_.store(false, std::memory_order_release);
        emergency_stop_candidate_count_ = 0;
        emergency_clear_candidate_count_ = 0;
        ROS_INFO("[EVENT][LASER_CLEAR] now=%.3f min=%.3f m threshold=%.3f m stop_duration=%.2f s; resuming=%s.",
                 ros::Time::now().toSec(), last_scan_min_distance_,
                 emergency_stop_dist_, emergency_stop_duration_,
                 has_last_goal_ ? "goal_active" : "request_planning");
        update_velocity = true;
        velocity_to_apply = current_max_vel_x_;
        if (!has_last_goal_)
          requestPlanning();
      }
    }
  }
  publishSafetyStop(emergency_stopped_);
  lock.unlock();
  // Dynamic-reconfigure is a synchronous ROS service.  Never hold the scan
  // state mutex while waiting for it; a delayed service response must not
  // block the laser watchdog or the next scan callback.
  if (update_velocity)
    setMaxVelocity(velocity_to_apply);
}

void NavigationDriver::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_odom_received_time_ = ros::Time::now();
  last_odom_stamp_ = msg->header.stamp;
  last_odom_twist_ = msg->twist.twist;
  last_odom_frame_id_ = msg->header.frame_id;
  last_odom_child_frame_id_ = msg->child_frame_id;
}

void NavigationDriver::objectCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  try
  {
    geometry_msgs::PoseStamped pose_map;
    tf_buffer_.transform(*msg, pose_map, "map", ros::Duration(0.2));
    const ros::Time now = ros::Time::now();
    const bool object_pose_changed = !has_last_known_object_pose_ ||
        distance(last_known_object_pose_.position.x, last_known_object_pose_.position.y,
                 pose_map.pose.position.x, pose_map.pose.position.y) >= 0.20;
    const bool object_goal_active = has_last_goal_ && current_goal_is_object_;
    object_pose_map_ = pose_map.pose;
    object_found_ = true;
    last_object_time_ = now;
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

    // Object detectors commonly publish the same track at camera rate. Keep
    // the current TEB goal stable unless the target moved materially; also
    // respect the global goal-switch holdoff to avoid cancel/re-send churn.
    if (object_goal_active && !object_pose_changed)
      return;
    if (object_goal_active &&
        (now - last_goal_switch_time_).toSec() < min_goal_switch_interval_)
      return;

    stopSmartRotation();
    stopDirectDrive();
    stopEscapeBackoff();
    ROS_INFO("[OBJECT] Detected at (%.2f, %.2f). Approaching.",
             object_pose_map_.position.x, object_pose_map_.position.y);
    cancelActiveGoal();
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
       std::fabs(msg->info.resolution - current_map_.info.resolution) > 1e-9 ||
       std::fabs(msg->info.origin.position.x - current_map_.info.origin.position.x) > 1e-6 ||
       std::fabs(msg->info.origin.position.y - current_map_.info.origin.position.y) > 1e-6);
  // Cartographer normally grows the grid by changing width/height and origin.
  // Failed frontiers are stored in map-frame world coordinates, so they remain
  // valid when the occupancy-grid cell origin changes.
  const bool cell_index_mapping_changed = map_received_ &&
      (std::fabs(msg->info.resolution - current_map_.info.resolution) > 1e-9 ||
       std::fabs(msg->info.origin.position.x - current_map_.info.origin.position.x) >
           std::max(1e-6, current_map_.info.resolution * 0.5) ||
       std::fabs(msg->info.origin.position.y - current_map_.info.origin.position.y) >
           std::max(1e-6, current_map_.info.resolution * 0.5));
  current_map_ = *msg;
  last_map_stamp_ = msg->header.stamp;
  last_map_frame_id_ = msg->header.frame_id;
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
      if (cell_index_mapping_changed)
      {
        last_map_change_time_ = ros::Time::now();
        ROS_INFO_THROTTLE(2.0,
            "[MAP] Map origin/resolution changed; reset stability timer and retain world-coordinate blacklist.");
      }
      else
      {
        ROS_INFO_THROTTLE(2.0,
            "[MAP] Map bounds expanded to %d x %d; retaining stability timer and blacklist.",
            last_map_width_, last_map_height_);
      }
    }
  }
}

void NavigationDriver::globalFootprintCallback(
    const geometry_msgs::PolygonStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  global_footprint_ = *msg;
  has_global_footprint_ = true;
}

void NavigationDriver::localFootprintCallback(
    const geometry_msgs::PolygonStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  local_footprint_ = *msg;
  has_local_footprint_ = true;
}

void NavigationDriver::requestPlanning()
{
  goal_request_pending_ = true;
}

void NavigationDriver::blacklistFrontier(const geometry_msgs::PoseStamped& goal)
{
  const std::pair<double, double> failed(
      goal.pose.position.x, goal.pose.position.y);
  for (const auto& existing : failed_frontiers_)
  {
    if (distance(existing.first, existing.second, failed.first, failed.second) <
        frontier_cluster_radius_)
      return;
  }
  failed_frontiers_.push_back(failed);
  const size_t max_blacklist_size = 200;
  if (failed_frontiers_.size() > max_blacklist_size)
    failed_frontiers_.erase(failed_frontiers_.begin());
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
  if (!has_scan_ ||
      (last_scan_tf_status_ != "ok" && last_scan_tf_status_ != "latest"))
    return false;
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

void NavigationDriver::publishCmdVel(const geometry_msgs::Twist& cmd)
{
  last_cmd_vel_ = cmd;
  last_cmd_vel_time_ = ros::Time::now();
  cmd_vel_pub_.publish(cmd);
}

void NavigationDriver::publishSafetyStop(bool stopped)
{
  safety_stop_atomic_.store(stopped, std::memory_order_release);
  const ros::Time now = ros::Time::now();
  if (!last_safety_publish_time_.isZero() &&
      (now - last_safety_publish_time_).toSec() < 0.10 &&
      !stopped)
    return;
  std_msgs::Bool msg;
  msg.data = stopped;
  safety_stop_pub_.publish(msg);
  last_safety_publish_time_ = now;
}

void NavigationDriver::safetyHeartbeatCallback(const ros::TimerEvent&)
{
  // Keep this path independent of data_mutex_: timerCallback can legitimately
  // spend hundreds of milliseconds in map search or a ROS service call.
  const std::int64_t scan_nsec =
      last_scan_received_nsec_.load(std::memory_order_acquire);
  const std::int64_t now_nsec =
      static_cast<std::int64_t>(ros::Time::now().toNSec());
  const std::int64_t timeout_nsec = static_cast<std::int64_t>(
      std::max(0.10, scan_timeout_) * 1e9);
  const bool scan_fresh = scan_nsec > 0 && now_nsec >= scan_nsec &&
      (now_nsec - scan_nsec) <= timeout_nsec;
  if (!scan_fresh)
    safety_stop_atomic_.store(true, std::memory_order_release);

  std_msgs::Bool msg;
  msg.data = safety_stop_atomic_.load(std::memory_order_acquire);
  safety_stop_pub_.publish(msg);
}

void NavigationDriver::logDiagnosticSnapshot(const ros::Time& now)
{
  if (!last_diagnostic_log_time_.isZero() &&
      (now - last_diagnostic_log_time_).toSec() < 5.0)
    return;
  last_diagnostic_log_time_ = now;

  const actionlib::SimpleClientGoalState action_state = ac_.getState();
  double map_odom_age = -1.0;
  double odom_base_age = -1.0;
  std::string map_odom_status = "unavailable";
  std::string odom_base_status = "unavailable";
  try
  {
    const geometry_msgs::TransformStamped tf_map_odom =
        tf_buffer_.lookupTransform("map", "odom", ros::Time(0));
    map_odom_age = (now - tf_map_odom.header.stamp).toSec();
    map_odom_status = "ok";
  }
  catch (const tf2::TransformException&)
  {
  }
  try
  {
    const geometry_msgs::TransformStamped tf_odom_base =
        tf_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
    odom_base_age = (now - tf_odom_base.header.stamp).toSec();
    odom_base_status = "ok";
  }
  catch (const tf2::TransformException&)
  {
  }

  const double scan_age = last_scan_received_time_.isZero()
      ? -1.0 : (now - last_scan_received_time_).toSec();
  const double scan_stamp_age = last_scan_stamp_.isZero()
      ? -1.0 : (now - last_scan_stamp_).toSec();
  const double map_stamp_age = last_map_stamp_.isZero()
      ? -1.0 : (now - last_map_stamp_).toSec();
  const double cmd_age = last_cmd_vel_time_.isZero()
      ? -1.0 : (now - last_cmd_vel_time_).toSec();
  const double odom_age = last_odom_received_time_.isZero()
      ? -1.0 : (now - last_odom_received_time_).toSec();
  const double odom_stamp_age = last_odom_stamp_.isZero()
      ? -1.0 : (now - last_odom_stamp_).toSec();

  std::ostringstream global_fp;
  if (has_global_footprint_)
  {
    global_fp << "frame=" << global_footprint_.header.frame_id << " points=";
    for (size_t i = 0; i < global_footprint_.polygon.points.size(); ++i)
    {
      if (i > 0) global_fp << ";";
      global_fp << "(" << global_footprint_.polygon.points[i].x << ","
                << global_footprint_.polygon.points[i].y << ")";
    }
  }
  else
  {
    global_fp << "missing";
  }

  std::ostringstream local_fp;
  if (has_local_footprint_)
  {
    local_fp << "frame=" << local_footprint_.header.frame_id << " points=";
    for (size_t i = 0; i < local_footprint_.polygon.points.size(); ++i)
    {
      if (i > 0) local_fp << ";";
      local_fp << "(" << local_footprint_.polygon.points[i].x << ","
               << local_footprint_.polygon.points[i].y << ")";
    }
  }
  else
  {
    local_fp << "missing";
  }

  std::ostringstream goal;
  if (has_last_goal_)
  {
    goal << (current_goal_is_object_ ? "object" : "frontier")
         << " pose=(" << last_goal_.pose.position.x << ","
         << last_goal_.pose.position.y << ",yaw="
         << tf2::getYaw(last_goal_.pose.orientation) * 180.0 / M_PI << "deg)"
         << " age=" << (now - last_goal_sent_time_).toSec() << "s";
  }
  else
  {
    goal << "none";
  }

  ROS_INFO("[STATE] now=%.3f map=%s frame=%s size=%dx%d res=%.3f map_change_age=%.2f map_stamp_age=%.2f scan=%s frame=%s recv_age=%.2f stamp_age=%.2f count=%d angles=[%.2f,%.2f]deg inc=%.4fdeg range=[%.2f,%.2f]m min=%.3f m laser_angle=%.2f deg base_angle=%.2f deg center=(min=%.3f valid=%d near=%d floor=%d cluster=%d) fov_valid=%d fov_invalid=%d sectors=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] action=%s goal=%s emergency=%s rotate=%s direct=%s cmd=(%.3f,%.3f) cmd_age=%.2f max_vel=%.3f teb_limit=%.3f tf_map_odom=%s age=%.4f tf_odom_base=%s age=%.4f failures=%d/%d footprint_global={%s} footprint_local={%s} odom=frame:%s child:%s recv_age:%.2f stamp_age:%.2f twist:(%.3f,%.3f) scan_tf=%s xyz:(%.3f,%.3f) yaw:%.2fdeg age:%.4f",
           now.toSec(), map_received_ ? "received" : "missing",
           last_map_frame_id_.c_str(), last_map_width_, last_map_height_,
           current_map_.info.resolution,
           last_map_change_time_.isZero() ? -1.0 : (now - last_map_change_time_).toSec(),
           map_stamp_age, has_scan_ ? "received" : "missing",
           last_scan_frame_id_.c_str(), scan_age, scan_stamp_age,
           static_cast<int>(last_scan_total_count_),
           last_scan_angle_min_ * 180.0 / M_PI,
           last_scan_angle_max_ * 180.0 / M_PI,
           last_scan_angle_increment_ * 180.0 / M_PI,
           last_scan_range_min_, last_scan_range_max_,
           last_scan_min_distance_, last_scan_min_laser_angle_ * 180.0 / M_PI,
           last_scan_min_base_angle_ * 180.0 / M_PI,
           last_center_min_distance_, last_center_valid_count_,
           last_center_near_count_, last_center_floor_count_, last_center_cluster_,
           last_scan_valid_count_, last_scan_invalid_count_,
           last_front_sector_min_distance_[0], last_front_sector_min_distance_[1],
           last_front_sector_min_distance_[2], last_front_sector_min_distance_[3],
           last_front_sector_min_distance_[4], last_front_sector_min_distance_[5],
           last_front_sector_min_distance_[6], action_state.toString().c_str(),
           goal.str().c_str(), emergency_stopped_ ? "ON" : "OFF",
           is_smart_rotating_ ? "ON" : "OFF", is_direct_drive_ ? "ON" : "OFF",
           last_cmd_vel_.linear.x, last_cmd_vel_.angular.z, cmd_age,
           current_max_vel_x_, last_vel_set_, map_odom_status.c_str(), map_odom_age,
           odom_base_status.c_str(), odom_base_age, consecutive_failures_,
           max_consecutive_failures_, global_fp.str().c_str(), local_fp.str().c_str(),
           last_odom_frame_id_.c_str(), last_odom_child_frame_id_.c_str(), odom_age,
           odom_stamp_age, last_odom_twist_.linear.x, last_odom_twist_.angular.z,
           last_scan_tf_status_.c_str(), last_scan_tf_x_, last_scan_tf_y_,
           last_scan_tf_yaw_ * 180.0 / M_PI, last_scan_tf_age_);
}

void NavigationDriver::cancelActiveGoal()
{
  const actionlib::SimpleClientGoalState state = ac_.getState();
  if (state == actionlib::SimpleClientGoalState::ACTIVE ||
      state == actionlib::SimpleClientGoalState::PENDING)
  {
    // Invalidate callbacks for the canceled goal before asking actionlib to
    // cancel it.  The cancel acknowledgement can arrive after a replacement
    // goal has already been sent.
    ++goal_generation_;
    ac_.cancelGoal();
  }
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
  cancelActiveGoal();
  has_last_goal_ = false;
  stopSmartRotation();
}

void NavigationDriver::stopDirectDrive()
{
  if (!is_direct_drive_) return;
  is_direct_drive_ = false;
  geometry_msgs::Twist stop;
  publishCmdVel(stop);
  ROS_INFO("[DIRECT] Stopped direct drive mode.");
}

double NavigationDriver::findBestDirection()
{
  if (!has_scan_) return 0.0;
  if (last_scan_tf_status_ != "ok" && last_scan_tf_status_ != "latest") return 0.0;

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
    const double laser_angle = latest_scan_.angle_min + i * latest_scan_.angle_increment;
    double angle = laser_angle + last_scan_tf_yaw_;
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;

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
      const std::vector<std::pair<double, double>> empty_blacklist;
      if (findFrontierCandidates(candidates, max_search_range_, empty_blacklist) ||
          findFrontierCandidates(candidates, fallback_range_, empty_blacklist))
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
    double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment + last_scan_tf_yaw_;
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
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
  publishCmdVel(cmd);
}

void NavigationDriver::timerCallback(const ros::TimerEvent&)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  const ros::Time now = ros::Time::now();
  logDiagnosticSnapshot(now);
  const bool scan_stale = !has_scan_ || last_scan_received_time_.isZero() ||
      (now - last_scan_received_time_).toSec() > scan_timeout_;
  if (scan_stale)
  {
    if (!emergency_stopped_)
    {
      emergency_stopped_ = true;
      safety_stop_atomic_.store(true, std::memory_order_release);
      emergency_stop_time_ = now;
      emergency_clear_candidate_count_ = 0;
      cancelActiveGoal();
      has_last_goal_ = false;
      current_goal_is_object_ = false;
      stopEscapeBackoff();
      stopSmartRotation();
      stopDirectDrive();
      ROS_WARN("[EVENT][SCAN_WATCHDOG] No usable scan for %.2f s (timeout %.2f s); holding zero velocity.",
               (last_scan_received_time_.isZero()
                    ? -1.0 : (now - last_scan_received_time_).toSec()),
               scan_timeout_);
    }
    geometry_msgs::Twist stop;
    publishCmdVel(stop);
    publishSafetyStop(true);
    return;
  }
  if (task_finished_ || emergency_stopped_)
  {
    geometry_msgs::Twist stop;
    publishCmdVel(stop);
    publishSafetyStop(emergency_stopped_);
    return;
  }
  publishSafetyStop(false);
  if (!map_received_) return;

  if (is_direct_drive_)
  {
    directDriveControl();
    return;
  }
  if (is_escape_backing_)
    return;

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
        publishCmdVel(stop);
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
      blacklistFrontier(last_goal_);
    }
    cancelActiveGoal();
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
    // The raw SLAM grid changes while Cartographer expands and corrects the
    // map.  Do not let one transient BFS result preempt an otherwise valid
    // move_base goal.  Object approach goals are intentionally left to the
    // costmap/TEB feasibility checks as well.
    if (!current_goal_is_object_ && isMapStable() && !isCurrentGoalReachable())
    {
      ++goal_unreachable_count_;
      ROS_WARN_THROTTLE(5.0,
          "[TIMER] Frontier reachability check failed (%d/%d); retaining goal while move_base replans.",
          goal_unreachable_count_, goal_recheck_confirmations_);
      if (goal_unreachable_count_ >= goal_recheck_confirmations_)
      {
        ROS_WARN("[TIMER] Frontier remained unreachable for %d checks; canceling and replanning.",
                 goal_unreachable_count_);
        blacklistFrontier(last_goal_);
        cancelActiveGoal();
        has_last_goal_ = false;
        current_goal_is_object_ = false;
        goal_unreachable_count_ = 0;
        last_fail_time_ = now;
        requestPlanning();
        return;
      }
    }
    else
    {
      goal_unreachable_count_ = 0;
    }
  }

  if ((now - last_progress_check_time_).toSec() < progress_check_interval_) return;
  last_progress_check_time_ = now;

  if (current_state != actionlib::SimpleClientGoalState::ACTIVE ||
      goal_age < progress_grace_period_)
  {
    // A pending goal, a just-sent goal, or a temporary planner handoff is not
    // evidence of a stall.  Start a fresh baseline after that grace period.
    has_last_robot_pose_ = false;
    progress_stall_count_ = 0;
    return;
  }

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
      ++progress_stall_count_;
      ROS_WARN_THROTTLE(5.0,
          "[TIMER] Low progress (%.2f m, %.1f deg in %.1f s), stall check %d/%d.",
          moved, yaw_change * 180.0 / M_PI, progress_check_interval_,
          progress_stall_count_, progress_confirmations_);
      if (progress_stall_count_ < progress_confirmations_)
      {
        last_robot_pose_ = current_pose;
        return;
      }

      ROS_WARN("[TIMER] No progress confirmed for %d checks; stopping and replanning.",
               progress_stall_count_);
      if (!current_goal_is_object_)
      {
        blacklistFrontier(last_goal_);
      }
      cancelActiveGoal();
      geometry_msgs::Twist stop;
      publishCmdVel(stop);
      has_last_goal_ = false;
      current_goal_is_object_ = false;
      last_fail_time_ = now;
      has_last_robot_pose_ = false;
      progress_stall_count_ = 0;
      if (++consecutive_failures_ >= max_consecutive_failures_)
      {
        consecutive_failures_ = 0;
        executeEscape();
      }
      else
      {
        requestPlanning();
      }
      return;
    }
    else
    {
      progress_stall_count_ = 0;
    }
  }
  last_robot_pose_ = current_pose;
  has_last_robot_pose_ = true;
}

void NavigationDriver::doneCallback(const actionlib::SimpleClientGoalState& state,
                                    const move_base_msgs::MoveBaseResultConstPtr&,
                                    std::uint64_t goal_generation)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (goal_generation != goal_generation_ || !has_last_goal_ ||
      task_finished_ || emergency_stopped_)
  {
    ROS_DEBUG("[DONE] Ignoring terminal callback for stale goal generation %llu (current %llu).",
              static_cast<unsigned long long>(goal_generation),
              static_cast<unsigned long long>(goal_generation_));
    return;
  }

  const bool goal_was_object = current_goal_is_object_;
  if (state == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    has_last_goal_ = false;
    current_goal_is_object_ = false;
    goal_unreachable_count_ = 0;
    progress_stall_count_ = 0;
    has_last_robot_pose_ = false;
    requestPlanning();
    return;
  }

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    has_last_goal_ = false;
    current_goal_is_object_ = false;
    consecutive_failures_ = 0;
    has_last_robot_pose_ = false;
    goal_unreachable_count_ = 0;
    progress_stall_count_ = 0;

    if (goal_was_object && object_found_)
    {
      task_finished_ = true;
      ROS_INFO("[DONE] TASK COMPLETED! Robot arrived at object.");
      ac_.cancelAllGoals();
      geometry_msgs::Twist stop;
      publishCmdVel(stop);
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
    blacklistFrontier(last_goal_);
  }

  ++fail_count_;
  ++consecutive_failures_;
  last_fail_time_ = ros::Time::now();
  has_last_goal_ = false;
  current_goal_is_object_ = false;
  has_last_robot_pose_ = false;
  goal_unreachable_count_ = 0;
  progress_stall_count_ = 0;
  ROS_WARN("[DONE] Goal failed (%s). Fail: %d, consecutive: %d.",
           state.toString().c_str(), fail_count_, consecutive_failures_);

  if (consecutive_failures_ >= max_consecutive_failures_)
  {
    consecutive_failures_ = 0;
    executeEscape();
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
      candidates, 2.0, std::vector<std::pair<double, double>>());
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

  const ros::Time now = ros::Time::now();
  if (!last_goal_switch_time_.isZero() &&
      (now - last_goal_switch_time_).toSec() < min_goal_switch_interval_)
  {
    // Record the suppressed attempt as well.  Otherwise the timer enters this
    // branch on every tick while the switch holdoff is active.
    last_planning_attempt_time_ = now;
    ROS_INFO_THROTTLE(5.0, "[SEND] Goal switch holdoff active (%.1f s remaining).",
                      min_goal_switch_interval_ - (now - last_goal_switch_time_).toSec());
    requestPlanning();
    return;
  }

  last_planning_attempt_time_ = now;

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
  last_goal_sent_time_ = now;
  last_goal_switch_time_ = now;
  goal_unreachable_count_ = 0;
  progress_stall_count_ = 0;
  has_last_robot_pose_ = false;
  random_goal_attempts_ = 0;

  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose = new_goal;
  const std::uint64_t goal_generation = ++goal_generation_;
  ac_.sendGoal(mb_goal, boost::bind(&NavigationDriver::doneCallback, this,
                                    _1, _2, goal_generation));

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
    const std::vector<std::pair<double, double>>& blacklist)
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
  // Keep the lowest-cost candidates irrespective of their row/column position
  // in the occupancy grid.  The previous first-N truncation favored the
  // beginning of the scan order and could discard a later elevator/doorway
  // frontier before scoring and clustering.
  typedef std::pair<double, geometry_msgs::PoseStamped> ScoredCandidate;
  struct HighestCostFirst
  {
    bool operator()(const ScoredCandidate& lhs,
                    const ScoredCandidate& rhs) const
    {
      return lhs.first < rhs.first;
    }
  };
  std::priority_queue<ScoredCandidate, std::vector<ScoredCandidate>,
                     HighestCostFirst> best_raw_candidates;
  size_t reachable_cells_checked = 0;
  size_t uncertain_cells = 0;
  size_t adjacent_unknown_cells = 0;
  size_t clearance_rejections = 0;
  size_t blacklist_rejections = 0;
  size_t distance_rejections = 0;
  size_t cells_visited = 0;
  bool timed_out = false;
  static const int kNeighborDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kNeighborDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  for (int y = min_y; y <= max_y && !timed_out; ++y)
  {
    for (int x = min_x; x <= max_x && !timed_out; ++x)
    {
      if ((++cells_visited & 0x3fU) == 0U &&
          (ros::WallTime::now() - start_time).toSec() >= frontier_search_timeout_)
      {
        timed_out = true;
        break;
      }
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

      const double world_x = origin_x + (x + 0.5) * resolution;
      const double world_y = origin_y + (y + 0.5) * resolution;
      bool blacklisted_nearby = false;
      for (const auto& failed : blacklist)
      {
        if (distance(world_x, world_y, failed.first, failed.second) <
            frontier_cluster_radius_)
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
      const int branch_unknown = countUnknownNeighbors(x, y, 6);
      const int opening_gain = std::max(0, branch_unknown - information_gain);
      const double lateral_factor = std::sin(
          std::min(heading_difference, M_PI / 2.0));
      const double side_opening_gain = lateral_factor * opening_gain;
      // The smaller neighborhood rewards immediate sensing.  The larger
      // neighborhood estimates whether the frontier opens into a side branch
      // (elevator lobby, doorway, or room) instead of ending at a wall.  This
      // term is gated by heading difference, so a forward corridor frontier
      // cannot receive the same branch bonus and starve a lateral opening.
      const double cost = goal_distance +
                          heading_penalty_weight_ * heading_difference -
                          frontier_unknown_weight_ * information_gain -
                          frontier_branch_weight_ * side_opening_gain;

      geometry_msgs::PoseStamped candidate;
      candidate.header.frame_id = "map";
      // Frontier poses are map-frame commands; zero selects the latest TF
      // when move_base transforms the goal into its costmap frame.
      candidate.header.stamp = ros::Time(0);
      candidate.pose.position.x = world_x;
      candidate.pose.position.y = world_y;
      candidate.pose.orientation = tf2::toMsg(
          tf2::Quaternion(tf2::Vector3(0, 0, 1), heading));
      ScoredCandidate scored(cost, candidate);
      if (static_cast<int>(best_raw_candidates.size()) < frontier_max_raw_candidates_)
      {
        best_raw_candidates.push(scored);
      }
      else if (cost < best_raw_candidates.top().first)
      {
        best_raw_candidates.pop();
        best_raw_candidates.push(scored);
      }
    }
  }

  std::vector<std::pair<geometry_msgs::PoseStamped, double>> raw_candidates;
  raw_candidates.reserve(best_raw_candidates.size());
  while (!best_raw_candidates.empty())
  {
    const ScoredCandidate scored = best_raw_candidates.top();
    best_raw_candidates.pop();
    raw_candidates.push_back({scored.second, scored.first});
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
      "[FRONTIER] Search %.3f s%s: reachable=%zu uncertain=%zu adjacent=%zu "
      "reject(clearance=%zu blacklist=%zu distance=%zu) raw=%zu clustered=%zu.",
      elapsed, timed_out ? " (deadline)" : "", reachable_cells_checked,
      uncertain_cells, adjacent_unknown_cells,
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
  goal.header.stamp = ros::Time(0);
  goal.pose.position.x = goal_x;
  goal.pose.position.y = goal_y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), angle_to_obj));
  return snapGoalToReachableFreeCell(goal, 0.75);
}

void NavigationDriver::executeEscape()
{
  ROS_INFO("[ESCAPE] Starting escape procedure...");

  if (is_escape_backing_) return;
  if (is_smart_rotating_) stopSmartRotation();
  if (is_direct_drive_) stopDirectDrive();
  cancelActiveGoal();
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
    startEscapeBackoff();
    return;
  }

  double yaw0 = tf2::getYaw(current.pose.orientation);
  std::vector<geometry_msgs::PoseStamped> attempts;

  // Generate every bounded recovery direction.  Truncating this list at ten
  // candidates made the far-side and forward-side alternatives unreachable in
  // a number of dead-end layouts.
  for (double dist : {1.5, 1.0, 0.5}) {
    for (double angle : {M_PI, M_PI*3/4, M_PI*5/4, M_PI/2, -M_PI/2}) {
      geometry_msgs::PoseStamped p = current;
      p.pose.position.x += dist * cos(yaw0 + angle);
      p.pose.position.y += dist * sin(yaw0 + angle);
      p.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), yaw0 + angle));
      attempts.push_back(p);
    }
  }

  ROS_INFO("[ESCAPE] Generated %zu escape candidates", attempts.size());

  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(attempts.begin(), attempts.end(), g);

  // Build the connected-free mask once.  Calling isGoalReachable() for every
  // escape pose repeats the same full-map BFS and can stall the navigation
  // callback on a large Cartographer grid.
  std::vector<unsigned char> reachable;
  int robot_mx = 0;
  int robot_my = 0;
  const bool have_reachable_mask = buildReachableMask(reachable, robot_mx, robot_my);
  const int map_width = static_cast<int>(current_map_.info.width);
  const int clearance_cells = current_map_.info.resolution > 0.0
      ? std::max(1, static_cast<int>(std::ceil(
          frontier_clearance_ / current_map_.info.resolution)))
      : 1;

  int checked = 0;
  for (const auto& goal : attempts) {
    checked++;
    ROS_INFO("[ESCAPE] Trying escape point %d/%zu...", checked, attempts.size());

    int goal_mx = 0;
    int goal_my = 0;
    bool goal_reachable = false;
    if (have_reachable_mask && worldToMap(goal.pose.position.x, goal.pose.position.y,
                                          goal_mx, goal_my))
    {
      const size_t goal_index = static_cast<size_t>(goal_my) * map_width + goal_mx;
      goal_reachable = goal_index < reachable.size() && reachable[goal_index] &&
          !hasOccupiedNeighbor(goal_mx, goal_my, clearance_cells);
    }

    if (goal_reachable) {
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
      const std::uint64_t goal_generation = ++goal_generation_;
      ac_.sendGoal(mb_goal, boost::bind(&NavigationDriver::doneCallback, this,
                                        _1, _2, goal_generation));
      consecutive_failures_ = 0;
      return;
    }
  }

  ROS_WARN("[ESCAPE] No escape goal reachable after checking %d points. Trying bounded reverse recovery.", checked);
  startEscapeBackoff();
}

void NavigationDriver::startEscapeBackoff()
{
  if (is_escape_backing_) return;
  if (!has_scan_ ||
      (last_scan_tf_status_ != "ok" && last_scan_tf_status_ != "latest"))
  {
    ROS_WARN_THROTTLE(5.0, "[ESCAPE] Reverse recovery unavailable without a valid scan transform.");
    startSmartRotation();
    return;
  }

  double rear_clearance = std::numeric_limits<double>::max();
  int rear_valid = 0;
  for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
  {
    double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment + last_scan_tf_yaw_;
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    if (std::fabs(std::fabs(angle) - M_PI) > 30.0 * M_PI / 180.0) continue;
    double range = latest_scan_.ranges[i];
    if (std::isinf(range) && range > 0.0) range = latest_scan_.range_max;
    if (!std::isfinite(range) || range < latest_scan_.range_min || range > latest_scan_.range_max)
      continue;
    rear_clearance = std::min(rear_clearance, range);
    ++rear_valid;
  }

  if (rear_valid < 3 || rear_clearance < emergency_stop_dist_)
  {
    ROS_WARN_THROTTLE(5.0,
        "[ESCAPE] Reverse blocked (rear_clearance=%.2f m, valid=%d); trying bounded rotation.",
        rear_valid > 0 ? rear_clearance : -1.0, rear_valid);
    startSmartRotation();
    return;
  }

  cancelActiveGoal();
  has_last_goal_ = false;
  current_goal_is_object_ = false;
  is_escape_backing_ = true;
  escape_backoff_start_ = ros::Time::now();
  ROS_WARN("[ESCAPE] Reversing at 0.10 m/s for up to %.1f s (rear clearance %.2f m).",
           escape_backoff_duration_, rear_clearance);

  escape_timer_ = nh_.createTimer(ros::Duration(0.05), [this](const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!is_escape_backing_) return;
    if (emergency_stopped_ || !has_scan_ ||
        (last_scan_tf_status_ != "ok" && last_scan_tf_status_ != "latest"))
    {
      stopEscapeBackoff();
      requestPlanning();
      return;
    }

    double rear_clearance = std::numeric_limits<double>::max();
    int rear_valid = 0;
    for (size_t i = 0; i < latest_scan_.ranges.size(); ++i)
    {
      double angle = latest_scan_.angle_min + i * latest_scan_.angle_increment + last_scan_tf_yaw_;
      while (angle > M_PI) angle -= 2.0 * M_PI;
      while (angle < -M_PI) angle += 2.0 * M_PI;
      if (std::fabs(std::fabs(angle) - M_PI) > 30.0 * M_PI / 180.0) continue;
      double range = latest_scan_.ranges[i];
      if (std::isinf(range) && range > 0.0) range = latest_scan_.range_max;
      if (!std::isfinite(range) || range < latest_scan_.range_min || range > latest_scan_.range_max)
        continue;
      rear_clearance = std::min(rear_clearance, range);
      ++rear_valid;
    }

    if (rear_valid < 3 || rear_clearance < emergency_stop_hard_dist_ ||
        (ros::Time::now() - escape_backoff_start_).toSec() >= escape_backoff_duration_)
    {
      stopEscapeBackoff();
      clearCostmaps();
      requestPlanning();
      ROS_INFO("[ESCAPE] Reverse recovery completed; requesting a fresh plan.");
      return;
    }

    geometry_msgs::Twist cmd;
    cmd.linear.x = -0.10;
    publishCmdVel(cmd);
  });
}

void NavigationDriver::stopEscapeBackoff()
{
  if (!is_escape_backing_) return;
  is_escape_backing_ = false;
  escape_timer_.stop();
  geometry_msgs::Twist stop;
  publishCmdVel(stop);
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
    publishCmdVel(twist);
    accumulated_angle_ += rotate_speed_ * dt;
  });
}

void NavigationDriver::stopSmartRotation()
{
  if (!is_smart_rotating_) return;
  is_smart_rotating_ = false;
  rotate_timer_.stop();
  geometry_msgs::Twist stop;
  publishCmdVel(stop);
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
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
