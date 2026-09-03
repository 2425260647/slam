#include "nav_driver/my_navigation.h"
#include <cmath>
#include <algorithm>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <limits>
#include <sstream>
#include <queue>
#include <boost/bind.hpp>

namespace
{
// A planning callback can return through many guarded branches.  Keep one
// scope timer so a slow map search, TF lookup or action call is visible even
// when the callback exits early.
struct ScopedTimingLog
{
  explicit ScopedTimingLog(const char* label)
    : label_(label), start_(ros::WallTime::now()) {}

  ~ScopedTimingLog()
  {
    const double elapsed = (ros::WallTime::now() - start_).toSec();
    if (elapsed > 0.25)
      ROS_WARN_THROTTLE(5.0, "[TIMING] %s callback took %.3f s (slow path).", label_, elapsed);
    else
      ROS_INFO_THROTTLE(5.0, "[TIMING] %s callback took %.3f s.", label_, elapsed);
  }

  const char* label_;
  ros::WallTime start_;
};
}

const std::string NavigationDriver::VERSION = "3.7.3-navigation-cleanup";

NavigationDriver::NavigationDriver(ros::NodeHandle& nh)
  : nh_(nh)
  , ac_("move_base", true)
  , tf_listener_(tf_buffer_)
  , map_received_(false)
  , object_found_(false)
  , task_finished_(false)
  , last_object_time_(ros::Time(0))
  , has_last_known_object_pose_(false)
  , enable_visual_servo_(true)
  , visual_area_stop_threshold_(0.033)
  , visual_stop_confirmations_(3)
  , visual_detection_timeout_(0.50)
  , visual_angle_gain_(1.8)
  , visual_angle_sign_(-1.0)
  , visual_max_linear_speed_(0.15)
  , visual_max_angular_speed_(0.60)
  , visual_align_angle_deg_(15.0)
  , visual_sync_tolerance_(0.20)
  , visual_min_front_clearance_(0.53)
  , last_visual_direction_deg_(0.0)
  , last_visual_area_ratio_(0.0)
  , last_visual_direction_time_(ros::Time(0))
  , last_visual_area_time_(ros::Time(0))
  , visual_stop_count_(0)
  , visual_control_engaged_(false)
  , visual_target_reached_(false)
  , current_goal_timeout_(30.0)
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
  , last_progress_check_time_(0)
  , has_last_robot_pose_(false)
  , has_last_camera_view_pose_(false)
  , goal_request_pending_(true)
  , current_goal_is_object_(false)
  , current_goal_is_bridge_(false)
  , last_planning_attempt_time_(0)
  , no_frontier_rotations_(0)
  , last_no_frontier_time_(0)
  , goal_generation_(0)
  , has_exploration_heading_(false)
  , exploration_heading_(0.0)
  , last_safety_publish_time_(0)
  , safety_stop_atomic_(true)
  , last_scan_arrival_nsec_(0)
  , last_scan_received_nsec_(0)
  , emergency_stopped_(false)
  , emergency_stop_duration_(1.0)
  , emergency_stop_candidate_count_(0)
  , emergency_clear_candidate_count_(0)
  , emergency_front_observation_until_(0)
  , emergency_escape_delay_(2.0)
  , emergency_escape_attempted_(false)
  , emergency_escape_attempt_count_(0)
  , emergency_escape_max_attempts_(1)
  , emergency_front_observation_duration_(1.0)
  , is_smart_rotating_(false)
  , accumulated_angle_(0.0)
  , max_rotate_angle_(M_PI)
  , is_escape_backing_(false)
  , escape_backoff_start_(0)
  , has_scan_(false)
  , current_max_vel_x_(0.20)
  , diagnostic_log_period_(15.0)
  , last_diagnostic_log_time_(0)
  , last_scan_received_time_(0)
  , last_scan_stamp_(0)
  , last_map_stamp_(0)
  , last_scan_tf_status_("unavailable")
  , last_scan_tf_x_(0.0)
  , last_scan_tf_y_(0.0)
  , last_scan_tf_yaw_(0.0)
  , last_scan_tf_age_(-1.0)
  , last_scan_processing_duration_(0.0)
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
  nh_.param<std::string>("object_topic", object_topic_, "/object_detected");
  nh_.param<std::string>("object_direction_topic", object_direction_topic_,
                         "/object_direction");
  nh_.param<std::string>("object_area_topic", object_area_topic_, "/object_area");
  nh_.param("enable_visual_servo", enable_visual_servo_, true);
  nh_.param("visual_area_stop_threshold", visual_area_stop_threshold_, 0.033);
  nh_.param("visual_stop_confirmations", visual_stop_confirmations_, 3);
  nh_.param("visual_detection_timeout", visual_detection_timeout_, 0.50);
  nh_.param("visual_angle_gain", visual_angle_gain_, 1.8);
  nh_.param("visual_angle_sign", visual_angle_sign_, -1.0);
  nh_.param("visual_max_linear_speed", visual_max_linear_speed_, 0.15);
  nh_.param("visual_max_angular_speed", visual_max_angular_speed_, 0.60);
  nh_.param("visual_align_angle_deg", visual_align_angle_deg_, 15.0);
  nh_.param("visual_sync_tolerance", visual_sync_tolerance_, 0.20);
  nh_.param("visual_min_front_clearance", visual_min_front_clearance_, 0.53);
  nh_.param("stop_distance", stop_distance_, 1.0);
  nh_.param("object_timeout", object_timeout_, 5.0);
  nh_.param("exploration_frequency", exploration_frequency_, 1.0);
  nh_.param("frontier_min_dist", frontier_min_dist_, 0.5);
  nh_.param("fov_horizontal", fov_horizontal_, 70.0);
  nh_.param("fov_range", fov_range_, 1.5);
  nh_.param("max_search_range", max_search_range_, 6.0);
  nh_.param("extended_search_range", extended_search_range_, 8.0);
  nh_.param("goal_timeout", goal_timeout_, 22.0);
  current_goal_timeout_ = goal_timeout_;
  nh_.param("fail_backoff_time", fail_backoff_time_, 2.0);
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
  nh_.param("emergency_hard_confirm_scans", emergency_hard_confirm_scans_, 2);
  nh_.param("emergency_clear_confirm_scans", emergency_clear_confirm_scans_, 3);
  nh_.param("emergency_escape_delay", emergency_escape_delay_, 2.0);
  nh_.param("emergency_escape_max_attempts", emergency_escape_max_attempts_, 1);
  nh_.param("emergency_front_observation_duration",
            emergency_front_observation_duration_, 1.0);
  nh_.param("rotate_speed", rotate_speed_, 0.4);
  nh_.param("max_rotate_angle_deg", max_rotate_angle_, 180.0);
  nh_.param("start_delay", start_delay_, 10.0);
  nh_.param("min_goal_switch_interval", min_goal_switch_interval_, 3.0);
  nh_.param("vel_update_interval", vel_update_interval_, 1.0);
  nh_.param("vel_change_threshold", vel_change_threshold_, 0.1);
  nh_.param("progress_check_interval", progress_check_interval_, 4.0);
  nh_.param("min_progress_dist", min_progress_dist_, 0.1);
  nh_.param("min_progress_yaw", min_progress_yaw_, 0.15);
  nh_.param("progress_grace_period", progress_grace_period_, 10.0);
  nh_.param("progress_confirmations", progress_confirmations_, 2);
  nh_.param("corridor_heading_weight", corridor_heading_weight_, 1.0);
  // 稳定性参数
  nh_.param("map_stable_time", map_stable_time_, 1.0);
  nh_.param("max_consecutive_failures", max_consecutive_failures_, 5);
  nh_.param("planning_retry_interval", planning_retry_interval_, 2.0);
  nh_.param("frontier_clearance", frontier_clearance_, 0.25);
  nh_.param("frontier_unknown_search_radius", frontier_unknown_search_radius_, 0.10);
  nh_.param("frontier_cluster_radius", frontier_cluster_radius_, 0.45);
  nh_.param("frontier_search_timeout", frontier_search_timeout_, 0.35);
  nh_.param("frontier_max_raw_candidates", frontier_max_raw_candidates_, 4000);
  nh_.param("camera_coverage_radius", camera_coverage_radius_, 0.0);
  nh_.param("camera_coverage_angle_deg", camera_coverage_angle_deg_, 70.0);
  nh_.param("camera_record_distance", camera_record_distance_, 0.20);
  nh_.param("camera_record_yaw_deg", camera_record_yaw_deg_, 10.0);
  nh_.param("min_unsearched_novelty", min_unsearched_novelty_, 0.25);
  nh_.param("rotation_clearance", rotation_clearance_, 0.45);
  nh_.param("escape_backoff_duration", escape_backoff_duration_, 1.5);
  nh_.param("max_autonomous_speed", max_autonomous_speed_, 0.25);
  nh_.param("no_frontier_retry_interval", no_frontier_retry_interval_, 5.0);
  nh_.param("max_no_frontier_rotations", max_no_frontier_rotations_, 1);
  nh_.param("free_cell_threshold", free_cell_threshold_, 49);
  nh_.param("occupied_cell_threshold", occupied_cell_threshold_, 65);
  nh_.param("frontier_failure_ttl", frontier_failure_ttl_, 120.0);
  nh_.param("bridge_failure_ttl", bridge_failure_ttl_, 60.0);
  nh_.param("stall_failure_ttl", stall_failure_ttl_, 45.0);
  nh_.param("diagnostic_log_period", diagnostic_log_period_, 15.0);
  nh_.param<std::string>("safety_stop_topic", safety_stop_topic_,
                         "/navigation_driver/safety_stop");

  free_cell_threshold_ = std::max(0, std::min(free_cell_threshold_, 49));
  occupied_cell_threshold_ = std::max(free_cell_threshold_ + 1,
                                      std::min(occupied_cell_threshold_, 100));
  // Keep max_vel_x above the TEB penalty epsilon used by this launch.
  max_autonomous_speed_ = std::max(0.15, std::min(max_autonomous_speed_, 0.30));
  max_no_frontier_rotations_ = std::max(0, max_no_frontier_rotations_);
  no_frontier_retry_interval_ = std::max(1.0, no_frontier_retry_interval_);
  progress_check_interval_ = std::max(2.0, progress_check_interval_);
  progress_grace_period_ = std::max(progress_check_interval_, progress_grace_period_);
  progress_confirmations_ = std::max(2, progress_confirmations_);
  progress_stall_count_ = 0;
  corridor_heading_weight_ = std::max(0.0, corridor_heading_weight_);
  frontier_unknown_search_radius_ = std::max(
      current_map_.info.resolution > 0.0 ? current_map_.info.resolution : 0.05,
      std::min(frontier_unknown_search_radius_, 1.00));
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
  frontier_failure_ttl_ = std::max(10.0, frontier_failure_ttl_);
  bridge_failure_ttl_ = std::max(10.0, bridge_failure_ttl_);
  stall_failure_ttl_ = std::max(10.0, stall_failure_ttl_);
  emergency_stop_confirm_scans_ = std::max(1, emergency_stop_confirm_scans_);
  emergency_hard_confirm_scans_ = std::max(2, emergency_hard_confirm_scans_);
  emergency_clear_confirm_scans_ = std::max(1, emergency_clear_confirm_scans_);
  emergency_escape_delay_ = std::max(0.5, std::min(10.0, emergency_escape_delay_));
  emergency_escape_max_attempts_ = std::max(0, std::min(3, emergency_escape_max_attempts_));
  emergency_front_observation_duration_ = std::max(
      0.2, std::min(10.0, emergency_front_observation_duration_));
  visual_area_stop_threshold_ = std::max(0.001, std::min(1.0, visual_area_stop_threshold_));
  visual_stop_confirmations_ = std::max(1, visual_stop_confirmations_);
  visual_detection_timeout_ = std::max(0.10, visual_detection_timeout_);
  visual_angle_gain_ = std::max(0.0, visual_angle_gain_);
  visual_max_linear_speed_ = std::max(0.0, std::min(0.25, visual_max_linear_speed_));
  visual_max_angular_speed_ = std::max(0.0, std::min(1.0, visual_max_angular_speed_));
  visual_align_angle_deg_ = std::max(0.0, std::min(90.0, visual_align_angle_deg_));
  visual_sync_tolerance_ = std::max(0.02, std::min(1.0, visual_sync_tolerance_));
  visual_min_front_clearance_ = std::max(
      emergency_stop_dist_ + 0.05, visual_min_front_clearance_);
  if (!std::isfinite(camera_coverage_radius_) || camera_coverage_radius_ <= 0.0)
    camera_coverage_radius_ = fov_range_;
  camera_coverage_radius_ = std::max(0.5, std::min(10.0, camera_coverage_radius_));
  camera_coverage_angle_deg_ = std::max(20.0, std::min(120.0, camera_coverage_angle_deg_));
  camera_record_distance_ = std::max(0.05, std::min(1.0, camera_record_distance_));
  camera_record_yaw_deg_ = std::max(2.0, std::min(45.0, camera_record_yaw_deg_));
  min_unsearched_novelty_ = std::max(0.0, std::min(1.0, min_unsearched_novelty_));
  diagnostic_log_period_ = std::max(5.0, diagnostic_log_period_);
  if (std::fabs(visual_angle_sign_) < 0.5)
    visual_angle_sign_ = visual_angle_sign_ < 0.0 ? -1.0 : 1.0;

  max_rotate_angle_ = max_rotate_angle_ * M_PI / 180.0;
  node_start_time_ = ros::Time::now();
  last_goal_switch_time_ = node_start_time_;
  last_vel_update_time_ = node_start_time_;
  last_progress_check_time_ = node_start_time_;
  last_goal_sent_time_ = node_start_time_;
  last_planning_attempt_time_ = node_start_time_;

  tf_buffer_.setUsingDedicatedThread(true);

  object_sub_ = nh_.subscribe(object_topic_, 1, &NavigationDriver::objectCallback, this);
  object_direction_sub_ = nh_.subscribe(
      object_direction_topic_, 5, &NavigationDriver::objectDirectionCallback, this);
  object_area_sub_ = nh_.subscribe(
      object_area_topic_, 5, &NavigationDriver::objectAreaCallback, this);
  map_sub_ = nh_.subscribe("/map", 1, &NavigationDriver::mapCallback, this);
  laser_sub_ = nh_.subscribe("/scan", 1, &NavigationDriver::laserCallback, this);
  nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
  odom_sub_ = nh_.subscribe(odom_topic_, 10, &NavigationDriver::odomCallback, this);
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
  visual_servo_timer_ = nh_.createTimer(
      ros::Duration(0.05), &NavigationDriver::visualServoTimerCallback, this, false, true);
  ROS_INFO("Navigation driver initialized (emergency stop at %.2f m, start delay %.1f s).",
           emergency_stop_dist_, start_delay_);
  ROS_INFO("[CONFIG] Topics: map=/map scan=/scan odom=%s object=%s cmd_vel=%s safety_stop=%s action=/move_base",
           odom_topic_.c_str(), object_topic_.c_str(), cmd_vel_topic_.c_str(), safety_stop_topic_.c_str());
  ROS_INFO("[CONFIG] Visual servo: enabled=%s direction=%s area=%s stop=%.3f confirmations=%d timeout=%.2f s sync=%.2f s angle_gain=%.2f sign=%.1f max_speed=%.2f m/s max_yaw=%.2f rad/s align=%.1f deg front_clearance=%.2f m.",
           enable_visual_servo_ ? "true" : "false", object_direction_topic_.c_str(),
           object_area_topic_.c_str(), visual_area_stop_threshold_,
           visual_stop_confirmations_, visual_detection_timeout_, visual_sync_tolerance_,
           visual_angle_gain_,
           visual_angle_sign_, visual_max_linear_speed_, visual_max_angular_speed_,
           visual_align_angle_deg_, visual_min_front_clearance_);
  ROS_INFO("[CONFIG] Frames: global=map local=odom base=base_link; scan FOV=%.1f deg; object_fov_range=%.2f m.",
           fov_horizontal_, fov_range_);
  ROS_INFO("[CONFIG] Laser emergency FOV is base_link [%.1f,%.1f] deg; laser zero is transformed through base_link<-laser_link. confirm=%d hard_confirm=%d clear=%d hard_stop=%.2f m near_points=%d cluster_points=%d floor_margin=%.3f m escape_delay=%.1f s escape_attempts=%d/%d front_observation=%.1f s.",
           -fov_horizontal_ / 2.0, fov_horizontal_ / 2.0,
           emergency_stop_confirm_scans_, emergency_hard_confirm_scans_,
           emergency_clear_confirm_scans_, emergency_stop_hard_dist_,
           emergency_min_near_points_, emergency_min_cluster_points_, emergency_range_floor_margin_,
           emergency_escape_delay_, emergency_escape_attempt_count_,
           emergency_escape_max_attempts_, emergency_front_observation_duration_);
  ROS_INFO("[CONFIG] Runtime footprint topics: global=/move_base/global_costmap/footprint local=/move_base/local_costmap/footprint.");
  ROS_INFO("[CONFIG] Limits: max_vel_x=%.2f m/s max_vel_theta=%.2f rad/s emergency_stop=%.2f m.",
           max_autonomous_speed_, 0.8, emergency_stop_dist_);
  ROS_INFO("[CONFIG] Timing: exploration=%.2f Hz map_stable=%.2f s goal_timeout=%.1f s planning_retry=%.1f s diagnostics=%.1f s.",
           exploration_frequency_, map_stable_time_, goal_timeout_, planning_retry_interval_,
           diagnostic_log_period_);
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
  ROS_INFO("Exploration mode: connected free-space frontiers with explicit bridge recovery.");
  ROS_INFO("Real-map thresholds: free <= %d, occupied >= %d, clearance %.2f m, speed cap %.2f m/s.",
           free_cell_threshold_, occupied_cell_threshold_, frontier_clearance_,
           max_autonomous_speed_);
  ROS_INFO("[CONFIG] Frontier score: normalized information/spatial/branch/novelty terms; "
           "corridor_heading_weight=%.2f cluster_radius=%.2f m info_radius=%.2f m.",
           corridor_heading_weight_, frontier_cluster_radius_,
           frontier_unknown_search_radius_);
  ROS_INFO("[CONFIG] Exploration failure TTL: frontier=%.1f s bridge=%.1f s stall=%.1f s.",
           frontier_failure_ttl_, bridge_failure_ttl_, stall_failure_ttl_);
  ROS_INFO("[CONFIG] Coverage memory: radius=%.2f m angle=%.1f deg record=(%.2f m, %.1f deg) min_unsearched=%.2f.",
           camera_coverage_radius_, camera_coverage_angle_deg_,
           camera_record_distance_, camera_record_yaw_deg_,
           min_unsearched_novelty_);
}

NavigationDriver::~NavigationDriver()
{
  stopEscapeBackoff();
  stopSmartRotation();
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
  // Dynamic-reconfigure rebuilds TEB's internal planner.  Only update when
  // the limit changed materially and the minimum update period elapsed;
  // previously the time condition alone caused a service call roughly every
  // second even when the requested velocity was unchanged.
  if (fabs(vel - last_vel_set_) < vel_change_threshold_)
    return;
  if ((now - last_vel_update_time_).toSec() < vel_update_interval_)
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
  // Record arrival before trying to acquire the navigation-state lock.  A
  // planning/map callback can legitimately hold that lock for a few hundred
  // milliseconds; scans must not be classified as stale merely because their
  // analysis was deferred by that contention.
  last_scan_arrival_nsec_.store(
      static_cast<std::int64_t>(ros::Time::now().toNSec()),
      std::memory_order_release);
  // A timer callback may be performing a full-map operation or waiting on a
  // service.  Do not queue a laser callback behind that lock: fail safe and
  // let the next scan recover normal operation.
  std::unique_lock<std::mutex> lock(data_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
  {
    // A transient callback collision is not evidence of an obstacle.  Do not
    // publish a zero on the navigation command topic here: the arbiter gives
    // that topic priority over move_base, so a one-cycle zero would manifest
    // as visible stop/start jitter.  The independent scan watchdog publishes
    // a real safety stop if contention lasts beyond scan_timeout_.
    const std::int64_t now_nsec =
        static_cast<std::int64_t>(ros::Time::now().toNSec());
    const std::int64_t arrival_nsec =
        last_scan_arrival_nsec_.load(std::memory_order_acquire);
    const double arrival_age = arrival_nsec > 0 && now_nsec >= arrival_nsec
        ? (now_nsec - arrival_nsec) / 1e9 : -1.0;
    ROS_WARN_THROTTLE(2.0,
        "[EVENT][SAFETY_LOCK_CONTENTION] Laser callback could not acquire "
        "navigation state lock; dropping this scan (arrival_age=%.3f s, "
        "safety_atomic=%s).",
        arrival_age,
        safety_stop_atomic_.load(std::memory_order_acquire) ? "ON" : "OFF");
    return;
  }
  bool update_velocity = false;
  double velocity_to_apply = current_max_vel_x_;
  const ros::WallTime scan_processing_start = ros::WallTime::now();
  // Publish the watchdog timestamp only after this callback has completed its
  // guarded scan analysis.  A long map/planning callback can otherwise make
  // the heartbeat believe that an unprocessed scan is still fresh.
  const auto mark_scan_processed = [&]() {
    const ros::Time completed = ros::Time::now();
    last_scan_received_time_ = completed;
    last_scan_received_nsec_.store(
        static_cast<int64_t>(completed.toNSec()), std::memory_order_release);
    last_scan_processing_duration_ =
        (ros::WallTime::now() - scan_processing_start).toSec();
  };
  latest_scan_ = *msg;
  has_scan_ = true;
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

  if (task_finished_)
  {
    mark_scan_processed();
    return;
  }

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
      ROS_WARN("[STOP_REASON][SCAN_UNUSABLE] tf=%s tf_age=%.3f valid=%d/%d "
               "invalid=%d frame=%s; navigation command forced to zero.",
               scan_transform_available ? last_scan_tf_status_.c_str() : "missing",
               last_scan_tf_age_, valid_count, static_cast<int>(msg->ranges.size()),
               invalid_count, msg->header.frame_id.c_str());
      emergency_stopped_ = true;
      emergency_escape_attempted_ = false;
      emergency_escape_attempt_count_ = 0;
      emergency_front_observation_until_ = ros::Time(0);
      visual_control_engaged_ = false;
      emergency_stop_time_ = ros::Time::now();
      geometry_msgs::Twist stop;
      publishCmdVel(stop);
      cancelActiveGoal();
      has_last_goal_ = false;
      current_goal_is_object_ = false;
      current_goal_is_bridge_ = false;
      stopEscapeBackoff();
      stopSmartRotation();
    }
    publishSafetyStop(true);
    mark_scan_processed();
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

  const int hard_confirm_scans = emergency_hard_confirm_scans_;
  const bool confirm_stop =
      (hard_obstacle_in_base_fov &&
       emergency_stop_candidate_count_ >= hard_confirm_scans) ||
      emergency_stop_candidate_count_ >= emergency_stop_confirm_scans_;
  const bool scan_clear = center_valid_count > 0 &&
      (center_min_dist >= emergency_stop_dist_ + emergency_clear_margin_ ||
       !obstacle_in_base_fov);
  if (center_near_count > 0 && !confirm_stop)
  {
    // Keep a low-rate record of near returns that did not pass the stop
    // confirmation.  This is useful for distinguishing glass multipath,
    // range-floor saturation and real obstacles without weakening safety.
    ROS_INFO_THROTTLE(1.0,
        "[LASER_NEAR] center_min=%.3f threshold=%.3f hard=%.3f near=%d floor=%d cluster=%d required=%d angle=%.2f deg valid=%d",
        center_min_dist, emergency_stop_dist_, emergency_stop_hard_dist_,
        center_near_count, center_floor_count, max_center_near_cluster,
        required_cluster_points, min_base_angle * 180.0 / M_PI,
        center_valid_count);
  }
  // A confirmed obstacle blocks forward motion.  During the one guarded
  // reverse-recovery maneuver the front return is expected and must not
  // immediately cancel the rearward command; the escape timer independently
  // checks the rear 60-degree sector on every cycle.
  if (confirm_stop && !is_escape_backing_)
  {
    if (!emergency_stopped_)
    {
      const actionlib::SimpleClientGoalState state = ac_.getState();
      geometry_msgs::Twist cmd_snapshot;
      {
        std::lock_guard<std::mutex> cmd_lock(cmd_publish_mutex_);
        cmd_snapshot = last_cmd_vel_;
      }
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
               cmd_snapshot.linear.x, cmd_snapshot.angular.z);
      ROS_WARN("[STOP_REASON][LASER_OBSTACLE] confirmed central obstacle: "
               "center_min=%.3f threshold=%.3f hard=%.3f near=%d cluster=%d "
               "confirm=%d/%d base_angle=%.2f deg; canceling active goal and "
               "forcing safety stop.",
               center_min_dist, emergency_stop_dist_, emergency_stop_hard_dist_,
               center_near_count, max_center_near_cluster,
               emergency_stop_candidate_count_, emergency_stop_confirm_scans_,
               min_base_angle * 180.0 / M_PI);
      emergency_stopped_ = true;
      emergency_escape_attempted_ = false;
      emergency_escape_attempt_count_ = 0;
      emergency_front_observation_until_ = ros::Time(0);
      visual_control_engaged_ = false;
      emergency_stop_time_ = ros::Time::now();
      geometry_msgs::Twist stop;
      publishCmdVel(stop);
      cancelActiveGoal();
      has_last_goal_ = false;
      current_goal_is_object_ = false;
      current_goal_is_bridge_ = false;
      stopEscapeBackoff();
      stopSmartRotation();
    }
    emergency_clear_candidate_count_ = 0;
    const ros::Time now = ros::Time::now();
    const bool observation_complete =
        emergency_front_observation_until_.isZero() ||
        now >= emergency_front_observation_until_;
    // If the front is still blocked after the observation window, permit a
    // second attempt only when the configured cap has not been reached.  The
    // default is one attempt, which turns an unrecoverable glass return into a
    // latched stop instead of an endless reverse loop.
    if (emergency_escape_attempted_ && observation_complete &&
        emergency_escape_attempt_count_ < emergency_escape_max_attempts_)
    {
      emergency_escape_attempted_ = false;
      emergency_front_observation_until_ = ros::Time(0);
    }

    if (emergency_stopped_ && !emergency_escape_attempted_ &&
        !is_escape_backing_ &&
        emergency_escape_attempt_count_ < emergency_escape_max_attempts_ &&
        (now - emergency_stop_time_).toSec() >= emergency_escape_delay_)
    {
      emergency_escape_attempted_ = true;
      ++emergency_escape_attempt_count_;
      emergency_front_observation_until_ = ros::Time(0);
      ROS_WARN("[EVENT][LASER_ESCAPE] Front obstacle remained for %.1f s; checking rear clearance for bounded reverse recovery attempt %d/%d.",
               emergency_escape_delay_, emergency_escape_attempt_count_,
               emergency_escape_max_attempts_);
      startEscapeBackoff();
      if (!is_escape_backing_)
      {
        emergency_front_observation_until_ =
            now + ros::Duration(emergency_front_observation_duration_);
        ROS_WARN("[EVENT][LASER_ESCAPE] Reverse recovery was not started; safety stop remains active for %.1f s before re-evaluation.",
                 emergency_front_observation_duration_);
      }
    }
    else if (emergency_stopped_ && observation_complete &&
             emergency_escape_attempt_count_ >= emergency_escape_max_attempts_)
    {
      ROS_WARN_THROTTLE(5.0,
          "[EVENT][LASER_ESCAPE_LIMIT] Front obstacle remains after %d/%d recovery attempts; holding zero velocity.",
          emergency_escape_attempt_count_, emergency_escape_max_attempts_);
    }
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
      const ros::Time now = ros::Time::now();
      const bool observation_complete =
          emergency_front_observation_until_.isZero() ||
          now >= emergency_front_observation_until_;
      if (observation_complete &&
          (now - emergency_stop_time_).toSec() > emergency_stop_duration_ &&
          emergency_clear_candidate_count_ >= emergency_clear_confirm_scans_)
      {
        emergency_stopped_ = false;
        emergency_stop_candidate_count_ = 0;
        emergency_clear_candidate_count_ = 0;
        emergency_escape_attempted_ = false;
        emergency_escape_attempt_count_ = 0;
        emergency_front_observation_until_ = ros::Time(0);
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
  mark_scan_processed();
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

void NavigationDriver::objectDirectionCallback(const std_msgs::Float32ConstPtr& msg)
{
  if (!std::isfinite(msg->data)) return;
  std::lock_guard<std::mutex> lock(data_mutex_);
  // A horizontal bearing is bounded by the camera FOV.  Rejecting impossible
  // values prevents a malformed detector message from commanding a full-spin
  // angular velocity.
  if (msg->data < -180.0f || msg->data > 180.0f) return;
  last_visual_direction_deg_ = static_cast<double>(msg->data);
  last_visual_direction_time_ = ros::Time::now();
}

void NavigationDriver::objectAreaCallback(const std_msgs::Float32ConstPtr& msg)
{
  if (!std::isfinite(msg->data)) return;
  std::lock_guard<std::mutex> lock(data_mutex_);
  // The detector publishes area/image_area, so values outside [0, 1] are
  // invalid and must not be interpreted as an immediate target hit.
  if (msg->data < 0.0f || msg->data > 1.0f) return;
  last_visual_area_ratio_ = static_cast<double>(msg->data);
  last_visual_area_time_ = ros::Time::now();
  if (last_visual_area_ratio_ >= visual_area_stop_threshold_)
  {
    if (visual_stop_count_ < visual_stop_confirmations_)
      ++visual_stop_count_;
    if (visual_stop_count_ >= visual_stop_confirmations_ &&
        !visual_target_reached_)
    {
      visual_target_reached_ = true;
      ROS_INFO("[VISUAL] Mouse area threshold reached: area=%.4f threshold=%.4f confirmations=%d.",
               last_visual_area_ratio_, visual_area_stop_threshold_,
               visual_stop_confirmations_);
    }
  }
  else
  {
    visual_stop_count_ = 0;
  }
}

void NavigationDriver::objectCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // TF lookup can wait up to the configured timeout. Do it before taking the
  // navigation-state mutex so a camera-rate detector cannot starve LaserScan
  // safety processing while the transform tree is catching up.
  geometry_msgs::PoseStamped pose_map;
  try
  {
    tf_buffer_.transform(*msg, pose_map, "map", ros::Duration(0.2));
  }
  catch (const tf2::TransformException &ex)
  {
    ROS_WARN_THROTTLE(5.0, "[TF] Object transform failed: %s", ex.what());
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  {
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
    failed_goals_.clear();

    // P0修复1: 不立即清除emergency_stopped_，等待laserCallback确认路径安全
    if (emergency_stopped_) {
      ROS_WARN("[OBJECT] Object detected but emergency stop is active. Waiting for clear path...");
      // 不清除emergency_stopped_，不发送新目标
      return;
    }
    if (is_escape_backing_)
    {
      ROS_WARN_THROTTLE(2.0,
          "[OBJECT] Object update ignored while emergency reverse recovery is active.");
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
    stopEscapeBackoff();
    ROS_INFO("[OBJECT] Detected at (%.2f, %.2f). Approaching.",
             object_pose_map_.position.x, object_pose_map_.position.y);
    cancelActiveGoal();
    has_last_goal_ = false;
    current_goal_is_object_ = false;
    current_goal_is_bridge_ = false;
    requestPlanning();
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

void NavigationDriver::blacklistExplorationGoal(
    const geometry_msgs::PoseStamped& goal, double ttl,
    const std::string& reason)
{
  const ros::Time now = ros::Time::now();
  pruneFailedGoals(now);
  const double failed_x = goal.pose.position.x;
  const double failed_y = goal.pose.position.y;
  for (auto& existing : failed_goals_)
  {
    if (distance(existing.x, existing.y, failed_x, failed_y) <
        frontier_cluster_radius_)
    {
      existing.expires_at = std::max(existing.expires_at, now + ros::Duration(ttl));
      existing.reason = reason;
      return;
    }
  }
  FailedGoalRecord failed;
  failed.x = failed_x;
  failed.y = failed_y;
  failed.expires_at = now + ros::Duration(ttl);
  failed.reason = reason;
  failed_goals_.push_back(failed);
  const size_t max_blacklist_size = 200;
  if (failed_goals_.size() > max_blacklist_size)
    failed_goals_.erase(failed_goals_.begin());
  ROS_WARN("[GOAL_MEMORY] blocked goal=(%.2f,%.2f) reason=%s ttl=%.1f s active=%zu.",
           failed_x, failed_y, reason.c_str(), ttl, failed_goals_.size());
}

void NavigationDriver::pruneFailedGoals(const ros::Time& now)
{
  failed_goals_.erase(
      std::remove_if(failed_goals_.begin(), failed_goals_.end(),
          [&now](const FailedGoalRecord& failed) {
            return !failed.expires_at.isZero() && failed.expires_at <= now;
          }),
      failed_goals_.end());
}

void NavigationDriver::recordExplorationSuccess(
    const geometry_msgs::PoseStamped& goal)
{
  exploration_visit_history_.push_back(
      {goal.pose.position.x, goal.pose.position.y});
  if (exploration_visit_history_.size() > 200)
    exploration_visit_history_.erase(exploration_visit_history_.begin());

  exploration_heading_ = tf2::getYaw(goal.pose.orientation);
  has_exploration_heading_ = true;

  failed_goals_.erase(
      std::remove_if(failed_goals_.begin(), failed_goals_.end(),
          [this, &goal](const FailedGoalRecord& failed) {
            return distance(failed.x, failed.y, goal.pose.position.x,
                            goal.pose.position.y) < frontier_cluster_radius_;
          }),
      failed_goals_.end());
}

bool NavigationDriver::isMapStable()
{
  if (!map_received_) return false;
  return (ros::Time::now() - last_map_change_time_).toSec() > map_stable_time_;
}

void NavigationDriver::updateCameraCoverage(const ros::Time& now)
{
  geometry_msgs::TransformStamped transform;
  if (!getFreshRobotTransform(transform)) return;

  const double x = transform.transform.translation.x;
  const double y = transform.transform.translation.y;
  const double yaw = tf2::getYaw(transform.transform.rotation);
  const double yaw_threshold = camera_record_yaw_deg_ * M_PI / 180.0;
  bool should_record = !has_last_camera_view_pose_;
  if (has_last_camera_view_pose_)
  {
    const double moved = distance(
        x, y, last_camera_view_pose_.position.x,
        last_camera_view_pose_.position.y);
    const double dyaw = std::fabs(std::atan2(
        std::sin(yaw - tf2::getYaw(last_camera_view_pose_.orientation)),
        std::cos(yaw - tf2::getYaw(last_camera_view_pose_.orientation))));
    should_record = moved >= camera_record_distance_ || dyaw >= yaw_threshold;
  }
  if (!should_record) return;

  CameraViewRecord record;
  record.x = x;
  record.y = y;
  record.yaw = yaw;
  record.stamp = now;
  camera_view_history_.push_back(record);
  // A bounded history is sufficient for a building corridor and prevents
  // exploration memory from growing without limit during a long run.
  const size_t max_history = 2000;
  if (camera_view_history_.size() > max_history)
    camera_view_history_.erase(camera_view_history_.begin(),
                               camera_view_history_.begin() +
                                   (camera_view_history_.size() - max_history));

  last_camera_view_pose_.position.x = x;
  last_camera_view_pose_.position.y = y;
  last_camera_view_pose_.orientation = transform.transform.rotation;
  has_last_camera_view_pose_ = true;
}

double NavigationDriver::cameraViewNovelty(double x, double y, double yaw) const
{
  if (camera_view_history_.empty()) return 1.0;

  const double half_fov = camera_coverage_angle_deg_ * M_PI / 360.0;
  double maximum_overlap = 0.0;
  for (const auto& view : camera_view_history_)
  {
    const double d = distance(x, y, view.x, view.y);
    if (d > camera_coverage_radius_) continue;
    const double dyaw = std::fabs(std::atan2(
        std::sin(yaw - view.yaw), std::cos(yaw - view.yaw)));
    if (dyaw > half_fov) continue;

    const double distance_overlap = std::max(
        0.0, 1.0 - d / std::max(1e-3, camera_coverage_radius_));
    const double angle_overlap = std::max(
        0.0, 1.0 - dyaw / std::max(1e-3, half_fov));
    maximum_overlap = std::max(maximum_overlap,
                               distance_overlap * angle_overlap);
  }
  return std::max(0.0, std::min(1.0, 1.0 - maximum_overlap));
}

void NavigationDriver::preferUnsearchedCandidates(
    std::vector<std::pair<geometry_msgs::PoseStamped, double>>& candidates) const
{
  if (candidates.empty() || camera_view_history_.empty()) return;

  std::vector<std::pair<geometry_msgs::PoseStamped, double>> novel;
  novel.reserve(candidates.size());
  for (const auto& candidate : candidates)
  {
    const double yaw = tf2::getYaw(candidate.first.pose.orientation);
    if (cameraViewNovelty(candidate.first.pose.position.x,
                          candidate.first.pose.position.y, yaw) >=
        min_unsearched_novelty_)
      novel.push_back(candidate);
  }

  // This helper is used by normal exploration candidates only.  If every
  // candidate overlaps an already observed camera view, return an empty list
  // so the caller can enter the explicit bridge/recovery path instead of
  // silently sending the robot back into a searched corridor.
  candidates.swap(novel);
}

bool NavigationDriver::rotationObservedNear(double x, double y) const
{
  for (const auto& point : rotation_observation_history_)
  {
    if (distance(x, y, point.first, point.second) < 0.80)
      return true;
  }
  return false;
}

bool NavigationDriver::findBacktrackGoal(geometry_msgs::PoseStamped& goal)
{
  if (!map_received_ || camera_view_history_.empty()) return false;

  geometry_msgs::TransformStamped robot_transform;
  if (!getFreshRobotTransform(robot_transform)) return false;
  const double robot_x = robot_transform.transform.translation.x;
  const double robot_y = robot_transform.transform.translation.y;
  const int width = static_cast<int>(current_map_.info.width);
  const int clearance_cells = footprintClearanceCells();

  std::vector<unsigned char> reachable;
  int robot_mx = 0;
  int robot_my = 0;
  if (!buildReachableMask(reachable, robot_mx, robot_my,
                          std::max(12.0, extended_search_range_)))
    return false;

  double best_cost = std::numeric_limits<double>::max();
  bool found = false;
  // A bridge is useful only when its selected heading actually points toward
  // map cells that are still unknown.  Checking the ray (rather than merely
  // checking heading novelty) prevents a previously observed corridor from
  // being revisited for an arbitrary, already-known direction.
  const double resolution = current_map_.info.resolution;
  auto hasUnknownAhead = [&](double x, double y, double yaw) {
    const int ray_steps = std::max(1, static_cast<int>(std::ceil(2.5 / resolution)));
    const int lateral_cells = std::max(1, static_cast<int>(std::ceil(0.20 / resolution)));
    for (int step = std::max(1, static_cast<int>(std::ceil(0.45 / resolution)));
         step <= ray_steps; ++step)
    {
      const double distance_ahead = step * resolution;
      const double cx = x + distance_ahead * std::cos(yaw);
      const double cy = y + distance_ahead * std::sin(yaw);
      int mx = 0;
      int my = 0;
      // Cartographer crops /map to observed cells.  Leaving the current array
      // after traversing known free cells is valid unseen-space evidence, not
      // proof that the branch does not exist.
      if (!worldToMap(cx, cy, mx, my)) return true;
      const size_t center_index = static_cast<size_t>(my) * width + mx;
      if (center_index >= current_map_.data.size()) break;
      if (current_map_.data[center_index] == -1) return true;
      // Unknown space behind an occupied or uncertain return is occluded and
      // cannot justify revisiting this bridge with a forward-facing camera.
      if (!isKnownFreeCell(mx, my)) break;
      for (int dy = -lateral_cells; dy <= lateral_cells; ++dy)
      {
        for (int dx = -lateral_cells; dx <= lateral_cells; ++dx)
        {
          const int nx = mx + dx;
          const int ny = my + dy;
          if (nx < 0 || ny < 0 || nx >= width || ny >= static_cast<int>(current_map_.info.height))
            continue;
          const size_t index = static_cast<size_t>(ny) * width + nx;
          if (index < current_map_.data.size() && current_map_.data[index] == -1)
            return true;
        }
      }
    }
    return false;
  };
  // Camera views are sampled densely along the travelled path.  Select only
  // spatially separated records so a long corridor does not generate a
  // sequence of almost identical backtrack goals.
  std::vector<std::pair<double, double>> selected_records;
  for (auto it = camera_view_history_.rbegin();
       it != camera_view_history_.rend(); ++it)
  {
    const double node_x = it->x;
    const double node_y = it->y;
    if (distance(robot_x, robot_y, node_x, node_y) < 0.90)
      continue;

    bool too_close = false;
    for (const auto& selected : selected_records)
    {
      if (distance(node_x, node_y, selected.first, selected.second) < 0.60)
      {
        too_close = true;
        break;
      }
    }
    if (too_close) continue;
    selected_records.push_back({node_x, node_y});

    int node_mx = 0;
    int node_my = 0;
    if (!worldToMap(node_x, node_y, node_mx, node_my)) continue;

    for (int k = 0; k < 16; ++k)
    {
      const double branch_yaw = -M_PI + (k + 0.5) * (2.0 * M_PI / 16.0);
      const double novelty = cameraViewNovelty(node_x, node_y, branch_yaw);
      if (novelty < min_unsearched_novelty_) continue;
      if (!hasUnknownAhead(node_x, node_y, branch_yaw)) continue;

      // Aim one metre into the branch when a known-free approach cell exists;
      // otherwise use the remembered node itself and rotate there.
      const double candidate_distance = 1.0;
      double candidate_x = node_x + candidate_distance * std::cos(branch_yaw);
      double candidate_y = node_y + candidate_distance * std::sin(branch_yaw);
      int candidate_mx = 0;
      int candidate_my = 0;
      bool candidate_ok = worldToMap(candidate_x, candidate_y,
                                     candidate_mx, candidate_my);
      if (candidate_ok)
      {
        const size_t index = static_cast<size_t>(candidate_my) * width + candidate_mx;
        candidate_ok = index < reachable.size() && reachable[index] &&
            isKnownFreeCell(candidate_mx, candidate_my) &&
            !hasOccupiedNeighbor(candidate_mx, candidate_my, clearance_cells);
      }
      if (!candidate_ok)
      {
        candidate_x = node_x;
        candidate_y = node_y;
        if (!worldToMap(candidate_x, candidate_y, candidate_mx, candidate_my))
          continue;
        const size_t index = static_cast<size_t>(candidate_my) * width + candidate_mx;
        candidate_ok = index < reachable.size() && reachable[index] &&
            isKnownFreeCell(candidate_mx, candidate_my) &&
            !hasOccupiedNeighbor(candidate_mx, candidate_my, clearance_cells);
      }
      if (!candidate_ok) continue;

      const double travel_distance = distance(robot_x, robot_y,
                                              candidate_x, candidate_y);
      const double cost = travel_distance - 2.0 * novelty;
      if (cost >= best_cost) continue;

      goal.header.frame_id = "map";
      goal.header.stamp = ros::Time(0);
      goal.pose.position.x = candidate_x;
      goal.pose.position.y = candidate_y;
      goal.pose.position.z = 0.0;
      goal.pose.orientation = tf2::toMsg(
          tf2::Quaternion(tf2::Vector3(0, 0, 1), branch_yaw));
      best_cost = cost;
      found = true;
    }
  }

  if (found)
  {
    ROS_WARN("[BACKTRACK] Reusing known path only to reach an unsearched camera direction: goal=(%.2f, %.2f) yaw=%.1f deg.",
             goal.pose.position.x, goal.pose.position.y,
             tf2::getYaw(goal.pose.orientation) * 180.0 / M_PI);
  }
  return found;
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

bool NavigationDriver::getFreshRobotTransform(
    geometry_msgs::TransformStamped& transform) const
{
  try
  {
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(5.0, "[TF] map->base_link unavailable for planning: %s", ex.what());
    return false;
  }

  if (transform.header.stamp.isZero())
  {
    ROS_WARN_THROTTLE(5.0, "[TF] map->base_link returned a zero timestamp; refusing to plan.");
    return false;
  }

  const double age = std::fabs((ros::Time::now() - transform.header.stamp).toSec());
  // Planning on a stale Cartographer correction can select a frontier behind
  // the robot and make an otherwise valid goal appear unreachable.  Keep this
  // threshold looser than the scan transform watchdog to tolerate low-rate
  // map->odom publication while still rejecting genuinely old poses.
  const double max_age = std::max(0.50, 2.0 * scan_tf_max_age_);
  if (age > max_age)
  {
    ROS_WARN_THROTTLE(5.0,
        "[TF] map->base_link is stale (age=%.3f s, limit=%.3f s); refusing to plan.",
        age, max_age);
    return false;
  }
  return true;
}

int NavigationDriver::footprintClearanceCells() const
{
  // Frontier reachability must account for the robot footprint, but using the
  // circumscribed diagonal radius rejects narrow corridors that the exact
  // polygon costmap can safely traverse.  Use the largest axis-aligned
  // half-extent plus padding here; move_base/TEB remains the final collision
  // checker with the full polygon footprint.
  double max_extent = 0.0;
  const auto accumulate = [&max_extent](const geometry_msgs::PolygonStamped& polygon,
                                    bool available) {
    if (!available) return;
    // Costmap's footprint topic is normally published after transforming the
    // polygon into the costmap global frame (map/odom), so its coordinates are
    // absolute robot-world positions.  Computing hypot(point.x, point.y) for
    // those messages would incorrectly turn the robot's distance from the map
    // origin into a multi-metre footprint radius.  Only a base_link-relative
    // polygon can be used directly here; otherwise use the configured fallback
    // below.
    if (!polygon.header.frame_id.empty() && polygon.header.frame_id != "base_link")
      return;
    for (const auto& point : polygon.polygon.points)
      max_extent = std::max(max_extent,
                            std::max(std::fabs(static_cast<double>(point.x)),
                                     std::fabs(static_cast<double>(point.y))));
  };
  accumulate(global_footprint_, has_global_footprint_);
  accumulate(local_footprint_, has_local_footprint_);

  // Keep startup safe before costmap publishes its runtime footprint.  This
  // matches the polygon in nav_driver.launch (0.64 x 0.57 m) plus padding.
  if (max_extent < 1e-3) max_extent = std::max(0.32, 0.285);
  double radius = max_extent + 0.02;
  radius = std::max(radius, frontier_clearance_);
  return std::max(1, static_cast<int>(std::ceil(
      radius / current_map_.info.resolution)));
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
      // Match GlobalPlanner's allow_unknown=false policy: both unknown cells
      // and gray/occupied cells block the footprint.  Information rays may
      // look beyond this eroded free mask, but motion goals may not overlap it.
      if (current_map_.data[index] < 0 ||
          current_map_.data[index] > free_cell_threshold_) return true;
    }
  }
  return false;
}

bool NavigationDriver::buildReachableMask(
    std::vector<unsigned char>& reachable, int& robot_mx, int& robot_my,
    double search_range) const
{
  const int width = static_cast<int>(current_map_.info.width);
  const int height = static_cast<int>(current_map_.info.height);
  if (!map_received_ || width <= 0 || height <= 0 ||
      current_map_.data.size() != static_cast<size_t>(width * height))
  {
    return false;
  }

  geometry_msgs::TransformStamped transform;
  if (!getFreshRobotTransform(transform)) return false;

  if (!worldToMap(transform.transform.translation.x,
                  transform.transform.translation.y, robot_mx, robot_my))
  {
    ROS_WARN_THROTTLE(5.0, "[GRID] Robot pose is outside the occupancy grid.");
    return false;
  }

  const int clearance_cells = footprintClearanceCells();
  if (!isKnownFreeCell(robot_mx, robot_my) ||
      hasOccupiedNeighbor(robot_mx, robot_my, clearance_cells))
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
        const int candidate_x = robot_mx + dx;
        const int candidate_y = robot_my + dy;
        if (isKnownFreeCell(candidate_x, candidate_y) &&
            !hasOccupiedNeighbor(candidate_x, candidate_y, clearance_cells))
        {
          best_x = candidate_x;
          best_y = candidate_y;
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
  // Planning only needs connectivity inside the active search horizon. A
  // full-map flood fill becomes several million cell visits after Cartographer
  // expands the map and can starve the scan callback. Keep a one-metre margin
  // so a frontier just outside the nominal radius is still reachable.
  int min_x = 0;
  int max_x = width - 1;
  int min_y = 0;
  int max_y = height - 1;
  if (std::isfinite(search_range))
  {
    const int radius_cells = std::max(
        1, static_cast<int>(std::ceil((search_range + 1.0) /
                                       current_map_.info.resolution)));
    min_x = std::max(0, robot_mx - radius_cells);
    max_x = std::min(width - 1, robot_mx + radius_cells);
    min_y = std::max(0, robot_my - radius_cells);
    max_y = std::min(height - 1, robot_my + radius_cells);
  }

  // Precompute footprint clearance for the bounded search window.  Calling
  // hasOccupiedNeighbor() for every BFS edge repeats the same O(r^2) work and
  // exceeded the scan watchdog on large, tightly cropped Cartographer maps.
  const int expanded_min_x = std::max(0, min_x - clearance_cells);
  const int expanded_max_x = std::min(width - 1, max_x + clearance_cells);
  const int expanded_min_y = std::max(0, min_y - clearance_cells);
  const int expanded_max_y = std::min(height - 1, max_y + clearance_cells);
  const int expanded_width = expanded_max_x - expanded_min_x + 1;
  const int expanded_height = expanded_max_y - expanded_min_y + 1;
  const int prefix_stride = expanded_width + 1;
  std::vector<int> blocked_row_prefix(
      static_cast<size_t>(expanded_height) * prefix_stride, 0);
  for (int local_y = 0; local_y < expanded_height; ++local_y)
  {
    const int map_y = expanded_min_y + local_y;
    const size_t prefix_row = static_cast<size_t>(local_y) * prefix_stride;
    for (int local_x = 0; local_x < expanded_width; ++local_x)
    {
      const int map_x = expanded_min_x + local_x;
      const int value = current_map_.data[static_cast<size_t>(map_y) * width + map_x];
      const int blocked = value < 0 || value > free_cell_threshold_ ? 1 : 0;
      blocked_row_prefix[prefix_row + local_x + 1] =
          blocked_row_prefix[prefix_row + local_x] + blocked;
    }
  }

  const auto cellHasClearance = [&](int x, int y) {
    for (int dy = -clearance_cells; dy <= clearance_cells; ++dy)
    {
      const int map_y = y + dy;
      const int dx_limit = static_cast<int>(std::floor(std::sqrt(
          static_cast<double>(clearance_cells * clearance_cells - dy * dy))));
      const int left = x - dx_limit;
      const int right = x + dx_limit;
      if (map_y < 0 || map_y >= height || left < 0 || right >= width)
        return false;
      const int local_y = map_y - expanded_min_y;
      const int local_left = left - expanded_min_x;
      const int local_right = right - expanded_min_x;
      if (local_y < 0 || local_y >= expanded_height || local_left < 0 ||
          local_right >= expanded_width)
        return false;
      const size_t row = static_cast<size_t>(local_y) * prefix_stride;
      if (blocked_row_prefix[row + local_right + 1] -
              blocked_row_prefix[row + local_left] > 0)
        return false;
    }
    return true;
  };

  std::vector<unsigned char> traversable(
      static_cast<size_t>(width * height), 0);
  for (int y = min_y; y <= max_y; ++y)
    for (int x = min_x; x <= max_x; ++x)
      if (cellHasClearance(x, y))
        traversable[static_cast<size_t>(y) * width + x] = 1;

  std::vector<int> queue;
  queue.reserve(static_cast<size_t>((max_x - min_x + 1) *
                                    (max_y - min_y + 1)));
  const int start_index = robot_my * width + robot_mx;
  if (!traversable[start_index])
  {
    ROS_WARN_THROTTLE(5.0,
        "[GRID] Robot seed has no footprint clearance in the bounded mask.");
    return false;
  }
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
      if (nx < min_x || nx > max_x || ny < min_y || ny > max_y) continue;
      const int neighbor_index = ny * width + nx;
      if (!traversable[neighbor_index]) continue;
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
  if (!buildReachableMask(reachable, robot_mx, robot_my,
                          std::max(3.0, extended_search_range_))) return false;

  int goal_mx = 0;
  int goal_my = 0;
  if (!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
    return false;

  const int width = static_cast<int>(current_map_.info.width);
  const int radius = std::max(
      0, static_cast<int>(std::ceil(tolerance / current_map_.info.resolution)));
  const int clearance_cells = footprintClearanceCells();
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
  size_t valid_count = 0;
  size_t near_count = 0;
  int max_near_cluster = 0;
  int current_near_cluster = 0;
  std::vector<unsigned char> near_flags;
  near_flags.reserve(latest_scan_.ranges.size());
  for (double raw_range : latest_scan_.ranges)
  {
    double range = raw_range;
    if (std::isinf(range) && range > 0.0) range = latest_scan_.range_max;
    if (!std::isfinite(range) ||
        range < latest_scan_.range_min || range > latest_scan_.range_max)
    {
      near_flags.push_back(0);
      current_near_cluster = 0;
      continue;
    }
    const bool near = range < rotation_clearance_;
    near_flags.push_back(near ? 1 : 0);
    ++valid_count;
    if (near)
    {
      ++near_count;
      ++current_near_cluster;
      max_near_cluster = std::max(max_near_cluster, current_near_cluster);
    }
    else
    {
      current_near_cluster = 0;
    }
  }
  // One isolated floor/chassis return should not veto a 180-degree turn, but
  // a contiguous near cluster remains a hard clearance failure.  Merge the
  // cluster that wraps from the end of the scan to its beginning.
  if (!near_flags.empty() && near_flags.front() && near_flags.back())
  {
    size_t first = 0;
    while (first < near_flags.size() && near_flags[first]) ++first;
    size_t last = near_flags.size();
    while (last > 0 && near_flags[last - 1]) --last;
    if (first > 0 && last < near_flags.size())
      max_near_cluster = std::max(
          max_near_cluster, static_cast<int>(first + near_flags.size() - last));
  }
  const size_t required_valid = std::max<size_t>(
      static_cast<size_t>(scan_min_valid_count_), latest_scan_.ranges.size() / 4);
  const int required_cluster = std::max(3, emergency_min_cluster_points_);
  const int required_near = std::max(3, emergency_min_near_points_);
  const bool clustered_obstacle = near_count >= static_cast<size_t>(required_near) &&
      max_near_cluster >= required_cluster;
  return valid_count >= required_valid && !clustered_obstacle;
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
  std::lock_guard<std::mutex> publish_lock(cmd_publish_mutex_);
  last_cmd_vel_ = cmd;
  last_cmd_vel_time_ = ros::Time::now();
  cmd_vel_pub_.publish(cmd);
}

void NavigationDriver::publishSafetyStop(bool stopped)
{
  const bool previous = safety_stop_atomic_.exchange(
      stopped, std::memory_order_acq_rel);
  if (previous != stopped)
  {
    ROS_INFO("[STATE_CHANGE][SAFETY_STOP] %s -> %s.",
             previous ? "ON" : "OFF", stopped ? "ON" : "OFF");
  }
  std::lock_guard<std::mutex> publish_lock(safety_publish_mutex_);
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
  const std::int64_t arrival_nsec =
      last_scan_arrival_nsec_.load(std::memory_order_acquire);
  const std::int64_t processed_nsec =
      last_scan_received_nsec_.load(std::memory_order_acquire);
  const std::int64_t now_nsec =
      static_cast<std::int64_t>(ros::Time::now().toNSec());
  const std::int64_t timeout_nsec = static_cast<std::int64_t>(
      std::max(0.10, scan_timeout_) * 1e9);
  // A short planning lock can defer scan analysis without meaning that the
  // sensor stopped.  Conversely, an always-arriving but never-completed scan
  // callback is a real processing fault.  Use a bounded grace window for the
  // latter so lock contention does not disable the safety watchdog forever.
  const std::int64_t processing_grace_nsec = static_cast<std::int64_t>(
      std::max(2.0, 4.0 * std::max(0.10, scan_timeout_)) * 1e9);
  const bool arrival_fresh = arrival_nsec > 0 && now_nsec >= arrival_nsec &&
      (now_nsec - arrival_nsec) <= timeout_nsec;
  const bool processing_fresh = processed_nsec > 0 && now_nsec >= processed_nsec &&
      (now_nsec - processed_nsec) <= processing_grace_nsec;
  const bool scan_fresh = arrival_fresh && processing_fresh;
  if (!scan_fresh)
  {
    const double arrival_age = arrival_nsec > 0 && now_nsec >= arrival_nsec
        ? (now_nsec - arrival_nsec) / 1e9 : -1.0;
    const double processed_age = processed_nsec > 0 && now_nsec >= processed_nsec
        ? (now_nsec - processed_nsec) / 1e9 : -1.0;
    ROS_WARN_THROTTLE(2.0,
        "[EVENT][SCAN_WATCHDOG] Scan freshness failed (arrival_age=%.3f s, processed_age=%.3f s, arrival_timeout=%.3f s, processing_grace=%.3f s); safety stop latched.",
        arrival_age, processed_age, timeout_nsec / 1e9,
        processing_grace_nsec / 1e9);
    ROS_WARN_THROTTLE(2.0,
        "[STOP_REASON][SCAN_STALE] safety heartbeat is stopping output: "
        "arrival_age=%.3f/%.3f processed_age=%.3f/%.3f.",
        arrival_age, timeout_nsec / 1e9, processed_age,
        processing_grace_nsec / 1e9);
  }

  // A freshness failure must actively latch the published stop.  Reading the
  // previous atomic value here would otherwise leave a stale false value on
  // the arbiter when the heartbeat is the first callback to detect the fault.
  publishSafetyStop(scan_fresh ?
      safety_stop_atomic_.load(std::memory_order_acquire) : true);
}

void NavigationDriver::logDiagnosticSnapshot(const ros::Time& now)
{
  const actionlib::SimpleClientGoalState action_state = ac_.getState();
  const bool safety_stop = safety_stop_atomic_.load(std::memory_order_acquire);

  // The five-second snapshot is intentionally throttled, but state edges are
  // emitted immediately from the same 1 Hz planning timer.  This prevents a
  // short emergency stop, action preemption or visual hand-off from being
  // hidden between two snapshots.
  static bool state_initialized = false;
  static std::string previous_action;
  static bool previous_safety_stop = false;
  static bool previous_emergency = false;
  static bool previous_task_finished = false;
  static bool previous_has_goal = false;
  static bool previous_goal_object = false;
  static bool previous_goal_bridge = false;
  static bool previous_goal_pending = false;
  static bool previous_escape = false;
  static bool previous_rotation = false;
  static bool previous_visual = false;
  static bool previous_visual_target = false;
  const std::string action_name = action_state.toString();
  std::ostringstream changed;
  if (!state_initialized || previous_action != action_name)
    changed << " action=" << action_name;
  if (!state_initialized || previous_safety_stop != safety_stop)
    changed << " safety_stop=" << (safety_stop ? "ON" : "OFF");
  if (!state_initialized || previous_emergency != emergency_stopped_)
    changed << " emergency=" << (emergency_stopped_ ? "ON" : "OFF");
  if (!state_initialized || previous_task_finished != task_finished_)
    changed << " task_finished=" << (task_finished_ ? "true" : "false");
  if (!state_initialized || previous_has_goal != has_last_goal_)
    changed << " has_goal=" << (has_last_goal_ ? "true" : "false");
  if (!state_initialized || previous_goal_object != current_goal_is_object_ ||
      previous_goal_bridge != current_goal_is_bridge_)
    changed << " goal_type="
            << (has_last_goal_ ? (current_goal_is_object_ ? "object" :
                                  (current_goal_is_bridge_ ? "bridge" : "frontier")) : "none");
  if (!state_initialized || previous_goal_pending != goal_request_pending_)
    changed << " goal_pending=" << (goal_request_pending_ ? "true" : "false");
  if (!state_initialized || previous_escape != is_escape_backing_)
    changed << " escape_backing=" << (is_escape_backing_ ? "ON" : "OFF");
  if (!state_initialized || previous_rotation != is_smart_rotating_)
    changed << " smart_rotation=" << (is_smart_rotating_ ? "ON" : "OFF");
  if (!state_initialized || previous_visual != visual_control_engaged_)
    changed << " visual_control=" << (visual_control_engaged_ ? "ON" : "OFF");
  if (!state_initialized || previous_visual_target != visual_target_reached_)
    changed << " visual_target=" << (visual_target_reached_ ? "true" : "false");
  if (!changed.str().empty())
  {
    ROS_INFO("[STATE_CHANGE] t=%.3f%s", now.toSec(), changed.str().c_str());
    state_initialized = true;
    previous_action = action_name;
    previous_safety_stop = safety_stop;
    previous_emergency = emergency_stopped_;
    previous_task_finished = task_finished_;
    previous_has_goal = has_last_goal_;
    previous_goal_object = current_goal_is_object_;
    previous_goal_bridge = current_goal_is_bridge_;
    previous_goal_pending = goal_request_pending_;
    previous_escape = is_escape_backing_;
    previous_rotation = is_smart_rotating_;
    previous_visual = visual_control_engaged_;
    previous_visual_target = visual_target_reached_;
  }

  if (!last_diagnostic_log_time_.isZero() &&
      (now - last_diagnostic_log_time_).toSec() < diagnostic_log_period_)
    return;
  last_diagnostic_log_time_ = now;

  geometry_msgs::Twist cmd_snapshot;
  ros::Time cmd_time_snapshot;
  {
    std::lock_guard<std::mutex> cmd_lock(cmd_publish_mutex_);
    cmd_snapshot = last_cmd_vel_;
    cmd_time_snapshot = last_cmd_vel_time_;
  }
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
  const double cmd_age = cmd_time_snapshot.isZero()
      ? -1.0 : (now - cmd_time_snapshot).toSec();
  const double visual_direction_age = last_visual_direction_time_.isZero()
      ? -1.0 : (now - last_visual_direction_time_).toSec();
  const double visual_area_age = last_visual_area_time_.isZero()
      ? -1.0 : (now - last_visual_area_time_).toSec();
  const double odom_age = last_odom_received_time_.isZero()
      ? -1.0 : (now - last_odom_received_time_).toSec();
  const double odom_stamp_age = last_odom_stamp_.isZero()
      ? -1.0 : (now - last_odom_stamp_).toSec();
  const std::int64_t now_nsec = static_cast<std::int64_t>(now.toNSec());
  const std::int64_t arrival_nsec =
      last_scan_arrival_nsec_.load(std::memory_order_acquire);
  const std::int64_t processed_nsec =
      last_scan_received_nsec_.load(std::memory_order_acquire);
  const double scan_arrival_age = arrival_nsec > 0 && now_nsec >= arrival_nsec
      ? (now_nsec - arrival_nsec) / 1e9 : -1.0;
  const double scan_processed_age = processed_nsec > 0 && now_nsec >= processed_nsec
      ? (now_nsec - processed_nsec) / 1e9 : -1.0;
  const double scan_arrival_timeout = std::max(0.10, scan_timeout_);
  const double scan_processing_grace =
      std::max(2.0, 4.0 * scan_arrival_timeout);
  const bool scan_watchdog_fresh = scan_arrival_age >= 0.0 &&
      scan_arrival_age <= scan_arrival_timeout &&
      scan_processed_age >= 0.0 && scan_processed_age <= scan_processing_grace;
  const double map_change_age = last_map_change_time_.isZero()
      ? -1.0 : (now - last_map_change_time_).toSec();
  const bool map_stable = map_received_ && map_change_age >= map_stable_time_;
  const double start_elapsed = (now - node_start_time_).toSec();
  const double start_delay_remaining =
      std::max(0.0, start_delay_ - start_elapsed);
  const double goal_age = has_last_goal_ && !last_goal_sent_time_.isZero()
      ? (now - last_goal_sent_time_).toSec() : -1.0;
  const double planning_age = last_planning_attempt_time_.isZero()
      ? -1.0 : (now - last_planning_attempt_time_).toSec();
  const char* nav_cmd_owner = is_escape_backing_ ? "escape_backoff" :
      (is_smart_rotating_ ? "smart_rotation" :
       (visual_control_engaged_ ? "visual_servo" :
        (has_last_goal_ ? "move_base_goal" : "idle_or_stop")));

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
    goal << (current_goal_is_object_ ? "object" :
             (current_goal_is_bridge_ ? "bridge" : "frontier"))
         << " pose=(" << last_goal_.pose.position.x << ","
         << last_goal_.pose.position.y << ",yaw="
         << tf2::getYaw(last_goal_.pose.orientation) * 180.0 / M_PI << "deg)"
         << " age=" << (now - last_goal_sent_time_).toSec() << "s"
         << " timeout=" << current_goal_timeout_ << "s";
  }
  else
  {
    goal << "none";
  }

  const double escape_observation_remaining =
      emergency_front_observation_until_.isZero()
          ? 0.0
          : std::max(0.0,
                     (emergency_front_observation_until_ - now).toSec());

  ROS_INFO("[STATE] now=%.3f map=%s frame=%s size=%dx%d res=%.3f map_change_age=%.2f map_stamp_age=%.2f scan=%s frame=%s recv_age=%.2f stamp_age=%.2f proc=%.3fs count=%d angles=[%.2f,%.2f]deg inc=%.4fdeg range=[%.2f,%.2f]m min=%.3f m laser_angle=%.2f deg base_angle=%.2f deg center=(min=%.3f valid=%d near=%d floor=%d cluster=%d) fov_valid=%d fov_invalid=%d sectors=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] action=%s goal=%s emergency=%s escape=%d/%d obs_remaining=%.2f rotate=%s cmd=(%.3f,%.3f) cmd_age=%.2f max_vel=%.3f teb_limit=%.3f tf_map_odom=%s age=%.4f tf_odom_base=%s age=%.4f failures=%d/%d footprint_global={%s} footprint_local={%s} odom=frame:%s child:%s recv_age:%.2f stamp_age:%.2f twist:(%.3f,%.3f) scan_tf=%s xyz:(%.3f,%.3f) yaw:%.2fdeg age:%.4f visual=(enabled=%s engaged=%s target=%s angle=%.2fdeg area=%.4f confirm=%d/%d dir_age=%.2f area_age=%.2f)",
           now.toSec(), map_received_ ? "received" : "missing",
           last_map_frame_id_.c_str(), last_map_width_, last_map_height_,
           current_map_.info.resolution,
           last_map_change_time_.isZero() ? -1.0 : (now - last_map_change_time_).toSec(),
           map_stamp_age, has_scan_ ? "received" : "missing",
           last_scan_frame_id_.c_str(), scan_age, scan_stamp_age,
           last_scan_processing_duration_,
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
           emergency_escape_attempt_count_, emergency_escape_max_attempts_,
           escape_observation_remaining,
           is_smart_rotating_ ? "ON" : "OFF",
           cmd_snapshot.linear.x, cmd_snapshot.angular.z, cmd_age,
           current_max_vel_x_, last_vel_set_, map_odom_status.c_str(), map_odom_age,
           odom_base_status.c_str(), odom_base_age, consecutive_failures_,
           max_consecutive_failures_, global_fp.str().c_str(), local_fp.str().c_str(),
           last_odom_frame_id_.c_str(), last_odom_child_frame_id_.c_str(), odom_age,
           odom_stamp_age, last_odom_twist_.linear.x, last_odom_twist_.angular.z,
           last_scan_tf_status_.c_str(), last_scan_tf_x_, last_scan_tf_y_,
           last_scan_tf_yaw_ * 180.0 / M_PI, last_scan_tf_age_,
           enable_visual_servo_ ? "true" : "false",
           visual_control_engaged_ ? "true" : "false",
           visual_target_reached_ ? "true" : "false",
           last_visual_direction_deg_, last_visual_area_ratio_,
           visual_stop_count_, visual_stop_confirmations_,
           visual_direction_age, visual_area_age);

  ROS_INFO("[STATE_DETAIL] t=%.3f node_age=%.2f start_delay_remaining=%.2f "
           "lifecycle=(map=%s stable=%s task_finished=%s emergency=%s "
           "safety_atomic=%s) watchdog=(fresh=%s arrival_age=%.3f/%.3f "
           "processed_age=%.3f/%.3f stamp_age=%.3f tf=%s tf_age=%.3f "
           "valid=%d invalid=%d proc=%.3f) goal=(pending=%s has=%s type=%s "
           "action=%s age=%.2f/%.2f planning_age=%.2f) "
           "recovery=(escape=%s rotate=%s "
           "escape_attempt=%d/%d obs_remaining=%.2f) progress=(stall=%d/%d "
           "fail=%d consecutive=%d/%d) nav_cmd_owner=%s nav_cmd=(%.3f,%.3f) "
           "nav_cmd_age=%.2f max_vel=(%.3f teb=%.3f)",
           now.toSec(), start_elapsed, start_delay_remaining,
           map_received_ ? "received" : "missing", map_stable ? "true" : "false",
           task_finished_ ? "true" : "false", emergency_stopped_ ? "true" : "false",
           safety_stop ? "true" : "false", scan_watchdog_fresh ? "true" : "false",
           scan_arrival_age, scan_arrival_timeout,
           scan_processed_age, scan_processing_grace, scan_stamp_age,
           last_scan_tf_status_.c_str(), last_scan_tf_age_,
           last_scan_valid_count_, last_scan_invalid_count_,
           last_scan_processing_duration_, goal_request_pending_ ? "true" : "false",
           has_last_goal_ ? "true" : "false",
           has_last_goal_ ? (current_goal_is_object_ ? "object" :
                             (current_goal_is_bridge_ ? "bridge" : "frontier")) : "none",
           action_name.c_str(), goal_age, current_goal_timeout_, planning_age,
           is_escape_backing_ ? "ON" : "OFF", is_smart_rotating_ ? "ON" : "OFF",
           emergency_escape_attempt_count_, emergency_escape_max_attempts_,
           escape_observation_remaining,
           progress_stall_count_, progress_confirmations_, fail_count_,
           consecutive_failures_, max_consecutive_failures_, nav_cmd_owner,
           cmd_snapshot.linear.x, cmd_snapshot.angular.z, cmd_age,
           current_max_vel_x_, last_vel_set_);
  ROS_INFO("[COVERAGE] camera_views=%zu rotation_nodes=%zu; searched views are excluded from normal exploration unless used as a bridge to a novel direction.",
           camera_view_history_.size(), rotation_observation_history_.size());
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
    ROS_INFO("[GOAL_EVENT][CANCEL] state=%s generation=%llu -> %llu.",
             state.toString().c_str(),
             static_cast<unsigned long long>(goal_generation_),
             static_cast<unsigned long long>(goal_generation_ + 1));
    ++goal_generation_;
    ac_.cancelGoal();
  }
}

void NavigationDriver::visualServoTimerCallback(const ros::TimerEvent&)
{
  if (!enable_visual_servo_) return;

  // Keep visual control responsive without allowing it to block LaserScan or
  // the full-map planner.  The next 20 Hz tick retries if another callback is
  // using the navigation state.
  std::unique_lock<std::mutex> lock(data_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return;

  const ros::Time now = ros::Time::now();
  const bool direction_fresh = !last_visual_direction_time_.isZero() &&
      (now - last_visual_direction_time_).toSec() >= 0.0 &&
      (now - last_visual_direction_time_).toSec() <= visual_detection_timeout_;
  const bool area_fresh = !last_visual_area_time_.isZero() &&
      (now - last_visual_area_time_).toSec() >= 0.0 &&
      (now - last_visual_area_time_).toSec() <= visual_detection_timeout_;
  const bool visual_pair_synced = direction_fresh && area_fresh &&
      std::fabs((last_visual_direction_time_ - last_visual_area_time_).toSec()) <=
          visual_sync_tolerance_;
  const bool scan_fresh = has_scan_ && !last_scan_received_time_.isZero() &&
      (now - last_scan_received_time_).toSec() >= 0.0 &&
      (now - last_scan_received_time_).toSec() <= scan_timeout_;
  const bool visual_front_clear = last_center_valid_count_ >= scan_min_valid_count_ &&
      last_center_min_distance_ >= visual_min_front_clearance_;

  if (is_escape_backing_)
    return;

  if (task_finished_ || emergency_stopped_ || !scan_fresh ||
      (now - node_start_time_).toSec() < start_delay_)
  {
    if (visual_control_engaged_)
    {
      geometry_msgs::Twist stop;
      publishCmdVel(stop);
      visual_control_engaged_ = false;
    }
    return;
  }

  if (!direction_fresh || !area_fresh || !visual_pair_synced)
  {
    // A confirmation sequence is valid only within one uninterrupted stream
    // of detector frames.  Do not carry two high-area frames across a lost
    // camera update and count a later frame as the third confirmation.
    if ((!area_fresh || !visual_pair_synced) && visual_stop_count_ > 0 &&
        !visual_target_reached_)
    {
      visual_stop_count_ = 0;
      ROS_WARN_THROTTLE(5.0,
          "[VISUAL] Detection streams are stale or unsynchronized; reset stop confirmations.");
    }
    if (visual_control_engaged_)
    {
      geometry_msgs::Twist stop;
      publishCmdVel(stop);
      visual_control_engaged_ = false;
      ROS_WARN("[VISUAL] Detection timed out (direction_age=%.2f area_age=%.2f); releasing visual control.",
               last_visual_direction_time_.isZero()
                   ? -1.0 : (now - last_visual_direction_time_).toSec(),
               last_visual_area_time_.isZero()
                   ? -1.0 : (now - last_visual_area_time_).toSec());
    }
    return;
  }

  if (visual_target_reached_)
  {
    if (!task_finished_)
    {
      task_finished_ = true;
      stopEscapeBackoff();
      stopSmartRotation();
      cancelActiveGoal();
      has_last_goal_ = false;
      current_goal_is_object_ = false;
      current_goal_is_bridge_ = false;
      goal_request_pending_ = false;
      geometry_msgs::Twist stop;
      publishCmdVel(stop);
      visual_control_engaged_ = false;
      ROS_INFO("[VISUAL] Mouse found. Forward motion stopped and task completed.");
      ROS_INFO("[STOP_REASON][VISUAL_TARGET] area=%.4f threshold=%.4f "
               "confirmations=%d/%d; task_finished=true.",
               last_visual_area_ratio_, visual_area_stop_threshold_,
               visual_stop_count_, visual_stop_confirmations_);
    }
    return;
  }

  if (!visual_control_engaged_)
  {
    // Visual servo has priority over an exploratory move_base goal.  The
    // arbiter already gives navigation_driver commands priority, but canceling
    // the action avoids stale goal callbacks and planner churn in the logs.
    stopEscapeBackoff();
    stopSmartRotation();
    cancelActiveGoal();
    has_last_goal_ = false;
    current_goal_is_object_ = false;
    current_goal_is_bridge_ = false;
    goal_request_pending_ = false;
    visual_control_engaged_ = true;
    ROS_INFO("[VISUAL] Engaged: angle=%.2f deg area=%.4f.",
             last_visual_direction_deg_, last_visual_area_ratio_);
  }

  const double angle_rad = last_visual_direction_deg_ * M_PI / 180.0;
  const double abs_angle_deg = std::fabs(last_visual_direction_deg_);
  geometry_msgs::Twist cmd;
  cmd.angular.z = visual_angle_sign_ * visual_angle_gain_ * angle_rad;
  cmd.angular.z = std::max(-visual_max_angular_speed_,
                           std::min(visual_max_angular_speed_, cmd.angular.z));

  // Turn in place while the target is far off-axis.  Once reasonably aligned,
  // taper forward speed as the area approaches the 3.3% stop threshold.
  if (abs_angle_deg <= visual_align_angle_deg_)
  {
    const double remaining = std::max(
        0.0, std::min(1.0,
                      (visual_area_stop_threshold_ - last_visual_area_ratio_) /
                          visual_area_stop_threshold_));
    cmd.linear.x = visual_front_clear
        ? visual_max_linear_speed_ * remaining
        : 0.0;
  }
  else
  {
    cmd.linear.x = 0.0;
  }
  publishCmdVel(cmd);
  ROS_INFO_THROTTLE(1.0,
                    "[VISUAL] angle=%.2f deg area=%.4f/%0.4f front=%.3f/%.3f cmd=(%.3f,%.3f) confirmations=%d/%d.",
                    last_visual_direction_deg_, last_visual_area_ratio_,
                    visual_area_stop_threshold_, last_center_min_distance_,
                    visual_min_front_clearance_, cmd.linear.x, cmd.angular.z,
                    visual_stop_count_, visual_stop_confirmations_);
}

void NavigationDriver::timerCallback(const ros::TimerEvent&)
{
  ScopedTimingLog timing("timerCallback");
  // Planning and full-map BFS can take substantially longer than one
  // exploration tick.  Never make the laser callback wait behind this work;
  // the safety heartbeat/watchdog remains independent and the next timer tick
  // will retry planning if this one is skipped.
  std::unique_lock<std::mutex> lock(data_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
  {
    const std::int64_t arrival_nsec =
        last_scan_arrival_nsec_.load(std::memory_order_acquire);
    const std::int64_t processed_nsec =
        last_scan_received_nsec_.load(std::memory_order_acquire);
    const std::int64_t now_nsec =
        static_cast<std::int64_t>(ros::Time::now().toNSec());
    const double arrival_age = arrival_nsec > 0 && now_nsec >= arrival_nsec
        ? (now_nsec - arrival_nsec) / 1e9 : -1.0;
    const double processed_age = processed_nsec > 0 && now_nsec >= processed_nsec
        ? (now_nsec - processed_nsec) / 1e9 : -1.0;
    ROS_WARN_THROTTLE(2.0,
        "[EVENT][NAV_LOCK_CONTENTION] Planning tick skipped while another "
        "callback holds navigation state lock (scan_arrival_age=%.3f s, "
        "scan_processed_age=%.3f s, safety_atomic=%s).",
        arrival_age, processed_age,
        safety_stop_atomic_.load(std::memory_order_acquire) ? "ON" : "OFF");
    return;
  }
  const ros::Time now = ros::Time::now();
  updateCameraCoverage(now);
  logDiagnosticSnapshot(now);
  // Use the same two-stage freshness check as the independent watchdog.  The
  // planner owns data_mutex_ during bounded map searches, so a short delay in
  // scan analysis is tolerated, but a processing stall cannot run forever.
  const std::int64_t arrival_nsec =
      last_scan_arrival_nsec_.load(std::memory_order_acquire);
  const std::int64_t processed_nsec =
      last_scan_received_nsec_.load(std::memory_order_acquire);
  // Read the clock after both atomics.  The laser callback updates arrival
  // before taking data_mutex_; using the timer's older 'now' can otherwise
  // make a newly arrived scan appear to come from the future.
  const std::int64_t now_nsec = static_cast<std::int64_t>(
      ros::Time::now().toNSec());
  const std::int64_t arrival_timeout_nsec = static_cast<std::int64_t>(
      std::max(0.10, scan_timeout_) * 1e9);
  const std::int64_t processing_grace_nsec = static_cast<std::int64_t>(
      std::max(2.0, 4.0 * std::max(0.10, scan_timeout_)) * 1e9);
  const bool scan_stale = !has_scan_ || arrival_nsec <= 0 ||
      processed_nsec <= 0 ||
      (now_nsec - arrival_nsec) > arrival_timeout_nsec ||
      (now_nsec - processed_nsec) > processing_grace_nsec;
  if (scan_stale)
  {
    const double arrival_age = arrival_nsec > 0 && now_nsec >= arrival_nsec
        ? (now_nsec - arrival_nsec) / 1e9 : -1.0;
    const double processed_age = processed_nsec > 0 && now_nsec >= processed_nsec
        ? (now_nsec - processed_nsec) / 1e9 : -1.0;
    if (!emergency_stopped_)
    {
      emergency_stopped_ = true;
      emergency_stop_time_ = now;
      emergency_clear_candidate_count_ = 0;
      cancelActiveGoal();
      has_last_goal_ = false;
      current_goal_is_object_ = false;
      current_goal_is_bridge_ = false;
      stopEscapeBackoff();
      stopSmartRotation();
      ROS_WARN("[EVENT][SCAN_WATCHDOG] No usable scan for %.2f s (timeout %.2f s); holding zero velocity.",
               processed_age,
               processing_grace_nsec / 1e9);
      ROS_WARN("[STOP_REASON][SCAN_STALE] planner blocked navigation: "
               "has_scan=%s arrival_age=%.3f/%.3f processed_age=%.3f/%.3f "
               "tf=%s valid=%d; goal canceled.",
               has_scan_ ? "true" : "false", arrival_age,
               arrival_timeout_nsec / 1e9, processed_age,
               processing_grace_nsec / 1e9, last_scan_tf_status_.c_str(),
               last_scan_valid_count_);
    }
    geometry_msgs::Twist stop;
    publishCmdVel(stop);
    publishSafetyStop(true);
    return;
  }
  if (task_finished_ || emergency_stopped_)
  {
    ROS_INFO_THROTTLE(5.0,
        "[STOP_REASON][LATCHED_STATE] holding zero: task_finished=%s "
        "emergency=%s safety_atomic=%s.",
        task_finished_ ? "true" : "false", emergency_stopped_ ? "true" : "false",
        safety_stop_atomic_.load(std::memory_order_acquire) ? "true" : "false");
    geometry_msgs::Twist stop;
    publishCmdVel(stop);
    publishSafetyStop(emergency_stopped_);
    return;
  }
  if (is_escape_backing_)
  {
    // The escape timer is the sole owner of navigation_driver/cmd_vel during
    // a guarded reverse maneuver.  Visual and frontier callbacks must not
    // stop it or replace its command until the maneuver has ended.
    return;
  }
  publishSafetyStop(false);
  const bool visual_direction_fresh = enable_visual_servo_ &&
      !last_visual_direction_time_.isZero() &&
      (now - last_visual_direction_time_).toSec() >= 0.0 &&
      (now - last_visual_direction_time_).toSec() <= visual_detection_timeout_;
  const bool visual_area_fresh = enable_visual_servo_ &&
      !last_visual_area_time_.isZero() &&
      (now - last_visual_area_time_).toSec() >= 0.0 &&
      (now - last_visual_area_time_).toSec() <= visual_detection_timeout_;
  const bool visual_pair_synced = visual_direction_fresh && visual_area_fresh &&
      std::fabs((last_visual_direction_time_ - last_visual_area_time_).toSec()) <=
          visual_sync_tolerance_;
  // The 20 Hz visual timer owns cmd_vel while a complete detection pair is
  // fresh.  Do not let the 1 Hz frontier planner send a competing goal.
  if (visual_pair_synced)
  {
    // Recovery timers also publish on navigation_driver/cmd_vel.  They must be
    // stopped before visual control is allowed to own the command stream.
    stopEscapeBackoff();
    stopSmartRotation();
    ROS_INFO_THROTTLE(5.0,
        "[STOP_REASON][VISUAL_PRIORITY] frontier planner suppressed: "
        "direction=%.2f deg area=%.4f pair_age=(%.2f,%.2f) engaged=%s.",
        last_visual_direction_deg_, last_visual_area_ratio_,
        (now - last_visual_direction_time_).toSec(),
        (now - last_visual_area_time_).toSec(),
        visual_control_engaged_ ? "true" : "false");
    return;
  }
  if (!map_received_)
  {
    ROS_INFO_THROTTLE(5.0,
        "[STOP_REASON][MAP_MISSING] scan is usable but no occupancy map has "
        "been received; planner will not issue a goal.");
    return;
  }

  if (is_escape_backing_)
    return;

  if ((now - node_start_time_).toSec() < start_delay_)
  {
    ROS_INFO_THROTTLE(5.0,
        "[STOP_REASON][START_DELAY] waiting %.2f s before planning.",
        std::max(0.0, start_delay_ - (now - node_start_time_).toSec()));
    return;
  }

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
    geometry_msgs::TransformStamped transform;
    if (getFreshRobotTransform(transform))
    {
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
        ROS_INFO("[STOP_REASON][OBJECT_REACHED] distance=%.3f m threshold=%.3f m.",
                 distance_to_object, stop_distance_ + 0.3);
        return;
      }
    }
    else
    {
      ROS_WARN_THROTTLE(5.0, "[TIMER] Robot pose unavailable for object distance check.");
    }
  }

  if (is_smart_rotating_) return;

  if (!has_active_goal && has_last_goal_ && current_state.isDone())
  {
    // Actionlib normally invokes doneCallback(), but a dropped callback or a
    // restart of move_base must not leave the local state machine latched on a
    // terminal goal forever.
    ROS_WARN("[TIMER] Goal is terminal (%s) but completion callback was not applied; recovering state.",
             current_state.toString().c_str());
    ROS_WARN("[STOP_REASON][ACTION_TERMINAL_RECOVERY] state=%s generation=%llu; "
             "reconstructing local goal state.", current_state.toString().c_str(),
             static_cast<unsigned long long>(goal_generation_));
    const bool goal_was_object = current_goal_is_object_;
    const bool goal_was_bridge = current_goal_is_bridge_;
    if (!goal_was_object && current_state != actionlib::SimpleClientGoalState::SUCCEEDED)
      blacklistExplorationGoal(
          last_goal_, goal_was_bridge ? bridge_failure_ttl_
                                      : frontier_failure_ttl_,
          "terminal_without_callback");
    if (current_state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ++fail_count_;
      ++consecutive_failures_;
      last_fail_time_ = now;
    }
    else
    {
      consecutive_failures_ = 0;
      if (!goal_was_object) recordExplorationSuccess(last_goal_);
    }
    if (goal_was_object && current_state == actionlib::SimpleClientGoalState::SUCCEEDED &&
        object_found_)
    {
      task_finished_ = true;
      goal_request_pending_ = false;
      geometry_msgs::Twist stop;
      publishCmdVel(stop);
      ROS_INFO("[TIMER] Object goal reached; completing task through terminal-state recovery.");
    }
    has_last_goal_ = false;
    current_goal_is_object_ = false;
    current_goal_is_bridge_ = false;
    progress_stall_count_ = 0;
    has_last_robot_pose_ = false;
    current_goal_timeout_ = goal_timeout_;
    requestPlanning();
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
      ROS_INFO_THROTTLE(5.0,
          "[STOP_REASON][MAP_UNSTABLE] change_age=%.2f/%.2f s size=%dx%d.",
          last_map_change_time_.isZero() ? -1.0
              : (now - last_map_change_time_).toSec(),
          map_stable_time_, last_map_width_, last_map_height_);
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
  if (goal_age > current_goal_timeout_)
  {
    ROS_WARN("[TIMER] Goal timed out after %.1f s (limit %.1f s); canceling and backing off.",
             goal_age, current_goal_timeout_);
    ROS_WARN("[STOP_REASON][GOAL_TIMEOUT] type=%s age=%.2f/%.2f goal=(%.2f,%.2f) "
             "action=%s; canceling and requesting a new plan.",
             current_goal_is_object_ ? "object" :
                 (current_goal_is_bridge_ ? "bridge" : "frontier"), goal_age,
             current_goal_timeout_, last_goal_.pose.position.x,
             last_goal_.pose.position.y, current_state.toString().c_str());
    if (!current_goal_is_object_)
      blacklistExplorationGoal(
          last_goal_, current_goal_is_bridge_ ? bridge_failure_ttl_
                                              : frontier_failure_ttl_,
          "goal_timeout");
    cancelActiveGoal();
    has_last_goal_ = false;
    current_goal_is_object_ = false;
    current_goal_is_bridge_ = false;
    last_fail_time_ = now;
    current_goal_timeout_ = goal_timeout_;
    ++consecutive_failures_;
    requestPlanning();
    return;
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
  geometry_msgs::TransformStamped transform;
  if (getFreshRobotTransform(transform))
  {
    current_pose.position.x = transform.transform.translation.x;
    current_pose.position.y = transform.transform.translation.y;
    current_pose.orientation = transform.transform.rotation;
  }
  else
  {
    ROS_WARN_THROTTLE(5.0, "[TIMER] Progress check TF unavailable or stale.");
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
      ROS_WARN("[STOP_REASON][PROGRESS_STALL] goal_type=%s moved_below=%.3f m "
               "yaw_below=%.3f rad checks=%d/%d action=%s; canceling goal.",
               current_goal_is_object_ ? "object" :
                   (current_goal_is_bridge_ ? "bridge" : "frontier"),
               min_progress_dist_, min_progress_yaw_, progress_stall_count_,
               progress_confirmations_, current_state.toString().c_str());
      if (!current_goal_is_object_)
        blacklistExplorationGoal(last_goal_, stall_failure_ttl_,
                                 current_goal_is_bridge_
                                     ? "bridge_stall" : "frontier_stall");
      cancelActiveGoal();
      geometry_msgs::Twist stop;
      publishCmdVel(stop);
      has_last_goal_ = false;
      current_goal_is_object_ = false;
      current_goal_is_bridge_ = false;
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
  ROS_INFO("[GOAL_EVENT][DONE] generation=%llu current_generation=%llu state=%s "
           "has_goal=%s type=%s emergency=%s task_finished=%s.",
           static_cast<unsigned long long>(goal_generation),
           static_cast<unsigned long long>(goal_generation_),
           state.toString().c_str(), has_last_goal_ ? "true" : "false",
           has_last_goal_ ? (current_goal_is_object_ ? "object" :
                             (current_goal_is_bridge_ ? "bridge" : "frontier")) : "none",
           emergency_stopped_ ? "true" : "false", task_finished_ ? "true" : "false");
  if (goal_generation != goal_generation_ || !has_last_goal_ ||
      task_finished_ || emergency_stopped_)
  {
    ROS_DEBUG("[DONE] Ignoring terminal callback for stale goal generation %llu (current %llu).",
              static_cast<unsigned long long>(goal_generation),
              static_cast<unsigned long long>(goal_generation_));
    return;
  }

  const bool goal_was_object = current_goal_is_object_;
  const bool goal_was_bridge = current_goal_is_bridge_;
  if (state == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    ROS_INFO("[STOP_REASON][GOAL_PREEMPTED] generation=%llu; clearing local goal and requesting planning.",
             static_cast<unsigned long long>(goal_generation));
    has_last_goal_ = false;
    current_goal_is_object_ = false;
    current_goal_is_bridge_ = false;
    progress_stall_count_ = 0;
    has_last_robot_pose_ = false;
    current_goal_timeout_ = goal_timeout_;
    requestPlanning();
    return;
  }

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    if (!goal_was_object) recordExplorationSuccess(last_goal_);
    has_last_goal_ = false;
    current_goal_is_object_ = false;
    current_goal_is_bridge_ = false;
    consecutive_failures_ = 0;
    has_last_robot_pose_ = false;
    progress_stall_count_ = 0;
    current_goal_timeout_ = goal_timeout_;

    if (goal_was_object && object_found_)
    {
      task_finished_ = true;
      ROS_INFO("[DONE] TASK COMPLETED! Robot arrived at object.");
      ac_.cancelAllGoals();
      geometry_msgs::Twist stop;
      publishCmdVel(stop);
      ROS_INFO("[STOP_REASON][OBJECT_REACHED] actionlib reported SUCCEEDED for object goal.");
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
    blacklistExplorationGoal(
        last_goal_, goal_was_bridge ? bridge_failure_ttl_
                                    : frontier_failure_ttl_,
        goal_was_bridge ? "bridge_action_failed"
                        : "frontier_action_failed");

  ++fail_count_;
  ++consecutive_failures_;
  last_fail_time_ = ros::Time::now();
  has_last_goal_ = false;
  current_goal_is_object_ = false;
  current_goal_is_bridge_ = false;
  has_last_robot_pose_ = false;
  current_goal_timeout_ = goal_timeout_;
  progress_stall_count_ = 0;
  ROS_WARN("[DONE] Goal failed (%s). Fail: %d, consecutive: %d.",
           state.toString().c_str(), fail_count_, consecutive_failures_);
  ROS_WARN("[STOP_REASON][ACTION_FAILED] state=%s type=%s fail=%d consecutive=%d/%d "
           "goal=(%.2f,%.2f); local goal cleared.", state.toString().c_str(),
           goal_was_object ? "object" : (goal_was_bridge ? "bridge" : "frontier"), fail_count_,
           consecutive_failures_, max_consecutive_failures_,
           last_goal_.pose.position.x, last_goal_.pose.position.y);

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

void NavigationDriver::sendNextGoal()
{
  ScopedTimingLog timing("sendNextGoal");
  if (task_finished_ || emergency_stopped_) return;
  if (!map_received_) { ROS_INFO_THROTTLE(5.0, "[SEND] No map yet."); return; }

  const ros::Time now = ros::Time::now();
  pruneFailedGoals(now);
  updateCameraCoverage(now);
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
  bool goal_is_bridge = false;

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
      bool found = findFrontierCandidates(candidates, max_search_range_, failed_goals_);
      if (!found || candidates.empty())
        found = findFrontierCandidates(
            candidates, extended_search_range_, failed_goals_);

      // A previously viewed corridor may still be the only route to an
      // unseen branch.  This is the sole normal-planner exception to the
      // no-repeat rule: the endpoint must have a novel camera direction.
      if (!found || candidates.empty())
      {
        geometry_msgs::PoseStamped backtrack_goal;
        if (findBacktrackGoal(backtrack_goal))
        {
          new_goal = backtrack_goal;
          found = true;
          goal_is_bridge = true;
          candidates.clear();
          candidates.push_back({new_goal, -1.0});
          ROS_INFO("[SEND] Reusing searched corridor only as a bridge to an unsearched branch: (%.2f, %.2f).",
                   new_goal.pose.position.x, new_goal.pose.position.y);
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
        ROS_WARN_THROTTLE(5.0,
            "[STOP_REASON][NO_FRONTIER] no connected candidate: rotations=%d/%d "
            "retry_interval=%.2f s blacklist=%zu map=%dx%d.",
            no_frontier_rotations_, max_no_frontier_rotations_,
            no_frontier_retry_interval_, failed_goals_.size(),
            last_map_width_, last_map_height_);
        return;
      }
      if (!goal_is_bridge)
        new_goal = candidates.front().first;
    }
  }

  // A fixed timeout makes long exploration goals fail deterministically: at
  // 0.20 m/s an 8 m goal already needs about forty seconds before turning or
  // obstacle avoidance.  Scale the watchdog from the current straight-line
  // distance and leave margin for TEB's turns and recovery pauses.
  current_goal_timeout_ = goal_timeout_;
  geometry_msgs::TransformStamped robot_transform;
  if (getFreshRobotTransform(robot_transform))
  {
    const double goal_distance = distance(
        robot_transform.transform.translation.x,
        robot_transform.transform.translation.y,
        new_goal.pose.position.x, new_goal.pose.position.y);
    const double nominal_speed = std::max(
        0.10, std::min(max_autonomous_speed_, current_max_vel_x_));
    const double distance_timeout = goal_distance / nominal_speed * 2.5 + 15.0;
    current_goal_timeout_ = std::min(300.0,
                                     std::max(goal_timeout_, distance_timeout));
    ROS_INFO("[SEND] Goal watchdog %.1f s for straight distance %.2f m at nominal %.2f m/s.",
             current_goal_timeout_, goal_distance, nominal_speed);
  }

  last_goal_ = new_goal;
  no_frontier_rotations_ = 0;
  last_no_frontier_time_ = ros::Time(0);
  has_last_goal_ = true;
  current_goal_is_object_ = goal_is_object;
  current_goal_is_bridge_ = goal_is_bridge;
  goal_request_pending_ = false;
  last_goal_sent_time_ = now;
  last_goal_switch_time_ = now;
  progress_stall_count_ = 0;
  has_last_robot_pose_ = false;

  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose = new_goal;
  const std::uint64_t goal_generation = ++goal_generation_;
  ac_.sendGoal(mb_goal, boost::bind(&NavigationDriver::doneCallback, this,
                                    _1, _2, goal_generation));

  ROS_INFO("[SEND] %s goal: (%.2f, %.2f, yaw=%.1f deg).",
           goal_is_object ? "Object" : (goal_is_bridge ? "Bridge" : "Frontier"),
           new_goal.pose.position.x, new_goal.pose.position.y,
           tf2::getYaw(new_goal.pose.orientation) * 180.0 / M_PI);
  ROS_INFO("[GOAL_EVENT][SENT] generation=%llu type=%s distance_timeout=%.2f s "
           "map_stable=%s max_vel=%.3f pending=%s.",
           static_cast<unsigned long long>(goal_generation),
           goal_is_object ? "object" : (goal_is_bridge ? "bridge" : "frontier"),
           current_goal_timeout_,
           isMapStable() ? "true" : "false", current_max_vel_x_,
           goal_request_pending_ ? "true" : "false");
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

NavigationDriver::FrontierEvidence NavigationDriver::evaluateFrontierEvidence(
    int cx, int cy, int radius_cells) const
{
  FrontierEvidence evidence{false, false, 0, 0.0, 0.0};
  if (!map_received_ || radius_cells <= 0) return evidence;

  const int width = static_cast<int>(current_map_.info.width);
  const int height = static_cast<int>(current_map_.info.height);
  const int ray_count = 24;
  for (int ray_index = 0; ray_index < ray_count; ++ray_index)
  {
    const double angle = -M_PI +
        (ray_index + 0.5) * (2.0 * M_PI / ray_count);
    const double ray_x = std::cos(angle);
    const double ray_y = std::sin(angle);
    int previous_x = cx;
    int previous_y = cy;
    for (int step = 1; step <= radius_cells; ++step)
    {
      const int mx = cx + static_cast<int>(std::lround(step * ray_x));
      const int my = cy + static_cast<int>(std::lround(step * ray_y));
      if (mx == previous_x && my == previous_y) continue;
      previous_x = mx;
      previous_y = my;

      if (mx < 0 || my < 0 || mx >= width || my >= height)
      {
        evidence.has_map_edge = true;
        ++evidence.visible_rays;
        evidence.direction_x += ray_x;
        evidence.direction_y += ray_y;
        break;
      }

      const size_t index = static_cast<size_t>(my) * width + mx;
      if (index >= current_map_.data.size()) break;
      const int value = current_map_.data[index];
      if (value == -1)
      {
        evidence.has_internal_unknown = true;
        ++evidence.visible_rays;
        evidence.direction_x += ray_x;
        evidence.direction_y += ray_y;
        break;
      }
      // Unknown information behind a gray or occupied cell is occluded.  The
      // candidate remains on the eroded known-free mask and never crosses it.
      if (value > free_cell_threshold_) break;
    }
  }
  return evidence;
}

bool NavigationDriver::findFrontierCandidates(
    std::vector<std::pair<geometry_msgs::PoseStamped, double>>& candidates,
    double search_range,
    const std::vector<FailedGoalRecord>& blacklist)
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
  if (!buildReachableMask(reachable, robot_mx, robot_my, search_range)) return false;

  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_yaw = 0.0;
  geometry_msgs::TransformStamped transform;
  if (!getFreshRobotTransform(transform)) return false;
  robot_x = transform.transform.translation.x;
  robot_y = transform.transform.translation.y;
  tf2::Quaternion q;
  tf2::fromMsg(transform.transform.rotation, q);
  robot_yaw = tf2::getYaw(q);

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
  size_t internal_frontier_cells = 0;
  size_t map_edge_frontier_cells = 0;
  size_t blacklist_rejections = 0;
  size_t distance_rejections = 0;
  size_t cells_visited = 0;
  bool timed_out = false;
  const int information_radius_cells = std::max(
      1, static_cast<int>(std::ceil(
          frontier_unknown_search_radius_ / resolution)));
  const int candidate_stride = std::max(
      1, static_cast<int>(std::lround(0.10 / resolution)));

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
      if ((x - min_x) % candidate_stride != 0 ||
          (y - min_y) % candidate_stride != 0)
        continue;

      const FrontierEvidence evidence = evaluateFrontierEvidence(
          x, y, information_radius_cells);
      if (evidence.visible_rays == 0) continue;
      ++adjacent_unknown_cells;
      if (evidence.has_internal_unknown) ++internal_frontier_cells;
      if (evidence.has_map_edge) ++map_edge_frontier_cells;
      const double world_x = origin_x + (x + 0.5) * resolution;
      const double world_y = origin_y + (y + 0.5) * resolution;
      bool blacklisted_nearby = false;
      for (const auto& failed : blacklist)
      {
        if (distance(world_x, world_y, failed.x, failed.y) <
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
      const int boundary_gain = 4 * evidence.visible_rays;
      const int information_gain = countUnknownNeighbors(x, y, 4) + boundary_gain;
      const int branch_unknown = countUnknownNeighbors(x, y, 6) + boundary_gain;
      const int opening_gain = std::max(0, branch_unknown - information_gain);
      const double lateral_factor = std::sin(
          std::min(heading_difference, M_PI / 2.0));
      const double side_opening_gain = lateral_factor * opening_gain;
      const double candidate_yaw =
          std::hypot(evidence.direction_x, evidence.direction_y) > 0.1
              ? std::atan2(evidence.direction_y, evidence.direction_x)
              : heading;
      const double novelty = cameraViewNovelty(world_x, world_y, candidate_yaw);
      if (!camera_view_history_.empty() && novelty < min_unsearched_novelty_)
        continue;
      // A covered camera view is not a valid normal exploration target.  It
      // may still be crossed by move_base on the way to a novel frontier, but
      // the endpoint itself must provide a new observation whenever one is
      // available.
      // Normalize heterogeneous terms before combining them.  This keeps a
      // 0.5 m distance advantage from overwhelming the information/branch
      // gain of a large elevator lobby.
      const double distance_norm = std::max(0.0, std::min(1.0,
          goal_distance / std::max(0.1, search_range)));
      const double heading_norm = heading_difference / M_PI;
      const double information_norm = std::max(0.0, std::min(1.0,
          static_cast<double>(information_gain) / 80.0));
      const double spatial_norm = std::max(0.0, std::min(1.0,
          static_cast<double>(branch_unknown) / 168.0));
      const double branch_norm = std::max(0.0, std::min(1.0,
          side_opening_gain / 88.0));
      const double boundary_norm = std::max(0.0, std::min(1.0,
          static_cast<double>(evidence.visible_rays) / 16.0));
      const double corridor_heading = has_exploration_heading_
          ? exploration_heading_ : robot_yaw;
      const double corridor_heading_difference = std::fabs(std::atan2(
          std::sin(heading - corridor_heading),
          std::cos(heading - corridor_heading)));
      const double utility =
          1.8 * information_norm + 1.2 * spatial_norm +
          0.8 * branch_norm + 1.0 * boundary_norm + 0.8 * novelty -
          0.6 * distance_norm - 0.3 * heading_norm -
          0.35 * corridor_heading_weight_ * corridor_heading_difference / M_PI;
      const double cost = -utility;

      geometry_msgs::PoseStamped candidate;
      candidate.header.frame_id = "map";
      // Frontier poses are map-frame commands; zero selects the latest TF
      // when move_base transforms the goal into its costmap frame.
      candidate.header.stamp = ros::Time(0);
      candidate.pose.position.x = world_x;
      candidate.pose.position.y = world_y;
      candidate.pose.orientation = tf2::toMsg(
          tf2::Quaternion(tf2::Vector3(0, 0, 1), candidate_yaw));
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

  // A partial scan is not a valid exploration decision.  Returning its
  // candidates makes the row/column iteration order choose the goal and can
  // repeatedly bias the robot toward one side of a growing map.
  if (timed_out)
  {
    candidates.clear();
    ROS_WARN_THROTTLE(5.0,
        "[FRONTIER] Search deadline reached after %zu cells; discarding partial candidates.",
        cells_visited);
    return false;
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

  preferUnsearchedCandidates(candidates);

  const double elapsed = (ros::WallTime::now() - start_time).toSec();
  ROS_INFO_THROTTLE(2.0,
      "[FRONTIER] Search %.3f s%s: reachable=%zu uncertain=%zu adjacent=%zu "
      "internal=%zu map_edge=%zu "
      "reject(blacklist=%zu distance=%zu) raw=%zu clustered=%zu.",
      elapsed, timed_out ? " (deadline)" : "", reachable_cells_checked,
      uncertain_cells, adjacent_unknown_cells, internal_frontier_cells,
      map_edge_frontier_cells,
      blacklist_rejections, distance_rejections,
      raw_candidates.size(), candidates.size());

  return !candidates.empty();
}

bool NavigationDriver::computeFinalGoal(const geometry_msgs::Pose& object_map,
                                        geometry_msgs::PoseStamped& goal)
{
  geometry_msgs::TransformStamped tf;
  if (!getFreshRobotTransform(tf)) return false;

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
  if (is_escape_backing_) return;
  if (is_smart_rotating_) stopSmartRotation();
  cancelActiveGoal();
  has_last_goal_ = false;
  current_goal_is_object_ = false;
  current_goal_is_bridge_ = false;
  clearCostmaps();
  ROS_WARN("[RECOVERY_EVENT][PLANNER_ESCAPE] Consecutive goal failures reached the limit; "
           "cleared costmaps and starting deterministic bounded recovery.");
  startEscapeBackoff();
}

void NavigationDriver::startEscapeBackoff()
{
  if (is_escape_backing_) return;
  if (!has_scan_ ||
      (last_scan_tf_status_ != "ok" && last_scan_tf_status_ != "latest"))
  {
    ROS_WARN_THROTTLE(5.0, "[ESCAPE] Reverse recovery unavailable without a valid scan transform.");
    if (!emergency_stopped_)
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
    ROS_WARN_THROTTLE(5.0,
        "[STOP_REASON][REVERSE_BLOCKED] rear_clearance=%.3f threshold=%.3f "
        "valid=%d emergency=%s; reverse command not issued.",
        rear_valid > 0 ? rear_clearance : -1.0, emergency_stop_dist_, rear_valid,
        emergency_stopped_ ? "true" : "false");
    // A latched emergency stop must not be bypassed by a rotation command.
    // Normal planner-recovery calls (which are not emergency-latched) may
    // still use the bounded all-around-clearance rotation fallback.
    if (!emergency_stopped_)
      startSmartRotation();
    return;
  }

  cancelActiveGoal();
  has_last_goal_ = false;
  current_goal_is_object_ = false;
  current_goal_is_bridge_ = false;
  is_escape_backing_ = true;
  visual_control_engaged_ = false;
  const bool emergency_escape = emergency_stopped_;
  escape_backoff_start_ = ros::Time::now();
  // The rear sector has just passed the clearance check.  Permit only the
  // dedicated reverse command while retaining the laser callback's rear
  // monitoring; a new front command cannot pass the arbiter during this mode.
  emergency_stopped_ = false;
  publishSafetyStop(false);
  ROS_WARN("[ESCAPE] Reversing at 0.10 m/s for up to %.1f s (rear clearance %.2f m).",
           escape_backoff_duration_, rear_clearance);
  ROS_WARN("[RECOVERY_EVENT][ESCAPE_START] emergency=%s rear_clearance=%.3f "
           "duration=%.2f attempt=%d/%d.", emergency_escape ? "true" : "false",
           rear_clearance, escape_backoff_duration_, emergency_escape_attempt_count_,
           emergency_escape_max_attempts_);

  escape_timer_ = nh_.createTimer(ros::Duration(0.05), [this, emergency_escape](const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!is_escape_backing_) return;
    if (emergency_stopped_ || !has_scan_ ||
        (last_scan_tf_status_ != "ok" && last_scan_tf_status_ != "latest"))
    {
      stopEscapeBackoff();
      // Losing the scan while reversing is a safety fault.  Keep the vehicle
      // stopped and let the scan watchdog/clear-confirmation path decide when
      // it is safe to resume; do not immediately request a new goal.
      emergency_stopped_ = true;
      emergency_stop_time_ = ros::Time::now();
      emergency_front_observation_until_ =
          emergency_stop_time_ + ros::Duration(emergency_front_observation_duration_);
      emergency_clear_candidate_count_ = 0;
      publishSafetyStop(true);
      ROS_WARN("[ESCAPE] Reverse recovery interrupted by unusable scan; holding safety stop.");
      ROS_WARN("[STOP_REASON][ESCAPE_SCAN_LOST] reverse canceled because scan became unusable.");
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
      if (!emergency_escape)
      {
        requestPlanning();
        ROS_INFO("[ESCAPE] Planner recovery reverse completed; requesting a fresh plan.");
        return;
      }
      emergency_stopped_ = true;
      emergency_stop_time_ = ros::Time::now();
      emergency_front_observation_until_ =
          emergency_stop_time_ + ros::Duration(emergency_front_observation_duration_);
      emergency_clear_candidate_count_ = 0;
      publishSafetyStop(true);
      ROS_WARN("[ESCAPE] Reverse recovery ended (rear_clearance=%.2f m, valid=%d); observing front for %.1f s, attempt %d/%d.",
               rear_valid > 0 ? rear_clearance : -1.0, rear_valid,
               emergency_front_observation_duration_,
               emergency_escape_attempt_count_, emergency_escape_max_attempts_);
      ROS_WARN("[RECOVERY_EVENT][ESCAPE_END] emergency stop latched after reverse; "
               "rear_clearance=%.3f valid=%d.",
               rear_valid > 0 ? rear_clearance : -1.0, rear_valid);
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
  ROS_INFO("[RECOVERY_EVENT][ESCAPE_STOP] reverse recovery command released.");
}

void NavigationDriver::startSmartRotation()
{
  if (is_smart_rotating_) return;

  geometry_msgs::TransformStamped rotation_tf;
  if (getFreshRobotTransform(rotation_tf))
  {
    const double rotation_x = rotation_tf.transform.translation.x;
    const double rotation_y = rotation_tf.transform.translation.y;
    if (rotationObservedNear(rotation_x, rotation_y))
    {
      ROS_WARN_THROTTLE(5.0,
          "[SMART_ROTATE] This corridor node was already observed; suppressing repeated rotation.");
      ROS_WARN_THROTTLE(5.0,
          "[STOP_REASON][ROTATION_SUPPRESSED] node already observed; waiting for map growth or recovery.");
      requestPlanning();
      return;
    }
  }

  if (!hasRotationClearance())
  {
    ROS_WARN_THROTTLE(5.0,
        "[SMART_ROTATE] Insufficient all-around clearance (need %.2f m); staying stopped.",
        rotation_clearance_);
    ROS_WARN_THROTTLE(5.0,
        "[STOP_REASON][ROTATION_BLOCKED] all-around clearance below %.3f m; "
        "no rotation command issued.", rotation_clearance_);
    requestPlanning();
    return;
  }

  if (getFreshRobotTransform(rotation_tf))
  {
    rotation_observation_history_.push_back({
        rotation_tf.transform.translation.x,
        rotation_tf.transform.translation.y});
    if (rotation_observation_history_.size() > 200)
      rotation_observation_history_.erase(rotation_observation_history_.begin());
  }

  const actionlib::SimpleClientGoalState rotation_action_state = ac_.getState();
  const bool rotation_goal_active =
      rotation_action_state == actionlib::SimpleClientGoalState::ACTIVE ||
      rotation_action_state == actionlib::SimpleClientGoalState::PENDING;
  if (rotation_goal_active)
    ac_.cancelGoal();
  has_last_goal_ = false;
  current_goal_is_object_ = false;
  current_goal_is_bridge_ = false;

  is_smart_rotating_ = true;
  accumulated_angle_ = 0.0;
  rotate_start_time_ = ros::Time::now();

  ROS_WARN("[SMART_ROTATE] Starting smart rotation (max %.1f°).",
           max_rotate_angle_ * 180.0 / M_PI);
  ROS_WARN("[RECOVERY_EVENT][ROTATION_START] speed=%.3f rad/s max_angle=%.1f deg.",
           rotate_speed_, max_rotate_angle_ * 180.0 / M_PI);

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

    const ros::Time now = ros::Time::now();
    double dt = (now - rotate_start_time_).toSec();
    rotate_start_time_ = now;

    // The normal planning timer exits while smart rotation owns cmd_vel, so
    // record camera poses here as the vehicle sweeps through new headings.
    // Otherwise a real 180-degree observation would be absent from coverage
    // memory and could be selected again as an apparently novel direction.
    updateCameraCoverage(now);

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
  ROS_INFO("[RECOVERY_EVENT][ROTATION_STOP] accumulated_angle=%.1f deg.",
           accumulated_angle_ * 180.0 / M_PI);
}

double NavigationDriver::distance(double x1, double y1, double x2, double y2) const
{
  return hypot(x1-x2, y1-y2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_driver");
  ros::NodeHandle nh("~");
  NavigationDriver driver(nh);
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
