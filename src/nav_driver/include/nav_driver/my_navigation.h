#ifndef MY_NAVIGATION_H
#define MY_NAVIGATION_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <array>
#include <xmlrpcpp/XmlRpcValue.h>
#include <cstdint>

class NavigationDriver
{
public:
  NavigationDriver(ros::NodeHandle& nh);
  ~NavigationDriver();

private:
  static const std::string VERSION;

  struct FrontierEvidence
  {
    bool has_internal_unknown;
    bool has_map_edge;
    int visible_rays;
    double direction_x;
    double direction_y;
  };

  struct FailedGoalRecord
  {
    double x;
    double y;
    ros::Time expires_at;
    std::string reason;
  };

  // 回调
  void objectCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void objectDirectionCallback(const std_msgs::Float32ConstPtr& msg);
  void objectAreaCallback(const std_msgs::Float32ConstPtr& msg);
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void globalFootprintCallback(const geometry_msgs::PolygonStampedConstPtr& msg);
  void localFootprintCallback(const geometry_msgs::PolygonStampedConstPtr& msg);
  void timerCallback(const ros::TimerEvent& event);
  void visualServoTimerCallback(const ros::TimerEvent& event);
  void safetyHeartbeatCallback(const ros::TimerEvent& event);
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const move_base_msgs::MoveBaseResultConstPtr& result,
                    std::uint64_t goal_generation);

  // 核心功能
  bool computeFinalGoal(const geometry_msgs::Pose& object_map,
                        geometry_msgs::PoseStamped& goal);
  bool findFrontierCandidates(
      std::vector<std::pair<geometry_msgs::PoseStamped, double>>& candidates,
      double search_range,
      const std::vector<FailedGoalRecord>& blacklist);
  double distance(double x1, double y1, double x2, double y2) const;
  int    countUnknownNeighbors(int cx, int cy, int radius);
  void   sendNextGoal();
  void   executeEscape();
  void   startEscapeBackoff();
  void   stopEscapeBackoff();
  void   startSmartRotation();
  void   stopSmartRotation();
  void   setMaxVelocity(double vel);
  void   clearCostmaps();
  void   cancelActiveGoal();
  void   publishCmdVel(const geometry_msgs::Twist& cmd);
  void   publishSafetyStop(bool stopped);
  void   logDiagnosticSnapshot(const ros::Time& now);
  void   requestPlanning();
  void   blacklistExplorationGoal(const geometry_msgs::PoseStamped& goal,
                                  double ttl, const std::string& reason);
  void   pruneFailedGoals(const ros::Time& now);
  void   recordExplorationSuccess(const geometry_msgs::PoseStamped& goal);

  // 占据栅格上的确定性可达性与 Frontier 辅助函数
  bool   worldToMap(double wx, double wy, int& mx, int& my) const;
  bool   isKnownFreeCell(int mx, int my) const;
  bool   hasOccupiedNeighbor(int mx, int my, int radius) const;
  bool   buildReachableMask(std::vector<unsigned char>& reachable,
                            int& robot_mx, int& robot_my,
                            double search_range) const;
  bool   snapGoalToReachableFreeCell(geometry_msgs::PoseStamped& goal,
                                     double tolerance) const;
  bool   hasRotationClearance() const;
  bool   getFreshRobotTransform(geometry_msgs::TransformStamped& transform) const;
  int    footprintClearanceCells() const;
  FrontierEvidence evaluateFrontierEvidence(int cx, int cy,
                                            int radius_cells) const;
  void   updateCameraCoverage(const ros::Time& now);
  double cameraViewNovelty(double x, double y, double yaw) const;
  void   preferUnsearchedCandidates(
      std::vector<std::pair<geometry_msgs::PoseStamped, double>>& candidates) const;
  bool   findBacktrackGoal(geometry_msgs::PoseStamped& goal);
  bool   rotationObservedNear(double x, double y) const;

  // 地图稳定性检测
  bool   isMapStable();

  // ROS 接口
  ros::NodeHandle nh_;
  ros::Subscriber object_sub_;
  ros::Subscriber object_direction_sub_;
  ros::Subscriber object_area_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber odom_sub_;
  std::string odom_topic_;
  ros::Subscriber global_footprint_sub_;
  ros::Subscriber local_footprint_sub_;
  ros::Timer      explore_timer_;
  ros::Timer      visual_servo_timer_;
  ros::Timer      safety_heartbeat_timer_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  ros::ServiceClient clear_costmap_client_;

  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 地图与状态
  nav_msgs::OccupancyGrid current_map_;
  bool map_received_;
  bool object_found_;
  bool task_finished_;
  geometry_msgs::Pose object_pose_map_;
  ros::Time last_object_time_;
  geometry_msgs::Pose last_known_object_pose_;
  bool      has_last_known_object_pose_;

  // 地图稳定性
  int  last_map_width_;
  int  last_map_height_;
  ros::Time last_map_change_time_;

  // 参数
  double camera_offset_x_;
  std::string cmd_vel_topic_;
  std::string object_topic_;
  std::string object_direction_topic_;
  std::string object_area_topic_;
  bool   enable_visual_servo_;
  double visual_area_stop_threshold_;
  int    visual_stop_confirmations_;
  double visual_detection_timeout_;
  double visual_angle_gain_;
  double visual_angle_sign_;
  double visual_max_linear_speed_;
  double visual_max_angular_speed_;
  double visual_align_angle_deg_;
  double visual_sync_tolerance_;
  double visual_min_front_clearance_;
  double last_visual_direction_deg_;
  double last_visual_area_ratio_;
  ros::Time last_visual_direction_time_;
  ros::Time last_visual_area_time_;
  int    visual_stop_count_;
  bool   visual_control_engaged_;
  bool   visual_target_reached_;
  double stop_distance_;
  double object_timeout_;
  double exploration_frequency_;
  double frontier_min_dist_;
  double fov_horizontal_;
  double fov_range_;
  double max_search_range_;
  double extended_search_range_;
  double goal_timeout_;
  double current_goal_timeout_;
  double fail_backoff_time_;
  double emergency_stop_dist_;
  double emergency_stop_hard_dist_;
  double emergency_clear_margin_;
  double emergency_range_floor_margin_;
  int    emergency_min_near_points_;
  int    emergency_min_cluster_points_;
  double scan_timeout_;
  double scan_tf_max_age_;
  int    scan_min_valid_count_;
  int    emergency_stop_confirm_scans_;
  int    emergency_hard_confirm_scans_;
  int    emergency_clear_confirm_scans_;
  double emergency_escape_delay_;
  bool   emergency_escape_attempted_;
  int    emergency_escape_attempt_count_;
  int    emergency_escape_max_attempts_;
  double emergency_front_observation_duration_;
  double start_delay_;
  double min_goal_switch_interval_;
  double vel_update_interval_;
  double vel_change_threshold_;
  double progress_check_interval_;
  double min_progress_dist_;
  double min_progress_yaw_;
  double progress_grace_period_;
  int    progress_confirmations_;
  int    progress_stall_count_;
  double corridor_heading_weight_;
  // 稳定性参数
  double map_stable_time_;
  int    max_consecutive_failures_;
  double planning_retry_interval_;
  double frontier_clearance_;
  double frontier_unknown_search_radius_;
  double frontier_cluster_radius_;
  double frontier_search_timeout_;
  int    frontier_max_raw_candidates_;
  double camera_coverage_radius_;
  double camera_coverage_angle_deg_;
  double camera_record_distance_;
  double camera_record_yaw_deg_;
  double min_unsearched_novelty_;
  double rotation_clearance_;
  double escape_backoff_duration_;
  double max_autonomous_speed_;
  double no_frontier_retry_interval_;
  int    max_no_frontier_rotations_;
  int    free_cell_threshold_;
  int    occupied_cell_threshold_;
  double frontier_failure_ttl_;
  double bridge_failure_ttl_;
  double stall_failure_ttl_;

  // 目标状态
  geometry_msgs::PoseStamped last_goal_;
  bool has_last_goal_;
  int  fail_count_;
  ros::Time last_fail_time_;
  ros::Time last_goal_sent_time_;
  int  consecutive_failures_;
  std::vector<FailedGoalRecord> failed_goals_;

  // 限频与速度
  ros::Time last_goal_switch_time_;
  ros::Time last_vel_update_time_;
  double last_vel_set_;
  ros::Time last_progress_check_time_;
  geometry_msgs::Pose last_robot_pose_;
  bool has_last_robot_pose_;
  bool goal_request_pending_;
  bool current_goal_is_object_;
  bool current_goal_is_bridge_;
  ros::Time last_planning_attempt_time_;
  int no_frontier_rotations_;
  ros::Time last_no_frontier_time_;
  std::uint64_t goal_generation_;
  // Successful exploration endpoints anchor corridor-direction continuity.
  std::vector<std::pair<double, double>> exploration_visit_history_;
  bool has_exploration_heading_;
  double exploration_heading_;

  struct CameraViewRecord
  {
    double x;
    double y;
    double yaw;
    ros::Time stamp;
  };
  std::vector<CameraViewRecord> camera_view_history_;
  geometry_msgs::Pose last_camera_view_pose_;
  bool has_last_camera_view_pose_;
  std::vector<std::pair<double, double>> rotation_observation_history_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher safety_stop_pub_;
  std::string    safety_stop_topic_;
  ros::Time      last_safety_publish_time_;
  std::mutex     safety_publish_mutex_;
  std::mutex     cmd_publish_mutex_;
  std::mutex     data_mutex_;
  std::atomic<bool> safety_stop_atomic_;
  // Arrival time is updated before the guarded scan analysis.  The planner
  // callback may briefly hold data_mutex_; that must not look like a sensor
  // outage to the independent watchdog when scans are still arriving.
  std::atomic<std::int64_t> last_scan_arrival_nsec_;
  // ROS-time nanosecond stamp of the last scan that completed the guarded
  // callback.  The safety heartbeat reads this without taking data_mutex_.
  std::atomic<std::int64_t> last_scan_received_nsec_;

  // 紧急停止
  bool emergency_stopped_;
  ros::Time emergency_stop_time_;
  double emergency_stop_duration_;
  int emergency_stop_candidate_count_;
  int emergency_clear_candidate_count_;
  ros::Time emergency_front_observation_until_;

  // 智能旋转
  bool is_smart_rotating_;
  ros::Time rotate_start_time_;
  double rotate_speed_;
  double accumulated_angle_;
  double max_rotate_angle_;
  ros::Timer rotate_timer_;
  ros::Timer escape_timer_;
  bool is_escape_backing_;
  ros::Time escape_backoff_start_;
  sensor_msgs::LaserScan latest_scan_;
  bool has_scan_;

  double current_max_vel_x_;
  ros::Time node_start_time_;

  // Runtime diagnostics copied into the launch console for post-run analysis.
  double diagnostic_log_period_;
  ros::Time last_diagnostic_log_time_;
  ros::Time last_scan_received_time_;
  ros::Time last_scan_stamp_;
  ros::Time last_map_stamp_;
  std::string last_scan_frame_id_;
  std::string last_map_frame_id_;
  std::string last_scan_tf_status_;
  double last_scan_tf_x_;
  double last_scan_tf_y_;
  double last_scan_tf_yaw_;
  double last_scan_tf_age_;
  double last_scan_processing_duration_;
  double last_scan_angle_min_;
  double last_scan_angle_max_;
  double last_scan_angle_increment_;
  double last_scan_range_min_;
  double last_scan_range_max_;
  size_t last_scan_total_count_;
  double last_scan_min_distance_;
  double last_scan_min_laser_angle_;
  double last_scan_min_base_angle_;
  double last_scan_min_angle_;
  double last_center_min_distance_;
  int last_center_valid_count_;
  int last_center_near_count_;
  int last_center_floor_count_;
  int last_center_cluster_;
  int last_scan_valid_count_;
  int last_scan_invalid_count_;
  std::array<double, 7> last_front_sector_min_distance_;
  geometry_msgs::Twist last_cmd_vel_;
  ros::Time last_cmd_vel_time_;
  ros::Time last_odom_received_time_;
  ros::Time last_odom_stamp_;
  geometry_msgs::Twist last_odom_twist_;
  std::string last_odom_frame_id_;
  std::string last_odom_child_frame_id_;
  geometry_msgs::PolygonStamped global_footprint_;
  geometry_msgs::PolygonStamped local_footprint_;
  bool has_global_footprint_;
  bool has_local_footprint_;

};

#endif
