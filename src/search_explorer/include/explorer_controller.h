#ifndef EXPLORER_CONTROLLER_H
#define EXPLORER_CONTROLLER_H

#include <ros/ros.h>

#include <angles/angles.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

constexpr double ROBOT_LENGTH = 0.64;
constexpr double ROBOT_WIDTH = 0.57;
constexpr double ROBOT_ROTATION_RADIUS = 0.50;

constexpr double COVERAGE_RANGE = 2.0;
constexpr double COVERAGE_HALF_ANGLE_DEG = 30.0;
constexpr double WAYPOINT_SPACING = 1.5;
constexpr double ALIGN_TOLERANCE_DEG = 8.0;
constexpr double COVERAGE_TARGET = 0.85;
constexpr double WAYPOINT_SKIP_RATIO = 0.70;
constexpr int MAX_WAYPOINT_VISITS = 2;
constexpr int POSITION_HISTORY_SIZE = 80;
constexpr double POSITION_REVISIT_R = 0.50;
constexpr double MIN_SCAN_TIME = 0.8;
constexpr double MAX_SCAN_TIME = 2.5;
constexpr double POSE_STALE_TIMEOUT = 6.5;
constexpr double LOCAL_MAP_STALE_TIMEOUT = 4.0;
constexpr double GOAL_PROGRESS_TIMEOUT = 8.0;

struct GridCell {
    int x;
    int y;

    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const GridCell& other) const {
        return !(*this == other);
    }
};

struct GridCellHash {
    std::size_t operator()(const GridCell& cell) const {
        return static_cast<std::size_t>(cell.y) * 65536U +
               static_cast<std::size_t>(cell.x);
    }
};

struct AStarNode {
    GridCell cell;
    double g_cost;
    double f_cost;
    GridCell parent;

    bool operator>(const AStarNode& other) const {
        return f_cost > other.f_cost;
    }
};

enum class RobotState {
    INIT,
    ALIGN,
    EXPLORE,
    NAVIGATE,
    SCAN,
    AVOID,
    STUCK,
    DONE
};

struct WaypointInfo {
    GridCell cell;
    int index;
    bool visited;
    int times_reached;
    double last_coverage;
    ros::Time last_visit;

    WaypointInfo()
        : cell{0, 0}
        , index(0)
        , visited(false)
        , times_reached(0)
        , last_coverage(0.0)
        , last_visit(0) {}
};

struct PosRecord {
    double x;
    double y;
    ros::Time stamp;
};

class ExplorerController {
public:
    ExplorerController();
    ~ExplorerController() = default;

    void spin();

private:
    static constexpr int8_t FREE_CELL = 0;
    static constexpr int8_t UNKNOWN_CELL = -1;
    static constexpr int8_t OCCUPIED_CELL = 100;
    static constexpr int STUCK_THRESHOLD = 10;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher cmd_vel_pub_;
    ros::Publisher coverage_goal_pub_;
    ros::Publisher global_path_pub_;
    ros::Publisher coverage_marker_pub_;

    ros::Subscriber map_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber warning_sub_;
    ros::Subscriber dynamic_points_sub_;
    ros::Subscriber local_map_sub_;
    ros::Subscriber object_goal_sub_;

    ros::Timer control_timer_;
    ros::Timer exploration_timer_;
    ros::Timer dynamic_decay_timer_;
    ros::Timer coverage_update_timer_;
    ros::Timer health_report_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void warningCallback(const std_msgs::Bool::ConstPtr& msg);
    void dynamicPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void localMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void objectGoalCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

    void controlLoop(const ros::TimerEvent& event);
    void explorationLoop(const ros::TimerEvent& event);
    void dynamicDecayLoop(const ros::TimerEvent& event);
    void coverageUpdateLoop(const ros::TimerEvent& event);
    void healthReportLoop(const ros::TimerEvent& event);

    std::vector<GridCell> main_route_;
    std::vector<WaypointInfo> waypoints_;
    std::vector<uint8_t> dominant_safe_mask_;
    std::vector<int> edge_distance_cells_;
    int route_index_;
    bool route_computed_;
    int total_circuits_;

    void planMainRoute();
    void rebuildWaypointCache();
    int findNearestRouteIndex(int rx, int ry) const;
    int findRouteIndexWithHeading(int rx, int ry) const;
    int findNextUnvisitedWaypoint(int start);

    void handleAlign();
    void handleExplore();
    void handleScan();
    void handleAvoid();
    void handleStuck();

    std::vector<uint8_t> coverage_grid_;
    int coverage_cells_total_;
    void markCoverageSector(double rx, double ry, double ryaw);
    void markCoverageDisk(double wx, double wy, double radius);
    double getLocalCov(int mx, int my, double radius) const;
    double getRobotLocalCov() const;
    double getWaypointCov(const GridCell& waypoint) const;
    double getGlobalCov() const;
    std::vector<uint8_t> buildPlanningFreeMask(bool use_inflation) const;
    std::vector<uint8_t> buildStaticSafetyMask(double safety_radius,
                                               int min_obstacle_cluster_cells) const;

    std::deque<PosRecord> pos_history_;
    void recordPos(double x, double y);
    double lastVisitAge(double x, double y, double radius) const;

    double computeWaypointHeading(int route_index) const;

    nav_msgs::OccupancyGrid static_map_;
    nav_msgs::OccupancyGrid local_dynamic_map_;
    bool map_received_;
    bool local_map_received_;
    bool worldToMap(double wx, double wy, int& mx, int& my) const;
    void mapToWorld(int mx, int my, double& wx, double& wy) const;
    bool isInMap(int mx, int my) const;
    bool refreshPoseFromTf(const std::string& reason);
    bool hasFreshLocalMap() const;
    bool hasFreshScan() const;
    bool updateStartupGate();

    int8_t getStaticCost(int mx, int my) const;
    int8_t getDynamicCost(int mx, int my) const;
    int8_t getTotalCost(int mx, int my) const;
    bool isCellBlocked(int mx, int my) const;
    bool isLocalMapBlocked(double wx, double wy) const;
    bool isLocalMapAreaBlocked(double wx, double wy,
                               double radius,
                               double min_ratio,
                               int min_hits) const;
    bool handleFrontStaticBlock(const ros::Time& now,
                                double lookahead_x,
                                double lookahead_y);

    geometry_msgs::Pose robot_pose_;
    bool pose_received_;
    double robot_yaw_;
    RobotState state_;
    ros::Time state_enter_time_;

    std::vector<geometry_msgs::Point> global_path_;
    GridCell current_goal_cell_;
    bool has_active_goal_;
    geometry_msgs::PointStamped object_goal_;
    bool object_goal_active_;
    bool navigating_to_object_;
    ros::Time last_object_goal_time_;
    geometry_msgs::Point last_object_goal_pos_;
    int last_selected_wp_;
    bool planToObjectGoal(double object_wx, double object_wy,
                          GridCell& approach_cell,
                          std::vector<geometry_msgs::Point>& path);
    double heuristic(const GridCell& a, const GridCell& b) const;
    std::vector<GridCell> getNeighbors(const GridCell& cell) const;
    bool aStarSearch(const GridCell& start, const GridCell& goal, std::vector<GridCell>& output);
    bool planToGoal(const GridCell& start, const GridCell& goal,
                    std::vector<geometry_msgs::Point>& waypoints);
    bool planAlongRouteToGoal(int start_index, int goal_index,
                              std::vector<geometry_msgs::Point>& waypoints) const;
    bool findReachableRouteIndex(int robot_x, int robot_y, int preferred_index,
                                 int& selected_index,
                                 std::vector<geometry_msgs::Point>& path);
    bool calcLookAhead(const std::vector<geometry_msgs::Point>& path,
                       geometry_msgs::Point& look_ahead);
    bool isPathBlocked() const;
    void replanToCurrentGoal(const std::string& reason);

    std::vector<int8_t> dynamic_layer_;
    std::vector<ros::Time> dynamic_timestamps_;
    bool dynamic_obstacle_active_;
    ros::Time last_dynamic_alert_time_;
    void updateDynamicLayer(const sensor_msgs::PointCloud2& cloud);
    void decayDynamicLayer();
    bool xformCloud(const sensor_msgs::PointCloud2& input, sensor_msgs::PointCloud2& output);

    double max_lin_;
    double max_ang_;
    double goal_tol_;
    double lookahead_;
    double path_replan_thresh_;
    double dynamic_clear_time_;
    double inflation_radius_;
    double route_safety_radius_;
    double cov_range_;
    double cov_half_angle_;
    double align_ang_;
    double wp_spacing_;
    double wp_cov_radius_;
    double scan_cov_target_;
    double health_interval_;
    bool phase1_static_global_only_;
    bool enable_object_goal_;
    bool use_edge_route_;
    double edge_side_clearance_;
    double edge_center_offset_;
    double edge_offset_tolerance_;
    double edge_preferred_center_distance_;
    double edge_min_center_distance_;
    double edge_max_center_distance_;
    int min_static_obstacle_cluster_cells_;
    int edge_reachable_search_window_;
    double object_standoff_;
    bool require_local_map_before_navigation_;
    bool startup_gate_enabled_;
    bool startup_gate_released_;
    bool startup_require_clear_warning_;
    double startup_stable_duration_;
    double startup_pose_max_age_;
    double startup_local_map_max_age_;
    double startup_scan_max_age_;
    double laser_front_angle_;
    double front_block_replan_delay_;
    int front_block_skip_count_;
    int startup_min_pose_messages_;
    ros::Time startup_gate_ready_since_;
    ros::Time controller_start_time_;
    ros::Time last_scan_msg_time_;
    ros::Time last_scan_stamp_;
    ros::Time front_block_since_;
    int scan_msg_ct_;
    double min_front_scan_range_;

    void publishCmdVel(double linear, double angular);
    void eStop();
    void setState(RobotState new_state, const std::string& reason);

    void visPath(const std::vector<geometry_msgs::Point>& path);
    void visGoal(const GridCell& goal);
    void pubCovMarkers();
    void pubRouteMarkers();

    int stuck_ctr_;
    double last_sx_;
    double last_sy_;
    ros::Time last_stuck_check_time_;
    bool isStuck();

    ros::Time last_report_;
    int pose_msg_ct_;
    int path_ok_;
    int path_fail_;
    int replan_ct_;
    int wp_skipped_;
    int wp_visited_;
    double total_dist_;
    ros::Time last_pose_msg_time_;
    ros::Time last_map_msg_time_;
    ros::Time last_lmap_msg_time_;
    ros::Time last_goal_progress_time_;
    double current_goal_start_distance_;
    bool latest_obstacle_warning_;
    bool soft_avoid_active_;
    ros::Time last_soft_avoid_time_;

    std::string stateName(RobotState state) const;
    std::string formatPose() const;
};

#endif
