#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

struct PipelineStats {
    int raw_points;
    int angle_filtered;
    int dist_filtered;
    int map_filtered;
    int clustered;
    int clusters_found;
    double angle_drop_pct;
    double dist_drop_pct;
    double map_drop_pct;
    double cluster_drop_pct;

    void reset() {
        raw_points = 0;
        angle_filtered = 0;
        dist_filtered = 0;
        map_filtered = 0;
        clustered = 0;
        clusters_found = 0;
        angle_drop_pct = 0.0;
        dist_drop_pct = 0.0;
        map_drop_pct = 0.0;
        cluster_drop_pct = 0.0;
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "raw=" << raw_points
            << " angle=" << angle_filtered
            << " dist=" << dist_filtered
            << " map=" << map_filtered
            << " cluster_points=" << clustered
            << " clusters=" << clusters_found;
        return oss.str();
    }
};

struct TransmissionTracker {
    std::string topic_name;
    ros::Time last_rx_time;
    ros::Time last_tx_time;
    uint64_t rx_count;
    uint64_t tx_count;
    uint64_t rx_drop_count;
    bool is_active;

    TransmissionTracker()
        : rx_count(0)
        , tx_count(0)
        , rx_drop_count(0)
        , is_active(false) {}

    void recordRx(const ros::Time& stamp) {
        if (last_rx_time.toSec() > 0.0) {
            const double dt_ms = (stamp - last_rx_time).toSec() * 1000.0;
            if (dt_ms > 500.0 && rx_count > 10) {
                ++rx_drop_count;
            }
        }
        last_rx_time = stamp;
        ++rx_count;
        is_active = true;
    }

    void recordTx() {
        last_tx_time = ros::Time::now();
        ++tx_count;
    }

    std::string statusStr() const {
        std::ostringstream oss;
        oss << topic_name << ": rx=" << rx_count
            << " tx=" << tx_count
            << " drops=" << rx_drop_count;
        if (last_rx_time.toSec() > 0.0) {
            const double age = (ros::Time::now() - last_rx_time).toSec();
            oss << " age=" << std::fixed << std::setprecision(1) << age << "s";
        }
        return oss.str();
    }
};

class DynamicObstacleDetector {
public:
    DynamicObstacleDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber map_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber object_angle_sub_;
    ros::Subscriber object_x_sub_;
    ros::Subscriber cmd_vel_sub_;

    ros::Publisher dynamic_cloud_pub_;
    ros::Publisher local_map_pub_;
    ros::Publisher warn_pub_;
    ros::Publisher closest_obs_pub_;
    ros::Publisher object_goal_pub_;
    ros::Publisher object_info_pub_;

    ros::Timer health_timer_;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void objectAngleCallback(const std_msgs::Float32::ConstPtr& msg);
    void objectXCallback(const std_msgs::Float32::ConstPtr& msg);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    std::vector<std::vector<geometry_msgs::Point>> euclideanCluster(
        const std::vector<geometry_msgs::Point>& points,
        double tolerance,
        int min_points);

    bool hysteresisFilter(bool has_obstacle_this_frame);
    bool isRobotWellLocalized(double robot_x, double robot_y);
    int getMapOccupancy(double wx, double wy);
    bool worldToMap(double wx, double wy, int& mx, int& my);

    void publishDynamicCloud(
        const std::vector<geometry_msgs::Point>& points,
        const ros::Time& stamp,
        const std::string& frame_id);

    void publishLocalMap(
        const std::vector<geometry_msgs::Point>& dynamic_points,
        const std::vector<geometry_msgs::Point>& corner_points,
        const ros::Time& stamp,
        double robot_x,
        double robot_y,
        bool localization_ready);

    void publishWarning(bool state);
    void publishClosestObstacle(
        const std::vector<geometry_msgs::Point>& base_points,
        const ros::Time& stamp);

    void logPipelineStats(const PipelineStats& stats);
    void logTransmissionSummary();
    void publishHealthReport(const ros::Time& now);
    void logTfLookup(const std::string& from,
                     const std::string& to,
                     bool success,
                     double duration_ms);

    double dynamic_angle_deg_;
    double angle_min_rel_;
    double angle_max_rel_;
    double dist_threshold_;
    double cluster_tolerance_;
    int min_cluster_points_;
    int enter_frames_;
    int exit_frames_;
    double local_map_size_;
    double local_map_res_;
    std::string laser_frame_;
    std::string base_frame_;
    double map_wait_timeout_;
    double loc_unknown_thresh_;
    std::string camera_frame_;
    double object_max_range_;
    double object_fov_half_deg_;
    bool object_x_is_range_;
    double object_projection_start_range_;
    double pipeline_log_interval_;
    double health_report_interval_;
    double manual_pose_report_interval_;
    double tx_summary_interval_;
    double max_scan_age_;
    double max_scan_future_age_;
    double tf_latest_fallback_age_;
    bool include_static_map_in_local_map_;
    bool enable_corner_filter_;
    double corner_threshold_;
    int corner_window_;
    double noise_min_range_;
    double noise_search_radius_;
    int noise_min_neighbors_;
    int local_map_dilate_cells_;

    std::vector<geometry_msgs::Point> latest_corner_points_;
    ros::Time latest_corner_stamp_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    nav_msgs::OccupancyGrid map_;
    bool map_received_;
    bool alarm_active_;
    int alarm_enter_count_;
    int alarm_exit_count_;
    bool map_tf_ever_available_;
    bool localization_ready_;

    bool object_goal_received_;
    bool object_x_received_;
    double object_x_;
    ros::Time object_x_stamp_;
    ros::Time node_start_time_;
    ros::Time last_local_map_stamp_;
    ros::Time last_cmd_vel_stamp_;
    double last_cmd_linear_;
    double last_cmd_angular_;

    PipelineStats last_stats_;

    uint64_t frame_count_;
    uint64_t frame_skip_count_;

    TransmissionTracker scan_tracker_;
    TransmissionTracker map_tracker_;
    TransmissionTracker dynamic_cloud_tracker_;
    TransmissionTracker local_map_tracker_;
    TransmissionTracker warning_tracker_;
    TransmissionTracker closest_obs_tracker_;
    TransmissionTracker object_goal_tracker_;

    int tf_lookup_total_;
    int tf_lookup_fail_;
    double tf_lookup_max_ms_;
    double tf_lookup_avg_ms_;

    ros::Time last_pipeline_log_time_;
    ros::Time last_tx_summary_time_;
};

#endif
