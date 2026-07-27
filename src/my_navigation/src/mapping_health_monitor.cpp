#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class MappingHealthMonitor {
public:
    MappingHealthMonitor(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_) {
        pnh_.param<std::string>("scan_topic", scan_topic_, "/scan");
        pnh_.param<std::string>("map_topic", map_topic_, "/map");
        pnh_.param<std::string>("local_grid_topic", local_grid_topic_, "/local_occupancy_grid");
        pnh_.param<std::string>("map_frame", map_frame_, "map");
        pnh_.param<std::string>("base_frame", base_frame_, "base_link");
        pnh_.param<double>("report_period", report_period_, 2.0);
        pnh_.param<double>("tf_timeout", tf_timeout_, 0.10);
        pnh_.param<double>("scan_min_hz", scan_min_hz_, 5.0);
        pnh_.param<double>("map_timeout", map_timeout_, 5.0);
        pnh_.param<double>("local_grid_timeout", local_grid_timeout_, 2.0);
        pnh_.param<double>("pose_window_sec", pose_window_sec_, 10.0);
        pnh_.param<double>("static_jitter_warn_m", static_jitter_warn_m_, 0.08);
        pnh_.param<double>("static_yaw_warn_rad", static_yaw_warn_rad_, 0.08);

        scan_sub_ = nh_.subscribe(scan_topic_, 50, &MappingHealthMonitor::scanCallback, this);
        map_sub_ = nh_.subscribe(map_topic_, 5, &MappingHealthMonitor::mapCallback, this);
        local_grid_sub_ = nh_.subscribe(local_grid_topic_, 10, &MappingHealthMonitor::localGridCallback, this);
        report_timer_ = nh_.createTimer(ros::Duration(report_period_), &MappingHealthMonitor::report, this);

        ROS_INFO("[MAPPING_HEALTH] scan=%s map=%s local_grid=%s tf=%s->%s period=%.1fs",
                 scan_topic_.c_str(), map_topic_.c_str(), local_grid_topic_.c_str(),
                 map_frame_.c_str(), base_frame_.c_str(), report_period_);
    }

private:
    struct PoseSample {
        ros::Time stamp;
        double x;
        double y;
        double yaw;
    };

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        const ros::Time now = ros::Time::now();
        if (!last_scan_wall_time_.isZero()) {
            const double dt = (now - last_scan_wall_time_).toSec();
            if (dt > 1e-4) {
                scan_dt_ema_ = scan_dt_ema_ <= 0.0 ? dt : 0.9 * scan_dt_ema_ + 0.1 * dt;
            }
        }
        last_scan_wall_time_ = now;
        last_scan_stamp_ = scan->header.stamp;
        last_scan_frame_ = scan->header.frame_id;
        ++scan_count_;

        int finite = 0;
        for (const float range : scan->ranges) {
            if (std::isfinite(range)) {
                ++finite;
            }
        }
        last_scan_points_ = static_cast<int>(scan->ranges.size());
        last_scan_finite_ = finite;
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map) {
        last_map_wall_time_ = ros::Time::now();
        last_map_stamp_ = map->header.stamp;
        last_map_frame_ = map->header.frame_id;
        map_width_ = map->info.width;
        map_height_ = map->info.height;
        map_resolution_ = map->info.resolution;
        ++map_count_;

        int unknown = 0;
        int free = 0;
        int occupied = 0;
        for (const int8_t value : map->data) {
            if (value < 0) {
                ++unknown;
            } else if (value >= 50) {
                ++occupied;
            } else {
                ++free;
            }
        }
        map_unknown_ = unknown;
        map_free_ = free;
        map_occupied_ = occupied;
    }

    void localGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid) {
        last_local_grid_wall_time_ = ros::Time::now();
        local_grid_width_ = grid->info.width;
        local_grid_height_ = grid->info.height;
        ++local_grid_count_;

        int unknown = 0;
        int free = 0;
        int occupied = 0;
        for (const int8_t value : grid->data) {
            if (value < 0) {
                ++unknown;
            } else if (value >= 50) {
                ++occupied;
            } else {
                ++free;
            }
        }
        local_unknown_ = unknown;
        local_free_ = free;
        local_occupied_ = occupied;
    }

    static double yawFromTransform(const geometry_msgs::TransformStamped& transform) {
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }

    static double angleDistance(double a, double b) {
        return std::atan2(std::sin(a - b), std::cos(a - b));
    }

    void updatePoseSample(const ros::Time& now, double x, double y, double yaw) {
        poses_.push_back({now, x, y, yaw});
        while (!poses_.empty() && (now - poses_.front().stamp).toSec() > pose_window_sec_) {
            poses_.pop_front();
        }
    }

    void computePoseWindow(double& span_xy, double& span_yaw) const {
        span_xy = 0.0;
        span_yaw = 0.0;
        if (poses_.size() < 2) {
            return;
        }

        double min_x = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();
        for (const PoseSample& pose : poses_) {
            min_x = std::min(min_x, pose.x);
            max_x = std::max(max_x, pose.x);
            min_y = std::min(min_y, pose.y);
            max_y = std::max(max_y, pose.y);
            span_yaw = std::max(span_yaw, std::abs(angleDistance(pose.yaw, poses_.front().yaw)));
        }
        span_xy = std::hypot(max_x - min_x, max_y - min_y);
    }

    void report(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        const double scan_hz = scan_dt_ema_ > 1e-4 ? 1.0 / scan_dt_ema_ : 0.0;
        const double map_age = last_map_wall_time_.isZero() ? std::numeric_limits<double>::infinity()
                                                            : (now - last_map_wall_time_).toSec();
        const double local_age = last_local_grid_wall_time_.isZero() ? std::numeric_limits<double>::infinity()
                                                                     : (now - last_local_grid_wall_time_).toSec();

        bool tf_ok = false;
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        std::string tf_error;
        try {
            const geometry_msgs::TransformStamped transform =
                tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(tf_timeout_));
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;
            yaw = yawFromTransform(transform);
            tf_ok = std::isfinite(x) && std::isfinite(y) && std::isfinite(yaw);
            if (tf_ok) {
                updatePoseSample(now, x, y, yaw);
            }
        } catch (const tf2::TransformException& ex) {
            tf_error = ex.what();
        }

        double pose_span_xy = 0.0;
        double pose_span_yaw = 0.0;
        computePoseWindow(pose_span_xy, pose_span_yaw);

        const bool scan_ok = scan_hz >= scan_min_hz_;
        const bool map_ok = map_count_ > 0 && map_age <= map_timeout_;
        const bool local_ok = local_grid_count_ > 0 && local_age <= local_grid_timeout_;
        const bool jitter_warn = poses_.size() >= 3 &&
                                 (pose_span_xy > static_jitter_warn_m_ ||
                                  pose_span_yaw > static_yaw_warn_rad_);
        const bool ok = scan_ok && map_ok && local_ok && tf_ok && !jitter_warn;

        const int map_total = map_unknown_ + map_free_ + map_occupied_;
        const double map_free_pct = map_total > 0 ? 100.0 * map_free_ / map_total : 0.0;
        const double map_occ_pct = map_total > 0 ? 100.0 * map_occupied_ / map_total : 0.0;
        const int local_total = local_unknown_ + local_free_ + local_occupied_;
        const double local_free_pct = local_total > 0 ? 100.0 * local_free_ / local_total : 0.0;
        const double local_occ_pct = local_total > 0 ? 100.0 * local_occupied_ / local_total : 0.0;

        if (ok) {
            ROS_INFO("[MAPPING_HEALTH] OK scan=%.1fHz frame=%s finite=%d/%d map=%ux%u free=%.1f%% occ=%.1f%% local=%ux%u free=%.1f%% occ=%.1f%% pose=(%.3f, %.3f, yaw=%.3f) jitter%.0fs=%.3fm/%.3frad",
                     scan_hz, last_scan_frame_.c_str(), last_scan_finite_, last_scan_points_,
                     map_width_, map_height_, map_free_pct, map_occ_pct,
                     local_grid_width_, local_grid_height_, local_free_pct, local_occ_pct,
                     x, y, yaw, pose_window_sec_, pose_span_xy, pose_span_yaw);
        } else {
            ROS_WARN("[MAPPING_HEALTH] WARN scan=%s %.1fHz map=%s age=%.2fs local=%s age=%.2fs tf=%s pose=(%.3f, %.3f, yaw=%.3f) jitter%.0fs=%.3fm/%.3frad tf_error='%s'",
                     scan_ok ? "OK" : "BAD", scan_hz,
                     map_ok ? "OK" : "BAD", map_age,
                     local_ok ? "OK" : "BAD", local_age,
                     tf_ok ? "OK" : "BAD", x, y, yaw,
                     pose_window_sec_, pose_span_xy, pose_span_yaw,
                     tf_error.c_str());
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber local_grid_sub_;
    ros::Timer report_timer_;

    std::string scan_topic_;
    std::string map_topic_;
    std::string local_grid_topic_;
    std::string map_frame_;
    std::string base_frame_;
    double report_period_;
    double tf_timeout_;
    double scan_min_hz_;
    double map_timeout_;
    double local_grid_timeout_;
    double pose_window_sec_;
    double static_jitter_warn_m_;
    double static_yaw_warn_rad_;

    ros::Time last_scan_wall_time_;
    ros::Time last_scan_stamp_;
    ros::Time last_map_wall_time_;
    ros::Time last_map_stamp_;
    ros::Time last_local_grid_wall_time_;
    double scan_dt_ema_ = 0.0;
    uint64_t scan_count_ = 0;
    uint64_t map_count_ = 0;
    uint64_t local_grid_count_ = 0;

    std::string last_scan_frame_;
    std::string last_map_frame_;
    int last_scan_points_ = 0;
    int last_scan_finite_ = 0;
    uint32_t map_width_ = 0;
    uint32_t map_height_ = 0;
    double map_resolution_ = 0.0;
    int map_unknown_ = 0;
    int map_free_ = 0;
    int map_occupied_ = 0;
    uint32_t local_grid_width_ = 0;
    uint32_t local_grid_height_ = 0;
    int local_unknown_ = 0;
    int local_free_ = 0;
    int local_occupied_ = 0;
    std::deque<PoseSample> poses_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapping_health_monitor");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    MappingHealthMonitor monitor(nh, pnh);
    ros::spin();
    return 0;
}
