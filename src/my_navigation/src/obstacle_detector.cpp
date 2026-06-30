#include "obstacle_detector.h"

#include <sensor_msgs/point_cloud2_iterator.h>

namespace {

double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

}  // namespace

DynamicObstacleDetector::DynamicObstacleDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , tf_listener_(tf_buffer_)
    , map_received_(false)
    , alarm_active_(false)
    , alarm_enter_count_(0)
    , alarm_exit_count_(0)
    , map_tf_ever_available_(false)
    , localization_ready_(false)
    , object_goal_received_(false)
    , object_x_received_(false)
    , object_x_(0.0)
    , last_cmd_linear_(0.0)
    , last_cmd_angular_(0.0)
    , frame_count_(0)
    , frame_skip_count_(0)
    , tf_lookup_total_(0)
    , tf_lookup_fail_(0)
    , tf_lookup_max_ms_(0.0)
    , tf_lookup_avg_ms_(0.0) {
    pnh_.param<double>("dynamic_angle_deg", dynamic_angle_deg_, 150.0);
    pnh_.param<double>("distance_threshold", dist_threshold_, 0.6);
    pnh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.15);
    pnh_.param<int>("min_cluster_points", min_cluster_points_, 15);
    pnh_.param<int>("enter_frames", enter_frames_, 2);
    pnh_.param<int>("exit_frames", exit_frames_, 5);
    pnh_.param<std::string>("laser_frame", laser_frame_, "laser_link");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<double>("local_map_size", local_map_size_, 2.0);
    pnh_.param<double>("local_map_res", local_map_res_, 0.05);
    pnh_.param<double>("map_wait_timeout", map_wait_timeout_, 10.0);
    pnh_.param<double>("localization_unknown_ratio_threshold", loc_unknown_thresh_, 0.8);
    pnh_.param<std::string>("camera_frame", camera_frame_, "camera_link");
    pnh_.param<double>("object_max_range", object_max_range_, 2.0);
    pnh_.param<double>("object_fov_half_deg", object_fov_half_deg_, 30.0);
    pnh_.param<bool>("object_x_is_range", object_x_is_range_, false);
    pnh_.param<double>("object_projection_start_range", object_projection_start_range_, 0.45);
    pnh_.param<double>("pipeline_log_interval", pipeline_log_interval_, 2.0);
    pnh_.param<double>("health_report_interval", health_report_interval_, 2.0);
    pnh_.param<double>("manual_pose_report_interval", manual_pose_report_interval_, 1.0);
    pnh_.param<double>("tx_summary_interval", tx_summary_interval_, 2.0);
    pnh_.param<double>("max_scan_age", max_scan_age_, 1.0);
    pnh_.param<double>("max_scan_future_age", max_scan_future_age_, 0.20);
    pnh_.param<double>("tf_latest_fallback_age", tf_latest_fallback_age_, 0.75);
    pnh_.param<bool>("include_static_map_in_local_map",
                     include_static_map_in_local_map_, false);
    pnh_.param<bool>("enable_corner_filter", enable_corner_filter_, true);
    pnh_.param<double>("corner_threshold", corner_threshold_, 0.20);
    pnh_.param<int>("corner_window", corner_window_, 2);
    pnh_.param<double>("noise_min_range", noise_min_range_, 0.30);
    pnh_.param<double>("noise_search_radius", noise_search_radius_, 0.25);
    pnh_.param<int>("noise_min_neighbors", noise_min_neighbors_, 2);
    pnh_.param<int>("local_map_dilate_cells", local_map_dilate_cells_, 1);

    const double half_angle = dynamic_angle_deg_ * M_PI / 360.0;
    angle_min_rel_ = -half_angle;
    angle_max_rel_ = half_angle;

    node_start_time_ = ros::Time::now();
    last_local_map_stamp_ = ros::Time(0);
    last_cmd_vel_stamp_ = ros::Time(0);
    manual_pose_report_interval_ = std::max(0.5, manual_pose_report_interval_);
    health_report_interval_ = std::min(health_report_interval_, manual_pose_report_interval_);
    max_scan_age_ = std::max(0.20, max_scan_age_);
    max_scan_future_age_ = std::max(0.05, max_scan_future_age_);
    tf_latest_fallback_age_ = std::max(0.05, tf_latest_fallback_age_);

    map_sub_ = nh_.subscribe("/map", 1, &DynamicObstacleDetector::mapCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 10, &DynamicObstacleDetector::scanCallback, this);
    object_angle_sub_ = nh_.subscribe("/object_direction", 1,
                                      &DynamicObstacleDetector::objectAngleCallback, this);
    object_x_sub_ = nh_.subscribe("/object_x", 1,
                                  &DynamicObstacleDetector::objectXCallback, this);
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10,
                                 &DynamicObstacleDetector::cmdVelCallback, this);

    dynamic_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_obstacles", 10);
    local_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_dynamic_map", 10);
    warn_pub_ = nh_.advertise<std_msgs::Bool>("/obstacle_warning", 10);
    closest_obs_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/closest_obstacle", 10);
    object_goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/object_goal", 1);
    object_info_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/object_info", 1);

    health_timer_ = nh_.createTimer(
        ros::Duration(health_report_interval_),
        [this](const ros::TimerEvent&) { publishHealthReport(ros::Time::now()); });

    scan_tracker_.topic_name = "/scan";
    map_tracker_.topic_name = "/map";
    dynamic_cloud_tracker_.topic_name = "/dynamic_obstacles";
    local_map_tracker_.topic_name = "/local_dynamic_map";
    warning_tracker_.topic_name = "/obstacle_warning";
    closest_obs_tracker_.topic_name = "/closest_obstacle";
    object_goal_tracker_.topic_name = "/object_goal";

    ROS_INFO("============================================================");
    ROS_INFO("[DETECT] Dynamic obstacle detector started");
    ROS_INFO("[DETECT] laser_frame=%s base_frame=%s local_map=%.2fm@%.2fm",
             laser_frame_.c_str(), base_frame_.c_str(),
             local_map_size_, local_map_res_);
    ROS_INFO("[DETECT] obstacle FOV=%.1fdeg threshold=%.2fm cluster_tol=%.2fm",
             dynamic_angle_deg_, dist_threshold_, cluster_tolerance_);
    ROS_INFO("[DETECT] object tracking half-FOV=%.1fdeg max_range=%.2fm x_mode=%s",
             object_fov_half_deg_, object_max_range_,
             object_x_is_range_ ? "range" : "pixel");
    ROS_INFO("[DETECT] object projection start=%.2fm topic chain: /object_direction,/object_x -> /object_goal",
             object_projection_start_range_);
    ROS_INFO("[DETECT] scan freshness max_age=%.2fs future_tolerance=%.2fs tf_latest_fallback<=%.2fs",
             max_scan_age_, max_scan_future_age_, tf_latest_fallback_age_);
    ROS_INFO("[DETECT] local_map static layer=%s",
             include_static_map_in_local_map_ ? "ON" : "OFF");
    ROS_INFO("[DETECT] corner_filter=%s threshold=%.2f neighbors=%d",
             enable_corner_filter_ ? "ON" : "OFF",
             corner_threshold_, noise_min_neighbors_);
    ROS_INFO("============================================================");
}

void DynamicObstacleDetector::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_ = *msg;
    const bool first_map = !map_received_;
    map_received_ = true;
    map_tracker_.recordRx(msg->header.stamp);
    if (first_map) {
        ROS_INFO("[DETECT] /map received: %dx%d @ %.3f",
                 msg->info.width, msg->info.height, msg->info.resolution);
    }
}

void DynamicObstacleDetector::objectXCallback(const std_msgs::Float32::ConstPtr& msg) {
    object_x_ = msg->data;
    object_x_received_ = true;
    object_x_stamp_ = ros::Time::now();
    ROS_INFO_THROTTLE(1.0, "[OBJECT-IN] /object_x=%.2f mode=%s",
                      object_x_, object_x_is_range_ ? "range_m" : "pixel_x");
}

void DynamicObstacleDetector::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    last_cmd_linear_ = msg->linear.x;
    last_cmd_angular_ = msg->angular.z;
    last_cmd_vel_stamp_ = ros::Time::now();
}

void DynamicObstacleDetector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ++frame_count_;
    scan_tracker_.recordRx(scan->header.stamp);
    const ros::Time now = ros::Time::now();
    const double scan_age = (now - scan->header.stamp).toSec();

    if (scan_age > max_scan_age_ || scan_age < -max_scan_future_age_) {
        ++frame_skip_count_;
        ROS_WARN_THROTTLE(1.0,
                          "[DETECT] Drop stale /scan: age=%.2fs stamp=%.3f now=%.3f. Check lidar/cloud_to_scan time sync.",
                          scan_age,
                          scan->header.stamp.toSec(),
                          now.toSec());
        publishWarning(false);
        logTransmissionSummary();
        return;
    }

    if (!map_received_) {
        ++frame_skip_count_;
        ROS_WARN_THROTTLE(2.0, "[DETECT] Waiting for static map");
        return;
    }

    const ros::Time tf_start = ros::Time::now();
    bool map_tf_ok = false;
    bool use_latest_tf_for_points = false;
    geometry_msgs::TransformStamped map_to_laser;
    try {
        map_to_laser = tf_buffer_.lookupTransform("map", laser_frame_,
                                                  scan->header.stamp, ros::Duration(0.25));
        map_tf_ok = true;
        map_tf_ever_available_ = true;
    } catch (const tf2::TransformException& ex) {
        if (std::fabs(scan_age) <= tf_latest_fallback_age_) {
            try {
                map_to_laser = tf_buffer_.lookupTransform("map", laser_frame_,
                                                          ros::Time(0), ros::Duration(0.10));
                map_tf_ok = true;
                map_tf_ever_available_ = true;
                use_latest_tf_for_points = true;
                ROS_WARN_THROTTLE(2.0,
                                  "[DETECT] map->%s TF unavailable at scan stamp, using latest TF fallback: %s",
                                  laser_frame_.c_str(), ex.what());
            } catch (const tf2::TransformException& fallback_ex) {
                ROS_WARN_THROTTLE(2.0, "[DETECT] map->%s TF unavailable: %s | latest fallback failed: %s",
                                  laser_frame_.c_str(), ex.what(), fallback_ex.what());
            }
        } else {
            ROS_WARN_THROTTLE(2.0, "[DETECT] map->%s TF unavailable: %s",
                              laser_frame_.c_str(), ex.what());
        }
    }
    logTfLookup("map", laser_frame_, map_tf_ok,
                (ros::Time::now() - tf_start).toSec() * 1000.0);

    geometry_msgs::TransformStamped base_to_laser;
    const ros::Time tf_start_base = ros::Time::now();
    try {
        base_to_laser = tf_buffer_.lookupTransform(base_frame_, laser_frame_,
                                                   scan->header.stamp, ros::Duration(0.15));
        logTfLookup(base_frame_, laser_frame_, true,
                    (ros::Time::now() - tf_start_base).toSec() * 1000.0);
    } catch (const tf2::TransformException& ex) {
        try {
            base_to_laser = tf_buffer_.lookupTransform(base_frame_, laser_frame_,
                                                       ros::Time(0), ros::Duration(0.10));
            logTfLookup(base_frame_, laser_frame_, true,
                        (ros::Time::now() - tf_start_base).toSec() * 1000.0);
            use_latest_tf_for_points = true;
            ROS_WARN_THROTTLE(2.0,
                              "[DETECT] %s->%s TF unavailable at scan stamp, using latest TF fallback: %s",
                              base_frame_.c_str(), laser_frame_.c_str(), ex.what());
        } catch (const tf2::TransformException& fallback_ex) {
            logTfLookup(base_frame_, laser_frame_, false,
                        (ros::Time::now() - tf_start_base).toSec() * 1000.0);
            ++frame_skip_count_;
            ROS_WARN_THROTTLE(2.0, "[DETECT] %s->%s TF unavailable: %s | latest fallback failed: %s",
                              base_frame_.c_str(), laser_frame_.c_str(), ex.what(), fallback_ex.what());
            return;
        }
    }

    double robot_map_x = 0.0;
    double robot_map_y = 0.0;
    localization_ready_ = false;
    if (map_tf_ok) {
        robot_map_x = map_to_laser.transform.translation.x;
        robot_map_y = map_to_laser.transform.translation.y;
        try {
            const geometry_msgs::TransformStamped map_to_base =
                tf_buffer_.lookupTransform("map", base_frame_,
                                           scan->header.stamp, ros::Duration(0.15));
            robot_map_x = map_to_base.transform.translation.x;
            robot_map_y = map_to_base.transform.translation.y;
        } catch (const tf2::TransformException& ex) {
            try {
                const geometry_msgs::TransformStamped map_to_base =
                    tf_buffer_.lookupTransform("map", base_frame_,
                                               ros::Time(0), ros::Duration(0.10));
                robot_map_x = map_to_base.transform.translation.x;
                robot_map_y = map_to_base.transform.translation.y;
                ROS_WARN_THROTTLE(2.0,
                                  "[DETECT] map->%s TF unavailable at scan stamp, using latest TF for local map center: %s",
                                  base_frame_.c_str(), ex.what());
            } catch (const tf2::TransformException& fallback_ex) {
                ROS_WARN_THROTTLE(2.0,
                                  "[DETECT] map->%s TF unavailable, local map will use laser center: %s | latest fallback failed: %s",
                                  base_frame_.c_str(), ex.what(), fallback_ex.what());
            }
        }
        localization_ready_ = isRobotWellLocalized(robot_map_x, robot_map_y);
    }

    struct BeamInfo {
        geometry_msgs::Point map_point;
        geometry_msgs::Point base_point;
        double angle_base;
        double dist_base;
        bool keep_for_corner;
    };

    std::vector<BeamInfo> corner_candidates;
    std::vector<geometry_msgs::Point> close_base_points;
    std::vector<geometry_msgs::Point> dynamic_candidates;
    PipelineStats stats;
    stats.reset();

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        const double range = scan->ranges[i];
        if (range < scan->range_min || range > scan->range_max ||
            std::isnan(range) || std::isinf(range)) {
            continue;
        }
        ++stats.raw_points;

        geometry_msgs::PointStamped laser_pt;
        laser_pt.header = scan->header;
        laser_pt.header.frame_id = laser_frame_;
        if (use_latest_tf_for_points) {
            laser_pt.header.stamp = ros::Time(0);
        }
        const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
        laser_pt.point.x = range * std::cos(angle);
        laser_pt.point.y = range * std::sin(angle);
        laser_pt.point.z = 0.0;

        geometry_msgs::PointStamped base_pt;
        geometry_msgs::PointStamped map_pt;
        try {
            tf_buffer_.transform(laser_pt, base_pt, base_frame_, ros::Duration(0.05));
            tf_buffer_.transform(laser_pt, map_pt, "map", ros::Duration(0.05));
        } catch (const tf2::TransformException&) {
            continue;
        }

        const double angle_base = std::atan2(base_pt.point.y, base_pt.point.x);
        const double dist_base = std::hypot(base_pt.point.x, base_pt.point.y);
        if (dist_base <= local_map_size_) {
            BeamInfo info;
            info.map_point = map_pt.point;
            info.base_point = base_pt.point;
            info.angle_base = angle_base;
            info.dist_base = dist_base;
            info.keep_for_corner = true;
            corner_candidates.push_back(info);
        }

        if (angle_base < angle_min_rel_ || angle_base > angle_max_rel_) {
            continue;
        }
        ++stats.angle_filtered;

        if (dist_base > dist_threshold_) {
            continue;
        }
        ++stats.dist_filtered;
        close_base_points.push_back(base_pt.point);

        if (!localization_ready_) {
            dynamic_candidates.push_back(map_pt.point);
            ++stats.map_filtered;
            continue;
        }

        const int occ = getMapOccupancy(map_pt.point.x, map_pt.point.y);
        // Only flag as dynamic if the map explicitly marks this cell as free.
        // Unknown cells (occ == -1) are ignored to prevent noisy/sparse maps
        // from generating false dynamic obstacles in unclassified areas.
        if (occ >= 0 && occ <= 50) {
            dynamic_candidates.push_back(map_pt.point);
            ++stats.map_filtered;
        }
    }

    if (enable_corner_filter_ && !corner_candidates.empty()) {
        std::sort(corner_candidates.begin(), corner_candidates.end(),
                  [](const BeamInfo& a, const BeamInfo& b) {
                      return a.angle_base < b.angle_base;
                  });

        for (size_t i = 0; i < corner_candidates.size(); ++i) {
            const double current = corner_candidates[i].dist_base;
            if (current <= noise_min_range_) {
                corner_candidates[i].keep_for_corner = true;
                continue;
            }

            double min_dist = current;
            double max_dist = current;
            int neighbors = 0;
            for (int offset = -corner_window_; offset <= corner_window_; ++offset) {
                if (offset == 0) {
                    continue;
                }
                const int idx = static_cast<int>(i) + offset;
                if (idx < 0 || idx >= static_cast<int>(corner_candidates.size())) {
                    continue;
                }
                const double angle_gap = std::fabs(corner_candidates[idx].angle_base -
                                                   corner_candidates[i].angle_base);
                if (angle_gap > 10.0 * M_PI / 180.0) {
                    continue;
                }
                min_dist = std::min(min_dist, corner_candidates[idx].dist_base);
                max_dist = std::max(max_dist, corner_candidates[idx].dist_base);
                ++neighbors;
            }
            corner_candidates[i].keep_for_corner =
                neighbors >= 2 && (max_dist - min_dist) >= corner_threshold_;
        }
    }

    std::vector<geometry_msgs::Point> filtered_corners;
    filtered_corners.reserve(corner_candidates.size());
    for (size_t i = 0; i < corner_candidates.size(); ++i) {
        if (!corner_candidates[i].keep_for_corner) {
            continue;
        }

        if (corner_candidates[i].dist_base > noise_min_range_) {
            int neighbor_count = 0;
            for (size_t j = 0; j < corner_candidates.size(); ++j) {
                if (i == j || !corner_candidates[j].keep_for_corner) {
                    continue;
                }
                const double dx = corner_candidates[i].map_point.x - corner_candidates[j].map_point.x;
                const double dy = corner_candidates[i].map_point.y - corner_candidates[j].map_point.y;
                if (dx * dx + dy * dy <= noise_search_radius_ * noise_search_radius_) {
                    ++neighbor_count;
                }
            }
            if (neighbor_count < noise_min_neighbors_) {
                continue;
            }
        }

        filtered_corners.push_back(corner_candidates[i].map_point);
    }

    latest_corner_points_ = filtered_corners;
    latest_corner_stamp_ = now;

    std::vector<std::vector<geometry_msgs::Point>> clusters =
        euclideanCluster(dynamic_candidates, cluster_tolerance_, min_cluster_points_);
    stats.clusters_found = static_cast<int>(clusters.size());

    std::vector<geometry_msgs::Point> dynamic_points;
    for (const std::vector<geometry_msgs::Point>& cluster : clusters) {
        dynamic_points.insert(dynamic_points.end(), cluster.begin(), cluster.end());
    }
    stats.clustered = static_cast<int>(dynamic_points.size());

    const double raw = std::max(1, stats.raw_points);
    stats.angle_drop_pct = clamp01(1.0 - static_cast<double>(stats.angle_filtered) / raw) * 100.0;
    stats.dist_drop_pct = stats.angle_filtered > 0
        ? clamp01(1.0 - static_cast<double>(stats.dist_filtered) /
                  static_cast<double>(stats.angle_filtered)) * 100.0
        : 0.0;
    stats.map_drop_pct = stats.dist_filtered > 0
        ? clamp01(1.0 - static_cast<double>(stats.map_filtered) /
                  static_cast<double>(stats.dist_filtered)) * 100.0
        : 0.0;
    stats.cluster_drop_pct = stats.map_filtered > 0
        ? clamp01(1.0 - static_cast<double>(stats.clustered) /
                  static_cast<double>(stats.map_filtered)) * 100.0
        : 0.0;

    last_stats_ = stats;
    logPipelineStats(stats);

    const bool should_alarm = hysteresisFilter(!dynamic_points.empty());
    publishWarning(should_alarm);
    if (should_alarm) {
        publishDynamicCloud(dynamic_points, now, "map");
        publishClosestObstacle(close_base_points, now);
    } else {
        publishDynamicCloud(std::vector<geometry_msgs::Point>(), now, "map");
    }

    publishLocalMap(dynamic_points, filtered_corners, now,
                    robot_map_x, robot_map_y, localization_ready_);
    logTransmissionSummary();
}

bool DynamicObstacleDetector::hysteresisFilter(bool has_obstacle_this_frame) {
    if (has_obstacle_this_frame) {
        alarm_exit_count_ = 0;
        ++alarm_enter_count_;
        if (alarm_enter_count_ >= enter_frames_ && !alarm_active_) {
            alarm_active_ = true;
            ROS_WARN("[DETECT] Obstacle warning activated");
        }
    } else {
        alarm_enter_count_ = 0;
        ++alarm_exit_count_;
        if (alarm_exit_count_ >= exit_frames_ && alarm_active_) {
            alarm_active_ = false;
            ROS_INFO("[DETECT] Obstacle warning cleared");
        }
    }
    return alarm_active_;
}

std::vector<std::vector<geometry_msgs::Point>> DynamicObstacleDetector::euclideanCluster(
    const std::vector<geometry_msgs::Point>& points,
    double tolerance,
    int min_points) {
    std::vector<std::vector<geometry_msgs::Point>> clusters;
    if (points.empty()) {
        return clusters;
    }

    std::vector<bool> visited(points.size(), false);
    for (size_t i = 0; i < points.size(); ++i) {
        if (visited[i]) {
            continue;
        }

        std::queue<size_t> pending;
        pending.push(i);
        visited[i] = true;

        std::vector<geometry_msgs::Point> cluster;
        while (!pending.empty()) {
            const size_t index = pending.front();
            pending.pop();
            cluster.push_back(points[index]);

            for (size_t j = 0; j < points.size(); ++j) {
                if (visited[j]) {
                    continue;
                }
                const double dx = points[index].x - points[j].x;
                const double dy = points[index].y - points[j].y;
                if (dx * dx + dy * dy <= tolerance * tolerance) {
                    visited[j] = true;
                    pending.push(j);
                }
            }
        }

        if (static_cast<int>(cluster.size()) >= min_points) {
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

bool DynamicObstacleDetector::isRobotWellLocalized(double robot_x, double robot_y) {
    if (!map_received_) {
        return false;
    }

    const double check_size = 0.5;
    const double resolution = map_.info.resolution;
    int known_cells = 0;
    int unknown_cells = 0;

    for (double dx = -check_size * 0.5; dx <= check_size * 0.5; dx += resolution) {
        for (double dy = -check_size * 0.5; dy <= check_size * 0.5; dy += resolution) {
            const int occ = getMapOccupancy(robot_x + dx, robot_y + dy);
            if (occ == -1) {
                ++unknown_cells;
            } else {
                ++known_cells;
            }
        }
    }

    if (known_cells + unknown_cells == 0) {
        return false;
    }

    const double unknown_ratio =
        static_cast<double>(unknown_cells) /
        static_cast<double>(known_cells + unknown_cells);
    return unknown_ratio <= loc_unknown_thresh_;
}

int DynamicObstacleDetector::getMapOccupancy(double wx, double wy) {
    int mx = 0;
    int my = 0;
    if (!worldToMap(wx, wy, mx, my)) {
        return -1;
    }
    return map_.data[my * map_.info.width + mx];
}

bool DynamicObstacleDetector::worldToMap(double wx, double wy, int& mx, int& my) {
    if (!map_received_) {
        return false;
    }

    const double origin_x = map_.info.origin.position.x;
    const double origin_y = map_.info.origin.position.y;
    const double resolution = map_.info.resolution;

    mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
    my = static_cast<int>(std::floor((wy - origin_y) / resolution));
    return mx >= 0 && my >= 0 &&
           mx < static_cast<int>(map_.info.width) &&
           my < static_cast<int>(map_.info.height);
}

void DynamicObstacleDetector::publishDynamicCloud(
    const std::vector<geometry_msgs::Point>& points,
    const ros::Time& stamp,
    const std::string& frame_id) {
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_dense = true;
    cloud.is_bigendian = false;

    sensor_msgs::PointField field;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;

    field.name = "x";
    field.offset = 0;
    cloud.fields.push_back(field);
    field.name = "y";
    field.offset = 4;
    cloud.fields.push_back(field);
    field.name = "z";
    field.offset = 8;
    cloud.fields.push_back(field);

    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);

    for (size_t i = 0; i < points.size(); ++i) {
        float* data_ptr = reinterpret_cast<float*>(&cloud.data[i * cloud.point_step]);
        data_ptr[0] = static_cast<float>(points[i].x);
        data_ptr[1] = static_cast<float>(points[i].y);
        data_ptr[2] = static_cast<float>(points[i].z);
    }

    dynamic_cloud_pub_.publish(cloud);
    dynamic_cloud_tracker_.recordTx();
}

void DynamicObstacleDetector::publishClosestObstacle(
    const std::vector<geometry_msgs::Point>& base_points,
    const ros::Time& stamp) {
    if (base_points.empty()) {
        return;
    }

    double best_dist_sq = std::numeric_limits<double>::max();
    geometry_msgs::Point closest;
    for (const geometry_msgs::Point& point : base_points) {
        const double dist_sq = point.x * point.x + point.y * point.y;
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            closest = point;
        }
    }

    geometry_msgs::PointStamped closest_msg;
    closest_msg.header.stamp = stamp;
    closest_msg.header.frame_id = base_frame_;
    closest_msg.point = closest;
    closest_obs_pub_.publish(closest_msg);
    closest_obs_tracker_.recordTx();
}

void DynamicObstacleDetector::publishLocalMap(
    const std::vector<geometry_msgs::Point>& dynamic_points,
    const std::vector<geometry_msgs::Point>& corner_points,
    const ros::Time& stamp,
    double robot_x,
    double robot_y,
    bool localization_ready) {
    if (!localization_ready) {
        ROS_WARN_THROTTLE(2.0,
            "[DETECT-MAP] Skip /local_dynamic_map publish because localization is not ready");
        return;
    }

    nav_msgs::OccupancyGrid local_map;
    local_map.header.stamp = stamp;
    local_map.header.frame_id = "map";

    const double resolution = local_map_res_;
    const double size = local_map_size_;
    const int cells_per_side = std::max(1, static_cast<int>(std::round(size / resolution)));

    local_map.info.resolution = resolution;
    local_map.info.width = cells_per_side;
    local_map.info.height = cells_per_side;
    local_map.info.origin.position.x = robot_x - size * 0.5;
    local_map.info.origin.position.y = robot_y - size * 0.5;
    local_map.info.origin.position.z = 0.0;
    local_map.info.origin.orientation.w = 1.0;
    local_map.data.assign(cells_per_side * cells_per_side, 0);

    int static_cells = 0;
    int unknown_cells = 0;
    int dynamic_cells = 0;
    int corner_cells = 0;

    if (map_received_ && include_static_map_in_local_map_) {
        for (int y = 0; y < cells_per_side; ++y) {
            for (int x = 0; x < cells_per_side; ++x) {
                const double wx = local_map.info.origin.position.x + (x + 0.5) * resolution;
                const double wy = local_map.info.origin.position.y + (y + 0.5) * resolution;
                const int occ = getMapOccupancy(wx, wy);
                if (occ == -1 || occ > 50) {
                    local_map.data[y * cells_per_side + x] = 100;
                    if (occ == -1) {
                        ++unknown_cells;
                    } else {
                        ++static_cells;
                    }
                }
            }
        }
    }

    for (const geometry_msgs::Point& point : dynamic_points) {
        const int mx = static_cast<int>(std::floor(
            (point.x - local_map.info.origin.position.x) / resolution));
        const int my = static_cast<int>(std::floor(
            (point.y - local_map.info.origin.position.y) / resolution));
        if (mx >= 0 && my >= 0 && mx < cells_per_side && my < cells_per_side) {
            if (local_map.data[my * cells_per_side + mx] < 50) {
                ++dynamic_cells;
            }
            local_map.data[my * cells_per_side + mx] = 100;
        }
    }

    // Corner points represent static laser returns (walls).
    // Only write them to the local map when the static map layer is also included;
    // without the static layer the planner has no way to distinguish these from
    // dynamic obstacles, causing false obstacle saturation of the local map.
    if (include_static_map_in_local_map_) {
        const int dilate = std::max(0, local_map_dilate_cells_);
        for (const geometry_msgs::Point& point : corner_points) {
            const int cx = static_cast<int>(std::floor(
                (point.x - local_map.info.origin.position.x) / resolution));
            const int cy = static_cast<int>(std::floor(
                (point.y - local_map.info.origin.position.y) / resolution));

            for (int dx = -dilate; dx <= dilate; ++dx) {
                for (int dy = -dilate; dy <= dilate; ++dy) {
                    const int mx = cx + dx;
                    const int my = cy + dy;
                    if (mx >= 0 && my >= 0 && mx < cells_per_side && my < cells_per_side) {
                        if (local_map.data[my * cells_per_side + mx] < 50) {
                            ++corner_cells;
                        }
                        local_map.data[my * cells_per_side + mx] = 100;
                    }
                }
            }
        }
    }

    local_map_pub_.publish(local_map);
    local_map_tracker_.recordTx();
    last_local_map_stamp_ = stamp;
    ROS_INFO_THROTTLE(2.0,
                      "[LOCAL MAP OUT] origin=(%.2f,%.2f) robot=(%.2f,%.2f) size=%dx%d occ_static=%d occ_unknown=%d occ_dynamic=%d occ_corner=%d loc=%s",
                      local_map.info.origin.position.x,
                      local_map.info.origin.position.y,
                      robot_x,
                      robot_y,
                      cells_per_side,
                      cells_per_side,
                      static_cells,
                      unknown_cells,
                      dynamic_cells,
                      corner_cells,
                      localization_ready ? "Y" : "N");
}

void DynamicObstacleDetector::publishWarning(bool state) {
    std_msgs::Bool warning_msg;
    warning_msg.data = state;
    warn_pub_.publish(warning_msg);
    warning_tracker_.recordTx();
}

void DynamicObstacleDetector::objectAngleCallback(const std_msgs::Float32::ConstPtr& msg) {
    if (!map_received_) {
        return;
    }

    ROS_INFO_THROTTLE(1.0, "[OBJECT-IN] /object_direction=%.2fdeg x_seen=%s x=%.2f",
                      msg->data,
                      object_x_received_ ? "Y" : "N",
                      object_x_);

    if (std::fabs(msg->data) > object_fov_half_deg_) {
        ROS_WARN_THROTTLE(2.0, "[DETECT] Ignore object angle %.1fdeg outside camera FOV",
                          msg->data);
        return;
    }
    if (object_x_received_ && (ros::Time::now() - object_x_stamp_).toSec() > 1.5) {
        object_x_received_ = false;
        ROS_WARN_THROTTLE(2.0, "[DETECT] Stale object_x dropped");
    }

    geometry_msgs::Vector3Stamped info;
    info.header.stamp = ros::Time::now();
    info.header.frame_id = camera_frame_;
    info.vector.x = object_x_received_ ? object_x_ : std::numeric_limits<double>::quiet_NaN();
    info.vector.y = msg->data;
    info.vector.z = 0.0;
    object_info_pub_.publish(info);

    geometry_msgs::TransformStamped camera_to_map;
    try {
        camera_to_map = tf_buffer_.lookupTransform("map", camera_frame_,
                                                   ros::Time(0), ros::Duration(0.4));
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(2.0, "[DETECT] Camera TF unavailable: %s", ex.what());
        return;
    }

    const double camera_yaw = tf2::getYaw(camera_to_map.transform.rotation);
    const double ray_yaw = camera_yaw + msg->data * M_PI / 180.0;
    const double step = std::max(0.02, map_.info.resolution * 0.25);
    const bool has_fresh_distance =
        object_x_is_range_ &&
        object_x_received_ &&
        object_x_ >= 0.10 &&
        object_x_ <= object_max_range_ &&
        (ros::Time::now() - object_x_stamp_).toSec() <= 1.5;

    geometry_msgs::PointStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.point.x = camera_to_map.transform.translation.x;
    goal.point.y = camera_to_map.transform.translation.y;
    goal.point.z = 0.0;

    bool found = false;
    const double ray_limit = has_fresh_distance ? object_x_ : object_max_range_;
    const double ray_start = std::min(std::max(0.0, object_projection_start_range_),
                                      std::max(0.0, ray_limit - step));
    for (double distance = ray_start; distance <= ray_limit; distance += step) {
        const double wx = camera_to_map.transform.translation.x + distance * std::cos(ray_yaw);
        const double wy = camera_to_map.transform.translation.y + distance * std::sin(ray_yaw);
        const int occ = getMapOccupancy(wx, wy);
        if (occ == -1 || occ > 50) {
            break;
        }
        goal.point.x = wx;
        goal.point.y = wy;
        found = true;
    }

    if (has_fresh_distance && found) {
        const double wx = camera_to_map.transform.translation.x + object_x_ * std::cos(ray_yaw);
        const double wy = camera_to_map.transform.translation.y + object_x_ * std::sin(ray_yaw);
        const int occ = getMapOccupancy(wx, wy);
        if (occ != -1 && occ <= 50) {
            goal.point.x = wx;
            goal.point.y = wy;
        }
    }

    if (found) {
        const double dist_to_goal = std::hypot(goal.point.x - camera_to_map.transform.translation.x,
                                               goal.point.y - camera_to_map.transform.translation.y);
        if (dist_to_goal < 0.10) {
            ROS_WARN_THROTTLE(2.0, "[DETECT] Skip object_goal because projected distance is too short");
            return;
        }
        object_goal_pub_.publish(goal);
        object_goal_tracker_.recordTx();
        object_goal_received_ = true;
        ROS_INFO("[OBJECT-OUT] Published /object_goal map=(%.2f, %.2f) angle=%.1fdeg dist=%.2fm source_x=%s",
                 goal.point.x,
                 goal.point.y,
                 msg->data,
                 dist_to_goal,
                 has_fresh_distance ? "range" : "ray_limit");
    }
}

void DynamicObstacleDetector::logPipelineStats(const PipelineStats& stats) {
    const ros::Time now = ros::Time::now();
    if (last_pipeline_log_time_.toSec() == 0.0 ||
        (now - last_pipeline_log_time_).toSec() >= pipeline_log_interval_) {
        last_pipeline_log_time_ = now;
        ROS_INFO("[DETECT-PIPE] frame=%lu %s drops(angle=%.0f%% dist=%.0f%% map=%.0f%% cluster=%.0f%%)",
                 frame_count_, stats.toString().c_str(),
                 stats.angle_drop_pct, stats.dist_drop_pct,
                 stats.map_drop_pct, stats.cluster_drop_pct);
    }
}

void DynamicObstacleDetector::logTransmissionSummary() {
    const ros::Time now = ros::Time::now();
    if (last_tx_summary_time_.toSec() == 0.0 ||
        (now - last_tx_summary_time_).toSec() >= tx_summary_interval_) {
        last_tx_summary_time_ = now;
        ROS_INFO("[DETECT-TX] %s", scan_tracker_.statusStr().c_str());
        ROS_INFO("[DETECT-TX] %s", map_tracker_.statusStr().c_str());
        ROS_INFO("[DETECT-TX] %s", local_map_tracker_.statusStr().c_str());
    }
}

void DynamicObstacleDetector::publishHealthReport(const ros::Time& now) {
    int map_width = 0;
    int map_height = 0;
    double map_res = 0.0;
    if (map_received_) {
        map_width = map_.info.width;
        map_height = map_.info.height;
        map_res = map_.info.resolution;
    }

    const double up = (now - node_start_time_).toSec();
    const double local_map_age = last_local_map_stamp_.toSec() > 0.0
        ? (now - last_local_map_stamp_).toSec()
        : -1.0;

    ROS_INFO("[DETECT-HEALTH] up=%.0fs frames=%lu skipped=%lu map=%dx%d@%.3f",
             up, frame_count_, frame_skip_count_, map_width, map_height, map_res);
    ROS_INFO("[DETECT-HEALTH] loc_ready=%s map_tf_seen=%s local_map_age=%.1fs alarm=%s",
             localization_ready_ ? "Y" : "N",
             map_tf_ever_available_ ? "Y" : "N",
             local_map_age,
             alarm_active_ ? "Y" : "N");
    ROS_INFO("[DETECT-HEALTH] %s", last_stats_.toString().c_str());
    ROS_INFO("[DETECT-HEALTH] %s", scan_tracker_.statusStr().c_str());
    ROS_INFO("[DETECT-HEALTH] %s", map_tracker_.statusStr().c_str());
    ROS_INFO("[DETECT-HEALTH] %s", local_map_tracker_.statusStr().c_str());
    ROS_INFO("[DETECT-HEALTH] TF fail=%d/%d max=%.1fms avg=%.1fms",
             tf_lookup_fail_, tf_lookup_total_, tf_lookup_max_ms_, tf_lookup_avg_ms_);

    try {
        const geometry_msgs::TransformStamped map_to_base =
            tf_buffer_.lookupTransform("map", base_frame_, ros::Time(0), ros::Duration(0.05));
        const double yaw = tf2::getYaw(map_to_base.transform.rotation);
        const double cmd_age = last_cmd_vel_stamp_.toSec() > 0.0
            ? (now - last_cmd_vel_stamp_).toSec()
            : -1.0;
        const bool cmd_fresh = cmd_age >= 0.0 && cmd_age <= 1.5;
        const double cmd_v = cmd_fresh ? last_cmd_linear_ : 0.0;
        const double cmd_w = cmd_fresh ? last_cmd_angular_ : 0.0;
        std::string motion = "stop";
        if (cmd_v > 0.02) {
            motion = "forward";
        } else if (cmd_v < -0.02) {
            motion = "backward";
        } else if (cmd_w > 0.05) {
            motion = "turn_left";
        } else if (cmd_w < -0.05) {
            motion = "turn_right";
        }
        ROS_INFO("[DETECT-POSE] planner_pose x=%.3f y=%.3f yaw=%.3frad %.1fdeg",
                 map_to_base.transform.translation.x,
                 map_to_base.transform.translation.y,
                 yaw,
                 yaw * 180.0 / M_PI);
        ROS_INFO("[DETECT-MANUAL] pose x=%.3f y=%.3f yaw=%.3frad %.1fdeg motion=%s cmd_v=%.3f cmd_w=%.3f cmd_age=%.1fs",
                 map_to_base.transform.translation.x,
                 map_to_base.transform.translation.y,
                 yaw,
                 yaw * 180.0 / M_PI,
                 motion.c_str(),
                 cmd_v,
                 cmd_w,
                 cmd_age);
    } catch (const tf2::TransformException& ex) {
        ROS_WARN("[DETECT-POSE] map->%s unavailable: %s",
                 base_frame_.c_str(), ex.what());
    }
}

void DynamicObstacleDetector::logTfLookup(const std::string&,
                                          const std::string&,
                                          bool success,
                                          double duration_ms) {
    ++tf_lookup_total_;
    if (!success) {
        ++tf_lookup_fail_;
    }
    tf_lookup_max_ms_ = std::max(tf_lookup_max_ms_, duration_ms);
    tf_lookup_avg_ms_ = tf_lookup_avg_ms_ * 0.9 + duration_ms * 0.1;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detector");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    DynamicObstacleDetector detector(nh, pnh);
    ros::spin();
    return 0;
}
