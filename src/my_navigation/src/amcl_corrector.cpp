#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <cmath>

/**
 * @brief Scan-to-map correlation corrector.
 *        Searches a small pose neighborhood for the best scan-map alignment
 *        and injects the result as /initialpose when drift is detected.
 */
class ScanMatchingCorrector
{
public:
    ScanMatchingCorrector() : tf_listener_(tf_buffer_)
    {
        ros::NodeHandle pnh("~");
        pnh.param("laser_frame",           laser_frame_,       std::string("laser_link"));
        pnh.param("search_radius_xy",      search_radius_xy_,  0.30);
        pnh.param("search_radius_yaw_deg", search_radius_yaw_, 15.0);
        pnh.param("xy_step",               xy_step_,           0.05);
        pnh.param("yaw_step_deg",          yaw_step_deg_,      2.0);
        pnh.param("min_correction_dist",   min_corr_dist_,     0.08);
        pnh.param("min_scan_score",        min_score_,         0.25);
        pnh.param("check_interval",        check_interval_,    10.0);
        pnh.param("covariance_trigger",    cov_trigger_,       0.08);
        pnh.param("cooldown_time",         cooldown_time_,     20.0);
        pnh.param("max_correction_dist",   max_corr_dist_,     0.25);

        initialpose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
        marker_pub_      = nh_.advertise<visualization_msgs::Marker>("scan_match_marker", 1);
        scan_sub_        = nh_.subscribe("/scan",      1, &ScanMatchingCorrector::scanCb,  this);
        map_sub_         = nh_.subscribe("/map",       1, &ScanMatchingCorrector::mapCb,   this);
        amcl_sub_        = nh_.subscribe("/amcl_pose", 1, &ScanMatchingCorrector::amclCb,  this);

        ROS_INFO("[ScanMatch] Ready. xy=%.2fm yaw=%.1fdeg interval=%.1fs cov_trigger=%.4f",
                 search_radius_xy_, search_radius_yaw_, check_interval_, cov_trigger_);
    }

    void run()
    {
        ros::Rate rate(10);
        while (ros::ok()) { ros::spinOnce(); tryCorrect(); rate.sleep(); }
    }

private:
    ros::NodeHandle            nh_;
    ros::Publisher             initialpose_pub_, marker_pub_;
    ros::Subscriber            scan_sub_, map_sub_, amcl_sub_;
    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string              laser_frame_;
    nav_msgs::OccupancyGrid  map_;
    sensor_msgs::LaserScan   scan_;
    bool map_ready_       = false;
    bool scan_ready_      = false;
    bool laser_tf_ready_  = false;

    // Static laser-to-base_link offset (cached once)
    double laser_dx_   = 0.0;
    double laser_dy_   = 0.0;
    double laser_dyaw_ = 0.0;

    double amcl_cov_         = 0.0;
    ros::Time last_check_time_;
    ros::Time last_correct_time_;

    double search_radius_xy_, search_radius_yaw_, xy_step_, yaw_step_deg_;
    double min_corr_dist_, min_score_, check_interval_, cov_trigger_, cooldown_time_, max_corr_dist_;

    // ── Callbacks ──────────────────────────────────────────────────────────

    void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        map_       = *msg;
        map_ready_ = true;
        ROS_INFO_ONCE("[ScanMatch] Map: %dx%d res=%.3fm",
                      msg->info.width, msg->info.height, msg->info.resolution);
    }

    void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        scan_       = *msg;
        scan_ready_ = true;

        if (!laser_tf_ready_) {
            try {
                auto tf = tf_buffer_.lookupTransform("base_link", laser_frame_, ros::Time(0));
                laser_dx_   = tf.transform.translation.x;
                laser_dy_   = tf.transform.translation.y;
                laser_dyaw_ = tf2::getYaw(tf.transform.rotation);
                laser_tf_ready_ = true;
                ROS_INFO("[ScanMatch] Laser TF cached: dx=%.3f dy=%.3f dyaw=%.3f rad",
                         laser_dx_, laser_dy_, laser_dyaw_);
            }
            catch (const tf2::TransformException&) {}
        }
    }

    void amclCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        amcl_cov_ = std::max(msg->pose.covariance[0], msg->pose.covariance[7]);
    }

    // ── Scan-to-map scoring ─────────────────────────────────────────────────

    // Returns fraction of valid scan beams whose endpoints land on occupied map cells.
    float scorePose(double cx, double cy, double cyaw) const
    {
        const double res = map_.info.resolution;
        const int    W   = static_cast<int>(map_.info.width);
        const int    H   = static_cast<int>(map_.info.height);
        const double ox  = map_.info.origin.position.x;
        const double oy  = map_.info.origin.position.y;

        // Laser origin/orientation in map frame under candidate robot pose
        const double ccos = std::cos(cyaw);
        const double csin = std::sin(cyaw);
        const double lx   = cx + ccos * laser_dx_ - csin * laser_dy_;
        const double ly   = cy + csin * laser_dx_ + ccos * laser_dy_;
        const double lyaw = cyaw + laser_dyaw_;

        int hits = 0, total = 0;
        double angle = scan_.angle_min;
        for (size_t i = 0; i < scan_.ranges.size(); ++i, angle += scan_.angle_increment) {
            const float r = scan_.ranges[i];
            if (r < scan_.range_min || r > scan_.range_max) continue;

            const double wx = lx + r * std::cos(lyaw + angle);
            const double wy = ly + r * std::sin(lyaw + angle);
            const int mx = static_cast<int>((wx - ox) / res);
            const int my = static_cast<int>((wy - oy) / res);

            ++total;
            if (mx >= 0 && my >= 0 && mx < W && my < H && map_.data[my * W + mx] > 50)
                ++hits;
        }
        return total > 0 ? static_cast<float>(hits) / total : 0.0f;
    }

    // ── Grid search ────────────────────────────────────────────────────────

    struct Pose2D { double x, y, yaw; float score; };

    Pose2D searchBest(double ix, double iy, double iyaw) const
    {
        Pose2D best{ix, iy, iyaw, 0.0f};
        const double yr = search_radius_yaw_ * M_PI / 180.0;
        const double ys = yaw_step_deg_      * M_PI / 180.0;

        for (double dy = -search_radius_xy_; dy <= search_radius_xy_ + 1e-9; dy += xy_step_)
        for (double dx = -search_radius_xy_; dx <= search_radius_xy_ + 1e-9; dx += xy_step_)
        for (double dth = -yr; dth <= yr + 1e-9; dth += ys) {
            const float s = scorePose(ix + dx, iy + dy,
                                      angles::normalize_angle(iyaw + dth));
            if (s > best.score)
                best = {ix + dx, iy + dy, angles::normalize_angle(iyaw + dth), s};
        }
        return best;
    }

    // ── Main correction logic ──────────────────────────────────────────────

    void tryCorrect()
    {
        if (!map_ready_ || !scan_ready_ || !laser_tf_ready_) return;

        const double now          = ros::Time::now().toSec();
        const bool   timer_fired  = (now - last_check_time_.toSec())   >= check_interval_;
        const bool   cov_high     = amcl_cov_ > cov_trigger_;
        const bool   in_cooldown  = (now - last_correct_time_.toSec()) <  cooldown_time_;

        if ((!timer_fired && !cov_high) || in_cooldown) return;
        last_check_time_ = ros::Time::now();

        geometry_msgs::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
        }
        catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "[ScanMatch] TF error: %s", ex.what());
            return;
        }

        const double rx   = tf.transform.translation.x;
        const double ry   = tf.transform.translation.y;
        const double ryaw = tf2::getYaw(tf.transform.rotation);

        const Pose2D best = searchBest(rx, ry, ryaw);
        const double cd   = std::hypot(best.x - rx, best.y - ry);
        const double cyaw = std::fabs(angles::shortest_angular_distance(ryaw, best.yaw));

        ROS_INFO("[ScanMatch] score=%.3f corr_xy=%.3fm corr_yaw=%.1fdeg cov=%.4f",
                 best.score, cd, cyaw * 180.0 / M_PI, amcl_cov_);

        if (best.score < min_score_) {
            ROS_WARN_THROTTLE(5.0, "[ScanMatch] Score %.3f below threshold, skip", best.score);
            return;
        }
        if (cd > max_corr_dist_) {
            ROS_WARN_THROTTLE(5.0, "[ScanMatch] Correction %.3fm exceeds max %.3fm, skip",
                              cd, max_corr_dist_);
            return;
        }
        if (cd < min_corr_dist_ && cyaw < 5.0 * M_PI / 180.0) {
            ROS_INFO_THROTTLE(10.0, "[ScanMatch] Pose aligned, no correction needed");
            return;
        }

        publishCorrection(best);
        last_correct_time_ = ros::Time::now();
    }

    void publishCorrection(const Pose2D& p)
    {
        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.header.stamp    = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.pose.position.x    = p.x;
        pose.pose.pose.position.y    = p.y;
        pose.pose.pose.orientation.z = std::sin(p.yaw / 2.0);
        pose.pose.pose.orientation.w = std::cos(p.yaw / 2.0);
        pose.pose.covariance[0]  = 0.02;   // x std ~0.14m
        pose.pose.covariance[7]  = 0.02;   // y std ~0.14m
        pose.pose.covariance[35] = 0.01;   // yaw std ~5.7°
        initialpose_pub_.publish(pose);

        ROS_WARN("[ScanMatch] Correction: (%.3f, %.3f, %.1fdeg) score=%.3f",
                 p.x, p.y, p.yaw * 180.0 / M_PI, p.score);

        visualization_msgs::Marker m;
        m.header.frame_id    = "map";
        m.header.stamp       = ros::Time::now();
        m.ns                 = "scan_match";
        m.id                 = 0;
        m.type               = visualization_msgs::Marker::ARROW;
        m.action             = visualization_msgs::Marker::ADD;
        m.pose.position.x    = p.x;
        m.pose.position.y    = p.y;
        m.pose.orientation.z = std::sin(p.yaw / 2.0);
        m.pose.orientation.w = std::cos(p.yaw / 2.0);
        m.scale.x = 0.8; m.scale.y = 0.15; m.scale.z = 0.15;
        m.color.a = 1.0; m.color.g = 1.0;
        m.lifetime = ros::Duration(3.0);
        marker_pub_.publish(m);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "amcl_corrector");
    ScanMatchingCorrector node;
    node.run();
    return 0;
}
