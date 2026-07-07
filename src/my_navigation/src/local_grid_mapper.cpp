#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class LocalGridMapper {
public:
    LocalGridMapper(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_) {
        pnh_.param<std::string>("scan_topic", scan_topic_, "/scan");
        pnh_.param<std::string>("output_topic", output_topic_, "/local_occupancy_grid");
        pnh_.param<std::string>("base_frame", base_frame_, "base_link");
        pnh_.param<double>("map_size", map_size_, 5.0);
        pnh_.param<double>("resolution", resolution_, 0.05);
        pnh_.param<double>("tf_timeout", tf_timeout_, 0.10);
        pnh_.param<double>("max_range", max_range_, 8.0);
        pnh_.param<int>("occupied_value", occupied_value_, 100);
        pnh_.param<int>("free_value", free_value_, 0);
        pnh_.param<int>("unknown_value", unknown_value_, -1);
        pnh_.param<int>("obstacle_dilation_cells", obstacle_dilation_cells_, 0);

        map_size_ = std::max(1.0, map_size_);
        resolution_ = std::max(0.01, resolution_);
        width_ = static_cast<int>(std::ceil(map_size_ / resolution_));
        if (width_ % 2 == 0) {
            ++width_;
        }
        height_ = width_;
        origin_x_ = -0.5 * width_ * resolution_;
        origin_y_ = -0.5 * height_ * resolution_;

        grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(output_topic_, 1);
        scan_sub_ = nh_.subscribe(scan_topic_, 5, &LocalGridMapper::scanCallback, this);

        ROS_INFO("[LOCAL_GRID] scan=%s output=%s frame=%s size=%.2fm res=%.3fm cells=%dx%d",
                 scan_topic_.c_str(), output_topic_.c_str(), base_frame_.c_str(),
                 map_size_, resolution_, width_, height_);
    }

private:
    bool worldToCell(double x, double y, int& mx, int& my) const {
        mx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
        my = static_cast<int>(std::floor((y - origin_y_) / resolution_));
        return mx >= 0 && mx < width_ && my >= 0 && my < height_;
    }

    int index(int mx, int my) const {
        return my * width_ + mx;
    }

    void setCell(std::vector<int8_t>& data, int mx, int my, int value) const {
        if (mx < 0 || mx >= width_ || my < 0 || my >= height_) {
            return;
        }
        data[index(mx, my)] = static_cast<int8_t>(value);
    }

    bool clipSegmentToMap(double x0, double y0, double x1, double y1,
                          double& clipped_x, double& clipped_y) const {
        const double min_x = origin_x_;
        const double max_x = origin_x_ + width_ * resolution_ - 1e-6;
        const double min_y = origin_y_;
        const double max_y = origin_y_ + height_ * resolution_ - 1e-6;
        const double dx = x1 - x0;
        const double dy = y1 - y0;
        double t0 = 0.0;
        double t1 = 1.0;

        const auto clip_edge = [&](double p, double q) -> bool {
            if (std::abs(p) < 1e-9) {
                return q >= 0.0;
            }
            const double r = q / p;
            if (p < 0.0) {
                if (r > t1) {
                    return false;
                }
                if (r > t0) {
                    t0 = r;
                }
            } else {
                if (r < t0) {
                    return false;
                }
                if (r < t1) {
                    t1 = r;
                }
            }
            return true;
        };

        if (!clip_edge(-dx, x0 - min_x) ||
            !clip_edge(dx, max_x - x0) ||
            !clip_edge(-dy, y0 - min_y) ||
            !clip_edge(dy, max_y - y0)) {
            return false;
        }

        clipped_x = x0 + t1 * dx;
        clipped_y = y0 + t1 * dy;
        return true;
    }

    void traceFree(std::vector<int8_t>& data, int x0, int y0, int x1, int y1) const {
        const int dx = std::abs(x1 - x0);
        const int sx = x0 < x1 ? 1 : -1;
        const int dy = -std::abs(y1 - y0);
        const int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        int x = x0;
        int y = y0;

        while (true) {
            if (x == x1 && y == y1) {
                break;
            }
            setCell(data, x, y, free_value_);
            const int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y += sy;
            }
        }
    }

    void dilateOccupied(std::vector<int8_t>& data, int mx, int my) const {
        if (obstacle_dilation_cells_ <= 0) {
            setCell(data, mx, my, occupied_value_);
            return;
        }
        for (int dy = -obstacle_dilation_cells_; dy <= obstacle_dilation_cells_; ++dy) {
            for (int dx = -obstacle_dilation_cells_; dx <= obstacle_dilation_cells_; ++dx) {
                if (dx * dx + dy * dy <= obstacle_dilation_cells_ * obstacle_dilation_cells_) {
                    setCell(data, mx + dx, my + dy, occupied_value_);
                }
            }
        }
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        geometry_msgs::TransformStamped laser_to_base;
        try {
            laser_to_base = tf_buffer_.lookupTransform(
                base_frame_, scan->header.frame_id, scan->header.stamp, ros::Duration(tf_timeout_));
        } catch (const tf2::TransformException& ex) {
            try {
                laser_to_base = tf_buffer_.lookupTransform(
                    base_frame_, scan->header.frame_id, ros::Time(0), ros::Duration(tf_timeout_));
                ROS_WARN_THROTTLE(2.0,
                                  "[LOCAL_GRID] TF at scan stamp unavailable, using latest %s->%s: %s",
                                  base_frame_.c_str(), scan->header.frame_id.c_str(), ex.what());
            } catch (const tf2::TransformException& latest_ex) {
                ROS_WARN_THROTTLE(2.0, "[LOCAL_GRID] TF failed %s->%s: %s | latest: %s",
                                  base_frame_.c_str(), scan->header.frame_id.c_str(),
                                  ex.what(), latest_ex.what());
                return;
            }
        }

        geometry_msgs::PointStamped laser_origin;
        laser_origin.header = scan->header;
        laser_origin.point.x = 0.0;
        laser_origin.point.y = 0.0;
        laser_origin.point.z = 0.0;

        geometry_msgs::PointStamped base_origin;
        tf2::doTransform(laser_origin, base_origin, laser_to_base);

        int origin_mx = 0;
        int origin_my = 0;
        if (!worldToCell(base_origin.point.x, base_origin.point.y, origin_mx, origin_my)) {
            origin_mx = width_ / 2;
            origin_my = height_ / 2;
        }

        nav_msgs::OccupancyGrid grid;
        grid.header.stamp = scan->header.stamp;
        grid.header.frame_id = base_frame_;
        grid.info.resolution = resolution_;
        grid.info.width = width_;
        grid.info.height = height_;
        grid.info.origin.position.x = origin_x_;
        grid.info.origin.position.y = origin_y_;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(width_ * height_, static_cast<int8_t>(unknown_value_));

        double angle = scan->angle_min;
        for (const float raw_range : scan->ranges) {
            const bool finite = std::isfinite(raw_range);
            const double range_min = std::max(static_cast<double>(scan->range_min), 0.01);
            const double usable_max_range =
                std::min(max_range_, scan->range_max > 0.0 ? static_cast<double>(scan->range_max) : max_range_);

            if (!finite && !std::isinf(raw_range)) {
                angle += scan->angle_increment;
                continue;
            }

            double range = finite ? static_cast<double>(raw_range) : usable_max_range;
            if (range < range_min) {
                angle += scan->angle_increment;
                continue;
            }

            const bool hit = finite && range <= usable_max_range;
            range = std::min(range, usable_max_range);

            geometry_msgs::PointStamped laser_point;
            laser_point.header = scan->header;
            laser_point.point.x = range * std::cos(angle);
            laser_point.point.y = range * std::sin(angle);
            laser_point.point.z = 0.0;

            geometry_msgs::PointStamped base_point;
            tf2::doTransform(laser_point, base_point, laser_to_base);

            int end_mx = 0;
            int end_my = 0;
            const bool endpoint_inside =
                worldToCell(base_point.point.x, base_point.point.y, end_mx, end_my);

            if (!endpoint_inside) {
                double clipped_x = 0.0;
                double clipped_y = 0.0;
                if (!clipSegmentToMap(base_origin.point.x, base_origin.point.y,
                                      base_point.point.x, base_point.point.y,
                                      clipped_x, clipped_y) ||
                    !worldToCell(clipped_x, clipped_y, end_mx, end_my)) {
                    angle += scan->angle_increment;
                    continue;
                }
            }

            if (origin_mx == end_mx && origin_my == end_my) {
                angle += scan->angle_increment;
                continue;
            }

            traceFree(grid.data, origin_mx, origin_my, end_mx, end_my);
            if (hit && endpoint_inside) {
                dilateOccupied(grid.data, end_mx, end_my);
            } else {
                setCell(grid.data, end_mx, end_my, free_value_);
            }

            angle += scan->angle_increment;
        }

        setCell(grid.data, width_ / 2, height_ / 2, free_value_);
        grid_pub_.publish(grid);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber scan_sub_;
    ros::Publisher grid_pub_;

    std::string scan_topic_;
    std::string output_topic_;
    std::string base_frame_;
    double map_size_;
    double resolution_;
    double tf_timeout_;
    double max_range_;
    double origin_x_;
    double origin_y_;
    int width_;
    int height_;
    int occupied_value_;
    int free_value_;
    int unknown_value_;
    int obstacle_dilation_cells_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_grid_mapper");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    LocalGridMapper mapper(nh, pnh);
    ros::spin();
    return 0;
}
