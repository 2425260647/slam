#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>

class SimEnvironment
{
public:
    SimEnvironment()
    {
        // 初始化地图参数
        map_.info.resolution = 0.05;
        map_.info.width = 200;
        map_.info.height = 200;
        map_.info.origin.position.x = -5.0;
        map_.info.origin.position.y = -5.0;
        map_.data.resize(map_.info.width * map_.info.height, 0);

        // 构建边界墙
        addWall(0, 0, 200, 1);                 // 下墙
        addWall(0, 199, 200, 1);               // 上墙
        addWall(0, 1, 1, 198);                 // 左墙
        addWall(199, 1, 1, 198);               // 右墙

        // 随机生成矩形障碍物
        srand(time(nullptr));
        generateRandomObstacles();

        // 小车初始位姿
        x_ = 0.0; y_ = 0.0; theta_ = 0.0;

        // 发布器
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 10);
        actual_path_pub_ = nh_.advertise<nav_msgs::Path>("/actual_path", 10);
        object_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/object_detected", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

        cmd_sub_ = nh_.subscribe("/cmd_vel", 10, &SimEnvironment::cmdCallback, this);

        // 发布静态地图
        map_.header.frame_id = "map";
        map_.header.stamp = ros::Time::now();
        map_pub_.publish(map_);

        // 设置物体检测：每2秒发布一次，模拟目标在相机前方
        object_timer_ = nh_.createTimer(ros::Duration(2.0), &SimEnvironment::publishObject, this);
    }

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        double dt = 0.1;
        double v = msg->linear.x;
        double w = msg->angular.z;
        theta_ += w * dt;
        x_ += v * cos(theta_) * dt;
        y_ += v * sin(theta_) * dt;
    }

    void publishTF(const ros::Time& now)
    {
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

        // map -> odom (固定)
        transformStamped.header.stamp = now;
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "odom";
        transformStamped.transform.translation.x = 0;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);

        // odom -> base_link (模拟运动)
        transformStamped.header.stamp = now;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = x_;
        transformStamped.transform.translation.y = y_;
        transformStamped.transform.translation.z = 0;
        q.setRPY(0, 0, theta_);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);

        // base_link -> camera_link (固定安装位置)
        transformStamped.header.stamp = now;
        transformStamped.header.frame_id = "base_link";
        transformStamped.child_frame_id = "camera_link";
        transformStamped.transform.translation.x = 0.32;
        transformStamped.transform.translation.y = 0;
        transformStamped.transform.translation.z = 0.3;
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        br.sendTransform(transformStamped);
    }

    void publishScan(const ros::Time& now)
    {
        sensor_msgs::LaserScan scan;
        scan.header.frame_id = "base_link";
        scan.header.stamp = now;
        scan.angle_min = -M_PI;
        scan.angle_max = M_PI;
        scan.angle_increment = M_PI/180.0;
        scan.range_min = 0.1;
        scan.range_max = 10.0;
        int num_ranges = (scan.angle_max - scan.angle_min) / scan.angle_increment + 1;
        scan.ranges.resize(num_ranges);

        for (int i = 0; i < num_ranges; ++i)
        {
            double angle = scan.angle_min + i * scan.angle_increment;
            double range = scan.range_max;
            for (double r = 0.0; r < scan.range_max; r += map_.info.resolution)
            {
                double px = x_ + r * cos(theta_ + angle);
                double py = y_ + r * sin(theta_ + angle);
                int mx = (px - map_.info.origin.position.x) / map_.info.resolution;
                int my = (py - map_.info.origin.position.y) / map_.info.resolution;
                if (mx < 0 || mx >= static_cast<int>(map_.info.width) ||
                    my < 0 || my >= static_cast<int>(map_.info.height))
                    break;
                if (map_.data[my * map_.info.width + mx] == 100)
                {
                    range = r;
                    break;
                }
            }
            scan.ranges[i] = range;
        }
        scan_pub_.publish(scan);
    }

    void publishActualPath(const ros::Time& now)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now;
        pose.pose.position.x = x_;
        pose.pose.position.y = y_;
        pose.pose.position.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        actual_path_.header.frame_id = "map";
        actual_path_.header.stamp = now;
        actual_path_.poses.push_back(pose);
        actual_path_pub_.publish(actual_path_);
    }

    void publishObject(const ros::TimerEvent&)
    {
        // 模拟目标固定在相机前方1.2米处
        geometry_msgs::PoseStamped obj;
        obj.header.frame_id = "camera_link";
        obj.header.stamp = ros::Time::now();
        obj.pose.position.x = 1.2;
        obj.pose.position.y = 0.0;
        obj.pose.position.z = 0.0;
        obj.pose.orientation.w = 1.0;
        object_pub_.publish(obj);
        // 同时发布可视化 marker（目标点）
        visualization_msgs::Marker obj_marker;
        obj_marker.header.frame_id = "camera_link";
        obj_marker.header.stamp = ros::Time::now();
        obj_marker.ns = "target";
        obj_marker.id = 0;
        obj_marker.type = visualization_msgs::Marker::SPHERE;
        obj_marker.action = visualization_msgs::Marker::ADD;
        obj_marker.pose.position = obj.pose.position;
        obj_marker.scale.x = 0.1; obj_marker.scale.y = 0.1; obj_marker.scale.z = 0.1;
        obj_marker.color.r = 1.0; obj_marker.color.g = 0.0; obj_marker.color.b = 0.0; obj_marker.color.a = 1.0;
        marker_pub_.publish(obj_marker);
    }

    void publishVisualization(const ros::Time& now)
    {
        // 小车轮廓（矩形）
        visualization_msgs::Marker robot_marker;
        robot_marker.header.frame_id = "base_link";
        robot_marker.header.stamp = now;
        robot_marker.ns = "robot";
        robot_marker.id = 0;
        robot_marker.type = visualization_msgs::Marker::CUBE;
        robot_marker.action = visualization_msgs::Marker::ADD;
        robot_marker.pose.position.x = 0.0;
        robot_marker.pose.position.y = 0.0;
        robot_marker.pose.position.z = 0.0;
        robot_marker.scale.x = 0.64;  // 长
        robot_marker.scale.y = 0.57;  // 宽
        robot_marker.scale.z = 0.56;  // 高
        robot_marker.color.r = 0.0; robot_marker.color.g = 0.5; robot_marker.color.b = 1.0; robot_marker.color.a = 0.8;
        marker_pub_.publish(robot_marker);

        // 相机视野扇形
        visualization_msgs::Marker fov_marker;
        fov_marker.header.frame_id = "camera_link";
        fov_marker.header.stamp = now;
        fov_marker.ns = "camera_fov";
        fov_marker.id = 1;
        fov_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        fov_marker.action = visualization_msgs::Marker::ADD;
        fov_marker.scale.x = 1.0; fov_marker.scale.y = 1.0; fov_marker.scale.z = 1.0;
        fov_marker.color.r = 0.0; fov_marker.color.g = 1.0; fov_marker.color.b = 0.0; fov_marker.color.a = 0.2;

        double half_fov = (70.0 / 2.0) * M_PI / 180.0;
        double range = 1.5;
        int num_segments = 20; // 扇形逼近精度
        // 原点 (0,0) 为相机位置
        geometry_msgs::Point origin;
        origin.x = 0; origin.y = 0; origin.z = 0;
        for (int i = 0; i < num_segments; ++i)
        {
            double a1 = -half_fov + i * (2 * half_fov) / num_segments;
            double a2 = -half_fov + (i + 1) * (2 * half_fov) / num_segments;
            geometry_msgs::Point p1, p2;
            p1.x = range * cos(a1); p1.y = range * sin(a1); p1.z = 0;
            p2.x = range * cos(a2); p2.y = range * sin(a2); p2.z = 0;
            fov_marker.points.push_back(origin);
            fov_marker.points.push_back(p1);
            fov_marker.points.push_back(p2);
        }
        marker_pub_.publish(fov_marker);
    }

    void spin()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            ros::Time now = ros::Time::now();
            publishTF(now);
            publishScan(now);
            publishActualPath(now);
            publishVisualization(now);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void addWall(int x, int y, int w, int h)
    {
        for (int i = y; i < y + h && i < static_cast<int>(map_.info.height); ++i)
            for (int j = x; j < x + w && j < static_cast<int>(map_.info.width); ++j)
                map_.data[i * map_.info.width + j] = 100;
    }

    void generateRandomObstacles()
    {
        // 随机生成 8~12 个矩形障碍物，尺寸随机
        int num_obs = 8 + rand() % 5;
        for (int k = 0; k < num_obs; ++k)
        {
            int w = 5 + rand() % 20;
            int h = 5 + rand() % 10;
            int x = 10 + rand() % (map_.info.width - w - 20);
            int y = 10 + rand() % (map_.info.height - h - 20);
            // 避免堵死起点附近区域 (0~2m 半径内)
            double center_x = (x + w/2.0) * map_.info.resolution + map_.info.origin.position.x;
            double center_y = (y + h/2.0) * map_.info.resolution + map_.info.origin.position.y;
            if (hypot(center_x - 0, center_y - 0) < 2.5) continue; // 起点附近不放置
            addWall(x, y, w, h);
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher map_pub_, scan_pub_, actual_path_pub_, object_pub_, marker_pub_;
    ros::Subscriber cmd_sub_;
    ros::Timer object_timer_;
    nav_msgs::OccupancyGrid map_;
    nav_msgs::Path actual_path_;
    double x_, y_, theta_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sim_env");
    SimEnvironment sim;
    sim.spin();
    return 0;
}
