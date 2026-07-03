#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <cmath>
#include <string>

class AloamPoseBridge {
public:
    AloamPoseBridge()
        : pnh_("~")
        , tf_listener_(tf_buffer_) {
        pnh_.param<std::string>("input_odom_topic", input_odom_topic_,
                                "/aft_mapped_to_init_high_frec");
        pnh_.param<std::string>("pose_topic", pose_topic_, "/aloam_pose");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/aloam_base_odom");
        pnh_.param<std::string>("map_frame", map_frame_, "map");
        pnh_.param<std::string>("base_frame", base_frame_, "base_link");
        pnh_.param<std::string>("laser_frame", laser_frame_, "laser_link");
        pnh_.param<double>("map_to_aloam_x", map_to_aloam_x_, 0.0);
        pnh_.param<double>("map_to_aloam_y", map_to_aloam_y_, 0.0);
        pnh_.param<double>("map_to_aloam_z", map_to_aloam_z_, 0.0);
        pnh_.param<double>("map_to_aloam_yaw", map_to_aloam_yaw_, 0.0);
        pnh_.param<double>("fallback_laser_x", fallback_laser_x_, 0.0);
        pnh_.param<double>("fallback_laser_y", fallback_laser_y_, 0.0);
        pnh_.param<double>("fallback_laser_z", fallback_laser_z_, 0.4);
        pnh_.param<double>("fallback_laser_yaw", fallback_laser_yaw_, -1.5708);
        pnh_.param<double>("pose_cov_xy", pose_cov_xy_, 0.04);
        pnh_.param<double>("pose_cov_yaw", pose_cov_yaw_, 0.02);

        tf2::Quaternion q_map_aloam;
        q_map_aloam.setRPY(0.0, 0.0, map_to_aloam_yaw_);
        map_T_aloam_.setOrigin(tf2::Vector3(map_to_aloam_x_, map_to_aloam_y_,
                                            map_to_aloam_z_));
        map_T_aloam_.setRotation(q_map_aloam);

        tf2::Quaternion q_base_laser;
        q_base_laser.setRPY(0.0, 0.0, fallback_laser_yaw_);
        base_T_laser_.setOrigin(tf2::Vector3(fallback_laser_x_, fallback_laser_y_,
                                             fallback_laser_z_));
        base_T_laser_.setRotation(q_base_laser);

        pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_, 5);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 5);
        odom_sub_ = nh_.subscribe(input_odom_topic_, 20, &AloamPoseBridge::odomCallback, this);

        ROS_INFO("[ALOAM-BRIDGE] input=%s pose=%s odom=%s frames %s->%s via %s",
                 input_odom_topic_.c_str(), pose_topic_.c_str(), odom_topic_.c_str(),
                 map_frame_.c_str(), base_frame_.c_str(), laser_frame_.c_str());
    }

private:
    void refreshLaserExtrinsic() {
        try {
            const geometry_msgs::TransformStamped tf =
                tf_buffer_.lookupTransform(base_frame_, laser_frame_, ros::Time(0),
                                           ros::Duration(0.02));
            tf2::fromMsg(tf.transform, base_T_laser_);
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0,
                              "[ALOAM-BRIDGE] Using fallback %s->%s extrinsic: %s",
                              base_frame_.c_str(), laser_frame_.c_str(), ex.what());
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        refreshLaserExtrinsic();

        tf2::Transform aloam_T_laser;
        tf2::Quaternion q_aloam_laser;
        tf2::fromMsg(msg->pose.pose.orientation, q_aloam_laser);
        aloam_T_laser.setOrigin(tf2::Vector3(msg->pose.pose.position.x,
                                             msg->pose.pose.position.y,
                                             msg->pose.pose.position.z));
        aloam_T_laser.setRotation(q_aloam_laser);
        const tf2::Transform map_T_base =
            map_T_aloam_ * aloam_T_laser * base_T_laser_.inverse();

        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = map_frame_;
        tf_msg.child_frame_id = base_frame_;
        tf_msg.transform = tf2::toMsg(map_T_base);
        tf_broadcaster_.sendTransform(tf_msg);

        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = tf_msg.header;
        pose_msg.pose.pose.position.x = map_T_base.getOrigin().x();
        pose_msg.pose.pose.position.y = map_T_base.getOrigin().y();
        pose_msg.pose.pose.position.z = map_T_base.getOrigin().z();
        pose_msg.pose.pose.orientation = tf2::toMsg(map_T_base.getRotation());
        pose_msg.pose.covariance[0] = pose_cov_xy_;
        pose_msg.pose.covariance[7] = pose_cov_xy_;
        pose_msg.pose.covariance[14] = pose_cov_xy_;
        pose_msg.pose.covariance[21] = 0.01;
        pose_msg.pose.covariance[28] = 0.01;
        pose_msg.pose.covariance[35] = pose_cov_yaw_;
        pose_pub_.publish(pose_msg);

        nav_msgs::Odometry odom_msg;
        odom_msg.header = tf_msg.header;
        odom_msg.child_frame_id = base_frame_;
        odom_msg.pose = pose_msg.pose;
        odom_msg.twist = msg->twist;
        odom_pub_.publish(odom_msg);

        const double yaw = tf2::getYaw(pose_msg.pose.pose.orientation);
        ROS_INFO_THROTTLE(2.0,
                          "[ALOAM-BRIDGE] base pose x=%.3f y=%.3f yaw=%.1fdeg",
                          pose_msg.pose.pose.position.x,
                          pose_msg.pose.pose.position.y,
                          yaw * 180.0 / M_PI);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber odom_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher odom_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string input_odom_topic_;
    std::string pose_topic_;
    std::string odom_topic_;
    std::string map_frame_;
    std::string base_frame_;
    std::string laser_frame_;
    double map_to_aloam_x_;
    double map_to_aloam_y_;
    double map_to_aloam_z_;
    double map_to_aloam_yaw_;
    double fallback_laser_x_;
    double fallback_laser_y_;
    double fallback_laser_z_;
    double fallback_laser_yaw_;
    double pose_cov_xy_;
    double pose_cov_yaw_;
    tf2::Transform map_T_aloam_;
    tf2::Transform base_T_laser_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aloam_pose_bridge");
    AloamPoseBridge bridge;
    ros::spin();
    return 0;
}
