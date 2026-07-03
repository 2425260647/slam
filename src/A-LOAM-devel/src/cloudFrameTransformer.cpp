#include <cmath>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CloudFrameTransformer
{
  public:
    CloudFrameTransformer()
        : private_nh_("~"), tf_listener_(tf_buffer_)
    {
        private_nh_.param<std::string>("input_topic", input_topic_, "/velodyne_points");
        private_nh_.param<std::string>("output_topic", output_topic_, "/aloam/velodyne_points_base");
        private_nh_.param<std::string>("target_frame", target_frame_, "base_link");
        private_nh_.param<bool>("prefer_tf", prefer_tf_, true);
        private_nh_.param<bool>("use_static_fallback", use_static_fallback_, true);
        private_nh_.param<double>("lookup_timeout", lookup_timeout_, 0.03);
        private_nh_.param<double>("static_x", static_x_, 0.0);
        private_nh_.param<double>("static_y", static_y_, 0.0);
        private_nh_.param<double>("static_z", static_z_, 0.17);
        private_nh_.param<double>("static_roll", static_roll_, 0.0);
        private_nh_.param<double>("static_pitch", static_pitch_, 0.0);
        private_nh_.param<double>("static_yaw", static_yaw_, -1.57079632679);

        static_tf_ = makeStaticTransform();
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 5);
        sub_ = nh_.subscribe(input_topic_, 5, &CloudFrameTransformer::cloudHandler, this);

        ROS_INFO_STREAM("A-LOAM cloud frame transformer: " << input_topic_ << " -> "
                                                          << output_topic_ << ", target_frame="
                                                          << target_frame_);
    }

  private:
    static std::string stripSlash(const std::string &frame)
    {
        if (!frame.empty() && frame[0] == '/')
        {
            return frame.substr(1);
        }
        return frame;
    }

    tf2::Transform makeStaticTransform() const
    {
        tf2::Quaternion q;
        q.setRPY(static_roll_, static_pitch_, static_yaw_);
        return tf2::Transform(q, tf2::Vector3(static_x_, static_y_, static_z_));
    }

    bool lookupTransform(const sensor_msgs::PointCloud2ConstPtr &msg, tf2::Transform *transform)
    {
        if (!prefer_tf_)
        {
            return false;
        }

        const std::string source_frame = stripSlash(msg->header.frame_id);
        const std::string target_frame = stripSlash(target_frame_);
        try
        {
            geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(
                target_frame, source_frame, msg->header.stamp, ros::Duration(lookup_timeout_));
            const geometry_msgs::Vector3 &t = tf_msg.transform.translation;
            const geometry_msgs::Quaternion &q = tf_msg.transform.rotation;
            *transform = tf2::Transform(tf2::Quaternion(q.x, q.y, q.z, q.w),
                                        tf2::Vector3(t.x, t.y, t.z));
            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(2.0, "TF lookup failed, using static fallback if enabled: %s", ex.what());
            return false;
        }
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        tf2::Transform transform;
        if (!lookupTransform(msg, &transform))
        {
            if (!use_static_fallback_)
            {
                return;
            }
            transform = static_tf_;
        }

        sensor_msgs::PointCloud2 out = *msg;
        out.header.frame_id = stripSlash(target_frame_);

        sensor_msgs::PointCloud2Iterator<float> iter_x(out, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(out, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(out, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z))
            {
                continue;
            }

            const tf2::Vector3 p = transform * tf2::Vector3(*iter_x, *iter_y, *iter_z);
            *iter_x = static_cast<float>(p.x());
            *iter_y = static_cast<float>(p.y());
            *iter_z = static_cast<float>(p.z());
        }

        pub_.publish(out);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2::Transform static_tf_;

    std::string input_topic_;
    std::string output_topic_;
    std::string target_frame_;
    bool prefer_tf_;
    bool use_static_fallback_;
    double lookup_timeout_;
    double static_x_;
    double static_y_;
    double static_z_;
    double static_roll_;
    double static_pitch_;
    double static_yaw_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aloam_cloud_frame_transformer");
    CloudFrameTransformer transformer;
    ros::spin();
    return 0;
}
