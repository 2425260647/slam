#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace nav_driver
{

class CmdVelArbiter
{
public:
  CmdVelArbiter()
      : nh_(), private_nh_("~"),
        have_move_base_(false), have_navigation_(false),
        have_safety_(false), safety_stop_(false),
        allow_move_base_reverse_(false),
        last_output_source_("none")
  {
    private_nh_.param<std::string>("move_base_cmd_vel_topic", move_base_topic_,
                                   "/move_base/cmd_vel");
    private_nh_.param<std::string>("navigation_cmd_vel_topic", navigation_topic_,
                                   "/navigation_driver/cmd_vel");
    private_nh_.param<std::string>("output_cmd_vel_topic", output_topic_, "/cmd_vel");
    private_nh_.param<std::string>("safety_stop_topic", safety_topic_,
                                   "/navigation_driver/safety_stop");
    private_nh_.param("move_base_timeout", move_base_timeout_, 0.50);
    private_nh_.param("navigation_timeout", navigation_timeout_, 0.60);
    private_nh_.param("safety_timeout", safety_timeout_, 1.50);
    private_nh_.param("publish_frequency", publish_frequency_, 30.0);
    private_nh_.param("max_linear_x", max_linear_x_, 0.60);
    private_nh_.param("max_linear_y", max_linear_y_, 0.60);
    private_nh_.param("max_angular_z", max_angular_z_, 1.20);
    private_nh_.param("allow_move_base_reverse", allow_move_base_reverse_, false);

    move_base_timeout_ = std::max(0.05, move_base_timeout_);
    navigation_timeout_ = std::max(0.05, navigation_timeout_);
    safety_timeout_ = std::max(0.0, safety_timeout_);
    publish_frequency_ = std::max(1.0, publish_frequency_);
    max_linear_x_ = std::max(0.0, max_linear_x_);
    max_linear_y_ = std::max(0.0, max_linear_y_);
    max_angular_z_ = std::max(0.0, max_angular_z_);

    move_base_sub_ = nh_.subscribe(move_base_topic_, 10,
                                   &CmdVelArbiter::moveBaseCallback, this);
    navigation_sub_ = nh_.subscribe(navigation_topic_, 10,
                                     &CmdVelArbiter::navigationCallback, this);
    safety_sub_ = nh_.subscribe(safety_topic_, 5,
                                &CmdVelArbiter::safetyCallback, this);
    output_pub_ = nh_.advertise<geometry_msgs::Twist>(output_topic_, 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_frequency_),
                             &CmdVelArbiter::publishCallback, this);

    ROS_INFO("[CMD_ARB] move_base=%s navigation=%s output=%s safety=%s",
             move_base_topic_.c_str(), navigation_topic_.c_str(),
             output_topic_.c_str(), safety_topic_.c_str());
    ROS_INFO("[CMD_ARB] timeouts: move_base=%.2f s navigation=%.2f s safety=%.2f s; rate=%.1f Hz",
             move_base_timeout_, navigation_timeout_, safety_timeout_,
             publish_frequency_);
    ROS_INFO("[CMD_ARB] move_base reverse commands: %s",
             allow_move_base_reverse_ ? "allowed" : "blocked");
  }

private:
  static bool finite(const geometry_msgs::Twist& cmd)
  {
    return std::isfinite(cmd.linear.x) && std::isfinite(cmd.linear.y) &&
           std::isfinite(cmd.linear.z) && std::isfinite(cmd.angular.x) &&
           std::isfinite(cmd.angular.y) && std::isfinite(cmd.angular.z);
  }

  geometry_msgs::Twist sanitize(const geometry_msgs::Twist& input) const
  {
    geometry_msgs::Twist output = input;
    if (!finite(output))
      return geometry_msgs::Twist();

    if (max_linear_x_ > 0.0)
      output.linear.x = std::max(-max_linear_x_, std::min(max_linear_x_, output.linear.x));
    if (max_linear_y_ > 0.0)
      output.linear.y = std::max(-max_linear_y_, std::min(max_linear_y_, output.linear.y));
    if (max_angular_z_ > 0.0)
      output.angular.z = std::max(-max_angular_z_, std::min(max_angular_z_, output.angular.z));

    // Scout Mini is non-holonomic: reject lateral and vertical/roll/pitch motion.
    output.linear.y = 0.0;
    output.linear.z = 0.0;
    output.angular.x = 0.0;
    output.angular.y = 0.0;
    return output;
  }

  void moveBaseCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    move_base_cmd_ = sanitize(*msg);
    if (!allow_move_base_reverse_ && move_base_cmd_.linear.x < 0.0)
    {
      ROS_WARN_THROTTLE(5.0,
          "[CMD_ARB] Blocking move_base reverse command %.3f m/s; "
          "navigation_driver recovery reverse remains available.",
          move_base_cmd_.linear.x);
      move_base_cmd_.linear.x = 0.0;
    }
    move_base_time_ = ros::Time::now();
    have_move_base_ = finite(*msg);
  }

  void navigationCallback(const geometry_msgs::Twist::ConstPtr& msg)
  {
    navigation_cmd_ = sanitize(*msg);
    navigation_time_ = ros::Time::now();
    have_navigation_ = finite(*msg);
  }

  void safetyCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    safety_stop_ = msg->data;
    safety_time_ = ros::Time::now();
    have_safety_ = true;
    if (safety_stop_)
      publishZeroNow("safety_stop");
  }

  bool fresh(const ros::Time& stamp, double timeout) const
  {
    if (stamp.isZero()) return false;
    const double age = (ros::Time::now() - stamp).toSec();
    return age >= 0.0 && age <= timeout;
  }

  void publishZeroNow(const std::string& source)
  {
    output_pub_.publish(geometry_msgs::Twist());
    if (source != last_output_source_)
    {
      ROS_WARN("[CMD_ARB] output source: %s", source.c_str());
      last_output_source_ = source;
    }
  }

  void publishCallback(const ros::TimerEvent&)
  {
    const bool safety_fresh = have_safety_ &&
        (safety_timeout_ <= 0.0 || fresh(safety_time_, safety_timeout_));
    if (safety_stop_ || !safety_fresh)
    {
      publishZeroNow(safety_stop_ ? "safety_stop" : "safety_timeout");
      return;
    }

    if (have_navigation_ && fresh(navigation_time_, navigation_timeout_))
    {
      output_pub_.publish(navigation_cmd_);
      if (last_output_source_ != "navigation_driver")
      {
        ROS_INFO("[CMD_ARB] output source: navigation_driver (override)");
        last_output_source_ = "navigation_driver";
      }
      return;
    }

    if (have_move_base_ && fresh(move_base_time_, move_base_timeout_))
    {
      output_pub_.publish(move_base_cmd_);
      if (last_output_source_ != "move_base")
      {
        ROS_INFO("[CMD_ARB] output source: move_base");
        last_output_source_ = "move_base";
      }
      return;
    }

    publishZeroNow("input_timeout");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber move_base_sub_;
  ros::Subscriber navigation_sub_;
  ros::Subscriber safety_sub_;
  ros::Publisher output_pub_;
  ros::Timer timer_;

  std::string move_base_topic_;
  std::string navigation_topic_;
  std::string output_topic_;
  std::string safety_topic_;
  double move_base_timeout_;
  double navigation_timeout_;
  double safety_timeout_;
  double publish_frequency_;
  double max_linear_x_;
  double max_linear_y_;
  double max_angular_z_;

  geometry_msgs::Twist move_base_cmd_;
  geometry_msgs::Twist navigation_cmd_;
  ros::Time move_base_time_;
  ros::Time navigation_time_;
  ros::Time safety_time_;
  bool have_move_base_;
  bool have_navigation_;
  bool have_safety_;
  bool safety_stop_;
  bool allow_move_base_reverse_;
  std::string last_output_source_;
};

}  // namespace nav_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmd_vel_arbiter");
  nav_driver::CmdVelArbiter arbiter;
  ros::spin();
  return 0;
}
