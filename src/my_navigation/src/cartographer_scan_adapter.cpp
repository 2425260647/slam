#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace
{
class CartographerScanAdapter
{
public:
  CartographerScanAdapter()
    : private_nh_("~")
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/scan");
    private_nh_.param<std::string>("output_topic", output_topic_,
                                   "/scan_cartographer");
    private_nh_.param("no_return_epsilon", no_return_epsilon_, 0.10);
    no_return_epsilon_ = std::max(0.01, no_return_epsilon_);

    publisher_ = nh_.advertise<sensor_msgs::LaserScan>(output_topic_, 2);
    subscriber_ = nh_.subscribe(input_topic_, 2,
                                &CartographerScanAdapter::scanCallback, this);
    ROS_INFO("[CARTO_SCAN] input=%s output=%s no_return_epsilon=%.3f m",
             input_topic_.c_str(), output_topic_.c_str(), no_return_epsilon_);
  }

private:
  void scanCallback(const sensor_msgs::LaserScanConstPtr& input)
  {
    if (!std::isfinite(input->range_max) || input->range_max <= input->range_min)
    {
      ROS_WARN_THROTTLE(5.0,
          "[CARTO_SCAN] Invalid scan limits range_min=%.3f range_max=%.3f; dropping frame.",
          input->range_min, input->range_max);
      return;
    }

    sensor_msgs::LaserScan output = *input;
    const float original_range_max = input->range_max;
    const float no_return_value = static_cast<float>(
        static_cast<double>(original_range_max) + no_return_epsilon_);
    output.range_max = no_return_value;

    std::size_t converted = 0;
    std::size_t finite = 0;
    std::size_t invalid = 0;
    for (float& range : output.ranges)
    {
      if (std::isinf(range) && range > 0.0f)
      {
        range = no_return_value;
        ++converted;
      }
      else if (std::isfinite(range))
      {
        ++finite;
      }
      else
      {
        ++invalid;
      }
    }

    publisher_.publish(output);
    ROS_INFO_THROTTLE(5.0,
        "[CARTO_SCAN] beams=%zu finite=%zu no_return=%zu invalid=%zu sentinel=%.3f m original_max=%.3f m.",
        output.ranges.size(), finite, converted, invalid, no_return_value,
        original_range_max);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::string input_topic_;
  std::string output_topic_;
  double no_return_epsilon_;
};
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cartographer_scan_adapter");
  CartographerScanAdapter adapter;
  ros::spin();
  return 0;
}
