#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include "agent_interfaces/msg/scan_range.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class LidarProcessing : public rclcpp::Node
{
  public:
    LidarProcessing() 
    : Node("lidar_processing")
    {
        subscriptionScan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarProcessing::ScanCallback, this, _1));

        publisherFarthest_ = this->create_publisher<std_msgs::msg::Float64>(
            "/closest_point", 10);
        publisherClosest_ = this->create_publisher<std_msgs::msg::Float64>(
            "/farthest_point", 10);
        publisherRange_ = this->create_publisher<agent_interfaces::msg::ScanRange>(
            "/scan_range", 10);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisherFarthest_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisherClosest_;
    rclcpp::Publisher<agent_interfaces::msg::ScanRange>::SharedPtr publisherRange_;

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriptionScan_;

    float MAX_RANGE = 28.0f;

    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        auto closestMsg = std_msgs::msg::Float64();
        auto farthestMsg = std_msgs::msg::Float64();
        auto rangeMsg = agent_interfaces::msg::ScanRange();
        closestMsg.data = 9999.0f;
        farthestMsg.data = 0.0f;

        for (long unsigned int i = 0; i < scan->ranges.size(); i++)
        {
            if (std::isnan(scan->ranges[i]) || std::isinf(scan->ranges[i])) continue;

            float currScan = scan->ranges[i];

            if (currScan < closestMsg.data) closestMsg.data = currScan;
            else if (currScan > farthestMsg.data && currScan <= MAX_RANGE) farthestMsg.data = currScan;
        }

        rangeMsg.closest_point = closestMsg.data;
        rangeMsg.farthest_point = farthestMsg.data;

        RCLCPP_INFO(this->get_logger(), ("Closest: %f, Farthest: %f"), rangeMsg.closest_point, rangeMsg.farthest_point);

        publisherClosest_->publish(closestMsg);
        publisherFarthest_->publish(farthestMsg);
        publisherRange_->publish(rangeMsg);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarProcessing>());
  rclcpp::shutdown();
  return 0;
}