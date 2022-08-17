#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "std_msgs/msg/float64.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

class EBrake : public rclcpp::Node
{
  public:
    EBrake() 
    : Node("e_brake")
    {
        //Subscriptions
        subscriptionScan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarProcessing::ScanCallback, this, _1));

        //Publishers
        publisherFarthest_ = this->create_publisher<std_msgs::msg::Float64>(
            "/closest_point", 10);
        publisherClosest_ = this->create_publisher<std_msgs::msg::Float64>(
            "/farthest_point", 10);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisherFarthest_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisherClosest_;

    private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriptionScan_;

    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        auto closestMsg = std_msgs::msg::Float64();
        auto farthestMsg = std_msgs::msg::Float64();
        closestMsg.data = 9999.0f;
        farthestMsg.data = 0.0f;

        // Iterate through the ranges of the scan to find the minimum and maximum distance
        for (long unsigned int i = 0; i < scan->ranges.size(); i++)
        {
            // Ignore nan and inf numbers
            if (std::isnan(scan->ranges[i]) || std::isinf(scan->ranges[i])) continue;

            float currScan = scan->ranges[i];

            // Set the message data if applicable
            if (currScan < closestMsg.data) 
              closestMsg.data = currScan;
            else if (currScan > farthestMsg.data && currScan <= scan->range_max - 2.0f) 
              farthestMsg.data = currScan;
        }

        // Publish the messages
        publisherClosest_->publish(closestMsg);
        publisherFarthest_->publish(farthestMsg);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EBrake>());
  rclcpp::shutdown();
  return 0;
}