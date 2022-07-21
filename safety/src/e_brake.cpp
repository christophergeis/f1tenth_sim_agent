#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
using std::placeholders::_1;
using namespace std::chrono_literals;

class EBrake : public rclcpp::Node
{
  public:
// Current Best Settings //
// TTC Threshold = 0.35
    EBrake() 
    : Node("e_brake")
    {
      this->declare_parameter<float>("ttcThresh", 0.35);
      this->get_parameter("ttcThresh", TTCThreshold);
      
      subscriptionScan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&EBrake::ScanCallback, this, _1));
      subscriptionOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10, std::bind(&EBrake::OdomCallback, this, _1));

      publisherBrake_ = this->create_publisher<std_msgs::msg::Float32>(
        "/brake", 10);
      publisherBrakeBool_ = this->create_publisher<std_msgs::msg::Bool>(
        "/brake_bool", 10);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisherBrake_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisherBrakeBool_;

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriptionScan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriptionOdom_;

    float speed = 0.0f;
    float TTCThreshold;
    bool startMsgSent;

    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
      auto brakeMsg = std_msgs::msg::Float32();
      auto brakeBoolMsg = std_msgs::msg::Bool();

      float T = 9999.0f;  // This is the smallest value in TTC

      float rangeRate;
      float TTC;

      for (long unsigned int i = 0; i < scan->ranges.size(); i++)
      {
        // Calculate the range rate
        rangeRate = speed * std::cos(scan->angle_min + scan->angle_increment * i);

        // Calculate the next TTC
        TTC = scan->ranges[i] / (std::max(rangeRate, 0.0f));
        if (std::isinf(TTC) || std::isnan(TTC)) TTC = 9999.0f;
        // Check if the new TTC is smaller than the current smallest value
        T = (TTC < T) ? TTC : T;
      }
      
      if (T <= TTCThreshold)
      {
        brakeMsg.data = 1.0f;
        brakeBoolMsg.data = true;
      }
      else
      {
        brakeMsg.data = 0.0f;
        brakeBoolMsg.data = false;
      }

      //if (brakeBoolMsg.data == true) RCLCPP_INFO(this->get_logger(), ("TTC = %f -> BRAKING! "), T);
      publisherBrake_->publish(brakeMsg);
      publisherBrakeBool_->publish(brakeBoolMsg);
    }

    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
      if (!startMsgSent)
        {
            RCLCPP_INFO(this->get_logger(), ("ttcThresh: %f"), TTCThreshold);
            startMsgSent = true;
        }

      speed = odom->twist.twist.linear.x;
    }

    // void PublishWallScan(std::vector<float> angles, std::vector<float> distances)
    // {
    //     auto msg = agent_interfaces::msg::WallScan();
    //     std::vector<float> anglesRad {DegToRad(angles[0]), DegToRad(angles[1])};
    //     msg.angle = anglesRad;
    //     msg.distance = distances;

    //     publisherWallScan_->publish(msg);
    // }

    float TimeToCollision(sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
      float T;  // This is the smallest value in TTC

      float rangeRate;
      float TTC;

      for (long unsigned int i = 0; i < scan->ranges.size(); i++)
      {
        // Calculate the range rate
        rangeRate = speed * std::cos(scan->angle_min + scan->angle_increment * i);

        // Calculate the next TTC
        // If the range rate is 0 or above, set TTC to max value to avoid inf errors
        TTC = scan->ranges[i] / (std::max(rangeRate, 0.0f));

        // Check if the new TTC is smaller than the current smallest value
        T = (TTC < T) ? TTC : T;
      }
      return T; // Return the smallest TTC found
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EBrake>());
  rclcpp::shutdown();
  return 0;
}