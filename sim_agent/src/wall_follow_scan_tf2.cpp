// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "agent_interfaces/msg/wall_scan.hpp"

#include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;

const double PI = 3.141592653589793238463;

class WallFollowScan : public rclcpp::Node
{
public:
  WallFollowScan()
  : Node("wall_follow_tf2_publisher")
  {
    wallScanSubscriber_ = this->create_subscription<agent_interfaces::msg::WallScan>(
            "/wall_scan", 10, std::bind(&WallFollowScan::WallScanCallback, this, _1));

    tfPublisherA_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tfPublisherB_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    timer_ = this->create_wall_timer(
      100ms, std::bind(&WallFollowScan::TimerCallback, this));
  }

private:
    std::vector<float> scanAngles = std::vector<float>();
    std::vector<float> scanDistances = std::vector<float>();

    void TimerCallback()
    {
        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped msgA;
        geometry_msgs::msg::TransformStamped msgB;

        // Angle A Message
        msgA.header.stamp = now;
        msgA.header.frame_id = "ego_racecar/base_link";
        msgA.child_frame_id = "wallA";
        msgA.transform.translation.x = 2.0f * sin(scanAngles[0]);
        msgA.transform.translation.y = 2.0f * cos(scanAngles[0]);
        msgA.transform.translation.z = 0.0;
        msgA.transform.rotation.x = 0.0;
        msgA.transform.rotation.y = 0.0;
        msgA.transform.rotation.z = 0.0;
        msgA.transform.rotation.w = 1.0;

        // Angle B Message
        msgB.header.stamp = now;
        msgB.header.frame_id = "ego_racecar/base_link";
        msgB.child_frame_id = "wallB";
        msgB.transform.translation.x = 2.0f * sin(scanAngles[1]);
        msgB.transform.translation.y = 2.0f * cos(scanAngles[1]);
        msgB.transform.translation.z = 0.0;
        msgB.transform.rotation.x = 0.0;
        msgB.transform.rotation.y = 0.0;
        msgB.transform.rotation.z = 0.0;
        msgB.transform.rotation.w = 1.0;

        tfPublisherA_->sendTransform(msgA);
        tfPublisherB_->sendTransform(msgB);
    }

    void WallScanCallback(const agent_interfaces::msg::WallScan::SharedPtr wallScan)
    {
        scanAngles = wallScan->angle;
        scanDistances = wallScan->distance;
        RCLCPP_INFO(this->get_logger(), "Distance A: %f, B: %f", scanDistances[0], scanDistances[1]);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<agent_interfaces::msg::WallScan>::SharedPtr wallScanSubscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfPublisherA_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfPublisherB_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollowScan>());
  rclcpp::shutdown();
  return 0;
}
