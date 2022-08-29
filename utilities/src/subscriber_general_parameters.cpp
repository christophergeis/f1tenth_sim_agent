#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "agent_interfaces/msg/scan_range.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class GeneralSubscriber : public rclcpp::Node
{
    public:
    GeneralSubscriber() 
    : Node("subscriber_general")
    {
        this->declare_parameter<float>("offset", 0.4);
        this->get_parameter("offset", rangeOffset);
        this->declare_parameter<float>("verbose", true);
        this->get_parameter("verbose", runVerbose);

        //Subscriptions
        subscriptionGen_ = this->create_subscription<agent_interfaces::msg::ScanRange>(
            "/scan_range", 10, std::bind(&GeneralSubscriber::SubscriptionCallback, this, _1));

        //Create a timer that will publish our message 4 times a second
        timer_ = this->create_wall_timer(
            250ms, std::bind(&GeneralSubscriber::TimerCallback, this));
    }

    private:
    rclcpp::Subscription<agent_interfaces::msg::ScanRange>::SharedPtr subscriptionGen_;

    float rangeOffset;
    bool runVerbose;

    //Variables used in data processing
    double closestPoint;
    double farthestPoint;
    
    void SubscriptionCallback(const agent_interfaces::msg::ScanRange::SharedPtr scan)
    {
        //Set global variables
        closestPoint = scan->closest_point;
        farthestPoint = scan->farthest_point;
    }
    
    void TimerCallback()
    {
        float range = farthestPoint + rangeOffset - closestPoint; //Process data
        if(runVerbose)
            RCLCPP_INFO(this->get_logger(), ("Range Difference: %f"), range);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeneralSubscriber>());
    rclcpp::shutdown();
    return 0;
}