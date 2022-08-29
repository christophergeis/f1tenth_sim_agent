#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "agent_interfaces/msg/scan_range.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class ClassName : public rclcpp::Node
{
    public:
    ClassName() 
    : Node("node_name")
    {
        this->declare_parameter<float>("number", 0.4);
        this->get_parameter("number", num);
        this->declare_parameter<float>("boolInput", true);
        this->get_parameter("boolInput", boolIn);
    }

    private:
    float num;
    bool boolIn;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GeneralSubscriber>());
    rclcpp::shutdown();
    return 0;
}

