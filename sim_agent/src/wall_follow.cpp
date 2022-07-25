#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
//#include "agent_interfaces/msg/wall_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class WallFollow : public rclcpp::Node
{
    public:
    WallFollow() 
    : Node("wall_follow")
    {
        //Get Parameters
        this->declare_parameter<float>("kp", 0.4);
        this->get_parameter("kp", kp);
        this->declare_parameter<float>("ki", 0.0);
        this->get_parameter("ki", ki);
        this->declare_parameter<float>("kd", 0.0004);
        this->get_parameter("kd", kd);
        this->declare_parameter<float>("L", 0.8);
        this->get_parameter("L", L);
        this->declare_parameter<float>("wallDist", 0.6);
        this->get_parameter("wallDist", DESIRED_DISTANCE_LEFT);
        this->declare_parameter<float>("angleA", 55.0); // deg
        this->get_parameter("angleA", angleA_);
        this->declare_parameter<float>("angleB", 100.0); // deg
        this->get_parameter("angleB", angleB_);

        //Subscriptions
        subscriptionScan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollow::ScanCallback, this, _1));
        subscriptionBrake_ = this->create_subscription<std_msgs::msg::Bool>(
            "/brake_bool", 10, std::bind(&WallFollow::BrakeCallback, this, _1));

        //Publishers
        publisherDrive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);

        // publisherWallScan_ = this->create_publisher<agent_interfaces::msg::WallScan>(
        //     "/wall_scan", 10);

        timer_ = this->create_wall_timer(100ms, std::bind(&WallFollow::TimerCallback, this));
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisherDrive_;
    // rclcpp::Publisher<agent_interfaces::msg::WallScan>::SharedPtr publisherWallScan_;

    private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriptionScan_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriptionBrake_;
    rclcpp::TimerBase::SharedPtr timer_;

    //VARIABLES//
    const double PI = 3.141592653589793238463;
    bool brake;
    bool brakeMsgSent;
    bool startMsgSent;
    std::vector<float> ranges = std::vector<float>();


    //PID CONTROL PARAMS//
    float kp; 
    float ki; 
    float kd; 
    float servo_offset = 0.0f;
    float prevError = 0.0f;
    float error = 0.0f;
    float integral = 0.0f;
    float maxSteeringAngle = 0.6f;  // rad

    //SCAN PARAMS//
    float ANGLE_MIN;
    float ANGLE_INC;
    float ANGLE_RANGE;              // Hokuyo 10LX has 270 degrees scan
    float ANGLE_FACTOR;
    float angleA_;
    float angleB_;
    std::vector<float> SCAN_ANGLES;
       // [0]: Angle A, the larger angled scan to be used in distance measurement
       // [1]: Angle B, the smaller angled scan to be used in distance measurement
    std::vector<float> trueScanAngles;
    bool SCAN_PARAMS_SET = false;

    //WALL FOLLOW PARAMS//
    float DESIRED_DISTANCE_RIGHT = 0.9; // meters
    float DESIRED_DISTANCE_LEFT;
    float CAR_LENGTH = 0.50;  // Traxxas Rally is 20 inches or 0.5 meters long
    float CAR_WIDTH = 0.23;   // Traxxas Rally is 10.5 inches or 0.23 meters long

    float angleDeg;
    float L;
    double CURR_TIME;
    double PREV_TIME;
    int integralCounter = 0;

    //CALLBACKS//
    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        if (!SCAN_PARAMS_SET)
        {
            ANGLE_MIN = scan->angle_min;
            ANGLE_INC = scan->angle_increment;
            ANGLE_RANGE = abs(scan->angle_max - ANGLE_MIN);
            ANGLE_FACTOR = ANGLE_RANGE / (3.0f * PI / 2.0f);
            SCAN_ANGLES = {angleA_, angleB_};
            SCAN_PARAMS_SET = true;
        }
        
        ranges = scan->ranges;

        //RCLCPP_INFO(this->get_logger(), ("Distance from Right: %f"), GetRange(0.0f));

        //send error to pid_control
        PIDController(error);
    }

    void BrakeCallback(const std_msgs::msg::Bool::SharedPtr brakeMsg)
    {
        if (brakeMsg->data) brake = true;
    }

    void TimerCallback()
    {
        prevError = error;
        error = FollowLeft(DESIRED_DISTANCE_LEFT);
        //RCLCPP_INFO(this->get_logger(), ("Error = %f"), error);

        if (!startMsgSent)
        {
            RCLCPP_INFO(this->get_logger(), ("P: %f, I: %f, D: %f"), kp, ki, kd);
            RCLCPP_INFO(this->get_logger(), ("L: %f, wallDist: %f"), L, DESIRED_DISTANCE_LEFT);
            RCLCPP_INFO(this->get_logger(), ("angleA: %f, angleB: %f"), SCAN_ANGLES[0], SCAN_ANGLES[1]);
            startMsgSent = true;
        }

        //if (!brake) RCLCPP_INFO(this->get_logger(), ("Error: %f, Angle = %f"), error, angleDeg);
        if (brake && !brakeMsgSent)
        {
            RCLCPP_INFO(this->get_logger(), ("Wall Encountered! Braking!"));
            brakeMsgSent = true;
        }
    }

    //WORKER FUNCTIONS//

    // angle: between -45 to 225 degrees, where 0 degrees is directly to the right
    // Outputs length in meters to object with angle in lidar scan field of view
    std::vector<float> GetRange(float angleIn)
    {
        float angleOut;

        // make sure to take care of nans etc.
        // Adjust the input angle so that it corresponds to the starting angle of the scan
        float angleRad = DegToRad(angleIn);

        long unsigned int index = ceil((angleRad - ANGLE_MIN) / ANGLE_INC); // Get the index

        // Check that the index is within the array
        if (index >= ranges.size()) index = ranges.size() - 1;

        // Make sure that the distance being output is valid
        if (RangeIsInvalid(index))
        {
            return std::vector<float>{0.0f, angleRad};
            // long unsigned int topIndex;
            // float topAngle;
            // long unsigned int bottomIndex;
            // float bottomAngle;
            // // Find the first upper index that contains a valid distance
            // for (topIndex = index; topIndex < ranges.size() - 1; topIndex++)
            // {
            //      if (!RangeIsInvalid(topIndex))
            //         break;
            // }
            // topAngle = ANGLE_MIN + ANGLE_OFFSET + topIndex * ANGLE_FACTOR;
            

            // // Find the first lower index that contains a valid distance
            // for (bottomIndex = index; bottomIndex > 0; bottomIndex--)
            // {
            //     if (!RangeIsInvalid(bottomIndex))
            //         break;
            // }
            // bottomAngle = ANGLE_MIN + ANGLE_OFFSET + bottomIndex * ANGLE_FACTOR;

            // if (!(RangeIsInvalid(topIndex) && RangeIsInvalid(bottomIndex))) 
            // {
            //     if (std::abs(topAngle - angleIn) <= std::abs(angleIn - bottomAngle))
            //     {
            //         index = topIndex;
            //         angleOut = topAngle;
            //     }
            //     else 
            //     {
            //         index = bottomIndex;
            //         angleOut = bottomAngle;
            //     }
            // }
            // else if (!RangeIsInvalid(topIndex) && RangeIsInvalid(bottomIndex))
            // {
            //     index = topIndex;
            //     angleOut = topAngle;
            // }
            // else if (RangeIsInvalid(topIndex) && !RangeIsInvalid(bottomIndex))
            // {
            //     index = bottomIndex;
            //     angleOut = bottomAngle;
            // }
            RCLCPP_INFO(this->get_logger(), "Invalid Range Encountered!\n Range: %f, Angle out: %f", ranges[index], angleOut);
        }
        else
        {
            angleOut = angleIn;
        }

        // Check if the result is nan before returning the value
        return std::vector<float>{ranges[index], angleOut};
    }

    bool RangeIsInvalid(long unsigned int indexIn)
    {
        return (std::isnan(ranges[indexIn]) || std::isinf(ranges[indexIn]) || ranges[indexIn] >= 27.0f);
    }

    float FollowLeft(float leftDist)
    {
        std::vector<float> scanA = GetRange(SCAN_ANGLES[0]);
        std::vector<float> scanB = GetRange(SCAN_ANGLES[1]);
        std::vector<float> distances = {scanA[0], scanB[0]};
        float theta = scanB[1] - scanA[1];
        
        float alpha = atan((distances[0] * cos(theta) - distances[1]) / (distances[0] * sin(theta)));
        float dist1 = distances[1] * cos(alpha);
        float dist2 = dist1 + L * sin(alpha); // Project vector forward to account for speed

        //PublishWallScan(std::vector<float>{SCAN_ANGLES[0] - 90.0f, SCAN_ANGLES[1] - 90.0f}, distances);

        return leftDist - dist2;
    }

    float RadToDeg(float rad)
    {
        return (rad * 180.0) / PI;
    }

    float DegToRad(float deg)
    {
        return deg / 180.0 * PI;
    }

    void PIDController(float currError)
    {
        PREV_TIME = CURR_TIME;
        CURR_TIME = this->get_clock()->now().seconds();
        float deltaTime = CURR_TIME - PREV_TIME;

        integral = integralCounter++ <= 10 ? prevError * deltaTime : 0.0f;

        // Use kp, ki & kd to implement a PID controller for angle
        float angleRad = -((kp * currError) + (ki * integral) + (kd * (currError - prevError) / deltaTime));

        //Limit the output angle
        if (angleRad > maxSteeringAngle)
        {
            angleRad = maxSteeringAngle;
            RCLCPP_INFO(this->get_logger(), "Max angle reached! Output angle: %f", RadToDeg(angleRad));
        }
        else if (angleRad < -maxSteeringAngle)
        {
            angleRad = -maxSteeringAngle;
            RCLCPP_INFO(this->get_logger(), "Max angle reached! Output angle: %f", RadToDeg(angleRad));
        }
        angleDeg = RadToDeg(angleRad);
        //RCLCPP_INFO(this->get_logger(), "Steering Angle: %f", angleDeg);
        
        float velocity;
        if (abs(angleDeg) <= 10) velocity = 1.5f;
        else if (abs(angleDeg) > 10 && abs(angleDeg) <= 20) velocity = 1.0f;
        else velocity = 0.5f;

        auto driveMsg = ackermann_msgs::msg::AckermannDriveStamped();
        driveMsg.header.stamp = this->get_clock()->now();
        driveMsg.header.frame_id = "laser";
        driveMsg.drive.steering_angle = brake ? 0.0f : angleRad;
        driveMsg.drive.speed = brake ? 0.0f : velocity;

        publisherDrive_->publish(driveMsg);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollow>());
  rclcpp::shutdown();
  return 0;
}