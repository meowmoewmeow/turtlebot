#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pose_pub_;
  int step_;

  const float ROBOT_WIDTH = 0.25f;
  const float CORRIDOR_WIDTH = 0.4f;
  const float TARGET_SPEED = 0.15f;
  const float OBSTACLE_DISTANCE = 0.35f;
  const float TURN_SPEED = 0.5f;

public:
  SelfDrive() : rclcpp::Node("self_drive"), step_(0)
  {
    auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", lidar_qos_profile, callback);
    
    auto vel_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", vel_qos_profile);
  }

  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    float front_dist = get_scan_min(scan, 0, 10) < get_scan_min(scan, 350, 360) 
                       ? get_scan_min(scan, 0, 10) : get_scan_min(scan, 350, 360);
    float left_dist = get_scan_min(scan, 80, 100);
    float right_dist = get_scan_min(scan, 260, 280);

    geometry_msgs::msg::Twist vel;

    if (front_dist < OBSTACLE_DISTANCE)
    {
      handle_corner(vel, left_dist, right_dist);
    }
    else
    {
      follow_lane(vel, left_dist, right_dist);
    }

    pose_pub_->publish(vel);
    step_++;
  }

private:
  float get_scan_min(const sensor_msgs::msg::LaserScan::SharedPtr scan, int start_angle, int end_angle)
  {
    float min_dist = std::numeric_limits<float>::infinity();
    int size = scan->ranges.size();

    for (int i = start_angle; i < end_angle; ++i)
    {
      int idx = i % size;
      if (!std::isinf(scan->ranges[idx]) && scan->ranges[idx] > 0.01)
      {
        if (scan->ranges[idx] < min_dist)
        {
          min_dist = scan->ranges[idx];
        }
      }
    }
    return min_dist;
  }

  void handle_corner(geometry_msgs::msg::Twist &vel, float left_dist, float right_dist)
  {
    vel.linear.x = 0.0;
    
    if (left_dist > right_dist)
    {
      vel.angular.z = TURN_SPEED;
    }
    else
    {
      vel.angular.z = -TURN_SPEED;
    }
  }

  void follow_lane(geometry_msgs::msg::Twist &vel, float left_dist, float right_dist)
  {
    vel.linear.x = TARGET_SPEED;

    float error = left_dist - right_dist;
    float kp = 1.5;

    if (std::isinf(left_dist)) left_dist = 2.0;
    if (std::isinf(right_dist)) right_dist = 2.0;

    vel.angular.z = kp * error;
    
    float max_ang = 1.0;
    if (vel.angular.z > max_ang) vel.angular.z = max_ang;
    if (vel.angular.z < -max_ang) vel.angular.z = -max_ang;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfDrive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
