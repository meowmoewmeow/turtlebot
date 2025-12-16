#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pose_pub_;
  int step_;

public:
  SelfDrive() : rclcpp::Node("self_drive"), step_(0)
  {
    auto lidar_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    lidar_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    auto callback = std::bind(&SelfDrive::subscribe_scan, this, std::placeholders::_1);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", lidar_qos_profile, callback);
   
    auto vel_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", vel_qos_profile);
  }

  void subscribe_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    geometry_msgs::msg::TwistStamped vel;
    vel.header.stamp = this->now();
    vel.header.frame_id = "base_link";

    calculate_command(scan, vel);

    pose_pub_->publish(vel);
    step_++;
  }

  float get_scan_avg(const sensor_msgs::msg::LaserScan::SharedPtr scan, int start_angle, int end_angle)
  {
    float sum = 0.0;
    int count = 0;
    int size = scan->ranges.size();

    for (int i = start_angle; i <= end_angle; ++i)
    {
      int idx = (i < 0) ? size + i : i;
      idx = idx % size;
      if (idx < 0) idx += size;

      float r = scan->ranges[idx];
      if (!std::isinf(r) && !std::isnan(r) && r > 0.01)
      {
        sum += r;
        count++;
      }
    }
   
    if (count == 0) return 0.0;
    return sum / count;
  }

  void calculate_command(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                         geometry_msgs::msg::TwistStamped &vel)
  {
    float front_dist = get_scan_avg(scan, -10, 10);
    float left_dist  = get_scan_avg(scan, 70, 110);
    float right_dist = get_scan_avg(scan, 250, 290);

    float max_range = 5.0;
    if (front_dist <= 0.01) front_dist = max_range;
    if (left_dist <= 0.01) left_dist = max_range;
    if (right_dist <= 0.01) right_dist = max_range;

    front_dist = std::min(front_dist, max_range);
    left_dist  = std::min(left_dist, max_range);
    right_dist = std::min(right_dist, max_range);

    if (front_dist < 0.45)
    {
      vel.twist.linear.x = 0.0;

      if (left_dist > right_dist)
        vel.twist.angular.z = 0.45;
      else
        vel.twist.angular.z = -0.45;
    }
    else
    {
      vel.twist.linear.x = 0.15;

      float error = left_dist - right_dist;
      float k_p = 1.8;

      vel.twist.angular.z = error * k_p;

      if (vel.twist.angular.z > 0.9) vel.twist.angular.z = 0.9;
      if (vel.twist.angular.z < -0.9) vel.twist.angular.z = -0.9;
    }
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
