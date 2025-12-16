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

using namespace std::chrono_literals;

class SelfDrive : public rclcpp::Node
{
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pose_pub_;

public:
  SelfDrive() : rclcpp::Node("self_drive")
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
  }

  float get_scan_avg(const sensor_msgs::msg::LaserScan::SharedPtr scan,
                     int start_angle, int end_angle)
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
    float front_center = get_scan_avg(scan, -10, 10);

    float left_area  = get_scan_avg(scan, 10, 50);
    float right_area = get_scan_avg(scan, -50, -10);

    float left_side  = get_scan_avg(scan, 85, 95);
    float right_side = get_scan_avg(scan, -95, -85);

    float max_range = 4.0;

    if (front_center <= 0.01) front_center = max_range;
    if (left_area <= 0.01) left_area = max_range;
    if (right_area <= 0.01) right_area = max_range;
    if (left_side <= 0.01) left_side = max_range;
    if (right_side <= 0.01) right_side = max_range;

    front_center = std::min(front_center, max_range);
    left_area = std::min(left_area, max_range);
    right_area = std::min(right_area, max_range);
    left_side = std::min(left_side, max_range);
    right_side = std::min(right_side, max_range);

    if (front_center < 0.45)
    {
      vel.twist.linear.x = 0.0;

      if (left_area > right_area)
        vel.twist.angular.z = 0.5;
      else
        vel.twist.angular.z = -0.5;
    }
    else
    {
      vel.twist.linear.x = 0.15;

      float error = left_area - right_area;
      float k_p = 1.6;

      vel.twist.angular.z = error * k_p;

      float max_turn = 0.8;
      if (vel.twist.angular.z > max_turn) vel.twist.angular.z = max_turn;
      if (vel.twist.angular.z < -max_turn) vel.twist.angular.z = -max_turn;
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
