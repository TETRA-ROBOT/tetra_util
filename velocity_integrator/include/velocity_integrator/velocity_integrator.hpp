#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class VelocityIntegrator
{
public:
  VelocityIntegrator(rclcpp::Node * node)
  : clock_(node->get_clock()),
    node_(node),
    distance_(0.0),
    prev_velocity_(0.0),
    first_call_(true)
  {
    distance_pub_ = node_->create_publisher<std_msgs::msg::Float64>("/distance", 10);
  }

  void update(double current_velocity)
  {
    rclcpp::Time now = clock_->now();

    if (first_call_)
    {
      last_time_ = now;
      prev_velocity_ = current_velocity;
      first_call_ = false;
      return;
    }

    double dt = (now - last_time_).seconds();
    double avg_velocity = 0.5 * (prev_velocity_ + current_velocity);
    distance_ += avg_velocity * dt;

    prev_velocity_ = current_velocity;
    last_time_ = now;

    std_msgs::msg::Float64 msg;
    msg.data = distance_;
    distance_pub_->publish(msg);
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Node * node_;
  rclcpp::Time last_time_;
  double distance_;
  double prev_velocity_;
  bool first_call_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
};

