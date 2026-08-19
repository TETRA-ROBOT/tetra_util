#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "velocity_integrator/velocity_integrator.hpp"

class VelocityIntegratorNode : public rclcpp::Node
{
public:
  VelocityIntegratorNode()
  : Node("velocity_integrator_node")
  {
    integrator_ = std::make_shared<VelocityIntegrator>(this);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav", 10, std::bind(&VelocityIntegratorNode::callback, this, std::placeholders::_1));
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    integrator_->update(msg->linear.x);
  }

  std::shared_ptr<VelocityIntegrator> integrator_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<VelocityIntegratorNode>());

  rclcpp::shutdown();
  return 0;
}

