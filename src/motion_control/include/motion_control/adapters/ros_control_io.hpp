#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "motion_control/domain/contracts.hpp"
namespace motion_control
{
class RosControlIO final : public Clock, public Gains, public Diagnostics, public CommandOutput {
  rclcpp::Node & node_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_, report_;

public:
  RosControlIO(
    rclcpp::Node & node,
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command,
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr report)
  : node_(node), command_(command), report_(report) {}
  double now() const override {return node_.get_clock()->now().seconds();}
  double yaw() const override {return node_.get_parameter("kp_yaw").as_double();}
  double distance() const override {return node_.get_parameter("kp_dist").as_double();}
  void write(const std::string & text) override
  {
    RCLCPP_INFO(node_.get_logger(), "%s", text.c_str());
  }
  void command(const MotorCommand & command) override
  {
    std_msgs::msg::Float32MultiArray message;
    message.data.assign(command.values.begin(), command.values.end());
    command_->publish(message);
  }
  void reached(const std::vector<float> & values) override
  {
    std_msgs::msg::Float32MultiArray message; message.data = values; report_->publish(message);
  }
};
}
