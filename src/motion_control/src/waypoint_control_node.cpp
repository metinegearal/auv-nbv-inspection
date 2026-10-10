#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "motion_control/use_cases/waypoint_controller.hpp"
#include "motion_control/adapters/ros_control_io.hpp"
#include "motion_control/adapters/thrusters.hpp"

using std::placeholders::_1;

class WaypointControlNode : public rclcpp::Node {
public:
  WaypointControlNode()
  : Node("waypoint_control_node")
  {
    // --- Declare and Get Parameters from config/params.yaml ---
    this->declare_parameter<double>("kp_yaw", 0.6);
    this->declare_parameter<double>("kp_dist", 0.75);
    this->declare_parameter<bool>("dvl_active", false);
    this->declare_parameter<double>("stopping_distance", 0.25);
    this->declare_parameter<double>("initial_waypoint_x", 10.0);
    this->declare_parameter<double>("initial_waypoint_y", 10.0);
    this->declare_parameter<double>("initial_waypoint_z", 5.0);
    this->declare_parameter<double>("max_yaw_command", 10.0);
    this->declare_parameter<double>("yaw_smoothing_rate", 0.8);
    this->declare_parameter<double>("min_forward_thrust", 1.0);
    this->declare_parameter<double>("max_forward_thrust", 10.0);
    this->declare_parameter<double>("command_multiplier", 2.0);

    // Publishers
    target_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/holo/cmd", 10);
    stop_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motion/report", 10);

    const motion_control::ControlConfig config{get_parameter("kp_yaw").as_double(),
      get_parameter("kp_dist").as_double(),
      get_parameter("dvl_active").as_bool(),
      get_parameter("stopping_distance").as_double(),
      get_parameter("initial_waypoint_x").as_double(),
      get_parameter("initial_waypoint_y").as_double(),
      get_parameter("initial_waypoint_z").as_double(),
      get_parameter("max_yaw_command").as_double(),
      get_parameter("yaw_smoothing_rate").as_double(),
      get_parameter("min_forward_thrust").as_double(),
      get_parameter("max_forward_thrust").as_double(),
      get_parameter("command_multiplier").as_double()};
    io_ = std::make_unique<motion_control::RosControlIO>(*this, target_pub_, stop_pub_);
    controller_ = std::make_unique<motion_control::WaypointController>(
      config, *io_, *io_, *io_, *io_, modes_, mixer_);
    // Subscribers
    waypoint_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/trajectory/waypoint", 10, std::bind(&WaypointControlNode::waypoint_callback, this, _1));
    position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "holocean/odom", 10, std::bind(&WaypointControlNode::position_callback, this, _1));
    distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "holocean/depth/distance", 10, std::bind(&WaypointControlNode::dist_callback, this, _1));
    depth_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "holocean/depth/scalar", 10, std::bind(&WaypointControlNode::depth_callback, this, _1));
    sub_mag_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
      "holocean/mag", 10, std::bind(&WaypointControlNode::magnetometer_callback, this, _1));
    obstacle_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "object/obstacle", 10, std::bind(&WaypointControlNode::obstacle_callback, this, _1));
    angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/aruco/angle", 10, std::bind(&WaypointControlNode::angle_callback, this, _1));
    // mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    //   "/movement/mode", 10, std::bind(&WaypointControlNode::mode_callback, this, _1));
    finished_execution_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/movement/finished_execution", 10,
      std::bind(&WaypointControlNode::finished_execution_callback, this, _1));
    dvl_reset_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/movement/dvl_reset", 10, std::bind(&WaypointControlNode::dvl_reset_callback, this, _1));

    controller_->start();
    RCLCPP_INFO(this->get_logger(), "✅ Waypoint Control Node Started (Cleaned Edition)");
  }

private:
  motion_control::ConfiguredModes modes_;
  motion_control::EightThrusterMixer mixer_;
  std::unique_ptr<motion_control::RosControlIO> io_;
  std::unique_ptr<motion_control::WaypointController> controller_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr stop_pub_;

  // --- Subscribers ---
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_, depth_sub_, angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_mag_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_execution_sub_, dvl_reset_sub_;

  void finished_execution_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {controller_->finished_execution_callback(msg->data);}
  void mode_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    controller_->mode_callback(msg->data);
  }
  void angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {controller_->angle_callback(msg->data);}
  void dvl_reset_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {controller_->dvl_reset_callback(msg->data);}
  void position_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    controller_->position_callback({msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z});
  }
  void magnetometer_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    controller_->magnetometer_callback({msg->magnetic_field.x, msg->magnetic_field.y,
        msg->magnetic_field.z});
  }
  void depth_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    controller_->depth_callback(msg->data);
  }
  void dist_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {controller_->dist_callback(msg->data);}
  void obstacle_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {controller_->obstacle_callback(motion_control::WaypointInput{msg->data});}
  void waypoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    controller_->waypoint_callback(motion_control::WaypointInput{msg->data});
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointControlNode>());
  rclcpp::shutdown();
  return 0;
}
