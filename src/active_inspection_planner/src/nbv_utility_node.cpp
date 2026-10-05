#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "active_inspection_planner/use_cases/mission.hpp"
#include "active_inspection_planner/adapters/maps.hpp"
#include "active_inspection_planner/adapters/mission_io.hpp"
#include <fstream>
#include <chrono>

class NBVUtilityNode : public rclcpp::Node {
public:
  NBVUtilityNode()
  : Node("nbv_utility_node")
  {
    const inspection::MapperConfig cfg{90.0, 256.0, 0.5,
      declare_parameter("voxel_resolution", 0.1), declare_parameter("max_depth", 5.0),
      declare_parameter("min_depth", 0.5), declare_parameter("z_min_target", -32.0),
      declare_parameter("info_gain_threshold", 0.05), declare_parameter("sparsity_multiplier", 0.8),
      declare_parameter("min_utility_threshold", 0.005)};
        // QoS for sensor data
    rclcpp::QoS qos(10);
    qos.best_effort();

        // Subscribers
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "holocean/depth/image_raw", qos,
            std::bind(&NBVUtilityNode::depth_callback, this, std::placeholders::_1));

    rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "holocean/cameraDown/image_raw", qos,
            std::bind(&NBVUtilityNode::rgb_callback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "holocean/odom", qos,
            std::bind(&NBVUtilityNode::odom_callback, this, std::placeholders::_1));

    mag_sub_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
            "holocean/mag", qos,
            std::bind(&NBVUtilityNode::mag_callback, this, std::placeholders::_1));

        // Publishers
    waypoint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("object/areaWaypoints",
      10);
    finished_pub_ = this->create_publisher<std_msgs::msg::Bool>("/movement/finished_execution", 10);

    diagnostics_ = std::make_unique<inspection::RosDiagnostics>(get_logger());
    map_ = std::make_unique<inspection::OctomapAdapter>(cfg.resolution, *diagnostics_);
    update_ = std::make_unique<inspection::MapUpdate>(cfg, *map_, *map_);
    route_ = std::make_unique<inspection::RoutePlanner>(cfg.resolution, *diagnostics_);
    candidates_ = std::make_unique<inspection::CandidatePlanner>(cfg, *map_, indexes_, *route_,
      *diagnostics_);
    output_ = std::make_unique<inspection::RosMissionOutput>(waypoint_pub_, finished_pub_);
    mission_ = std::make_unique<inspection::InspectionMission>(*update_, *candidates_, *map_, *map_,
            *output_, metrics_, clock_, *diagnostics_,
            "/home/metin-ege/AIEngineering/RoboticFocus/HoloSystem/NBV_paper_ros2/final_map_ros2.bt");
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]{mission_->timer_callback();});
    RCLCPP_INFO(get_logger(), "✅ Object Detector Node Started in C++");
  }

private:
  inspection::PclIndexes indexes_;
  inspection::SystemClock clock_;
  inspection::FileMissionMetrics metrics_{"evaluation_parameters.txt"};
  std::unique_ptr<inspection::RosDiagnostics> diagnostics_;
  std::unique_ptr<inspection::OctomapAdapter> map_;
  std::unique_ptr<inspection::MapUpdate> update_;
  std::unique_ptr<inspection::RoutePlanner> route_;
  std::unique_ptr<inspection::CandidatePlanner> candidates_;
  std::unique_ptr<inspection::RosMissionOutput> output_;
  std::unique_ptr<inspection::InspectionMission> mission_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_, rgb_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub_;
  rclcpp::TimerBase::SharedPtr timer_;


  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    if(!mission_->accepts_pose(position)) {return;}
    const auto & o = msg->pose.pose.orientation;
    tf2::Quaternion q(o.x, o.y, o.z, o.w); tf2::Matrix3x3 m(q);
    double roll, pitch, yaw; m.getRPY(roll, pitch, yaw);
    mission_->pose(inspection::Pose{position, yaw, pitch, roll});
  }
  void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    mission_->magnetic(msg->magnetic_field.x, msg->magnetic_field.y);
  }
  void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    mission_->rgb(inspection::color_values(cv_bridge::toCvCopy(msg, "bgr8")->image));
  }
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    mission_->depth(inspection::depth_values(cv_bridge::toCvCopy(msg, msg->encoding)->image));
  }
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NBVUtilityNode>());
  rclcpp::shutdown();
  return 0;
}
