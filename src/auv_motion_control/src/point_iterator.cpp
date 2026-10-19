#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include "auv_motion_control/use_cases/trajectory_queue.hpp"
class PathPlanner : public rclcpp::Node, public motion_control::WaypointOutput,
  public motion_control::Diagnostics {
public:
  PathPlanner()
  : Node("path_planner")
  {
    this->declare_parameter<double>("camera_x", 0.35);
    this->declare_parameter<double>("camera_y", 0.1125 + 0.275);
    this->declare_parameter<double>("camera_z", -0.5);
    this->declare_parameter<double>("initial_waypoint_x", -16.0);
    this->declare_parameter<double>("initial_waypoint_y", -81.0);
    this->declare_parameter<double>("initial_waypoint_z", -30.0);
    this->declare_parameter<double>("initial_waypoint_yaw", 0.0);
    this->declare_parameter<double>("timer_period_s", 1.0);
    this->declare_parameter<double>("waypoint_reached_distance", 0.4);

        // Publishers
    waypoint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/trajectory/waypoint",
      10);
    planner_pub_ = this->create_publisher<std_msgs::msg::Bool>("/planner/report", 10);
    request_nbv_pub_ = this->create_publisher<std_msgs::msg::Bool>("/planner/request_nbv", 1);

    const motion_control::QueueConfig config{
      get_parameter("camera_x").as_double(),
      get_parameter("camera_y").as_double(),
      get_parameter("camera_z").as_double(),
      get_parameter("initial_waypoint_x").as_double(),
      get_parameter("initial_waypoint_y").as_double(),
      get_parameter("initial_waypoint_z").as_double(),
      get_parameter("initial_waypoint_yaw").as_double(),
      get_parameter("timer_period_s").as_double(),
      get_parameter("waypoint_reached_distance").as_double()};
    queue_ = std::make_unique<motion_control::TrajectoryQueue>(config, *this, *this);
        // Subscribers
    stop_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/motion/report", 10,
      std::bind(&PathPlanner::stop_callback, this, std::placeholders::_1));
    point_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "object/detected", 10,
      std::bind(&PathPlanner::point_callback, this, std::placeholders::_1));
    point_segment_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "object/areaWaypoints", 10,
      std::bind(&PathPlanner::pointSegment_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(config.timer_period_s)), [this]{queue_->timer_callback();});
    RCLCPP_INFO(get_logger(), "✅ Path planner Started");
  }
  void write(const std::string & text) override {RCLCPP_INFO(get_logger(), "%s", text.c_str());}
  void waypoints(const std::vector<float> & values, std::size_t count) override
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.layout.dim.resize(2);
    msg.layout.dim[0].label = "waypoints"; msg.layout.dim[0].size = count;
    msg.layout.dim[0].stride = count * 4;
    msg.layout.dim[1].label = "coordinates"; msg.layout.dim[1].size = 4;
    msg.layout.dim[1].stride = 4; msg.data = values;
    waypoint_pub_->publish(msg);
  }
  void request_nbv() override
  {
    std_msgs::msg::Bool msg; msg.data = true; request_nbv_pub_->publish(msg);
  }

private:
  std::unique_ptr<motion_control::TrajectoryQueue> queue_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planner_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr request_nbv_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point_segment_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void point_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    queue_->point_callback(motion_control::WaypointInput{msg->data});
  }
  void pointSegment_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    queue_->pointSegment_callback(motion_control::WaypointInput{msg->data});
  }
  void stop_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    queue_->stop_callback(motion_control::WaypointInput{msg->data});
  }

};
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
