#include "active_inspection_planner/migration/adapters.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace inspection::migration {
class PlannerNode : public rclcpp::Node {
public:
  PlannerNode() : Node("nbv_utility_node") {
    Json config = defaults();
    for (auto it = config.begin(); it != config.end(); ++it) {
      auto &v = it.value();
      const auto &k = it.key();
      if (v.is_boolean())
        v = declare_parameter(k, v.get<bool>());
      else if (v.is_number_integer())
        v = declare_parameter(k, v.get<int64_t>());
      else if (v.is_number())
        v = declare_parameter(k, v.get<double>());
      else if (v.is_string())
        v = declare_parameter(k, v.get<std::string>());
      else
        v = declare_parameter(k, v.get<std::vector<double>>());
    }
    // Validate configuration before opening result files, as the Python adapter
    // does.
    if (config["route_max_candidates"].get<int64_t>() < 1)
      throw std::invalid_argument(
          "route_max_candidates must be a positive integer");
    for (auto k : {"route_stall_timeout_s", "route_stall_min_progress_m"}) {
      double v = config[k];
      if (!(v > 0 && v < INFINITY))
        throw std::invalid_argument(std::string(k) +
                                    " must be finite and positive");
    }
    map = std::make_unique<Octomap>(config["voxel_resolution"]);
    files = std::make_unique<FileSink>(config["results_dir"]);
    trajectory_pub = create_publisher<std_msgs::msg::Float32MultiArray>(
        "object/areaWaypoints", 10);
    completion_pub = create_publisher<std_msgs::msg::Bool>(
        "/movement/finished_execution", 10);
    MissionPorts ports;
    ports.telemetry = [this](auto &event) { files->record(event); };
    ports.evaluation = [this](auto &line) { files->evaluation(line); };
    ports.diagnostics = [this](const auto &level, const auto &message) {
      double throttle = (message.rfind("Pipeline blocked:", 0) == 0 ||
                         message.rfind("Depth projection yielded", 0) == 0 ||
                         message.rfind("Iteration ", 0) == 0)
                            ? 2
                        : message.rfind("Inserted ", 0) == 0 ? 1
                                                             : 0;
      std::string key = message.rfind("Iteration ", 0) == 0  ? "Iteration"
                        : message.rfind("Inserted ", 0) == 0 ? "Inserted"
                                                             : message;
      if (throttle) {
        double now = monotonic();
        if (last_log.count(key) && now - last_log[key] < throttle)
          return;
        last_log[key] = now;
      }
      if (level == "error")
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      else if (level == "warning")
        RCLCPP_WARN(get_logger(), "%s", message.c_str());
      else
        RCLCPP_INFO(get_logger(), "%s", message.c_str());
    };
    ports.publish = [this](auto &values) {
      std_msgs::msg::Float32MultiArray msg;
      msg.data = values;
      trajectory_pub->publish(msg);
    };
    ports.complete = [this](bool value) {
      std_msgs::msg::Bool msg;
      msg.data = value;
      completion_pub->publish(msg);
    };
    mission = std::make_unique<Mission>(config, *map, ports);
    state_group =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = state_group;
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
        "holocean/odom", rclcpp::QoS(1).best_effort(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          auto &p = msg->pose.pose.position;
          auto &q = msg->pose.pose.orientation;
          auto &t = msg->header.stamp;
          mission->odometry({t.sec + t.nanosec * 1e-9, V3(p.x, p.y, p.z)},
                            Eigen::Quaterniond(q.w, q.x, q.y, q.z));
        },
        opts);
    mag_sub = create_subscription<sensor_msgs::msg::MagneticField>(
        "holocean/mag", rclcpp::QoS(1).best_effort(),
        [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {
          auto &v = msg->magnetic_field;
          mission->magnetometer(V3(v.x, v.y, v.z));
        },
        opts);
    rgb_sub = create_subscription<sensor_msgs::msg::Image>(
        "holocean/cameraDown/image_raw", rclcpp::QoS(10).best_effort(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr) { mission->rgb(); });
    depth_sub = create_subscription<sensor_msgs::msg::Image>(
        "holocean/depth/image_raw", rclcpp::QoS(10).best_effort(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          auto image = cv_bridge::toCvShare(msg);
          DepthFrame f;
          f.rows = image->image.rows;
          f.cols = image->image.cols;
          f.channels = image->image.channels();
          f.uint16 = image->image.depth() == CV_16U;
          cv::Mat values;
          image->image.convertTo(values, CV_64F);
          for (int r = 0; r < f.rows; ++r) {
            auto *row = values.ptr<double>(r);
            f.values.insert(f.values.end(), row, row + f.cols * f.channels);
          }
          mission->depth(f, msg->header.stamp.sec +
                                msg->header.stamp.nanosec * 1e-9);
        });
    request_sub = create_subscription<std_msgs::msg::Bool>(
        "/planner/request_nbv", 1,
        [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          mission->request(msg->data);
        });
    timer = rclcpp::create_timer(this, get_clock(),
                                 rclcpp::Duration::from_seconds(.5),
                                 [this] { mission->tick(); });
  }
  ~PlannerNode() override {
    if (mission)
      mission->close();
  }

private:
  std::unique_ptr<Octomap> map;
  std::unique_ptr<FileSink> files;
  std::unique_ptr<Mission> mission;
  std::map<std::string, double> last_log;
  rclcpp::CallbackGroup::SharedPtr state_group;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr trajectory_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completion_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub, depth_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr request_sub;
};
} // namespace inspection::migration
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<inspection::migration::PlannerNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      2);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
