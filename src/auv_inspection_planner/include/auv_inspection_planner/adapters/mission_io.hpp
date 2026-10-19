#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include "auv_inspection_planner/domain/contracts.hpp"
namespace inspection
{
class RosDiagnostics final : public Diagnostics {
  rclcpp::Logger logger_;

public:
  explicit RosDiagnostics(rclcpp::Logger logger)
  :logger_(logger) {}
  void text(const std::string & text, bool error, bool flush) override
  {
    auto & stream = error ? std::cerr : std::cout; stream << text; if(flush) {stream.flush();}
  }
  void info(const std::string & message) override {RCLCPP_INFO(logger_, "%s", message.c_str());}
  void warning(const std::string & message) override {RCLCPP_WARN(logger_, "%s", message.c_str());}
  void error(const std::string & message) override {RCLCPP_ERROR(logger_, "%s", message.c_str());}
};
class SystemClock final : public Clock {
public:
  Time now() const override
  {
    return std::chrono::high_resolution_clock::now();
  }
};
class FileMissionMetrics final : public MissionMetrics {
  const std::string path_; std::ofstream stream_; bool started_ = false;

public:
  explicit FileMissionMetrics(std::string path)
  :path_(std::move(path)) {}
  void begin() override {if(!started_) {stream_.open(path_, std::ios::app); started_ = true;}}
  void record(double elapsed, std::size_t added, std::size_t leaves) override
  {
    stream_ << "Process Time (ms): " << elapsed << ", Exploration Score: " << added <<
      ", Total Leaves: " << leaves << "\n";
  }
};
class RosMissionOutput final : public MissionOutput {
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoints_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_;

public:
  RosMissionOutput(
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoints,
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished)
  :waypoints_(waypoints), finished_(finished) {}
  void trajectory(const std::vector<Eigen::Vector4d> & values) override
  {
    std_msgs::msg::Float32MultiArray msg;
    for(const auto & p:values) {
      msg.data.push_back(p.x());msg.data.push_back(p.y());msg.data.push_back(p.z());
      msg.data.push_back(p.w());
    }
    waypoints_->publish(msg);
  }
  void finished() override {std_msgs::msg::Bool msg;msg.data = true;finished_->publish(msg);}
};
inline std::shared_ptr<const ColorImage> color_values(const cv::Mat & image)
{
  std::vector<std::array<unsigned char, 3>> pixels;
  for(int v = 0; v < image.rows; ++v) {
    for(int u = 0; u < image.cols; ++u) {
      const auto p = image.at<cv::Vec3b>(v, u); pixels.push_back({p[0], p[1], p[2]});
    }
  }
  return std::make_shared<const ColorImage>(ColorImage{image.rows, image.cols, std::move(pixels)});
}
inline std::shared_ptr<const DepthImage> depth_values(const cv::Mat & image)
{
  std::vector<float> pixels;
  for(int v = 0; v < image.rows; ++v) {
    for(int u = 0; u < image.cols; ++u) {
      pixels.push_back(image.at<float>(v, u));
    }
  }
  return std::make_shared<const DepthImage>(DepthImage{image.rows, image.cols, std::move(pixels)});
}
}
