#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

class PathPlanner : public rclcpp::Node
{
public:
    PathPlanner() : Node("path_planner")
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

        this->get_parameter("camera_x", camera_position_x_);
        this->get_parameter("camera_y", camera_position_y_);
        this->get_parameter("camera_z", camera_position_z_);
        this->get_parameter("initial_waypoint_x", initial_waypoint_x_);
        this->get_parameter("initial_waypoint_y", initial_waypoint_y_);
        this->get_parameter("initial_waypoint_z", initial_waypoint_z_);
        this->get_parameter("initial_waypoint_yaw", initial_waypoint_yaw_);
        this->get_parameter("timer_period_s", timer_period_s_);
        this->get_parameter(
            "waypoint_reached_distance", waypoint_reached_distance_);

        // Initial setup
        camera_position_ = {camera_position_x_, camera_position_y_, camera_position_z_};
        waypoints_.push_back({
            initial_waypoint_x_, initial_waypoint_y_,
            initial_waypoint_z_, initial_waypoint_yaw_});
        // waypoints_.push_back({-56.0, -100.0, -30.0, 3.14});

        // Publishers
        waypoint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/trajectory/waypoint", 10);
        planner_pub_ = this->create_publisher<std_msgs::msg::Bool>("/planner/report", 10);
        request_nbv_pub_ = this->create_publisher<std_msgs::msg::Bool>("/planner/request_nbv", 1);

        // Subscribers
        stop_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/motion/report", 10, std::bind(&PathPlanner::stop_callback, this, std::placeholders::_1));
        point_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "object/detected", 10, std::bind(&PathPlanner::point_callback, this, std::placeholders::_1));
        point_segment_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "object/areaWaypoints", 10, std::bind(&PathPlanner::pointSegment_callback, this, std::placeholders::_1));

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(timer_period_s_)),
            std::bind(&PathPlanner::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "✅ Path planner Started");
    }

private:
    std::vector<double> camera_position_;
    std::vector<std::vector<double>> waypoints_;
    double camera_position_x_ = 0.35;
    double camera_position_y_ = 0.1125 + 0.275;
    double camera_position_z_ = -0.5;
    double initial_waypoint_x_ = -16.0;
    double initial_waypoint_y_ = -81.0;
    double initial_waypoint_z_ = -30.0;
    double initial_waypoint_yaw_ = 0.0;
    double timer_period_s_ = 1.0;
    double waypoint_reached_distance_ = 0.4;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planner_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr request_nbv_pub_;
    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point_segment_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void point_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        waypoints_.clear();
        std::vector<double> new_wp(msg->data.begin(), msg->data.end());
        waypoints_.push_back(new_wp);
        timer_callback();
        RCLCPP_INFO(this->get_logger(), "Published 1 waypoints (from object/detected).");
    }

    void pointSegment_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        std::vector<std::vector<double>> wps;
        
        // Replicates numpy.reshape(-1, 4)
        for (size_t i = 0; i + 3 < msg->data.size(); i += 4) {
            RCLCPP_INFO(this->get_logger(), "Adding segmentation waypoint...");
            std::vector<double> wp = {
                msg->data[i] - camera_position_[0],
                msg->data[i+1] - camera_position_[1],
                msg->data[i+2] - camera_position_[2],
                msg->data[i+3]
            };
            wps.push_back(wp);
        }

        if (!wps.empty()) {
            waypoints_ = wps;
            RCLCPP_INFO(this->get_logger(), "Received %zu segmentation waypoints.", wps.size());
            // Send the new route now: the vehicle keeps moving between timer ticks.
            timer_callback();
        }
    }

    void timer_callback()
    {
        if (!waypoints_.empty()) {
            auto msg = std_msgs::msg::Float32MultiArray();
            
            // Define strict 2D layout for the Python flight controller
            msg.layout.dim.resize(2);
            msg.layout.dim[0].label = "waypoints";
            msg.layout.dim[0].size = waypoints_.size();
            msg.layout.dim[0].stride = waypoints_.size() * 4;
            msg.layout.dim[1].label = "coordinates";
            msg.layout.dim[1].size = 4;
            msg.layout.dim[1].stride = 4;
            
            for (const auto& wp : waypoints_) {
                float x = 0.0f, y = 0.0f, z = -29.5f, yaw = 0.0f;
                
                // Safely pad every waypoint to exactly 4 floats to prevent numpy crashes
                if (wp.size() >= 2) { x = wp[0]; y = wp[1]; }
                if (wp.size() >= 3) { z = wp[2]; }
                if (wp.size() >= 4) { yaw = wp[3]; }
                
                msg.data.insert(msg.data.end(), {x, y, z, yaw});
            }

            waypoint_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published %zu waypoints.", waypoints_.size());
        }
    }

    void stop_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 3) return;

        std::vector<double> data = {msg->data[0], msg->data[1], msg->data[2]};

        if (!waypoints_.empty() && waypoints_[0].size() >= 2) {
            // Replaces python Tools.mapping2D.diff
            double dist = std::hypot(waypoints_[0][0] - data[0], waypoints_[0][1] - data[1]);
            
            if (dist < waypoint_reached_distance_) {
                waypoints_.erase(waypoints_.begin());
            }
        }

        RCLCPP_INFO(this->get_logger(), "Received stop signal, giving next waypoint.");

        // Replaces try/except blocks to remove specific target point from list safely
        auto it = std::find_if(waypoints_.begin(), waypoints_.end(), [&data](const std::vector<double>& wp) {
            if (wp.size() >= 3) {
                return (wp[0] == data[0] && wp[1] == data[1] && wp[2] == data[2]);
            }
            return false;
        });

        if (it != waypoints_.end()) {
            waypoints_.erase(it);
        }

        // Limit the waypoints array to the last 200 elements (self.waypoints[-200:])
        // if (waypoints_.size() > 200) {
        //     waypoints_.erase(waypoints_.begin(), waypoints_.end() - 200);
        // }

        // Trigger the publisher manually
        timer_callback();

        // NEW: If the queue just hit zero, instantly demand more waypoints!
        if (waypoints_.empty()) {
            RCLCPP_INFO(this->get_logger(), "Waypoint queue empty. Demanding new NBV trajectory NOW!");
            auto req_msg = std_msgs::msg::Bool();
            req_msg.data = true;
            request_nbv_pub_->publish(req_msg);
        }
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
