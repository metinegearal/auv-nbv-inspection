#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "realtime_stereo_mapper.hpp"
#include <fstream>
#include <chrono>

class NBVUtilityNode : public rclcpp::Node {
public:
    NBVUtilityNode() : Node("nbv_utility_node"), 
        started_mapping_(false), finished_mapping_(false), iter_count_(0) 
    {
        // Declare and read parameters
        MapperConfig cfg;
        cfg.resolution = this->declare_parameter("voxel_resolution", 0.1);
        cfg.max_depth = this->declare_parameter("max_depth", 5.0);
        cfg.min_depth = this->declare_parameter("min_depth", 0.5);
        cfg.z_min_target = this->declare_parameter("z_min_target", -34.0);
        cfg.info_gain_threshold = this->declare_parameter("info_gain_threshold", 0.05);
        cfg.sparsity_multiplier = this->declare_parameter("sparsity_multiplier", 0.8);
        cfg.min_utility_threshold = this->declare_parameter("min_utility_threshold", 0.005);

        // Initialize mapper with the struct
        mapper_ = std::make_unique<RealTimeStereoMapper>(cfg);
        
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
        waypoint_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("object/areaWaypoints", 10);
        finished_pub_ = this->create_publisher<std_msgs::msg::Bool>("/movement/finished_execution", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&NBVUtilityNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "✅ Object Detector Node Started in C++");
    }

private:
    bool odom_received_ = false;
    std::unique_ptr<RealTimeStereoMapper> mapper_;
    Eigen::Vector3d current_position_{0, 0, 0};
    double yaw_ = 0, pitch_ = 0, roll_ = 0;
    
    cv::Mat left_image_;
    cv::Mat depth_frame_;
    bool started_mapping_;
    bool finished_mapping_;
    int iter_count_;
    size_t previous_node_leaves_ = 0;

    // ROS objects...
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_, rgb_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        if (std::abs(msg->pose.pose.position.x) < 0.01 && 
            std::abs(msg->pose.pose.position.y) < 0.01 ) {
            return; 
        }

        current_position_ << msg->pose.pose.position.x, 
                             msg->pose.pose.position.y, 
                             msg->pose.pose.position.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll_, pitch_, yaw_);
        odom_received_ = true;
    }

    void mag_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
        double ya = std::atan2(msg->magnetic_field.x, msg->magnetic_field.y) - (M_PI / 2.0);
        if (ya < -M_PI) ya += 2 * M_PI;
        yaw_ = ya; // Override odom yaw with mag
    }

    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        left_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Passing msg->encoding dynamically adapts to the incoming format
        depth_frame_ = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    }

    void timer_callback() {
        RCLCPP_INFO(this->get_logger(), "Timer triggered. Iteration: %d, left_images: %d, depth_frames: %d, odom_received: %s", iter_count_, left_image_.empty() ? 0 : 1, depth_frame_.empty() ? 0 : 1, odom_received_ ? "true" : "false");
        if (finished_mapping_) return;
        if (left_image_.empty() || depth_frame_.empty()) return;
        if (!odom_received_) return;


        auto start_proc = std::chrono::high_resolution_clock::now();
        
        mapper_->process_frame(current_position_, yaw_, pitch_, roll_, left_image_, depth_frame_);
        if (iter_count_ == 11) {
            RCLCPP_INFO(this->get_logger(), "DEBUG MAP POS: AUV is at %.2f, %.2f, %.2f", 
                        current_position_.x(), current_position_.y(), current_position_.z());
        }
        
        if (iter_count_ % 10 == 0) {
            static std::ofstream eval_file("evaluation_parameters.txt", std::ios::app);
            auto start_time = std::chrono::high_resolution_clock::now();

            mapper_->save_map("final_map_ros2.bt");

            auto [viewpoints, trajectories] = mapper_->generate_candidate_viewpoints(current_position_);
            
            // --- NEW DEBUGGING BLOCK ---
            RCLCPP_INFO(this->get_logger(), "A* Pipeline Finished. Trajectories size: %zu", trajectories.size());
            if (!trajectories.empty()) {
                RCLCPP_INFO(this->get_logger(), "First Trajectory Point: [%.2f, %.2f, %.2f]", 
                            trajectories[0].x(), trajectories[0].y(), trajectories[0].z());
            }
            // ---------------------------

            if (!trajectories.empty()) {
                
                // CRITICAL SAFETY CHECK: Prevent publishing [0,0,0] if A* failed weirdly
                if (trajectories.size() < 2 && std::abs(trajectories[0].x()) < 0.1 && std::abs(trajectories[0].y()) < 0.1) {
                    RCLCPP_ERROR(this->get_logger(), "ALGORITHM RETURNED [0,0,0] INSTEAD OF A PATH. ABORTING PUBLISH!");
                } else {
                    started_mapping_ = true;
                    std_msgs::msg::Float32MultiArray wp_msg;
                    
                    // Flatten trajectories for ROS message
                    for (const auto& pt : trajectories) {
                        wp_msg.data.push_back(pt.x());
                        wp_msg.data.push_back(pt.y());
                        wp_msg.data.push_back(pt.z());
                        wp_msg.data.push_back(pt.w());
                    }
                    waypoint_pub_->publish(wp_msg);
                    RCLCPP_INFO(this->get_logger(), "Published valid trajectory to PathPlanner.");
                }
            } else if (started_mapping_) {
                finished_mapping_ = true;
                std_msgs::msg::Bool fin_msg;
                fin_msg.data = true;
                finished_pub_->publish(fin_msg);
                RCLCPP_INFO(this->get_logger(), "Mapping finished. Explored all accessible frontiers.");
            } else {
                RCLCPP_INFO(this->get_logger(), "No new viewpoints generated. Continuing exploration...");
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            auto process_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            
            size_t current_leaves = mapper_->get_current_leaves();
            size_t leaves_added = current_leaves - previous_node_leaves_;
            previous_node_leaves_ = current_leaves;
            
            eval_file << "Process Time (ms): " << process_time_ms 
                    << ", Exploration Score: " << leaves_added 
                    << ", Total Leaves: " << current_leaves << "\n";
        }
        iter_count_++;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NBVUtilityNode>());
    rclcpp::shutdown();
    return 0;
}