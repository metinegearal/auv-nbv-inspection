#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nbv_interfaces/action/compute_nbv.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "motion_control/use_cases/trajectory_queue.hpp"
class PathPlanner : public rclcpp::Node, public motion_control::WaypointOutput,
  public motion_control::Diagnostics {
public:
  using ComputeNBV = nbv_interfaces::action::ComputeNBV;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ComputeNBV>;
  using SteadyClock = std::chrono::steady_clock;
  PathPlanner()
  : Node("path_planner")
  {
    server_timeout_s_ = declare_parameter<double>("nbv_server_timeout_s", 30.0);
    feedback_timeout_s_ = declare_parameter<double>("nbv_feedback_timeout_s", 30.0);
    goal_timeout_s_ = declare_parameter<double>("nbv_goal_timeout_s", 120.0);
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
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
    nbv_client_ = rclcpp_action::create_client<ComputeNBV>(this, "/planner/compute_nbv");
    finished_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/movement/finished_execution", rclcpp::QoS(1).transient_local());
    state_pub_ = create_publisher<std_msgs::msg::String>(
      "/planner/state", rclcpp::QoS(1).transient_local());

    const motion_control::QueueConfig config{
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
      std::chrono::duration<double>(config.timer_period_s)), [this]{timer_callback();});
    waiting_since_ = SteadyClock::now();
    set_state("WaitingForNBVServer");
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
    if (terminal_) {return;}
    force_pending_ = true;
    if (goal_handle_ && !cancel_pending_) {
      // A monitoring goal may be waiting for the next periodic planning pass.
      // Replace it with an immediate request when the vehicle queue empties.
      cancel_pending_ = true;
      nbv_client_->async_cancel_goal(goal_handle_);
    }
    if (!in_flight_) {send_goal(true);}
  }

private:
  std::unique_ptr<motion_control::TrajectoryQueue> queue_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planner_pub_;
  rclcpp_action::Client<ComputeNBV>::SharedPtr nbv_client_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finished_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  bool terminal_ = false, in_flight_ = false, force_pending_ = true, cancel_pending_ = false;
  bool computation_active_ = false;
  double server_timeout_s_, feedback_timeout_s_, goal_timeout_s_;
  std::string world_frame_;
  SteadyClock::time_point waiting_since_, goal_since_, feedback_since_;

  void set_state(const std::string & state)
  {
    std_msgs::msg::String msg; msg.data = state; state_pub_->publish(msg);
  }
  void finish(const std::string & state, const std::string & reason)
  {
    terminal_ = true;
    queue_->clear();
    if (goal_handle_) {nbv_client_->async_cancel_goal(goal_handle_);}
    set_state(state);
    std_msgs::msg::Bool msg; msg.data = true; finished_pub_->publish(msg);
    RCLCPP_WARN(get_logger(), "%s: %s", state.c_str(), reason.c_str());
  }
  void send_goal(bool force)
  {
    if (terminal_ || in_flight_ || !nbv_client_->action_server_is_ready()) {return;}
    in_flight_ = true;
    force_pending_ = false;
    cancel_pending_ = false;
    computation_active_ = false;
    goal_since_ = feedback_since_ = SteadyClock::now();
    ComputeNBV::Goal goal; goal.trigger = true; goal.force_replan = force;
    auto options = rclcpp_action::Client<ComputeNBV>::SendGoalOptions();
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
        if (terminal_) {
          if (handle) {nbv_client_->async_cancel_goal(handle);}
          return;
        }
        if (!handle) {finish("Failed", "NBV goal rejected"); return;}
        goal_handle_ = handle;
        set_state("WaitingForSensors");
        if (force_pending_) {request_nbv();}
      };
    options.feedback_callback = [this](GoalHandle::SharedPtr,
      const std::shared_ptr<const ComputeNBV::Feedback> feedback) {
        if (terminal_) {return;}
        feedback_since_ = SteadyClock::now();
        const bool computing = feedback->state == "Computing" || feedback->state == "Blacklisting";
        if (computing && !computation_active_) {goal_since_ = feedback_since_;}
        computation_active_ = computing;
        set_state(feedback->state);
        RCLCPP_DEBUG(get_logger(), "NBV %s; strikes=%u; blacklisted=%u",
        feedback->state.c_str(), feedback->strikes, feedback->blacklisted_targets);
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        in_flight_ = false;
        goal_handle_.reset();
        cancel_pending_ = false;
        if (terminal_) {return;}
        if (result.code == rclcpp_action::ResultCode::ABORTED) {
          finish(result.result && result.result->strikeout ? "MissionComplete" : "Failed",
          result.result ? result.result->message : "NBV aborted without a result");
          return;
        }
        if (result.code == rclcpp_action::ResultCode::CANCELED && force_pending_) {
          send_goal(true); return;
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result) {
          finish("Failed", "NBV goal canceled or result unavailable"); return;
        }
        const auto & route = result.result->trajectory;
        if (result.result->frame_id != world_frame_ || route.empty() || route.size() % 4 != 0 ||
          !std::all_of(route.begin(), route.end(), [](float v) {return std::isfinite(v);}))
        {
          finish("Failed", "Invalid NBV trajectory or coordinate frame"); return;
        }
        std_msgs::msg::Bool resume; resume.data = false; finished_pub_->publish(resume);
        queue_->pointSegment_callback(motion_control::WaypointInput{route});
        set_state("ExecutingTrajectory");
      // Keep one action open for periodic/arrival/collision replanning.
        waiting_since_ = SteadyClock::now();
        send_goal(false);
      };
    nbv_client_->async_send_goal(goal, options);
  }
  void timer_callback()
  {
    if (terminal_) {return;}
    const auto now = SteadyClock::now();
    const auto elapsed = [now](SteadyClock::time_point start) {
        return std::chrono::duration<double>(now - start).count();
      };
    if (in_flight_) {
      if (elapsed(feedback_since_) > feedback_timeout_s_ ||
        (computation_active_ && elapsed(goal_since_) > goal_timeout_s_))
      {
        finish("Failed", "NBV server stopped responding or computation timed out"); return;
      }
    } else {
      if (elapsed(waiting_since_) > server_timeout_s_) {
        finish("Failed", "NBV action server unavailable"); return;
      }
      send_goal(force_pending_);
    }
    queue_->timer_callback();
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point_segment_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void point_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (terminal_) {return;}
    queue_->point_callback(motion_control::WaypointInput{msg->data});
  }
  void pointSegment_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // The topic is an observation/collision-invalidation channel. Only action
    // results may install NBV routes, avoiding duplicate or out-of-order routes.
    if (terminal_ || !msg->data.empty()) {return;}
    queue_->clear();
    std_msgs::msg::Bool hold; hold.data = true; finished_pub_->publish(hold);
    request_nbv();
  }
  void stop_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (terminal_) {return;}
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
