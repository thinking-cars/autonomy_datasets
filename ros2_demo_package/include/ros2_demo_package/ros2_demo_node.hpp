#pragma once

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <ros2_demo_package_interfaces/action/fibonacci.hpp>

namespace ros2_demo_package {

template <typename C>
struct is_vector : std::false_type {};
template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};
template <typename C>
inline constexpr bool is_vector_v = is_vector<C>::value;

/**
 * @brief Configuration parameters for topic diagnostics
 */
struct TopicDiagnosticConfig {
  /**
   * @brief Minimum acceptable frequency
   */
  double min_frequency;

  /**
   * @brief Maximum acceptable frequency
   */
  double max_frequency;

  /**
   * @brief Minimum acceptable difference between message timestamp and receipt time (in seconds)
   */
  double min_acceptable_timestamp_delta;

  /**
   * @brief Maximum acceptable difference between message timestamp and receipt time (in seconds)
   */
  double max_acceptable_timestamp_delta;
};

/**
 * @brief Ros2DemoNode class
 */
class Ros2DemoNode : public rclcpp::Node {
 public:
  Ros2DemoNode();

 private:
  /**
   * @brief Declares and loads a ROS parameter
   *
   * @param name name
   * @param param parameter variable to load into
   * @param description description
   * @param add_to_auto_reconfigurable_params enable reconfiguration of parameter
   * @param is_required whether failure to load parameter will stop node
   * @param read_only set parameter to read-only
   * @param from_value parameter range minimum
   * @param to_value parameter range maximum
   * @param step_value parameter range step
   * @param additional_constraints additional constraints description
   */
  template <typename T>
  void declareAndLoadParameter(const std::string& name, T& param, const std::string& description,
                               const bool add_to_auto_reconfigurable_params = true, const bool is_required = false,
                               const bool read_only = false, const std::optional<double>& from_value = std::nullopt,
                               const std::optional<double>& to_value = std::nullopt,
                               const std::optional<double>& step_value = std::nullopt,
                               const std::string& additional_constraints = "");

  /**
   * @brief Handles reconfiguration when a parameter value is changed
   *
   * @param parameters parameters
   * @return parameter change result
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief Sets up subscribers, publishers, etc. to configure the node
   */
  void setup();

  /**
   * @brief Processes messages received by a subscriber
   *
   * @param msg message
   */
  void topicCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr& msg);

  /**
   * @brief Processes service requests
   *
   * @param request service request
   * @param response service response
   */
  void serviceCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /**
   * @brief Processes action goal requests
   *
   * @param uuid unique goal identifier
   * @param goal action goal
   * @return goal response
   */
  rclcpp_action::GoalResponse actionHandleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ros2_demo_package_interfaces::action::Fibonacci::Goal> goal);

  /**
   * @brief Processes action cancel requests
   *
   * @param goal_handle action goal handle
   * @return cancel response
   */
  rclcpp_action::CancelResponse actionHandleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_demo_package_interfaces::action::Fibonacci>>
          goal_handle);

  /**
   * @brief Processes accepted action goal requests
   *
   * @param goal_handle action goal handle
   */
  void actionHandleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_demo_package_interfaces::action::Fibonacci>>
          goal_handle);

  /**
   * @brief Executes an action
   *
   * @param goal_handle action goal handle
   */
  void actionExecute(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ros2_demo_package_interfaces::action::Fibonacci>>
          goal_handle);

  /**
   * @brief Processes timer triggers
   */
  void timerCallback();

  /**
   * @brief Function called by diagnostic updater to populate diagnostics status
   */
  void health(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @brief Sets the health information and triggers publishing by diagnostic updater
   */
  void setHealth(const unsigned char status, const std::string& msg,
                 const std::map<std::string, std::string>& key_value_pairs = {});

 private:
  /**
   * @brief Auto-reconfigurable parameters for dynamic reconfiguration
   */
  std::vector<std::tuple<std::string, std::function<void(const rclcpp::Parameter&)>>> auto_reconfigurable_params_;

  /**
   * @brief Callback handle for dynamic parameter reconfiguration
   */
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_;

  /**
   * @brief Subscriber
   */
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscriber_;

  /**
   * @brief Publisher
   */
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;

  /**
   * @brief Service server
   */
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_;

  /**
   * @brief Action server
   */
  rclcpp_action::Server<ros2_demo_package_interfaces::action::Fibonacci>::SharedPtr action_server_;

  /**
   * @brief Timer
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Dummy parameter (parameter)
   */
  double param_ = 1.0;

  /**
   * @brief Diagnostic updater
   */
  diagnostic_updater::Updater diagnostic_updater_{this};

  /**
   * @brief Diagnostic status indicating node health
   */
  struct DiagnosticStatus {
    unsigned char status = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    std::string message = "";
    std::map<std::string, std::string> key_value_pairs = {};
  } health_;

  /**
   * @brief Topic diagnostic to auto-diagnose a subscribed topic
   */
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> topic_diagnostic_;

  /**
   * @brief Configuration for auto-diagnosed topic subscription
   */
  TopicDiagnosticConfig topic_diagnostic_config_;

  /**
   * @brief Diagnosed publisher
   */
  std::unique_ptr<diagnostic_updater::DiagnosedPublisher<geometry_msgs::msg::PointStamped>> diagnosed_publisher_;

  /**
   * @brief Configuration for auto-diagnosed publisher
   */
  TopicDiagnosticConfig diagnosed_publisher_config_;
};

}  // namespace ros2_demo_package
