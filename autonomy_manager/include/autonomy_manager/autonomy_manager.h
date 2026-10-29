//
// Created by aljoscha on 20.02.24.
//

#ifndef HECTOR_ROBOT_BASE_NODE_AUTONOMY_MANAGER_H
#define HECTOR_ROBOT_BASE_NODE_AUTONOMY_MANAGER_H

#include <autonomy_manager/monitors/action_monitor.h>
#include <autonomy_manager/monitors/controller_monitor.h>
#include <autonomy_manager/monitors/estop_monitor.h>
#include <autonomy_manager/monitors/topic_monitor.h>
#include <autonomy_manager/types.h>
#include <autonomy_manager_msgs/msg/autonomy_mode.hpp>
#include <hector_ros2_utils/node.hpp>
#include <std_msgs/msg/string.hpp>

namespace autonomy_manager
{

class AutonomyManager final : public hector::Node
{
public:
  /// Construct from ROS parameters (production use, composable node).
  explicit AutonomyManager( const rclcpp::NodeOptions &options = rclcpp::NodeOptions() );

  /// Construct from pre-built topic infos (backward compatibility / testing).
  explicit AutonomyManager( const std::vector<ObservedTopicInfo> &observedTopicInfos,
                            const rclcpp::NodeOptions &options = rclcpp::NodeOptions() );

  ~AutonomyManager() override = default;

  /// Access monitors for testing
  TopicMonitor *topicMonitor() { return topic_monitor_.get(); }

  EStopMonitor *estopMonitor() { return estop_monitor_.get(); }

  ActionMonitor *actionMonitor() { return action_monitor_.get(); }

  ControllerMonitor *controllerMonitor() { return controller_monitor_.get(); }

private:
  void setupMonitors();
  void setupAutonomyPublisher();
  void loadOperatingModeRules();
  void updateAutonomyMode();
  std::optional<std::string> resolveOperatingMode( const ModeWithContext &context ) const;
  void printConfig() const;

  // Monitors (optional — only created if configured)
  std::unique_ptr<EStopMonitor> estop_monitor_;
  std::unique_ptr<TopicMonitor> topic_monitor_;
  std::unique_ptr<ActionMonitor> action_monitor_;
  std::unique_ptr<ControllerMonitor> controller_monitor_;

  // Configuration
  autonomy_manager_msgs::msg::AutonomyMode stale_mode_;
  std::vector<OperatingModeRule> operating_mode_rules_;

  // Output
  rclcpp::Publisher<autonomy_manager_msgs::msg::AutonomyMode>::SharedPtr autonomy_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operating_mode_publisher_;
  ModeWithContext current_context_;
  std::string current_operating_mode_string_;
};

} // namespace autonomy_manager

#endif // HECTOR_ROBOT_BASE_NODE_AUTONOMY_MANAGER_H
