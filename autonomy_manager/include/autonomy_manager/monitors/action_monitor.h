//
// created by aljoscha on 26.02.26.
//

#ifndef AUTONOMY_MANAGER_ACTION_MONITOR_H
#define AUTONOMY_MANAGER_ACTION_MONITOR_H

#include <action_msgs/msg/goal_status_array.hpp>
#include <autonomy_manager/monitors/monitor_base.h>
#include <autonomy_manager/types.h>

namespace autonomy_manager
{

class ActionMonitor : public MonitorBase
{
public:
  ActionMonitor( rclcpp::Node::SharedPtr node,
                 const autonomy_manager_msgs::msg::AutonomyMode &stale_mode );

  void loadParameters() override;
  ModeWithContext currentMode() const override;
  void printConfig() const override;

  /// For testing: simulate a feedback arrival (resets the timer).
  void injectFeedback( const std::string &action_name );

private:
  void addAction( const std::string &action_name );
  void tryCreateFeedbackSubscription( const std::string &action_name );
  void statusCallback( const action_msgs::msg::GoalStatusArray::SharedPtr msg,
                       const std::string &action_name );
  void feedbackReceived( const std::string &action_name );
  void timeoutCallback( const std::string &action_name );

  struct ActionState {
    autonomy_manager_msgs::msg::AutonomyMode autonomy_type;
    autonomy_manager_msgs::msg::AutonomyMode current_mode;
    std::string operating_mode;
    double timeout{ 300.0 };
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub;
    rclcpp::GenericSubscription::SharedPtr feedback_sub;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr feedback_discovery_timer;
    bool executing{ false };
  };

  std::map<std::string, ActionState> actions_;
  autonomy_manager_msgs::msg::AutonomyMode stale_mode_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

} // namespace autonomy_manager
#endif // AUTONOMY_MANAGER_ACTION_MONITOR_H
