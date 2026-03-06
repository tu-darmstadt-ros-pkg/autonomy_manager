//
// created by aljoscha on 26.02.26.
//

#ifndef AUTONOMY_MANAGER_ESTOP_MONITOR_H
#define AUTONOMY_MANAGER_ESTOP_MONITOR_H

#include <autonomy_manager/monitors/monitor_base.h>
#include <std_msgs/msg/bool.hpp>

namespace autonomy_manager
{

class EStopMonitor : public MonitorBase
{
public:
  explicit EStopMonitor( rclcpp::Node::SharedPtr node );

  void loadParameters() override;
  ModeWithContext currentMode() const override;
  void printConfig() const override;

private:
  void hardEStopCallback( const std_msgs::msg::Bool::SharedPtr msg );
  void softEStopCallback( const std_msgs::msg::Bool::SharedPtr msg );

  std::string hard_estop_topic_;
  std::string soft_estop_topic_;
  bool hard_estop_active_{ false };
  bool soft_estop_active_{ false };
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hard_estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr soft_estop_sub_;
};

} // namespace autonomy_manager
#endif // AUTONOMY_MANAGER_ESTOP_MONITOR_H
