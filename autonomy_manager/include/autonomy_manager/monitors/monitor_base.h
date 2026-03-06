//
// created by aljoscha on 26.02.26.
//

#ifndef AUTONOMY_MANAGER_MONITOR_BASE_H
#define AUTONOMY_MANAGER_MONITOR_BASE_H

#include <autonomy_manager/types.h>
#include <autonomy_manager_msgs/msg/autonomy_mode.hpp>
#include <functional>
#include <hector_ros2_utils/parameters/reconfigurable_parameter.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autonomy_manager
{

/// Callback type: monitors call this when their autonomy mode assessment changes.
using ModeChangeCallback = std::function<void()>;

/// Abstract base for all monitor types.
class MonitorBase
{
public:
  explicit MonitorBase( rclcpp::Node::SharedPtr node ) : node_( std::move( node ) ) { }
  virtual ~MonitorBase() = default;

  /// Load parameters from the node and create subscriptions.
  virtual void loadParameters() = 0;

  /// Returns the current autonomy mode (with operating-mode context) as determined by this monitor.
  virtual ModeWithContext currentMode() const = 0;

  /// Log the current configuration at INFO level.
  virtual void printConfig() const = 0;

  /// Set the callback to invoke when this monitor's mode changes.
  void setModeChangeCallback( ModeChangeCallback cb ) { onModeChange_ = std::move( cb ); }

protected:
  void notifyModeChange()
  {
    if ( onModeChange_ )
      onModeChange_();
  }

  rclcpp::Node::SharedPtr node_;
  ModeChangeCallback onModeChange_;
  std::vector<hector::ParameterSubscription> param_subscriptions_;
};

} // namespace autonomy_manager
#endif // AUTONOMY_MANAGER_MONITOR_BASE_H
