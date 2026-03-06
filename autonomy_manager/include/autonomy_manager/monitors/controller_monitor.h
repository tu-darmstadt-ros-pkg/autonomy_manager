//
// created by aljoscha on 26.02.26.
//

#ifndef AUTONOMY_MANAGER_CONTROLLER_MONITOR_H
#define AUTONOMY_MANAGER_CONTROLLER_MONITOR_H

#include <autonomy_manager/monitors/monitor_base.h>
#include <autonomy_manager/types.h>
#include <controller_manager_msgs/msg/controller_manager_activity.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace autonomy_manager
{

class ControllerMonitor : public MonitorBase
{
public:
  explicit ControllerMonitor( rclcpp::Node::SharedPtr node );

  void loadParameters() override;
  ModeWithContext currentMode() const override;
  void printConfig() const override;

private:
  struct ControllerConfig {
    std::string name;
    autonomy_manager_msgs::msg::AutonomyMode autonomy_type;
    std::string operating_mode;
    std::vector<std::string> joints;
    bool joints_queried{ false };
    bool require_motion{ true };
  };

  void addController( const std::string &controller_name );
  void activityCallback( const controller_manager_msgs::msg::ControllerManagerActivity::SharedPtr msg );
  void jointStateCallback( const sensor_msgs::msg::JointState::SharedPtr msg );
  bool isControllerCommanding( const ControllerConfig &config ) const;
  void queryJoints( const std::string &controller_name );

  std::string activity_topic_;
  std::string joint_states_topic_;
  std::string list_controllers_service_;
  double velocity_threshold_{ 0.01 };

  std::vector<ControllerConfig> configs_;

  // Per-controller: is it active (from activity topic)?
  std::map<std::string, bool> controller_active_;

  // Per-joint: latest absolute velocity
  std::map<std::string, double> joint_velocities_;
  // Per-joint: previous position for delta-based motion detection
  std::map<std::string, double> prev_joint_positions_;
  bool has_prev_positions_{ false };

  rclcpp::Subscription<controller_manager_msgs::msg::ControllerManagerActivity>::SharedPtr activity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

} // namespace autonomy_manager
#endif // AUTONOMY_MANAGER_CONTROLLER_MONITOR_H
