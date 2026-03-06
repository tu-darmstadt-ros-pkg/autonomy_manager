#include <autonomy_manager/monitors/estop_monitor.h>

namespace autonomy_manager
{

EStopMonitor::EStopMonitor( rclcpp::Node::SharedPtr node ) : MonitorBase( std::move( node ) ) { }

void EStopMonitor::loadParameters()
{
  rcl_interfaces::msg::ParameterDescriptor hard_desc;
  hard_desc.description = "Topic for hardware e-stop (std_msgs/Bool)";
  hard_desc.read_only = true;
  hard_estop_topic_ = node_->declare_parameter<std::string>( "hard_e_stop_topic", "", hard_desc );

  rcl_interfaces::msg::ParameterDescriptor soft_desc;
  soft_desc.description = "Topic for software e-stop (std_msgs/Bool)";
  soft_desc.read_only = true;
  soft_estop_topic_ = node_->declare_parameter<std::string>( "soft_e_stop_topic", "", soft_desc );

  auto qos = rclcpp::QoS( 1 ).transient_local();

  if ( !hard_estop_topic_.empty() ) {
    hard_estop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        hard_estop_topic_, qos,
        [this]( const std_msgs::msg::Bool::SharedPtr msg ) { hardEStopCallback( msg ); } );
  } else {
    RCLCPP_WARN(
        node_->get_logger(),
        "Parameter 'hard_e_stop_topic' is not set. Hardware e-stop monitoring will be disabled." );
  }

  if ( !soft_estop_topic_.empty() ) {
    soft_estop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        soft_estop_topic_, qos,
        [this]( const std_msgs::msg::Bool::SharedPtr msg ) { softEStopCallback( msg ); } );
  } else {
    RCLCPP_WARN(
        node_->get_logger(),
        "Parameter 'soft_e_stop_topic' is not set. Software e-stop monitoring will be disabled." );
  }
}

ModeWithContext EStopMonitor::currentMode() const
{
  ModeWithContext result;
  result.operating_mode = "emergency";
  if ( hard_estop_active_ ) {
    result.autonomy_mode.autonomy_mode =
        autonomy_manager_msgs::msg::AutonomyMode::EMERGENCY_STOP_HARD;
    result.source = "EStopMonitor: hard e-stop active";
  } else if ( soft_estop_active_ ) {
    result.autonomy_mode.autonomy_mode =
        autonomy_manager_msgs::msg::AutonomyMode::EMERGENCY_STOP_SOFT;
    result.source = "EStopMonitor: soft e-stop active";
  } else {
    result.autonomy_mode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::INACTIVE;
    result.operating_mode = "";
  }
  return result;
}

void EStopMonitor::hardEStopCallback( const std_msgs::msg::Bool::SharedPtr msg )
{
  const bool prev = hard_estop_active_;
  hard_estop_active_ = msg->data;
  if ( hard_estop_active_ != prev ) {
    notifyModeChange();
  }
}

void EStopMonitor::softEStopCallback( const std_msgs::msg::Bool::SharedPtr msg )
{
  const bool prev = soft_estop_active_;
  soft_estop_active_ = msg->data;
  if ( soft_estop_active_ != prev ) {
    notifyModeChange();
  }
}

void EStopMonitor::printConfig() const
{
  RCLCPP_INFO( node_->get_logger(), "[EStopMonitor] Configuration:" );
  RCLCPP_INFO( node_->get_logger(), "  hard_e_stop_topic: %s",
               hard_estop_topic_.empty() ? "<not set>" : hard_estop_topic_.c_str() );
  RCLCPP_INFO( node_->get_logger(), "  soft_e_stop_topic: %s",
               soft_estop_topic_.empty() ? "<not set>" : soft_estop_topic_.c_str() );
}

} // namespace autonomy_manager
