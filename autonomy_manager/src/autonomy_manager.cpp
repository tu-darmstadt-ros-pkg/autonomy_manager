//
// Created by aljoscha on 20.02.24.
//

#include <autonomy_manager/autonomy_manager.h>

namespace autonomy_manager
{

AutonomyManager::AutonomyManager( const rclcpp::NodeOptions &options )
    : hector::Node( "autonomy_manager", options )
{
  // Stale mode is the only parameter loaded by the manager itself
  std::string stale_mode_str = "inactive";
  declare_readonly_parameter( "stale_mode", stale_mode_str, "Mode when no activity detected" );
  stale_mode_ = fromString( stale_mode_str );

  setupMonitors();
  loadOperatingModeRules();
  setupAutonomyPublisher();
  printConfig();
}

AutonomyManager::AutonomyManager( const std::vector<ObservedTopicInfo> &observedTopicInfos,
                                  const rclcpp::NodeOptions &options )
    : hector::Node( "autonomy_manager", options )
{
  stale_mode_.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::INACTIVE;

  // Create topic monitor directly from pre-built info (backward compat / testing)
  auto self = std::shared_ptr<rclcpp::Node>( this, []( rclcpp::Node * ) { } );
  topic_monitor_ = std::make_unique<TopicMonitor>( self, observedTopicInfos, stale_mode_ );
  topic_monitor_->setModeChangeCallback( [this]() { updateAutonomyMode(); } );

  setupAutonomyPublisher();
}

void AutonomyManager::setupMonitors()
{
  auto mode_change_cb = [this]() { updateAutonomyMode(); };

  // shared_from_this() not available in constructor, use non-deleting shared_ptr
  auto self = std::shared_ptr<rclcpp::Node>( this, []( rclcpp::Node * ) { } );

  // E-Stop monitor — always created, loadParameters decides what to subscribe to
  estop_monitor_ = std::make_unique<EStopMonitor>( self );
  estop_monitor_->loadParameters();
  estop_monitor_->setModeChangeCallback( mode_change_cb );

  // Topic monitor
  topic_monitor_ = std::make_unique<TopicMonitor>( self, stale_mode_ );
  topic_monitor_->loadParameters();
  topic_monitor_->setModeChangeCallback( mode_change_cb );

  // Action monitor
  action_monitor_ = std::make_unique<ActionMonitor>( self, stale_mode_ );
  action_monitor_->loadParameters();
  action_monitor_->setModeChangeCallback( mode_change_cb );

  // Controller monitor
  controller_monitor_ = std::make_unique<ControllerMonitor>( self );
  controller_monitor_->loadParameters();
  controller_monitor_->setModeChangeCallback( mode_change_cb );
}

void AutonomyManager::loadOperatingModeRules()
{
  // Load indexed rules: operating_mode_rules.rule_0.{autonomy, mode, output}, rule_1, ...
  // Stop at the first index where the ".autonomy" key is not provided in the config.
  for ( int i = 0;; ++i ) {
    const std::string prefix = "operating_mode_rules.rule_" + std::to_string( i );

    // Declare with empty default — if the value stays empty, no override was provided.
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = true;

    desc.description = "Autonomy level match for rule " + std::to_string( i );
    auto autonomy = declare_parameter<std::string>( prefix + ".autonomy", "", desc );
    if ( autonomy.empty() ) {
      break;
    }

    desc.description = "Operating mode match for rule " + std::to_string( i );
    auto mode = declare_parameter<std::string>( prefix + ".mode", "", desc );

    desc.description = "Output string for rule " + std::to_string( i );
    auto output = declare_parameter<std::string>( prefix + ".output", "", desc );

    operating_mode_rules_.push_back( { autonomy, mode, output } );
  }
}

void AutonomyManager::setupAutonomyPublisher()
{
  autonomy_publisher_ = create_publisher<autonomy_manager_msgs::msg::AutonomyMode>(
      "autonomy_mode", rclcpp::QoS( 10 ).transient_local() );
  operating_mode_publisher_ = create_publisher<std_msgs::msg::String>(
      "operating_mode", rclcpp::QoS( 10 ).transient_local() );

  current_context_.autonomy_mode = stale_mode_;
  autonomy_publisher_->publish( current_context_.autonomy_mode );

  // Only publish initial operating mode if a rule matches
  auto initial_op = resolveOperatingMode( current_context_ );
  if ( initial_op.has_value() ) {
    current_operating_mode_string_ = initial_op.value();
    std_msgs::msg::String op_msg;
    op_msg.data = current_operating_mode_string_;
    operating_mode_publisher_->publish( op_msg );
  }
}

void AutonomyManager::updateAutonomyMode()
{
  ModeWithContext best;
  best.autonomy_mode = stale_mode_;

  // Collect modes from all active monitors and pick highest priority
  auto consider = [&best]( const ModeWithContext &ctx ) {
    if ( autonomyPriority( ctx.autonomy_mode.autonomy_mode ) <
         autonomyPriority( best.autonomy_mode.autonomy_mode ) ) {
      best = ctx;
    }
  };

  if ( estop_monitor_ )
    consider( estop_monitor_->currentMode() );
  if ( topic_monitor_ )
    consider( topic_monitor_->currentMode() );
  if ( action_monitor_ )
    consider( action_monitor_->currentMode() );
  if ( controller_monitor_ )
    consider( controller_monitor_->currentMode() );

  bool autonomy_changed =
      best.autonomy_mode.autonomy_mode != current_context_.autonomy_mode.autonomy_mode;
  bool context_changed = autonomy_changed || best.operating_mode != current_context_.operating_mode;

  const std::string &reason = best.source.empty() ? "no active monitor (stale)" : best.source;

  if ( autonomy_changed ) {
    current_context_ = best;
    autonomy_publisher_->publish( current_context_.autonomy_mode );
    RCLCPP_INFO( get_logger(), "Autonomy mode -> %s (reason: %s)",
                 toString( current_context_.autonomy_mode ).c_str(), reason.c_str() );
  } else if ( context_changed ) {
    current_context_ = best;
  }

  if ( context_changed ) {
    auto new_op = resolveOperatingMode( current_context_ );
    // Only publish when a rule matches; no match = hold last published value
    if ( new_op.has_value() && new_op.value() != current_operating_mode_string_ ) {
      current_operating_mode_string_ = new_op.value();
      std_msgs::msg::String op_msg;
      op_msg.data = current_operating_mode_string_;
      operating_mode_publisher_->publish( op_msg );
      RCLCPP_INFO( get_logger(), "Operating mode -> '%s' (reason: %s)",
                   current_operating_mode_string_.c_str(), reason.c_str() );
    }
  }
}

std::optional<std::string> AutonomyManager::resolveOperatingMode( const ModeWithContext &context ) const
{
  const std::string autonomy_str = toString( context.autonomy_mode );

  for ( const auto &rule : operating_mode_rules_ ) {
    bool autonomy_match = ( rule.autonomy == "*" || rule.autonomy == autonomy_str );
    bool mode_match = ( rule.mode == "*" || rule.mode == context.operating_mode );
    if ( autonomy_match && mode_match ) {
      return rule.output;
    }
  }

  // No rule matched — hold last published value
  return std::nullopt;
}

void AutonomyManager::printConfig() const
{
  RCLCPP_INFO( get_logger(), "=== AutonomyManager Configuration ===" );
  RCLCPP_INFO( get_logger(), "stale_mode: %s", toString( stale_mode_ ).c_str() );
  if ( estop_monitor_ )
    estop_monitor_->printConfig();
  if ( topic_monitor_ )
    topic_monitor_->printConfig();
  if ( action_monitor_ )
    action_monitor_->printConfig();
  if ( controller_monitor_ )
    controller_monitor_->printConfig();

  RCLCPP_INFO( get_logger(), "[OperatingModeRules] %zu rules:", operating_mode_rules_.size() );
  for ( size_t i = 0; i < operating_mode_rules_.size(); ++i ) {
    const auto &rule = operating_mode_rules_[i];
    RCLCPP_INFO( get_logger(), "  rule_%zu: (autonomy=%s, mode=%s) -> \"%s\"", i,
                 rule.autonomy.c_str(), rule.mode.c_str(), rule.output.c_str() );
  }
  RCLCPP_INFO( get_logger(), "=====================================" );
}

} // namespace autonomy_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE( autonomy_manager::AutonomyManager )
