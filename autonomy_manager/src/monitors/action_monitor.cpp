#include <action_msgs/msg/goal_status.hpp>
#include <autonomy_manager/monitors/action_monitor.h>

namespace autonomy_manager
{

ActionMonitor::ActionMonitor( rclcpp::Node::SharedPtr node,
                              const autonomy_manager_msgs::msg::AutonomyMode &stale_mode )
    : MonitorBase( std::move( node ) ), stale_mode_( stale_mode )
{
}

void ActionMonitor::loadParameters()
{
  std::vector<std::string> observed_actions;
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = "List of action names to monitor";
  desc.read_only = true;
  observed_actions =
      node_->declare_parameter<std::vector<std::string>>( "observed_actions", {}, desc );

  for ( const auto &action_name : observed_actions ) { addAction( action_name ); }

  // Dynamic reconfiguration: detect new actions added to the list
  param_callback_ =
      node_->add_on_set_parameters_callback( [this]( const std::vector<rclcpp::Parameter> &params ) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for ( const auto &param : params ) {
          // Handle timeout updates for existing actions
          for ( auto &[name, state] : actions_ ) {
            if ( param.get_name() == name + ".timeout" ) {
              double new_timeout = param.as_double();
              state.timeout = new_timeout;
              // Recreate timer with new timeout
              if ( new_timeout > 0.0 && state.executing ) {
                state.timer =
                    node_->create_wall_timer( std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                  std::chrono::duration<double>( new_timeout ) ),
                                              [this, n = name]() { timeoutCallback( n ); } );
              } else if ( new_timeout <= 0.0 ) {
                state.timer.reset();
              }
            }
          }
        }
        return result;
      } );
}

void ActionMonitor::addAction( const std::string &action_name )
{
  if ( actions_.count( action_name ) )
    return;

  ActionState state;
  state.current_mode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::INACTIVE;

  // Load per-action parameters (namespaced: <action>.autonomy_type, <action>.timeout)
  std::string autonomy_type_str = "autonomy";
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Autonomy type when action '" + action_name + "' is executing";
    desc.read_only = true;
    autonomy_type_str = node_->declare_parameter<std::string>( action_name + ".autonomy_type",
                                                               autonomy_type_str, desc );
  }
  state.autonomy_type = fromString( autonomy_type_str );

  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Operating mode label for action '" + action_name + "'";
    desc.read_only = true;
    state.operating_mode =
        node_->declare_parameter<std::string>( action_name + ".operating_mode", "", desc );
  }

  state.timeout = 300.0; // default 5 minutes crash safety net
  param_subscriptions_.push_back( hector::createReconfigurableParameter(
      node_, action_name + ".timeout", std::ref( state.timeout ),
      "Feedback timeout in seconds for action '" + action_name + "' (crash detection)",
      hector::ParameterOptions<double>().setRange( 0.0, 3600.0, 0.1 ) ) );

  // Subscribe to status topic (typed — rtest compatible)
  const std::string status_topic = action_name + "/_action/status";
  state.status_sub = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
      status_topic, 10,
      [this, name = action_name]( const action_msgs::msg::GoalStatusArray::SharedPtr msg ) {
        statusCallback( msg, name );
      } );

  // Create timeout timer (cancelled until first feedback)
  if ( state.timeout > 0.0 ) {
    state.timer =
        node_->create_wall_timer( std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::duration<double>( state.timeout ) ),
                                  [this, name = action_name]() { timeoutCallback( name ); } );
    state.timer->cancel();
  }

  actions_[action_name] = std::move( state );

  // Try to discover and subscribe to the feedback topic
  tryCreateFeedbackSubscription( action_name );
}

void ActionMonitor::tryCreateFeedbackSubscription( const std::string &action_name )
{
  auto it = actions_.find( action_name );
  if ( it == actions_.end() || it->second.feedback_sub )
    return;

  const std::string feedback_topic = action_name + "/_action/feedback";
  auto topics_and_types = node_->get_topic_names_and_types();
  auto topic_it = topics_and_types.find( feedback_topic );

  if ( topic_it != topics_and_types.end() && !topic_it->second.empty() ) {
    const std::string &feedback_type = topic_it->second[0];
    it->second.feedback_sub = node_->create_generic_subscription(
        feedback_topic, feedback_type, rclcpp::QoS( 10 ),
        [this, name = action_name]( const std::shared_ptr<const rclcpp::SerializedMessage> &,
                                    const rclcpp::MessageInfo & ) { feedbackReceived( name ); } );
    RCLCPP_INFO( node_->get_logger(), "Subscribed to feedback topic '%s' (type: %s)",
                 feedback_topic.c_str(), feedback_type.c_str() );
    // Cancel discovery timer
    if ( it->second.feedback_discovery_timer ) {
      it->second.feedback_discovery_timer->cancel();
      it->second.feedback_discovery_timer.reset();
    }
  } else {
    // Retry periodically until the topic appears
    if ( !it->second.feedback_discovery_timer ) {
      it->second.feedback_discovery_timer =
          node_->create_wall_timer( std::chrono::seconds( 5 ), [this, name = action_name]() {
            tryCreateFeedbackSubscription( name );
          } );
      RCLCPP_INFO( node_->get_logger(),
                   "Feedback topic '%s' not yet available, retrying every 5 seconds",
                   feedback_topic.c_str() );
    }
  }
}

ModeWithContext ActionMonitor::currentMode() const
{
  ModeWithContext best;
  best.autonomy_mode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::INACTIVE;

  for ( const auto &[name, state] : actions_ ) {
    if ( autonomyPriority( state.current_mode.autonomy_mode ) <
         autonomyPriority( best.autonomy_mode.autonomy_mode ) ) {
      best.autonomy_mode = state.current_mode;
      best.operating_mode = state.operating_mode;
      best.source = "ActionMonitor: " + name;
    }
  }
  return best;
}

void ActionMonitor::injectFeedback( const std::string &action_name )
{
  feedbackReceived( action_name );
}

void ActionMonitor::statusCallback( const action_msgs::msg::GoalStatusArray::SharedPtr msg,
                                    const std::string &action_name )
{
  auto it = actions_.find( action_name );
  if ( it == actions_.end() )
    return;

  auto &state = it->second;
  const auto prev_mode = state.current_mode.autonomy_mode;

  // Check if any goal is currently executing
  bool executing = false;
  for ( const auto &status : msg->status_list ) {
    if ( status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
         status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ) {
      executing = true;
      break;
    }
  }

  state.executing = executing;

  if ( executing ) {
    state.current_mode = state.autonomy_type;
  } else {
    // All goals are terminal — immediately revert to INACTIVE
    state.current_mode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::INACTIVE;
    // Cancel the timeout timer since nothing is executing
    if ( state.timer ) {
      state.timer->cancel();
    }
  }

  if ( state.current_mode.autonomy_mode != prev_mode ) {
    notifyModeChange();
  }
}

void ActionMonitor::feedbackReceived( const std::string &action_name )
{
  auto it = actions_.find( action_name );
  if ( it == actions_.end() )
    return;

  auto &state = it->second;

  // Reset the timeout timer — action server is alive
  if ( state.timer && state.executing ) {
    state.timer->reset();
  }
}

void ActionMonitor::timeoutCallback( const std::string &action_name )
{
  auto it = actions_.find( action_name );
  if ( it == actions_.end() )
    return;

  auto &state = it->second;
  const auto prev_mode = state.current_mode.autonomy_mode;

  RCLCPP_WARN( node_->get_logger(),
               "Action '%s' feedback timeout — no feedback for %.1f seconds, assuming crash",
               action_name.c_str(), state.timeout );
  state.current_mode = stale_mode_;
  state.executing = false;

  if ( state.current_mode.autonomy_mode != prev_mode ) {
    notifyModeChange();
  }
}

void ActionMonitor::printConfig() const
{
  RCLCPP_INFO( node_->get_logger(), "[ActionMonitor] Configuration:" );
  for ( const auto &[name, state] : actions_ ) {
    RCLCPP_INFO(
        node_->get_logger(), "  Action '%s': autonomy_type=%s, operating_mode=%s, timeout=%.1f s",
        name.c_str(), toString( state.autonomy_type ).c_str(),
        state.operating_mode.empty() ? "<not set>" : state.operating_mode.c_str(), state.timeout );
  }
}

} // namespace autonomy_manager
