#include <autonomy_manager/monitors/controller_monitor.h>

#include <cmath>
#include <set>
#include <sstream>

namespace autonomy_manager
{

ControllerMonitor::ControllerMonitor( rclcpp::Node::SharedPtr node )
    : MonitorBase( std::move( node ) )
{
}

void ControllerMonitor::loadParameters()
{
  // Global controller monitor settings
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Topic for controller manager activity";
    desc.read_only = true;
    activity_topic_ =
        node_->declare_parameter<std::string>( "controller_manager_activity_topic", "", desc );
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Topic for joint states";
    desc.read_only = true;
    joint_states_topic_ = node_->declare_parameter<std::string>( "joint_states_topic", "", desc );
  }

  // Derive list_controllers service name from activity topic
  // e.g., "/controller_manager/controller_manager_activity" → "/controller_manager/list_controllers"
  std::string default_service;
  if ( !activity_topic_.empty() ) {
    auto last_slash = activity_topic_.rfind( '/' );
    if ( last_slash != std::string::npos ) {
      default_service = activity_topic_.substr( 0, last_slash ) + "/list_controllers";
    }
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Service name for querying controller joints (ListControllers)";
    desc.read_only = true;
    list_controllers_service_ =
        node_->declare_parameter<std::string>( "list_controllers_service", default_service, desc );
  }

  // Reconfigurable velocity threshold
  param_subscriptions_.push_back( hector::createReconfigurableParameter(
      node_, "joint_velocity_threshold", std::ref( velocity_threshold_ ),
      "Joint velocity threshold for motion detection (rad/s)",
      hector::ParameterOptions<double>().setRange( 0.001, 1.0, 0.001 ) ) );

  // Load controller list
  std::vector<std::string> observed_controllers;
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "List of controller names to monitor";
    desc.read_only = true;
    observed_controllers =
        node_->declare_parameter<std::vector<std::string>>( "observed_controllers", {}, desc );
  }

  for ( const auto &ctrl_name : observed_controllers ) { addController( ctrl_name ); }

  // Create subscriptions if configured
  if ( !activity_topic_.empty() ) {
    activity_sub_ =
        node_->create_subscription<controller_manager_msgs::msg::ControllerManagerActivity>(
            activity_topic_, rclcpp::QoS( 10 ).transient_local(),
            [this]( const controller_manager_msgs::msg::ControllerManagerActivity::SharedPtr msg ) {
              activityCallback( msg );
            } );
  }

  if ( !joint_states_topic_.empty() ) {
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, 10,
        [this]( const sensor_msgs::msg::JointState::SharedPtr msg ) { jointStateCallback( msg ); } );
  }

  // Create service client for joint discovery
  if ( !list_controllers_service_.empty() ) {
    list_controllers_client_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
        list_controllers_service_ );
  }

  // Param callback for dynamic require_motion changes
  param_callback_ =
      node_->add_on_set_parameters_callback( [this]( const std::vector<rclcpp::Parameter> &params ) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for ( const auto &param : params ) {
          for ( auto &config : configs_ ) {
            if ( param.get_name() == config.name + ".require_motion" ) {
              config.require_motion = param.as_bool();
              notifyModeChange(); // re-evaluate with new setting
            }
          }
        }
        return result;
      } );
}

void ControllerMonitor::addController( const std::string &controller_name )
{
  ControllerConfig config;
  config.name = controller_name;

  // Namespaced params: <controller>.autonomy_type, <controller>.require_motion
  std::string autonomy_type_str = "teleop";
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Autonomy type when controller '" + controller_name + "' is commanding";
    desc.read_only = true;
    autonomy_type_str = node_->declare_parameter<std::string>( controller_name + ".autonomy_type",
                                                               autonomy_type_str, desc );
  }
  config.autonomy_type = fromString( autonomy_type_str );

  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Operating mode label for controller '" + controller_name + "'";
    desc.read_only = true;
    config.operating_mode =
        node_->declare_parameter<std::string>( controller_name + ".operating_mode", "", desc );
  }

  // Joints are discovered dynamically via ListControllers service
  // (no parameter needed)

  // require_motion is reconfigurable
  param_subscriptions_.push_back( hector::createReconfigurableParameter(
      node_, controller_name + ".require_motion", std::ref( config.require_motion ),
      "Whether controller '" + controller_name + "' requires joint motion to be considered commanding",
      hector::ParameterOptions<bool>() ) );

  controller_active_[controller_name] = false;
  configs_.push_back( std::move( config ) );
}

ModeWithContext ControllerMonitor::currentMode() const
{
  ModeWithContext best;
  best.autonomy_mode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::INACTIVE;

  for ( const auto &config : configs_ ) {
    if ( isControllerCommanding( config ) ) {
      if ( autonomyPriority( config.autonomy_type.autonomy_mode ) <
           autonomyPriority( best.autonomy_mode.autonomy_mode ) ) {
        best.autonomy_mode = config.autonomy_type;
        best.operating_mode = config.operating_mode;
        best.source = "ControllerMonitor: " + config.name;
      }
    }
  }
  return best;
}

void ControllerMonitor::activityCallback(
    const controller_manager_msgs::msg::ControllerManagerActivity::SharedPtr msg )
{
  bool changed = false;
  for ( auto &config : configs_ ) {
    bool found_active = false;
    for ( const auto &controller : msg->controllers ) {
      if ( controller.name == config.name ) {
        found_active = ( controller.state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE );
        break;
      }
    }

    bool was_active = controller_active_[config.name];
    if ( was_active != found_active ) {
      controller_active_[config.name] = found_active;
      changed = true;
    }

    // Query joints when controller becomes active for the first time
    if ( found_active && !config.joints_queried ) {
      queryJoints( config.name );
    }
  }
  if ( changed ) {
    notifyModeChange();
  }
}

void ControllerMonitor::queryJoints( const std::string &controller_name )
{
  if ( !list_controllers_client_ || !list_controllers_client_->service_is_ready() ) {
    RCLCPP_WARN( node_->get_logger(),
                 "ListControllers service not available, cannot discover joints for '%s'",
                 controller_name.c_str() );
    return;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  list_controllers_client_->async_send_request(
      request,
      [this, controller_name](
          rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future ) {
        auto response = future.get();
        if ( !response ) {
          RCLCPP_WARN( node_->get_logger(), "ListControllers returned null response for '%s'",
                       controller_name.c_str() );
          return;
        }

        // Find the matching controller and extract joints from claimed_interfaces
        for ( const auto &ctrl : response->controller ) {
          if ( ctrl.name != controller_name )
            continue;

          std::set<std::string> joint_set;
          for ( const auto &iface : ctrl.claimed_interfaces ) {
            // Interface format: "<joint>/<type>" or "<chained_ctrl>/<joint>/<type>"
            // The joint name is always the second-to-last segment.
            auto last_slash = iface.rfind( '/' );
            if ( last_slash != std::string::npos && last_slash > 0 ) {
              auto prev_slash = iface.rfind( '/', last_slash - 1 );
              if ( prev_slash != std::string::npos ) {
                // Chained: extract between prev_slash and last_slash
                joint_set.insert( iface.substr( prev_slash + 1, last_slash - prev_slash - 1 ) );
              } else {
                // Direct: extract before last_slash
                joint_set.insert( iface.substr( 0, last_slash ) );
              }
            }
          }

          // Find and update the config
          for ( auto &config : configs_ ) {
            if ( config.name == controller_name ) {
              config.joints.assign( joint_set.begin(), joint_set.end() );
              config.joints_queried = true;

              std::ostringstream oss;
              for ( size_t i = 0; i < config.joints.size(); ++i ) {
                if ( i > 0 )
                  oss << ", ";
                oss << config.joints[i];
              }
              RCLCPP_INFO( node_->get_logger(), "Discovered joints for '%s': [%s]",
                           controller_name.c_str(), oss.str().c_str() );

              notifyModeChange();
              break;
            }
          }
          break;
        }
      } );
}

void ControllerMonitor::jointStateCallback( const sensor_msgs::msg::JointState::SharedPtr msg )
{
  bool changed = false;

  for ( size_t i = 0; i < msg->name.size(); ++i ) {
    const auto &joint_name = msg->name[i];

    // Update velocity: prefer published velocity, fall back to position delta
    if ( i < msg->velocity.size() ) {
      double prev = joint_velocities_.count( joint_name ) ? joint_velocities_[joint_name] : 0.0;
      joint_velocities_[joint_name] = std::abs( msg->velocity[i] );
      bool was_moving = prev > velocity_threshold_;
      bool is_moving = joint_velocities_[joint_name] > velocity_threshold_;
      if ( was_moving != is_moving )
        changed = true;
    } else if ( i < msg->position.size() && has_prev_positions_ ) {
      // Position delta fallback
      auto it = prev_joint_positions_.find( joint_name );
      if ( it != prev_joint_positions_.end() ) {
        double delta = std::abs( msg->position[i] - it->second );
        double prev = joint_velocities_.count( joint_name ) ? joint_velocities_[joint_name] : 0.0;
        joint_velocities_[joint_name] = delta; // approximate velocity as position delta
        bool was_moving = prev > velocity_threshold_;
        bool is_moving = delta > velocity_threshold_;
        if ( was_moving != is_moving )
          changed = true;
      }
    }

    // Store current position for next delta calculation
    if ( i < msg->position.size() ) {
      prev_joint_positions_[joint_name] = msg->position[i];
    }
  }

  has_prev_positions_ = true;

  if ( changed ) {
    notifyModeChange();
  }
}

bool ControllerMonitor::isControllerCommanding( const ControllerConfig &config ) const
{
  auto active_it = controller_active_.find( config.name );
  if ( active_it == controller_active_.end() || !active_it->second ) {
    return false; // controller not active
  }

  if ( !config.require_motion ) {
    return true; // active is enough
  }

  // Check if any of the controller's configured joints are moving
  for ( const auto &joint : config.joints ) {
    auto vel_it = joint_velocities_.find( joint );
    if ( vel_it != joint_velocities_.end() && vel_it->second > velocity_threshold_ ) {
      return true;
    }
  }
  return false;
}

void ControllerMonitor::printConfig() const
{
  RCLCPP_INFO( node_->get_logger(), "[ControllerMonitor] Configuration:" );
  RCLCPP_INFO( node_->get_logger(), "  activity_topic: %s",
               activity_topic_.empty() ? "<not set>" : activity_topic_.c_str() );
  RCLCPP_INFO( node_->get_logger(), "  joint_states_topic: %s",
               joint_states_topic_.empty() ? "<not set>" : joint_states_topic_.c_str() );
  RCLCPP_INFO( node_->get_logger(), "  list_controllers_service: %s",
               list_controllers_service_.empty() ? "<not set>" : list_controllers_service_.c_str() );
  RCLCPP_INFO( node_->get_logger(), "  joint_velocity_threshold: %.4f", velocity_threshold_ );
  for ( const auto &config : configs_ ) {
    RCLCPP_INFO( node_->get_logger(),
                 "  Controller '%s': autonomy_type=%s, operating_mode=%s, require_motion=%s",
                 config.name.c_str(), toString( config.autonomy_type ).c_str(),
                 config.operating_mode.empty() ? "<not set>" : config.operating_mode.c_str(),
                 config.require_motion ? "true" : "false" );
  }
}

} // namespace autonomy_manager
