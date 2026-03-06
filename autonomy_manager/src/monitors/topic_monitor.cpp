#include <autonomy_manager/monitors/topic_monitor.h>

namespace autonomy_manager
{

TopicMonitor::TopicMonitor( rclcpp::Node::SharedPtr node,
                            const autonomy_manager_msgs::msg::AutonomyMode &stale_mode )
    : MonitorBase( std::move( node ) ), stale_mode_( stale_mode )
{
}

TopicMonitor::TopicMonitor( rclcpp::Node::SharedPtr node,
                            const std::vector<ObservedTopicInfo> &topic_infos,
                            const autonomy_manager_msgs::msg::AutonomyMode &stale_mode )
    : MonitorBase( std::move( node ) ), stale_mode_( stale_mode )
{
  for ( const auto &info : topic_infos ) { addTopicFromInfo( info ); }
}

void TopicMonitor::loadParameters()
{
  std::vector<std::string> observed_topics;
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "List of topic names to monitor";
    desc.read_only = true;
    observed_topics =
        node_->declare_parameter<std::vector<std::string>>( "observed_topics", {}, desc );
  }

  for ( const auto &topic_name : observed_topics ) { addTopic( topic_name ); }

  // Parameter callback for dynamic updates to node lists, autonomy types, and timeouts
  param_callback_ =
      node_->add_on_set_parameters_callback( [this]( const std::vector<rclcpp::Parameter> &params ) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for ( auto &[topic_name, _] : topic_info_ ) {
          bool mapping_changed = false;
          for ( const auto &param : params ) {
            if ( param.get_name() == topic_name + ".nodes" ||
                 param.get_name() == topic_name + ".autonomy_types" ||
                 param.get_name() == topic_name + ".default_autonomy_type" ) {
              mapping_changed = true;
              break;
            }
          }

          if ( mapping_changed ) {
            try {
              // Resolve parameter values: prefer values from the incoming batch
              // (they haven't been committed to the node yet) over current node values.
              auto resolve_vec = [&]( const std::string &name ) -> std::vector<std::string> {
                for ( const auto &p : params ) {
                  if ( p.get_name() == name ) {
                    return p.get_value<std::vector<std::string>>();
                  }
                }
                return node_->get_parameter( name ).get_value<std::vector<std::string>>();
              };

              auto resolve_str = [&]( const std::string &name ) -> std::string {
                for ( const auto &p : params ) {
                  if ( p.get_name() == name ) {
                    return p.get_value<std::string>();
                  }
                }
                return node_->get_parameter( name ).get_value<std::string>();
              };

              auto nodes = resolve_vec( topic_name + ".nodes" );
              auto types = resolve_vec( topic_name + ".autonomy_types" );
              auto default_type = resolve_str( topic_name + ".default_autonomy_type" );
              if ( nodes.size() != types.size() ) {
                // When parameters are set individually (non-atomically), one may be
                // updated before the other. Accept the change and defer the rebuild
                // until both arrays have matching length.
                RCLCPP_WARN( node_->get_logger(),
                             "Topic '%s': nodes (%zu) and autonomy_types (%zu) have different "
                             "lengths, deferring node mapping rebuild",
                             topic_name.c_str(), nodes.size(), types.size() );
              } else {
                rebuildNodeMapping( topic_name, nodes, types, default_type );
              }
            } catch ( const rclcpp::ParameterTypeException & ) {
              // Parameter not yet set, skip
            }
          }

          // Handle timeout changes
          for ( const auto &param : params ) {
            if ( param.get_name() == topic_name + ".timeout" ) {
              double new_timeout = param.as_double();
              timeout_per_topic_[topic_name] = new_timeout;
              recreateTimer( topic_name, new_timeout );
            }
          }
        }
        return result;
      } );
}

void TopicMonitor::addTopic( const std::string &topic_name )
{
  if ( topic_info_.count( topic_name ) )
    return;

  // Namespaced params: <topic>.nodes, <topic>.autonomy_types, <topic>.timeout
  std::vector<std::string> nodes, autonomy_types;

  // nodes and autonomy_types are reconfigurable (can add new nodes dynamically)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Node names publishing to '" + topic_name + "'";
    nodes = node_->declare_parameter<std::vector<std::string>>( topic_name + ".nodes", {}, desc );
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Autonomy types per node for '" + topic_name + "' (same order as nodes)";
    autonomy_types = node_->declare_parameter<std::vector<std::string>>(
        topic_name + ".autonomy_types", {}, desc );
  }

  double timeout = 0.0;
  param_subscriptions_.push_back( hector::createReconfigurableParameter(
      node_, topic_name + ".timeout", std::ref( timeout ),
      "Stale timeout in seconds for topic '" + topic_name + "' (0 = no timeout)",
      hector::ParameterOptions<double>().setRange( 0.0, 3600.0, 0.1 ) ) );
  timeout = node_->get_parameter( topic_name + ".timeout" ).get_value<double>();

  std::string default_autonomy_type;
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Default autonomy type for unmatched nodes on '" + topic_name + "'";
    default_autonomy_type =
        node_->declare_parameter<std::string>( topic_name + ".default_autonomy_type", "", desc );
  }

  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description =
        "Operating mode label for topic '" + topic_name + "' (e.g. driving, manipulation)";
    desc.read_only = true;
    operating_mode_per_topic_[topic_name] =
        node_->declare_parameter<std::string>( topic_name + ".operating_mode", "", desc );
  }

  // Build node-to-autonomy mapping
  rebuildNodeMapping( topic_name, nodes, autonomy_types, default_autonomy_type );

  // Set initial state
  autonomy_per_topic_[topic_name] = stale_mode_;
  timeout_per_topic_[topic_name] = timeout;

  // Setup timeout timer
  if ( timeout > 0.0 ) {
    timer_per_topic_[topic_name] =
        node_->create_wall_timer( std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::duration<double>( timeout ) ),
                                  [this, topic = topic_name]() { invalidateStaleTopics( topic ); } );
    timer_per_topic_[topic_name]->cancel(); // don't start until first message
  }

  // Discover topic type from the ROS graph and create subscription
  tryDiscoverTopicType( topic_name );
}

void TopicMonitor::addTopicFromInfo( const ObservedTopicInfo &info )
{
  rebuildNodeMapping( info.topicName, info.nodeNames, info.autonomyModes, info.defaultAutonomyType );
  autonomy_per_topic_[info.topicName] = stale_mode_;
  timeout_per_topic_[info.topicName] = info.timeout;
  if ( info.timeout > 0.0 ) {
    timer_per_topic_[info.topicName] = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>( info.timeout ) ),
        [this, topic = info.topicName]() { invalidateStaleTopics( topic ); } );
    timer_per_topic_[info.topicName]->cancel();
  }

  if ( !info.topicType.empty() ) {
    topic_types_[info.topicName] = info.topicType;
    setupSubscription( info.topicName, info.topicType );
  } else {
    tryDiscoverTopicType( info.topicName );
  }
}

void TopicMonitor::tryDiscoverTopicType( const std::string &topic_name )
{
  if ( subscriptions_.count( topic_name ) )
    return; // already subscribed

  // Resolve to fully-qualified name (e.g. "cmd_vel" -> "/athena/cmd_vel")
  // to match the keys returned by get_topic_names_and_types().
  std::string resolved =
      rclcpp::expand_topic_or_service_name( topic_name, node_->get_name(), node_->get_namespace() );
  auto topics_and_types = node_->get_topic_names_and_types();
  auto it = topics_and_types.find( resolved );

  if ( it != topics_and_types.end() && !it->second.empty() ) {
    const std::string &discovered_type = it->second[0];
    topic_types_[topic_name] = discovered_type;
    setupSubscription( topic_name, discovered_type );
    RCLCPP_INFO( node_->get_logger(), "Discovered type '%s' for topic '%s'",
                 discovered_type.c_str(), topic_name.c_str() );
    // Cancel discovery timer
    if ( type_discovery_timers_.count( topic_name ) ) {
      type_discovery_timers_[topic_name]->cancel();
      type_discovery_timers_.erase( topic_name );
    }
  } else {
    // Retry periodically until the topic appears
    if ( !type_discovery_timers_.count( topic_name ) ) {
      type_discovery_timers_[topic_name] =
          node_->create_wall_timer( std::chrono::seconds( 5 ), [this, topic = topic_name]() {
            tryDiscoverTopicType( topic );
          } );
      RCLCPP_INFO( node_->get_logger(),
                   "Topic '%s' not yet available, retrying type discovery every 5 seconds",
                   topic_name.c_str() );
    }
  }
}

void TopicMonitor::setupSubscription( const std::string &topic_name, const std::string &topic_type )
{
  if ( topic_type.empty() ) {
    RCLCPP_ERROR( node_->get_logger(),
                  "Topic type not specified for topic '%s'. Cannot create generic subscription.",
                  topic_name.c_str() );
    return;
  }

  subscriptions_[topic_name] = node_->create_generic_subscription(
      topic_name, topic_type, rclcpp::QoS( 10 ),
      [this, topic = topic_name]( const std::shared_ptr<const rclcpp::SerializedMessage> &msg,
                                  const rclcpp::MessageInfo &msg_info ) {
        genericCallback( msg, msg_info, topic );
      } );
}

void TopicMonitor::rebuildNodeMapping( const std::string &topic_name,
                                       const std::vector<std::string> &nodes,
                                       const std::vector<std::string> &autonomy_types,
                                       const std::string &default_autonomy_type )
{
  AutonomyTopicInfo info;
  for ( size_t i = 0; i < nodes.size() && i < autonomy_types.size(); ++i ) {
    auto mode = fromString( autonomy_types[i] );
    if ( isGlobPattern( nodes[i] ) ) {
      info.wildcardPatterns.emplace_back( nodes[i], mode );
    } else {
      info.autonomyTypePerNode[nodes[i]] = mode;
    }
  }
  if ( !default_autonomy_type.empty() ) {
    info.defaultAutonomyType = fromString( default_autonomy_type );
  }
  topic_info_[topic_name] = std::move( info );
}

void TopicMonitor::recreateTimer( const std::string &topic_name, double timeout )
{
  if ( timer_per_topic_.count( topic_name ) ) {
    timer_per_topic_[topic_name]->cancel();
    timer_per_topic_.erase( topic_name );
  }

  if ( timeout > 0.0 ) {
    timer_per_topic_[topic_name] =
        node_->create_wall_timer( std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::duration<double>( timeout ) ),
                                  [this, topic = topic_name]() { invalidateStaleTopics( topic ); } );
    timer_per_topic_[topic_name]->cancel();
  }
}

ModeWithContext TopicMonitor::currentMode() const
{
  ModeWithContext best;
  best.autonomy_mode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::INACTIVE;

  for ( const auto &[topic, mode] : autonomy_per_topic_ ) {
    if ( autonomyPriority( mode.autonomy_mode ) <
         autonomyPriority( best.autonomy_mode.autonomy_mode ) ) {
      best.autonomy_mode = mode;
      auto op_it = operating_mode_per_topic_.find( topic );
      best.operating_mode = ( op_it != operating_mode_per_topic_.end() ) ? op_it->second : "";
      best.source = "TopicMonitor: " + topic;
    }
  }
  return best;
}

void TopicMonitor::injectActivity( const std::string &topic, const std::string &node_name )
{
  autonomyCallback( topic, node_name );
}

void TopicMonitor::genericCallback( const std::shared_ptr<const rclcpp::SerializedMessage> &msg,
                                    const rclcpp::MessageInfo &msg_info, const std::string &topic )
{
  (void)msg; // content not needed, only publisher identity matters
  auto node_name = getNodeName( msg_info, topic );
  RCLCPP_DEBUG( node_->get_logger(), "Received message on topic %s from %s", topic.c_str(),
                node_name.c_str() );

  if ( timeout_per_topic_[topic] > 0 && timer_per_topic_.count( topic ) ) {
    timer_per_topic_[topic]->reset();
  }

  autonomyCallback( topic, node_name );
}

std::string TopicMonitor::getNodeName( const rclcpp::MessageInfo &msg_info,
                                       const std::string &topic ) const
{
  auto gid = msg_info.get_rmw_message_info().publisher_gid;
  auto info = node_->get_publishers_info_by_topic( topic );
  for ( const auto &info_per_publisher : info ) {
    if ( std::equal( gid.data, gid.data + RMW_GID_STORAGE_SIZE,
                     info_per_publisher.endpoint_gid().data() ) ) {
      return info_per_publisher.node_name();
    }
  }
  return "Unknown";
}

autonomy_manager_msgs::msg::AutonomyMode
TopicMonitor::resolveAutonomyMode( const AutonomyTopicInfo &info, const std::string &node_name ) const
{
  // 1. Exact match
  auto exact_it = info.autonomyTypePerNode.find( node_name );
  if ( exact_it != info.autonomyTypePerNode.end() ) {
    return exact_it->second;
  }

  // 2. Wildcard match (first matching pattern wins)
  for ( const auto &[pattern, mode] : info.wildcardPatterns ) {
    if ( matchesGlob( pattern, node_name ) ) {
      return mode;
    }
  }

  // 3. Default
  if ( info.defaultAutonomyType.has_value() ) {
    return info.defaultAutonomyType.value();
  }

  // 4. No match
  autonomy_manager_msgs::msg::AutonomyMode unknown;
  unknown.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN;
  return unknown;
}

void TopicMonitor::autonomyCallback( const std::string &topic, const std::string &node_name )
{
  auto topic_it = topic_info_.find( topic );
  if ( topic_it == topic_info_.end() ) {
    RCLCPP_ERROR( node_->get_logger(), "Received message on unknown topic %s", topic.c_str() );
    return;
  }

  const auto prev_mode = autonomy_per_topic_[topic].autonomy_mode;
  auto resolved = resolveAutonomyMode( topic_it->second, node_name );

  if ( resolved.autonomy_mode == autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN ) {
    RCLCPP_WARN( node_->get_logger(),
                 "Node '%s' on topic '%s' matched no rule (exact/wildcard/default) -> UNKNOWN",
                 node_name.c_str(), topic.c_str() );
  } else {
    RCLCPP_DEBUG( node_->get_logger(), "Node '%s' on topic '%s' resolved to %s", node_name.c_str(),
                  topic.c_str(), toString( resolved ).c_str() );
  }

  autonomy_per_topic_[topic] = resolved;

  if ( autonomy_per_topic_[topic].autonomy_mode != prev_mode ) {
    notifyModeChange();
  }
}

void TopicMonitor::invalidateStaleTopics( const std::string &topic )
{
  RCLCPP_INFO( node_->get_logger(), "Topic %s timed out", topic.c_str() );

  // Cancel the timer — it will be reset when the next message arrives
  if ( timer_per_topic_.count( topic ) ) {
    timer_per_topic_[topic]->cancel();
  }

  const auto prev_mode = autonomy_per_topic_[topic].autonomy_mode;
  autonomy_per_topic_[topic] = stale_mode_;
  if ( autonomy_per_topic_[topic].autonomy_mode != prev_mode ) {
    notifyModeChange();
  }
}

void TopicMonitor::printConfig() const
{
  RCLCPP_INFO( node_->get_logger(), "[TopicMonitor] Configuration:" );
  for ( const auto &[topic_name, info] : topic_info_ ) {
    auto type_it = topic_types_.find( topic_name );
    std::string type_str = ( type_it != topic_types_.end() && !type_it->second.empty() )
                               ? type_it->second
                               : "<discovering...>";
    auto timeout_it = timeout_per_topic_.find( topic_name );
    double timeout = ( timeout_it != timeout_per_topic_.end() ) ? timeout_it->second : 0.0;

    auto op_it = operating_mode_per_topic_.find( topic_name );
    std::string op_mode = ( op_it != operating_mode_per_topic_.end() && !op_it->second.empty() )
                              ? op_it->second
                              : "<not set>";
    RCLCPP_INFO( node_->get_logger(),
                 "  Topic '%s' (type: %s, timeout: %.1f s, operating_mode: %s):", topic_name.c_str(),
                 type_str.c_str(), timeout, op_mode.c_str() );

    for ( const auto &[node_name, mode] : info.autonomyTypePerNode ) {
      RCLCPP_INFO( node_->get_logger(), "    node '%s' -> %s", node_name.c_str(),
                   toString( mode ).c_str() );
    }
    for ( const auto &[pattern, mode] : info.wildcardPatterns ) {
      RCLCPP_INFO( node_->get_logger(), "    pattern '%s' -> %s", pattern.c_str(),
                   toString( mode ).c_str() );
    }
    if ( info.defaultAutonomyType.has_value() ) {
      RCLCPP_INFO( node_->get_logger(), "    default -> %s",
                   toString( info.defaultAutonomyType.value() ).c_str() );
    }
  }
}

} // namespace autonomy_manager
