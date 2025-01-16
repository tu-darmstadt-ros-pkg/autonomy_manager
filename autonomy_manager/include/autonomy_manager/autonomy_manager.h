//
// Created by aljoscha on 20.02.24.
//

#ifndef HECTOR_ROBOT_BASE_NODE_AUTONOMY_MANAGER_H
#define HECTOR_ROBOT_BASE_NODE_AUTONOMY_MANAGER_H

#include <autonomy_manager/types.h>
#include <autonomy_manager_msgs/msg/autonomy_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autonomy_manager
{
using std::placeholders::_1;
using std::placeholders::_2;

class AutonomyManager final : public rclcpp::Node
{
public:
  explicit AutonomyManager( const rclcpp::NodeOptions &options = rclcpp::NodeOptions() )
      : Node( "autonomy_manager", options )
  {
    std::vector<ObservedTopicInfo> observedTopicInfos;
    std::vector<std::string> observedTopics;
    // first load observed topics
    declare_parameter( "observed_topics", std::vector<std::string>{} );
    get_parameter( "observed_topics", observedTopics );
    declare_parameter( "stale_autonomy_mode", "inactive" );
    std::string staleAutonomyMode;
    get_parameter( "stale_autonomy_mode", staleAutonomyMode );
    staleAutonomyMode_ = fromString( staleAutonomyMode );
    // for every observed topic load vector<string> <name>_nodes, vector<string> <name>_autonomy_types and int <name>_timeout
    std::vector<std::string> nodes, autonomy_types;
    int timeout;
    for ( const auto &topic : observedTopics ) {
      declare_parameter<std::vector<std::string>>( topic + "_nodes", std::vector<std::string>{} );
      declare_parameter<std::vector<std::string>>( topic + "_autonomy_types",
                                                   std::vector<std::string>{} );
      declare_parameter<int>( topic + "_timeout", 0 );
      get_parameter( topic + "_nodes", nodes );
      get_parameter( topic + "_autonomy_types", autonomy_types );
      get_parameter( topic + "_timeout", timeout );
      observedTopicInfos.emplace_back( topic, nodes, autonomy_types, timeout );
    }

    // setup subscriptions and publisher
    for ( const auto &observedTopicInfo : observedTopicInfos ) {
      setupSubscriptions( observedTopicInfo.nodeNames, observedTopicInfo.autonomyModes,
                          observedTopicInfo.topicName, observedTopicInfo.timeout );
    }
    setupAutonomyPublisher();
  }

  explicit AutonomyManager( const std::vector<ObservedTopicInfo> &observedTopicInfos,
                            const rclcpp::NodeOptions &options = rclcpp::NodeOptions() )
      : Node( "autonomy_manager", options )
  {
    // setup subscriptions and publisher
    staleAutonomyMode_.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::INACTIVE;
    for ( const auto &observedTopicInfo : observedTopicInfos ) {
      setupSubscriptions( observedTopicInfo.nodeNames, observedTopicInfo.autonomyModes,
                          observedTopicInfo.topicName, observedTopicInfo.timeout );
    }
    setupAutonomyPublisher();
  }

  void setupSubscriptions( const std::vector<std::string> &nodes,
                           const std::vector<std::string> &autonomy_types, const std::string &topic,
                           const int timeoutInSeconds )
  {
    for ( int i = 0; i < nodes.size(); i++ ) {
      autonomyTopicInfo_[topic].autonomyTypePerNode[nodes[i]] = fromString( autonomy_types[i] );
    }

    // read topic type from topic info
    auto info = get_topic_names_and_types();
    auto topicTypes = info[topic];
    for ( const auto &topicType : topicTypes ) {
      if ( topicType == "geometry_msgs/msg/Twist" ) {
        addSubscription<geometry_msgs::msg::Twist>( topic );
      } else if ( topicType == "geometry_msgs/msg/PoseStamped" ) {
        addSubscription<geometry_msgs::msg::PoseStamped>( topic );
      } else {
        RCLCPP_ERROR( get_logger(), "Unsupported topic type %s", topicType.c_str() );
      }
    }

    // addSubscription<geometry_msgs::msg::Twist>(topic);
    timeoutPerTopic_[topic] = timeoutInSeconds;
    if ( timeoutInSeconds > 0 ) {
      // add timer that will invalidate the topic if no message is received for a certain time
      timerPerTopic_[topic] =
          create_wall_timer( std::chrono::seconds( timeoutInSeconds ),
                             [this, topic]() { invalidateStaleTopics( topic ); } );
      timerPerTopic_[topic]->cancel(); // cancel the timer initially
    }
    autonomyTypePerTopic_[topic] = staleAutonomyMode_; // set initial autonomy mode
  }

  void setupAutonomyPublisher()
  {
    autonomyPublisher_ =
        create_publisher<autonomy_manager_msgs::msg::AutonomyMode>( "/autonomy_mode", 10 );
    autonomy_manager_msgs::msg::AutonomyMode msg;
    autonomyPublisher_->publish( staleAutonomyMode_ );
  }

  /**
   * @brief Adds a subscription to a topic.
   *
   * This function creates a subscription to a specified topic and stores it in the subscriptions_
   * map. The callback for the subscription is a lambda function that calls the subscriptionCallback
   * function, passing the received message, message info, and the topic name.
   *
   * @tparam MessageType The message type of the topic.
   * @param topic The name of the topic to subscribe to.
   */
  template<typename MessageType>
  void addSubscription( const std::string &topic )
  {
    subscriptions_[topic] = create_subscription<MessageType>(
        topic, 10,
        [this, topic]( const std::shared_ptr<MessageType> msg, const rclcpp::MessageInfo &msg_info ) {
          subscriptionCallback( msg, msg_info, topic );
        } );
  }

  /**
   * @brief Callback function for subscriptions.
   *
   * This function is called whenever a message is received on a subscribed topic.
   * It logs the topic name and the node name of the publisher.
   *
   * @tparam MessageType The message type of the topic.
   * @param msg The received message.
   * @param msg_info Information about the received message.
   * @param topic The name of the topic the message was received on.
   */
  template<typename MessageType>
  void subscriptionCallback( const std::shared_ptr<MessageType> msg,
                             const rclcpp::MessageInfo &msg_info, const std::string &topic )
  {
    (void)msg; // suppress unused parameter warning
    RCLCPP_INFO( get_logger(), "Received message on topic %s", topic.c_str() );
    auto node_name = getNodeName( msg_info, topic );
    RCLCPP_INFO( get_logger(), "Heartbeat from %s", node_name.c_str() );
    if ( timeoutPerTopic_[topic] > 0 )
      timerPerTopic_[topic]->reset(); // cancel and restart the timer
    autonomyCallback( topic, node_name );
  }

  /**
   * @brief Gets the name of the node that published a message.
   *
   * This function uses the publisher_gid from the message info to find the node name of the publisher.
   *
   * @param msg_info Information about the received message.
   * @param topic The name of the topic the message was received on.
   * @return The name of the node that published the message.
   */
  std::string getNodeName( const rclcpp::MessageInfo &msg_info, const std::string &topic ) const
  {
    auto gid = msg_info.get_rmw_message_info().publisher_gid;
    auto info = get_publishers_info_by_topic( topic );
    for ( auto &info_per_publisher : info ) {
      if ( std::equal( gid.data, gid.data + RMW_GID_STORAGE_SIZE,
                       info_per_publisher.endpoint_gid().data() ) ) {
        return info_per_publisher.node_name();
      }
    }
    return "Unknown";
  }

  void autonomyCallback( const std::string &topic, const std::string &node_name )
  {
    if ( autonomyTopicInfo_.find( topic ) == autonomyTopicInfo_.end() ) {
      RCLCPP_ERROR( get_logger(), "Received message on unknown topic %s", topic.c_str() );
      return;
    }
    const auto cacheOldAutonomyType = autonomyTypePerTopic_[topic];
    if ( autonomyTopicInfo_[topic].autonomyTypePerNode.find( node_name ) ==
         autonomyTopicInfo_[topic].autonomyTypePerNode.end() ) {
      RCLCPP_ERROR( get_logger(), "Received message from unknown node %s on topic %s",
                    node_name.c_str(), topic.c_str() );
      autonomyTypePerTopic_[topic].autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN;
    } else {
      // get the required autonomy type for the node
      autonomy_manager_msgs::msg::AutonomyMode requiredAutonomyType =
          autonomyTopicInfo_[topic].autonomyTypePerNode[node_name];
      autonomyTypePerTopic_[topic] = requiredAutonomyType;
    }
    if ( cacheOldAutonomyType.autonomy_mode != autonomyTypePerTopic_[topic].autonomy_mode )
      updateAutonomyMode();
  }

  void updateAutonomyMode()
  {
    // if one node is in unknown, the whole system is in unknown
    // if one node is in teleop (and no topic in unknown), the whole system is in teleop
    // if no mode in teleop, but one in semi_autonomy, the whole system is in semi_autonomy
    // if no mode in teleop or semi_autonomy, the whole system is in autonomy
    // iterate over all nodes and check if one is in teleop, semi_autonomy or unknown
    bool teleop = false, semi_autonomy = false, autonomy = false, unknown = false, inactive = false;
    for ( const auto &[fst, snd] : autonomyTypePerTopic_ ) {
      switch ( snd.autonomy_mode ) {
      case autonomy_manager_msgs::msg::AutonomyMode::TELEOPERATED:
        teleop = true;
        break;
      case autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS:
        semi_autonomy = true;
        break;
      case autonomy_manager_msgs::msg::AutonomyMode::AUTONOMOUS:
        autonomy = true;
        break;
      case autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN:
        unknown = true;
        break;
      case autonomy_manager_msgs::msg::AutonomyMode::INACTIVE:
        inactive = true;
        break;
      default:
        unknown = true;
        RCLCPP_ERROR( get_logger(), "Unknown autonomy mode" );
        break;
      }
    }
    if ( unknown ) {
      currentAutonomyMode_.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN;
    } else if ( teleop ) {
      currentAutonomyMode_.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::TELEOPERATED;
    } else if ( semi_autonomy ) {
      currentAutonomyMode_.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS;
    } else if ( autonomy ) {
      currentAutonomyMode_.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::AUTONOMOUS;
    } else if ( inactive ) {
      currentAutonomyMode_.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::INACTIVE;
    } else {
      RCLCPP_ERROR( get_logger(), "No valid autonomy mode found" );
    }
    autonomyPublisher_->publish( currentAutonomyMode_ );
    RCLCPP_INFO_STREAM( get_logger(), "Set autonomy mode to " << toString( currentAutonomyMode_ ) );
  }

  void invalidateStaleTopics( const std::string &topic )
  {
    RCLCPP_INFO( get_logger(), "Topic %s timed out", topic.c_str() );
    autonomyTypePerTopic_[topic] = staleAutonomyMode_;
    updateAutonomyMode();
  }

private:
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  // map of subscriptions per topic name
  std::map<std::string, autonomy_manager_msgs::msg::AutonomyMode> autonomyTypePerTopic_;
  // map of autonomy mode per subscribed topic
  std::map<std::string, rclcpp::TimerBase::SharedPtr> timerPerTopic_; // map of timers per topic
  std::map<std::string, int> timeoutPerTopic_;                        // map of timeouts per topic
  std::map<std::string, AutonomyTopicInfo> autonomyTopicInfo_; // map of autonomy types per NodeName per topic
  autonomy_manager_msgs::msg::AutonomyMode currentAutonomyMode_; // current autonomy mode
  autonomy_manager_msgs::msg::AutonomyMode staleAutonomyMode_;
  // autonomy mode to be published if no message is received
  rclcpp::Publisher<autonomy_manager_msgs::msg::AutonomyMode>::SharedPtr autonomyPublisher_;
};
} // namespace autonomy_manager

#endif // HECTOR_ROBOT_BASE_NODE_AUTONOMY_MANAGER_H
