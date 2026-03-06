//
// created by aljoscha on 26.02.26.
//

#ifndef AUTONOMY_MANAGER_TOPIC_MONITOR_H
#define AUTONOMY_MANAGER_TOPIC_MONITOR_H

#include <autonomy_manager/monitors/monitor_base.h>
#include <autonomy_manager/types.h>

namespace autonomy_manager
{

class TopicMonitor : public MonitorBase
{
public:
  TopicMonitor( rclcpp::Node::SharedPtr node,
                const autonomy_manager_msgs::msg::AutonomyMode &stale_mode );

  /// Construct from pre-built topic infos (backward compat / testing).
  TopicMonitor( rclcpp::Node::SharedPtr node, const std::vector<ObservedTopicInfo> &topic_infos,
                const autonomy_manager_msgs::msg::AutonomyMode &stale_mode );

  void loadParameters() override;
  ModeWithContext currentMode() const override;
  void printConfig() const override;

  /// For testing: directly inject a "message received from node X on topic Y".
  void injectActivity( const std::string &topic, const std::string &node_name );

private:
  void addTopic( const std::string &topic_name );
  void addTopicFromInfo( const ObservedTopicInfo &info );
  void tryDiscoverTopicType( const std::string &topic_name );
  void setupSubscription( const std::string &topic_name, const std::string &topic_type );
  void rebuildNodeMapping( const std::string &topic_name, const std::vector<std::string> &nodes,
                           const std::vector<std::string> &autonomy_types,
                           const std::string &default_autonomy_type = "" );
  void recreateTimer( const std::string &topic_name, double timeout );

  void genericCallback( const std::shared_ptr<const rclcpp::SerializedMessage> &msg,
                        const rclcpp::MessageInfo &msg_info, const std::string &topic );

  std::string getNodeName( const rclcpp::MessageInfo &msg_info, const std::string &topic ) const;

  void autonomyCallback( const std::string &topic, const std::string &node_name );
  autonomy_manager_msgs::msg::AutonomyMode resolveAutonomyMode( const AutonomyTopicInfo &info,
                                                                const std::string &node_name ) const;
  void invalidateStaleTopics( const std::string &topic );

  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::map<std::string, autonomy_manager_msgs::msg::AutonomyMode> autonomy_per_topic_;
  std::map<std::string, rclcpp::TimerBase::SharedPtr> timer_per_topic_;
  std::map<std::string, double> timeout_per_topic_;
  std::map<std::string, AutonomyTopicInfo> topic_info_;
  std::map<std::string, std::string> topic_types_;
  std::map<std::string, rclcpp::TimerBase::SharedPtr> type_discovery_timers_;
  std::map<std::string, std::string> operating_mode_per_topic_;
  autonomy_manager_msgs::msg::AutonomyMode stale_mode_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};

} // namespace autonomy_manager
#endif // AUTONOMY_MANAGER_TOPIC_MONITOR_H
