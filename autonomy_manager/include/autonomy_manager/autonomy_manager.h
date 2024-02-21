//
// Created by aljoscha on 20.02.24.
//

#ifndef HECTOR_ROBOT_BASE_NODE_AUTONOMYMANAGER_H
#define HECTOR_ROBOT_BASE_NODE_AUTONOMYMANAGER_H

#include <rclcpp/rclcpp.hpp>
#include <autonomy_manager/types.h>
#include <autonomy_manager_msgs/msg/autonomy_mode.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace autonomy_manager {
    using std::placeholders::_1;
    using std::placeholders::_2;

    class AutonomyManager : public rclcpp::Node {
    public:
        explicit AutonomyManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("autonomy_manager",
                                                                                                    options) {
            declare_parameter<std::vector<std::string>>("twist_nodes", std::vector<std::string>{});
            declare_parameter<std::vector<std::string>>("twist_autonomy_types", std::vector<std::string>{});
            declare_parameter<std::string>("twist_topic", "cmd_vel");
            std::vector<std::string> twist_nodes, twist_autonomy_types;
            std::string twist_topic;
            get_parameter("twist_nodes", twist_nodes);
            get_parameter("twist_autonomy_types", twist_autonomy_types);
            get_parameter("twist_topic", twist_topic);

            // setup subscriptions and publisher
            setupSubscriptions(twist_nodes, twist_autonomy_types, twist_topic);
            setupAutonomyPublisher();

        }

        AutonomyManager(const std::vector<std::string> &twist_nodes,
                        const std::vector<std::string> &twist_autonomy_types,
                        const std::string &twist_topic, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
                : Node("autonomy_manager", options) {
            // setup subscriptions and publisher
            setupSubscriptions(twist_nodes, twist_autonomy_types, twist_topic);
            setupAutonomyPublisher();
        }

        void setupSubscriptions(const std::vector<std::string> &twist_nodes,
                                const std::vector<std::string> &twist_autonomy_types,
                                const std::string &twist_topic) {
            for (int i = 0; i < twist_nodes.size(); i++) {
                autonomyTopicInfo_[twist_topic].autonomyTypePerNode[twist_nodes[i]] = fromString(
                        twist_autonomy_types[i]);
            }
            addSubscription<geometry_msgs::msg::Twist>(twist_topic);

            //TODO: add subscriptions for other topics (also adapt the config file) [e.g. topics for flexbe, etc.]

        }

        void setupAutonomyPublisher() {
            autonomyPublisher_ = create_publisher<autonomy_manager_msgs::msg::AutonomyMode>("/autonomy_mode", 10);
            autonomy_manager_msgs::msg::AutonomyMode msg;
            msg.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN;
            autonomyPublisher_->publish(msg);
        }

        /**
         * @brief Adds a subscription to a topic.
         *
         * This function creates a subscription to a specified topic and stores it in the subscriptions_ map.
         * The callback for the subscription is a lambda function that calls the subscriptionCallback function,
         * passing the received message, message info, and the topic name.
         *
         * @tparam T The message type of the topic.
         * @param topic The name of the topic to subscribe to.
         */
        template<typename T>
        void addSubscription(const std::string &topic) {
            subscriptions_[topic] = create_subscription<T>(topic, 5,
                                                           [this, topic](const std::shared_ptr<T> msg,
                                                                         const rclcpp::MessageInfo &msg_info) {
                                                               subscriptionCallback(msg, msg_info, topic);
                                                           }
            );
        }

        /**
         * @brief Callback function for subscriptions.
         *
         * This function is called whenever a message is received on a subscribed topic.
         * It logs the topic name and the node name of the publisher.
         *
         * @tparam T The message type of the topic.
         * @param msg The received message.
         * @param msg_info Information about the received message.
         * @param topic The name of the topic the message was received on.
         */
        template<typename T>
        void subscriptionCallback(const std::shared_ptr<T> msg, const rclcpp::MessageInfo &msg_info,
                                  const std::string &topic) {
            (void) msg; // suppress unused parameter warning
            RCLCPP_INFO(get_logger(), "Received message on topic %s", topic.c_str());
            auto node_name = getNodeName(msg_info, topic);
            RCLCPP_INFO(get_logger(), "Heartbeat from %s", node_name.c_str());
            autonomyCallback(topic, node_name);
        }

        /**
         * @brief Gets the name of the node that published a message.
         *
         * This function uses the publisher_gid from the message info to find the node name of the publisher.
         *
         * @param msg_info Information about the received message.
         * @return The name of the node that published the message.
         */
        std::string getNodeName(const rclcpp::MessageInfo &msg_info, const std::string &topic) {
            auto gid = msg_info.get_rmw_message_info().publisher_gid;
            auto info = get_publishers_info_by_topic(topic);
            for (auto &info_per_publisher: info) {
                if (std::equal(gid.data, gid.data + RMW_GID_STORAGE_SIZE, info_per_publisher.endpoint_gid().data())) {
                    return info_per_publisher.node_name();
                }
            }
            return "Unknown";
        }

        void autonomyCallback(const std::string &topic, const std::string &node_name) {
            if (autonomyTopicInfo_.find(topic) == autonomyTopicInfo_.end()) {
                RCLCPP_ERROR(get_logger(), "Received message on unknown topic %s", topic.c_str());
                return;
            }
            if (autonomyTopicInfo_[topic].autonomyTypePerNode.find(node_name) ==
                autonomyTopicInfo_[topic].autonomyTypePerNode.end()) {
                RCLCPP_ERROR(get_logger(), "Received message from unknown node %s on topic %s", node_name.c_str(),
                             topic.c_str());
                //TODO: how to set the mode for unknown nodes ??
                autonomy_manager_msgs::msg::AutonomyMode msg;
                msg.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN;
                autonomyPublisher_->publish(msg);
                return;
            }
            // get the required autonomy type for the node
            autonomy_manager_msgs::msg::AutonomyMode requiredAutonomyType = autonomyTopicInfo_[topic].autonomyTypePerNode[node_name];
            if (requiredAutonomyType.autonomy_mode != currentAutonomyMode.autonomy_mode) {
                // if the required autonomy type is different from the current autonomy type, publish the new autonomy type
                autonomyPublisher_->publish(requiredAutonomyType);
                currentAutonomyMode = requiredAutonomyType;
            }
        }


    private:
        std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_; // map of subscriptions per topic name
        std::map<std::string, AutonomyTopicInfo> autonomyTopicInfo_; // map of autonomy types per NodeName per topic
        rclcpp::Publisher<autonomy_manager_msgs::msg::AutonomyMode>::SharedPtr autonomyPublisher_;
        autonomy_manager_msgs::msg::AutonomyMode currentAutonomyMode;
    };

} // autonomy_manager

#endif //HECTOR_ROBOT_BASE_NODE_AUTONOMYMANAGER_H
