//
// Created by aljoscha on 21.02.24.
//

#ifndef AUTONOMY_MANAGER_TYPES_H
#define AUTONOMY_MANAGER_TYPES_H

#include <autonomy_manager_msgs/msg/autonomy_mode.hpp>
#include <map>

namespace autonomy_manager {

    autonomy_manager_msgs::msg::AutonomyMode fromString(const std::string &str) {
        std::string str_lower;
        std::transform(str.begin(), str.end(), std::back_inserter(str_lower),
                       [](unsigned char c) { return std::tolower(c); });
        autonomy_manager_msgs::msg::AutonomyMode autonomyMode;
        if (str_lower == "teleop") {
            autonomyMode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::TELEOPERATED;
        } else if (str_lower == "semi_autonomy") {
            autonomyMode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS;
        } else if (str_lower == "autonomy") {
            autonomyMode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::AUTONOMOUS;
        } else if (str_lower == "unknown") {
            autonomyMode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN;
        } else if (str_lower == "inactive") {
            autonomyMode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::INACTIVE;
        } else {
            throw std::invalid_argument("Invalid autonomy type: " + str);
        }
        return autonomyMode;
    }

    std::string toString(const autonomy_manager_msgs::msg::AutonomyMode &autonomyMode) {
        switch (autonomyMode.autonomy_mode) {
            case autonomy_manager_msgs::msg::AutonomyMode::TELEOPERATED:
                return "teleop";
            case autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS:
                return "semi_autonomy";
            case autonomy_manager_msgs::msg::AutonomyMode::AUTONOMOUS:
                return "autonomy";
            case autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN:
                return "unknown";
            case autonomy_manager_msgs::msg::AutonomyMode::INACTIVE:
                return "inactive";
        }
        return "not_found";
    }


    struct AutonomyTopicInfo {
        std::map<std::string, autonomy_manager_msgs::msg::AutonomyMode> autonomyTypePerNode;
    };

    struct ObservedTopicInfo {
        ObservedTopicInfo(const std::string &topicName, const std::vector<std::string> &nodeNames,
                          const std::vector<std::string> &autonomyModes, int timeout) : topicName(topicName),
                                                                                        nodeNames(nodeNames),
                                                                                        autonomyModes(autonomyModes),
                                                                                        timeout(timeout) {}

        std::string topicName;
        std::vector<std::string> nodeNames;
        std::vector<std::string> autonomyModes; // same order as nodeNames
        int timeout; // if <= 0, no timeout
    };

}
#endif //AUTONOMY_MANAGER_TYPES_H
