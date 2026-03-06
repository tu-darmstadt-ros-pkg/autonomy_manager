//
// Created by aljoscha on 21.02.24.
//

#ifndef AUTONOMY_MANAGER_TYPES_H
#define AUTONOMY_MANAGER_TYPES_H

#include <autonomy_manager_msgs/msg/autonomy_mode.hpp>
#include <fnmatch.h>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace autonomy_manager
{

inline autonomy_manager_msgs::msg::AutonomyMode fromString( const std::string &str )
{
  std::string str_lower;
  std::transform( str.begin(), str.end(), std::back_inserter( str_lower ),
                  []( unsigned char c ) { return std::tolower( c ); } );
  autonomy_manager_msgs::msg::AutonomyMode autonomyMode;
  if ( str_lower == "teleop" ) {
    autonomyMode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::TELEOPERATED;
  } else if ( str_lower == "semi_autonomy" ) {
    autonomyMode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS;
  } else if ( str_lower == "autonomy" ) {
    autonomyMode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::AUTONOMOUS;
  } else if ( str_lower == "unknown" ) {
    autonomyMode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN;
  } else if ( str_lower == "inactive" ) {
    autonomyMode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::INACTIVE;
  } else if ( str_lower == "emergency_stop_hard" || str_lower == "e_stop_hard" ) {
    autonomyMode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::EMERGENCY_STOP_HARD;
  } else if ( str_lower == "emergency_stop_soft" || str_lower == "e_stop_soft" ) {
    autonomyMode.autonomy_mode = autonomy_manager_msgs::msg::AutonomyMode::EMERGENCY_STOP_SOFT;
  } else {
    throw std::invalid_argument( "Invalid autonomy type: " + str );
  }
  return autonomyMode;
}

inline std::string toString( const autonomy_manager_msgs::msg::AutonomyMode &autonomyMode )
{
  switch ( autonomyMode.autonomy_mode ) {
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
  case autonomy_manager_msgs::msg::AutonomyMode::EMERGENCY_STOP_HARD:
    return "emergency_stop_hard";
  case autonomy_manager_msgs::msg::AutonomyMode::EMERGENCY_STOP_SOFT:
    return "emergency_stop_soft";
  }
  return "not_found";
}

/// Returns a priority value for the given autonomy mode. Lower value = higher precedence.
inline int autonomyPriority( int8_t mode )
{
  switch ( mode ) {
  case autonomy_manager_msgs::msg::AutonomyMode::EMERGENCY_STOP_HARD:
    return 0;
  case autonomy_manager_msgs::msg::AutonomyMode::EMERGENCY_STOP_SOFT:
    return 1;
  case autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN:
    return 2;
  case autonomy_manager_msgs::msg::AutonomyMode::TELEOPERATED:
    return 3;
  case autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS:
    return 4;
  case autonomy_manager_msgs::msg::AutonomyMode::AUTONOMOUS:
    return 5;
  case autonomy_manager_msgs::msg::AutonomyMode::INACTIVE:
    return 6;
  default:
    return 2; // treat unknown values as UNKNOWN priority
  }
}

/// Returns true if `name` matches the glob `pattern` (supports *, ?, [...]).
inline bool matchesGlob( const std::string &pattern, const std::string &name )
{
  return ::fnmatch( pattern.c_str(), name.c_str(), 0 ) == 0;
}

/// Returns true if `name` contains glob meta-characters (*, ?, [).
inline bool isGlobPattern( const std::string &name )
{
  return name.find_first_of( "*?[" ) != std::string::npos;
}

struct AutonomyTopicInfo {
  /// Exact node name → autonomy mode (checked first).
  std::map<std::string, autonomy_manager_msgs::msg::AutonomyMode> autonomyTypePerNode;

  /// Wildcard patterns → autonomy mode (checked second, in insertion order).
  std::vector<std::pair<std::string, autonomy_manager_msgs::msg::AutonomyMode>> wildcardPatterns;

  /// Optional default autonomy mode for unmatched nodes (checked third).
  /// When not set, unmatched nodes produce UNKNOWN mode.
  std::optional<autonomy_manager_msgs::msg::AutonomyMode> defaultAutonomyType;
};

/// Pairs an AutonomyMode with an operating-mode label (e.g. "driving", "manipulation").
struct ModeWithContext {
  autonomy_manager_msgs::msg::AutonomyMode autonomy_mode;
  std::string operating_mode; // empty = no specific operating mode
  std::string source;         // human-readable reason, e.g. "TopicMonitor: cmd_vel"
};

/// A single rule in the operating-mode rule table (first match wins).
struct OperatingModeRule {
  std::string autonomy; // autonomy level to match (e.g. "teleop", "*" for any)
  std::string mode;     // operating_mode to match (e.g. "driving", "*" for any)
  std::string output;   // output string to publish (e.g. "driving")
};

/// Backward-compatible struct for constructing TopicMonitor from pre-built info (testing).
struct ObservedTopicInfo {
  ObservedTopicInfo( const std::string &topicName, const std::vector<std::string> &nodeNames,
                     const std::vector<std::string> &autonomyModes, double timeout,
                     const std::string &topicType = "", const std::string &defaultAutonomyType = "" )
      : topicName( topicName ), nodeNames( nodeNames ), autonomyModes( autonomyModes ),
        timeout( timeout ), topicType( topicType ), defaultAutonomyType( defaultAutonomyType )
  {
  }

  std::string topicName;
  std::vector<std::string> nodeNames;
  std::vector<std::string> autonomyModes; // same order as nodeNames
  double timeout;                         // if <= 0, no timeout
  std::string topicType;                  // message type string (e.g. "geometry_msgs/msg/Twist")
  std::string defaultAutonomyType;        // fallback for unmatched nodes (empty = no default)
};

} // namespace autonomy_manager
#endif // AUTONOMY_MANAGER_TYPES_H
