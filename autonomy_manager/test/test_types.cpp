#include <autonomy_manager/types.h>
#include <gtest/gtest.h>

using namespace autonomy_manager;
using AutonomyMode = autonomy_manager_msgs::msg::AutonomyMode;

TEST( TypesTest, FromStringBasicModes )
{
  EXPECT_EQ( fromString( "teleop" ).autonomy_mode, AutonomyMode::TELEOPERATED );
  EXPECT_EQ( fromString( "semi_autonomy" ).autonomy_mode, AutonomyMode::SEMI_AUTONOMOUS );
  EXPECT_EQ( fromString( "autonomy" ).autonomy_mode, AutonomyMode::AUTONOMOUS );
  EXPECT_EQ( fromString( "unknown" ).autonomy_mode, AutonomyMode::UNKNOWN );
  EXPECT_EQ( fromString( "inactive" ).autonomy_mode, AutonomyMode::INACTIVE );
}

TEST( TypesTest, FromStringEStopModes )
{
  EXPECT_EQ( fromString( "emergency_stop_hard" ).autonomy_mode, AutonomyMode::EMERGENCY_STOP_HARD );
  EXPECT_EQ( fromString( "e_stop_hard" ).autonomy_mode, AutonomyMode::EMERGENCY_STOP_HARD );
  EXPECT_EQ( fromString( "emergency_stop_soft" ).autonomy_mode, AutonomyMode::EMERGENCY_STOP_SOFT );
  EXPECT_EQ( fromString( "e_stop_soft" ).autonomy_mode, AutonomyMode::EMERGENCY_STOP_SOFT );
}

TEST( TypesTest, FromStringCaseInsensitive )
{
  EXPECT_EQ( fromString( "TELEOP" ).autonomy_mode, AutonomyMode::TELEOPERATED );
  EXPECT_EQ( fromString( "Teleop" ).autonomy_mode, AutonomyMode::TELEOPERATED );
  EXPECT_EQ( fromString( "EMERGENCY_STOP_HARD" ).autonomy_mode, AutonomyMode::EMERGENCY_STOP_HARD );
}

TEST( TypesTest, FromStringInvalid )
{
  EXPECT_THROW( fromString( "invalid_mode" ), std::invalid_argument );
}

TEST( TypesTest, ToStringAllModes )
{
  AutonomyMode mode;
  mode.autonomy_mode = AutonomyMode::TELEOPERATED;
  EXPECT_EQ( toString( mode ), "teleop" );

  mode.autonomy_mode = AutonomyMode::SEMI_AUTONOMOUS;
  EXPECT_EQ( toString( mode ), "semi_autonomy" );

  mode.autonomy_mode = AutonomyMode::AUTONOMOUS;
  EXPECT_EQ( toString( mode ), "autonomy" );

  mode.autonomy_mode = AutonomyMode::UNKNOWN;
  EXPECT_EQ( toString( mode ), "unknown" );

  mode.autonomy_mode = AutonomyMode::INACTIVE;
  EXPECT_EQ( toString( mode ), "inactive" );

  mode.autonomy_mode = AutonomyMode::EMERGENCY_STOP_HARD;
  EXPECT_EQ( toString( mode ), "emergency_stop_hard" );

  mode.autonomy_mode = AutonomyMode::EMERGENCY_STOP_SOFT;
  EXPECT_EQ( toString( mode ), "emergency_stop_soft" );
}

TEST( TypesTest, PriorityOrdering )
{
  // Lower priority number = higher precedence
  EXPECT_LT( autonomyPriority( AutonomyMode::EMERGENCY_STOP_HARD ),
             autonomyPriority( AutonomyMode::EMERGENCY_STOP_SOFT ) );
  EXPECT_LT( autonomyPriority( AutonomyMode::EMERGENCY_STOP_SOFT ),
             autonomyPriority( AutonomyMode::UNKNOWN ) );
  EXPECT_LT( autonomyPriority( AutonomyMode::UNKNOWN ),
             autonomyPriority( AutonomyMode::TELEOPERATED ) );
  EXPECT_LT( autonomyPriority( AutonomyMode::TELEOPERATED ),
             autonomyPriority( AutonomyMode::SEMI_AUTONOMOUS ) );
  EXPECT_LT( autonomyPriority( AutonomyMode::SEMI_AUTONOMOUS ),
             autonomyPriority( AutonomyMode::AUTONOMOUS ) );
  EXPECT_LT( autonomyPriority( AutonomyMode::AUTONOMOUS ),
             autonomyPriority( AutonomyMode::INACTIVE ) );
}

TEST( TypesTest, FromStringRoundTrip )
{
  for ( const auto &name : { "teleop", "semi_autonomy", "autonomy", "unknown", "inactive",
                             "emergency_stop_hard", "emergency_stop_soft" } ) {
    EXPECT_EQ( toString( fromString( name ) ), name );
  }
}

// =============================================================================
// Glob Matching Helpers
// =============================================================================

TEST( MatchesGlob, ExactMatch )
{
  EXPECT_TRUE( matchesGlob( "teleop_node", "teleop_node" ) );
  EXPECT_FALSE( matchesGlob( "teleop_node", "nav_node" ) );
}

TEST( MatchesGlob, ContainsWildcard )
{
  EXPECT_TRUE( matchesGlob( "*nav2*", "nav2_controller" ) );
  EXPECT_TRUE( matchesGlob( "*nav2*", "my_nav2_node" ) );
  EXPECT_TRUE( matchesGlob( "*nav2*", "nav2" ) );
  EXPECT_FALSE( matchesGlob( "*nav2*", "nav_node" ) );
}

TEST( MatchesGlob, PrefixWildcard )
{
  EXPECT_TRUE( matchesGlob( "gamepad_*", "gamepad_left" ) );
  EXPECT_TRUE( matchesGlob( "gamepad_*", "gamepad_" ) );
  EXPECT_FALSE( matchesGlob( "gamepad_*", "my_gamepad" ) );
}

TEST( MatchesGlob, SuffixWildcard )
{
  EXPECT_TRUE( matchesGlob( "*_teleop", "arm_teleop" ) );
  EXPECT_FALSE( matchesGlob( "*_teleop", "teleop_arm" ) );
}

TEST( MatchesGlob, QuestionMarkWildcard )
{
  EXPECT_TRUE( matchesGlob( "node_?", "node_a" ) );
  EXPECT_FALSE( matchesGlob( "node_?", "node_ab" ) );
}

TEST( IsGlobPattern, DetectsStarPattern )
{
  EXPECT_TRUE( isGlobPattern( "*nav2*" ) );
  EXPECT_TRUE( isGlobPattern( "gamepad_*" ) );
  EXPECT_TRUE( isGlobPattern( "*_teleop" ) );
}

TEST( IsGlobPattern, DetectsQuestionMark ) { EXPECT_TRUE( isGlobPattern( "node_?" ) ); }

TEST( IsGlobPattern, DetectsBrackets ) { EXPECT_TRUE( isGlobPattern( "[abc]_node" ) ); }

TEST( IsGlobPattern, RejectsPlainNames )
{
  EXPECT_FALSE( isGlobPattern( "exact_node" ) );
  EXPECT_FALSE( isGlobPattern( "teleop_node" ) );
  EXPECT_FALSE( isGlobPattern( "" ) );
}
