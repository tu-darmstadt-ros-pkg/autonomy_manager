#include <autonomy_manager/autonomy_manager.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rtest/create_timer_mock.hpp>
#include <rtest/publisher_mock.hpp>

using namespace autonomy_manager;
using AutonomyMode = autonomy_manager_msgs::msg::AutonomyMode;

class TopicMonitorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Build the node with topic monitor via the ObservedTopicInfo constructor.
    // The GenericSubscription won't be created in tests (no topic type in DDS graph),
    // but we test the logic via injectActivity().
    ObservedTopicInfo cmd_vel_info( "/cmd_vel", { "teleop_node", "nav_node" },
                                    { "teleop", "autonomy" }, 3, "geometry_msgs/msg/Twist" );
    ObservedTopicInfo arm_info( "/arm_goal_pose", { "teleop_arm_node" }, { "teleop" }, 0,
                                "geometry_msgs/msg/PoseStamped" );

    node_ =
        std::make_shared<AutonomyManager>( std::vector<ObservedTopicInfo>{ cmd_vel_info, arm_info } );

    pub_mock_ = rtest::findPublisher<AutonomyMode>( node_, "/autonomy_mode" );
  }

  std::shared_ptr<AutonomyManager> node_;
  std::shared_ptr<rtest::PublisherMock<AutonomyMode>> pub_mock_;
};

TEST_F( TopicMonitorTest, PublisherCreated )
{
  ASSERT_TRUE( pub_mock_ );
  ASSERT_TRUE( node_->topicMonitor() );
}

TEST_F( TopicMonitorTest, InjectTeleopActivity )
{
  AutonomyMode teleop;
  teleop.autonomy_mode = AutonomyMode::TELEOPERATED;
  EXPECT_CALL( *pub_mock_, publish( teleop ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/cmd_vel", "teleop_node" );
}

TEST_F( TopicMonitorTest, InjectAutonomyActivity )
{
  AutonomyMode autonomous;
  autonomous.autonomy_mode = AutonomyMode::AUTONOMOUS;
  EXPECT_CALL( *pub_mock_, publish( autonomous ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/cmd_vel", "nav_node" );
}

TEST_F( TopicMonitorTest, UnknownNodeTriggersUnknownMode )
{
  AutonomyMode unknown;
  unknown.autonomy_mode = AutonomyMode::UNKNOWN;
  EXPECT_CALL( *pub_mock_, publish( unknown ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/cmd_vel", "unexpected_node" );
}

TEST_F( TopicMonitorTest, TeleopOverridesAutonomy )
{
  // Inject autonomy first
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  node_->topicMonitor()->injectActivity( "/cmd_vel", "nav_node" );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Inject teleop on arm topic — teleop should win
  AutonomyMode teleop;
  teleop.autonomy_mode = AutonomyMode::TELEOPERATED;
  EXPECT_CALL( *pub_mock_, publish( teleop ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/arm_goal_pose", "teleop_arm_node" );
}

TEST_F( TopicMonitorTest, TimeoutRevertsToInactive )
{
  // First inject activity
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  node_->topicMonitor()->injectActivity( "/cmd_vel", "teleop_node" );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Fire timeout timer (cmd_vel has timeout=3)
  auto timers = rtest::findTimers( node_ );
  ASSERT_FALSE( timers.empty() );

  AutonomyMode inactive;
  inactive.autonomy_mode = AutonomyMode::INACTIVE;
  EXPECT_CALL( *pub_mock_, publish( inactive ) ).Times( 1 );

  timers[0]->execute_callback( nullptr );
}

TEST_F( TopicMonitorTest, NoTimeoutTopicDoesNotRevert )
{
  // arm_goal_pose has timeout=0, so no timer should be created for it
  // Just verify the inject works and stays
  AutonomyMode teleop;
  teleop.autonomy_mode = AutonomyMode::TELEOPERATED;
  EXPECT_CALL( *pub_mock_, publish( teleop ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/arm_goal_pose", "teleop_arm_node" );
}

// =============================================================================
// Wildcard and Default Autonomy Type Tests
// =============================================================================

class TopicMonitorWildcardTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // cmd_vel: "*nav2*" -> autonomy, default = teleop
    ObservedTopicInfo cmd_vel_info( "/cmd_vel", { "*nav2*" }, { "autonomy" }, 0,
                                    "geometry_msgs/msg/Twist", "teleop" );

    // arm topic: "teleop_arm_*" -> teleop, "*planner*" -> autonomy, no default
    ObservedTopicInfo arm_info( "/arm_goal_pose", { "teleop_arm_*", "*planner*" },
                                { "teleop", "autonomy" }, 0, "geometry_msgs/msg/PoseStamped" );

    // mixed topic: exact "exact_node" -> teleop, "*auto*" -> autonomy, default = semi_autonomy
    ObservedTopicInfo mixed_info( "/mixed_topic", { "exact_node", "*auto*" },
                                  { "teleop", "autonomy" }, 0, "std_msgs/msg/String",
                                  "semi_autonomy" );

    node_ = std::make_shared<AutonomyManager>(
        std::vector<ObservedTopicInfo>{ cmd_vel_info, arm_info, mixed_info } );
    pub_mock_ = rtest::findPublisher<AutonomyMode>( node_, "/autonomy_mode" );
  }

  std::shared_ptr<AutonomyManager> node_;
  std::shared_ptr<rtest::PublisherMock<AutonomyMode>> pub_mock_;
};

TEST_F( TopicMonitorWildcardTest, WildcardMatchNav2 )
{
  // "*nav2*" should match "nav2_controller" -> autonomy
  AutonomyMode autonomous;
  autonomous.autonomy_mode = AutonomyMode::AUTONOMOUS;
  EXPECT_CALL( *pub_mock_, publish( autonomous ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/cmd_vel", "nav2_controller" );
}

TEST_F( TopicMonitorWildcardTest, WildcardMatchNav2InMiddle )
{
  // "*nav2*" should match "my_nav2_node"
  AutonomyMode autonomous;
  autonomous.autonomy_mode = AutonomyMode::AUTONOMOUS;
  EXPECT_CALL( *pub_mock_, publish( autonomous ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/cmd_vel", "my_nav2_node" );
}

TEST_F( TopicMonitorWildcardTest, DefaultFallbackForUnmatchedNode )
{
  // "random_node" matches no wildcard, default = teleop
  AutonomyMode teleop;
  teleop.autonomy_mode = AutonomyMode::TELEOPERATED;
  EXPECT_CALL( *pub_mock_, publish( teleop ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/cmd_vel", "random_node" );
}

TEST_F( TopicMonitorWildcardTest, ExactMatchTakesPriorityOverWildcard )
{
  // "exact_node" is an exact match -> teleop, even though "*auto*" would not match it
  AutonomyMode teleop;
  teleop.autonomy_mode = AutonomyMode::TELEOPERATED;
  EXPECT_CALL( *pub_mock_, publish( teleop ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/mixed_topic", "exact_node" );
}

TEST_F( TopicMonitorWildcardTest, WildcardTakesPriorityOverDefault )
{
  // "some_auto_node" matches "*auto*" -> autonomy, not default semi_autonomy
  AutonomyMode autonomous;
  autonomous.autonomy_mode = AutonomyMode::AUTONOMOUS;
  EXPECT_CALL( *pub_mock_, publish( autonomous ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/mixed_topic", "some_auto_node" );
}

TEST_F( TopicMonitorWildcardTest, DefaultUsedWhenNoWildcardMatch )
{
  // "unknown_node" matches nothing on /mixed_topic -> default = semi_autonomy
  AutonomyMode semi;
  semi.autonomy_mode = AutonomyMode::SEMI_AUTONOMOUS;
  EXPECT_CALL( *pub_mock_, publish( semi ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/mixed_topic", "unknown_node" );
}

TEST_F( TopicMonitorWildcardTest, NoDefaultNoMatchGivesUnknown )
{
  // /arm_goal_pose has no default; "unknown_node" matches no pattern -> UNKNOWN
  AutonomyMode unknown;
  unknown.autonomy_mode = AutonomyMode::UNKNOWN;
  EXPECT_CALL( *pub_mock_, publish( unknown ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/arm_goal_pose", "unknown_node" );
}

TEST_F( TopicMonitorWildcardTest, FirstWildcardPatternWins )
{
  // "teleop_arm_planner" matches both "teleop_arm_*" and "*planner*"
  // First pattern should win -> teleop
  AutonomyMode teleop;
  teleop.autonomy_mode = AutonomyMode::TELEOPERATED;
  EXPECT_CALL( *pub_mock_, publish( teleop ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/arm_goal_pose", "teleop_arm_planner" );
}

TEST_F( TopicMonitorWildcardTest, SecondWildcardPatternMatches )
{
  // "path_planner_node" matches only "*planner*" -> autonomy
  AutonomyMode autonomous;
  autonomous.autonomy_mode = AutonomyMode::AUTONOMOUS;
  EXPECT_CALL( *pub_mock_, publish( autonomous ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "/arm_goal_pose", "path_planner_node" );
}
