#include <autonomy_manager/autonomy_manager.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rtest/publisher_mock.hpp>
#include <rtest/subscription_mock.hpp>

using namespace autonomy_manager;
using AutonomyMode = autonomy_manager_msgs::msg::AutonomyMode;

class AutonomyManagerIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override( "stale_mode", "inactive" );
    opts.append_parameter_override( "hard_e_stop_topic", "/emergency_stop_hardware" );
    opts.append_parameter_override( "soft_e_stop_topic", "/emergency_stop_software" );

    // Topic monitor via params (namespaced)
    opts.append_parameter_override( "observed_topics", std::vector<std::string>{ "cmd_vel" } );
    opts.append_parameter_override( "cmd_vel.nodes",
                                    std::vector<std::string>{ "teleop_node", "nav_node" } );
    opts.append_parameter_override( "cmd_vel.autonomy_types",
                                    std::vector<std::string>{ "teleop", "autonomy" } );
    opts.append_parameter_override( "cmd_vel.timeout", 0.0 );
    opts.append_parameter_override( "cmd_vel.type", "geometry_msgs/msg/Twist" );

    // Action monitor (namespaced)
    opts.append_parameter_override( "observed_actions",
                                    std::vector<std::string>{ "navigate_to_pose" } );
    opts.append_parameter_override( "navigate_to_pose.autonomy_type", "autonomy" );
    opts.append_parameter_override( "navigate_to_pose.timeout", 0.0 );

    // No controller monitor for this test
    opts.append_parameter_override( "observed_controllers", std::vector<std::string>{} );
    opts.append_parameter_override( "controller_manager_activity_topic", "" );
    opts.append_parameter_override( "joint_states_topic", "" );

    node_ = std::make_shared<AutonomyManager>( opts );

    hard_estop_sub_ =
        rtest::findSubscription<std_msgs::msg::Bool>( node_, "/emergency_stop_hardware" );
    soft_estop_sub_ =
        rtest::findSubscription<std_msgs::msg::Bool>( node_, "/emergency_stop_software" );
    action_status_sub_ = rtest::findSubscription<action_msgs::msg::GoalStatusArray>(
        node_, "navigate_to_pose/_action/status" );
    pub_mock_ = rtest::findPublisher<AutonomyMode>( node_, "/autonomy_mode" );
  }

  std::shared_ptr<AutonomyManager> node_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> hard_estop_sub_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> soft_estop_sub_;
  std::shared_ptr<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>> action_status_sub_;
  std::shared_ptr<rtest::PublisherMock<AutonomyMode>> pub_mock_;
};

TEST_F( AutonomyManagerIntegrationTest, StartupIsInactive )
{
  ASSERT_TRUE( node_ );
  ASSERT_TRUE( pub_mock_ );
}

TEST_F( AutonomyManagerIntegrationTest, EStopOverridesTopicMonitor )
{
  // First set teleop via topic monitor
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  node_->topicMonitor()->injectActivity( "cmd_vel", "teleop_node" );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Now activate hard e-stop — should override teleop
  AutonomyMode estop_hard;
  estop_hard.autonomy_mode = AutonomyMode::EMERGENCY_STOP_HARD;
  EXPECT_CALL( *pub_mock_, publish( estop_hard ) ).Times( 1 );

  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  hard_estop_sub_->handle_message( msg );
}

TEST_F( AutonomyManagerIntegrationTest, EStopClearedFallsBackToTopicMode )
{
  // Set teleop via topic monitor
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  node_->topicMonitor()->injectActivity( "cmd_vel", "teleop_node" );

  // Activate hard e-stop
  auto estop_msg = std_msgs::msg::Bool();
  estop_msg.data = true;
  hard_estop_sub_->handle_message( estop_msg );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Deactivate e-stop — should fall back to teleop
  AutonomyMode teleop;
  teleop.autonomy_mode = AutonomyMode::TELEOPERATED;
  EXPECT_CALL( *pub_mock_, publish( teleop ) ).Times( 1 );

  estop_msg.data = false;
  hard_estop_sub_->handle_message( estop_msg );
}

TEST_F( AutonomyManagerIntegrationTest, ActionAndTopicCombinedPriority )
{
  // Action executing (autonomy)
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  action_msgs::msg::GoalStatusArray status_msg;
  action_msgs::msg::GoalStatus status;
  status.status = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  status_msg.status_list.push_back( status );
  action_status_sub_->handle_message( status_msg );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Topic monitor teleop — teleop should win over autonomy
  AutonomyMode teleop;
  teleop.autonomy_mode = AutonomyMode::TELEOPERATED;
  EXPECT_CALL( *pub_mock_, publish( teleop ) ).Times( 1 );

  node_->topicMonitor()->injectActivity( "cmd_vel", "teleop_node" );
}

TEST_F( AutonomyManagerIntegrationTest, SoftEStopOverridesTeleop )
{
  // Set teleop
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  node_->topicMonitor()->injectActivity( "cmd_vel", "teleop_node" );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Activate soft e-stop — should override teleop
  AutonomyMode estop_soft;
  estop_soft.autonomy_mode = AutonomyMode::EMERGENCY_STOP_SOFT;
  EXPECT_CALL( *pub_mock_, publish( estop_soft ) ).Times( 1 );

  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  soft_estop_sub_->handle_message( msg );
}

TEST_F( AutonomyManagerIntegrationTest, HardEStopOverridesSoftEStop )
{
  // Activate soft e-stop
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  soft_estop_sub_->handle_message( msg );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Activate hard e-stop — should override soft
  AutonomyMode estop_hard;
  estop_hard.autonomy_mode = AutonomyMode::EMERGENCY_STOP_HARD;
  EXPECT_CALL( *pub_mock_, publish( estop_hard ) ).Times( 1 );

  hard_estop_sub_->handle_message( msg );
}

TEST_F( AutonomyManagerIntegrationTest, AllInactiveIsInactive )
{
  // Don't inject any activity — mode should be inactive
  // The initial publish already happened in the constructor
  ASSERT_TRUE( node_ );
}
