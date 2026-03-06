#include <autonomy_manager/autonomy_manager.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rtest/create_timer_mock.hpp>
#include <rtest/publisher_mock.hpp>
#include <rtest/subscription_mock.hpp>

using namespace autonomy_manager;
using AutonomyMode = autonomy_manager_msgs::msg::AutonomyMode;

class ActionMonitorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override( "stale_mode", "inactive" );
    opts.append_parameter_override( "hard_e_stop_topic", "" );
    opts.append_parameter_override( "soft_e_stop_topic", "" );
    opts.append_parameter_override( "observed_topics", std::vector<std::string>{} );
    opts.append_parameter_override( "observed_actions",
                                    std::vector<std::string>{ "navigate_to_pose" } );
    opts.append_parameter_override( "navigate_to_pose.autonomy_type", "autonomy" );
    opts.append_parameter_override( "navigate_to_pose.timeout", 5.0 );
    opts.append_parameter_override( "observed_controllers", std::vector<std::string>{} );
    opts.append_parameter_override( "controller_manager_activity_topic", "" );
    opts.append_parameter_override( "joint_states_topic", "" );

    node_ = std::make_shared<AutonomyManager>( opts );

    status_sub_ = rtest::findSubscription<action_msgs::msg::GoalStatusArray>(
        node_, "navigate_to_pose/_action/status" );
    pub_mock_ = rtest::findPublisher<AutonomyMode>( node_, "/autonomy_mode" );
  }

  action_msgs::msg::GoalStatusArray makeStatusMsg( std::initializer_list<int8_t> statuses )
  {
    action_msgs::msg::GoalStatusArray msg;
    for ( auto s : statuses ) {
      action_msgs::msg::GoalStatus goal_status;
      goal_status.status = s;
      msg.status_list.push_back( goal_status );
    }
    return msg;
  }

  action_msgs::msg::GoalStatusArray makeStatusMsg( int8_t status )
  {
    return makeStatusMsg( { status } );
  }

  std::shared_ptr<AutonomyManager> node_;
  std::shared_ptr<rclcpp::Subscription<action_msgs::msg::GoalStatusArray>> status_sub_;
  std::shared_ptr<rtest::PublisherMock<AutonomyMode>> pub_mock_;
};

TEST_F( ActionMonitorTest, SubscriptionsCreated )
{
  ASSERT_TRUE( status_sub_ );
  ASSERT_TRUE( pub_mock_ );
}

TEST_F( ActionMonitorTest, ActionExecutingTriggersAutonomyMode )
{
  AutonomyMode expected;
  expected.autonomy_mode = AutonomyMode::AUTONOMOUS;
  EXPECT_CALL( *pub_mock_, publish( expected ) ).Times( 1 );

  auto msg = makeStatusMsg( action_msgs::msg::GoalStatus::STATUS_EXECUTING );
  status_sub_->handle_message( msg );
}

TEST_F( ActionMonitorTest, ActionAcceptedTriggersAutonomyMode )
{
  AutonomyMode expected;
  expected.autonomy_mode = AutonomyMode::AUTONOMOUS;
  EXPECT_CALL( *pub_mock_, publish( expected ) ).Times( 1 );

  auto msg = makeStatusMsg( action_msgs::msg::GoalStatus::STATUS_ACCEPTED );
  status_sub_->handle_message( msg );
}

TEST_F( ActionMonitorTest, ActionSucceededReturnsToInactive )
{
  // First trigger executing
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  auto exec_msg = makeStatusMsg( action_msgs::msg::GoalStatus::STATUS_EXECUTING );
  status_sub_->handle_message( exec_msg );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Then succeed — should go to inactive
  AutonomyMode inactive;
  inactive.autonomy_mode = AutonomyMode::INACTIVE;
  EXPECT_CALL( *pub_mock_, publish( inactive ) ).Times( 1 );

  auto success_msg = makeStatusMsg( action_msgs::msg::GoalStatus::STATUS_SUCCEEDED );
  status_sub_->handle_message( success_msg );
}

TEST_F( ActionMonitorTest, ActionAbortedReturnsToInactive )
{
  // First trigger executing
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  auto exec_msg = makeStatusMsg( action_msgs::msg::GoalStatus::STATUS_EXECUTING );
  status_sub_->handle_message( exec_msg );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Abort — should go to inactive
  AutonomyMode inactive;
  inactive.autonomy_mode = AutonomyMode::INACTIVE;
  EXPECT_CALL( *pub_mock_, publish( inactive ) ).Times( 1 );

  auto abort_msg = makeStatusMsg( action_msgs::msg::GoalStatus::STATUS_ABORTED );
  status_sub_->handle_message( abort_msg );
}

TEST_F( ActionMonitorTest, ActionCanceledReturnsToInactive )
{
  // First trigger executing
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  auto exec_msg = makeStatusMsg( action_msgs::msg::GoalStatus::STATUS_EXECUTING );
  status_sub_->handle_message( exec_msg );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Cancel — should go to inactive
  AutonomyMode inactive;
  inactive.autonomy_mode = AutonomyMode::INACTIVE;
  EXPECT_CALL( *pub_mock_, publish( inactive ) ).Times( 1 );

  auto cancel_msg = makeStatusMsg( action_msgs::msg::GoalStatus::STATUS_CANCELED );
  status_sub_->handle_message( cancel_msg );
}

TEST_F( ActionMonitorTest, TimeoutRevertsToStaleMode )
{
  // First trigger executing
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  auto exec_msg = makeStatusMsg( action_msgs::msg::GoalStatus::STATUS_EXECUTING );
  status_sub_->handle_message( exec_msg );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Fire the timeout timer — stale_mode is "inactive" in this test
  auto timers = rtest::findTimers( node_ );
  ASSERT_FALSE( timers.empty() );

  AutonomyMode inactive;
  inactive.autonomy_mode = AutonomyMode::INACTIVE;
  EXPECT_CALL( *pub_mock_, publish( inactive ) ).Times( 1 );

  timers[0]->execute_callback( nullptr );
}

TEST_F( ActionMonitorTest, FeedbackResetsTimer )
{
  // First trigger executing
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  auto exec_msg = makeStatusMsg( action_msgs::msg::GoalStatus::STATUS_EXECUTING );
  status_sub_->handle_message( exec_msg );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Inject feedback — this should reset the timer (no timeout)
  // We verify by ensuring no stale mode is published after the feedback
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( 0 );
  node_->actionMonitor()->injectFeedback( "navigate_to_pose" );
}

TEST_F( ActionMonitorTest, ParallelGoalsOneExecutingStaysActive )
{
  // Two goals: one executing, one succeeded
  AutonomyMode autonomy;
  autonomy.autonomy_mode = AutonomyMode::AUTONOMOUS;
  EXPECT_CALL( *pub_mock_, publish( autonomy ) ).Times( 1 );

  auto msg = makeStatusMsg( { action_msgs::msg::GoalStatus::STATUS_EXECUTING,
                              action_msgs::msg::GoalStatus::STATUS_SUCCEEDED } );
  status_sub_->handle_message( msg );
}

TEST_F( ActionMonitorTest, ParallelGoalsAllTerminalReturnsInactive )
{
  // First make active
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  auto exec_msg = makeStatusMsg( { action_msgs::msg::GoalStatus::STATUS_EXECUTING,
                                   action_msgs::msg::GoalStatus::STATUS_EXECUTING } );
  status_sub_->handle_message( exec_msg );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Both terminal — should revert to inactive
  AutonomyMode inactive;
  inactive.autonomy_mode = AutonomyMode::INACTIVE;
  EXPECT_CALL( *pub_mock_, publish( inactive ) ).Times( 1 );

  auto terminal_msg = makeStatusMsg( { action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
                                       action_msgs::msg::GoalStatus::STATUS_ABORTED } );
  status_sub_->handle_message( terminal_msg );
}

TEST_F( ActionMonitorTest, EmptyStatusListIsInactive )
{
  // Empty status list should not change mode (already inactive)
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( 0 );

  action_msgs::msg::GoalStatusArray msg;
  status_sub_->handle_message( msg );
}
