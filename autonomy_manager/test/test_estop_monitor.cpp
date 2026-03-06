#include <autonomy_manager/autonomy_manager.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rtest/publisher_mock.hpp>
#include <rtest/subscription_mock.hpp>

using namespace autonomy_manager;
using AutonomyMode = autonomy_manager_msgs::msg::AutonomyMode;

class EStopMonitorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override( "stale_mode", "inactive" );
    opts.append_parameter_override( "hard_e_stop_topic", "/emergency_stop_hardware" );
    opts.append_parameter_override( "soft_e_stop_topic", "/emergency_stop_software" );
    opts.append_parameter_override( "observed_topics", std::vector<std::string>{} );
    opts.append_parameter_override( "observed_actions", std::vector<std::string>{} );
    opts.append_parameter_override( "observed_controllers", std::vector<std::string>{} );
    opts.append_parameter_override( "controller_manager_activity_topic", "" );
    opts.append_parameter_override( "joint_states_topic", "" );

    node_ = std::make_shared<AutonomyManager>( opts );

    hard_sub_ = rtest::findSubscription<std_msgs::msg::Bool>( node_, "/emergency_stop_hardware" );
    soft_sub_ = rtest::findSubscription<std_msgs::msg::Bool>( node_, "/emergency_stop_software" );
    pub_mock_ = rtest::findPublisher<AutonomyMode>( node_, "/autonomy_mode" );
  }

  std::shared_ptr<AutonomyManager> node_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> hard_sub_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> soft_sub_;
  std::shared_ptr<rtest::PublisherMock<AutonomyMode>> pub_mock_;
};

TEST_F( EStopMonitorTest, SubscriptionsCreated )
{
  ASSERT_TRUE( hard_sub_ );
  ASSERT_TRUE( soft_sub_ );
  ASSERT_TRUE( pub_mock_ );
}

TEST_F( EStopMonitorTest, HardEStopActivation )
{
  AutonomyMode expected;
  expected.autonomy_mode = AutonomyMode::EMERGENCY_STOP_HARD;
  EXPECT_CALL( *pub_mock_, publish( expected ) ).Times( 1 );

  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  hard_sub_->handle_message( msg );
}

TEST_F( EStopMonitorTest, HardEStopDeactivation )
{
  // First activate
  auto msg = std_msgs::msg::Bool();
  msg.data = true;

  AutonomyMode estop_hard;
  estop_hard.autonomy_mode = AutonomyMode::EMERGENCY_STOP_HARD;
  EXPECT_CALL( *pub_mock_, publish( estop_hard ) ).Times( 1 );
  hard_sub_->handle_message( msg );

  // Then deactivate
  msg.data = false;
  AutonomyMode inactive;
  inactive.autonomy_mode = AutonomyMode::INACTIVE;
  EXPECT_CALL( *pub_mock_, publish( inactive ) ).Times( 1 );
  hard_sub_->handle_message( msg );
}

TEST_F( EStopMonitorTest, SoftEStopActivation )
{
  AutonomyMode expected;
  expected.autonomy_mode = AutonomyMode::EMERGENCY_STOP_SOFT;
  EXPECT_CALL( *pub_mock_, publish( expected ) ).Times( 1 );

  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  soft_sub_->handle_message( msg );
}

TEST_F( EStopMonitorTest, HardEStopTakesPrecedenceOverSoft )
{
  // Activate soft first
  auto soft_msg = std_msgs::msg::Bool();
  soft_msg.data = true;

  AutonomyMode estop_soft;
  estop_soft.autonomy_mode = AutonomyMode::EMERGENCY_STOP_SOFT;
  EXPECT_CALL( *pub_mock_, publish( estop_soft ) ).Times( 1 );
  soft_sub_->handle_message( soft_msg );

  // Activate hard — should override to hard
  auto hard_msg = std_msgs::msg::Bool();
  hard_msg.data = true;

  AutonomyMode estop_hard;
  estop_hard.autonomy_mode = AutonomyMode::EMERGENCY_STOP_HARD;
  EXPECT_CALL( *pub_mock_, publish( estop_hard ) ).Times( 1 );
  hard_sub_->handle_message( hard_msg );
}

TEST_F( EStopMonitorTest, DeactivateHardFallsBackToSoft )
{
  // Activate both
  auto msg = std_msgs::msg::Bool();
  msg.data = true;
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  soft_sub_->handle_message( msg );
  hard_sub_->handle_message( msg );

  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Deactivate hard — should fall back to soft
  msg.data = false;
  AutonomyMode estop_soft;
  estop_soft.autonomy_mode = AutonomyMode::EMERGENCY_STOP_SOFT;
  EXPECT_CALL( *pub_mock_, publish( estop_soft ) ).Times( 1 );
  hard_sub_->handle_message( msg );
}

TEST_F( EStopMonitorTest, NoChangeNoPublish )
{
  // Send false when already false — no publish expected (mode doesn't change)
  auto msg = std_msgs::msg::Bool();
  msg.data = false;
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( 0 );
  hard_sub_->handle_message( msg );
}
