/// @file test_dynamic_reconfiguration.cpp
/// @brief Integration tests for dynamic parameter reconfiguration of the autonomy manager.
///
/// These tests verify that reconfigurable parameters (topic nodes/autonomy_types/timeout,
/// action timeout, controller require_motion, joint_velocity_threshold, and e-stop topics)
/// take effect at runtime without restarting the node.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <action_msgs/msg/goal_status_array.hpp>
#include <autonomy_manager/autonomy_manager.h>
#include <autonomy_manager_msgs/msg/autonomy_mode.hpp>
#include <controller_manager_msgs/msg/controller_manager_activity.hpp>
#include <controller_manager_msgs/msg/named_lifecycle_state.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rclcpp/rclcpp.hpp>

#include <hector_testing_utils/hector_testing_utils.hpp>

using namespace std::chrono_literals;
using autonomy_manager_msgs::msg::AutonomyMode;
using hector_testing_utils::HectorTestFixtureWithContext;
using hector_testing_utils::RemoteParameterClient;

// =============================================================================
// Test Fixture
// =============================================================================

class DynamicReconfigurationTest : public HectorTestFixtureWithContext
{
protected:
  void SetUp() override
  {
    HectorTestFixtureWithContext::SetUp();

    // Build node options with initial configuration
    rclcpp::NodeOptions opts = context_->node_options();
    opts.append_parameter_override( "stale_mode", "inactive" );

    // E-Stop
    opts.append_parameter_override( "hard_e_stop_topic", "/hard_estop" );
    opts.append_parameter_override( "soft_e_stop_topic", "/soft_estop" );

    // Topic monitoring
    opts.append_parameter_override( "observed_topics", std::vector<std::string>{ "cmd_vel" } );
    opts.append_parameter_override( "cmd_vel.nodes", std::vector<std::string>{ "teleop_node" } );
    opts.append_parameter_override( "cmd_vel.autonomy_types",
                                    std::vector<std::string>{ "teleop" } );
    opts.append_parameter_override( "cmd_vel.timeout", 5.0 );
    opts.append_parameter_override( "cmd_vel.type", "geometry_msgs/msg/Twist" );

    // Action monitoring
    opts.append_parameter_override( "observed_actions",
                                    std::vector<std::string>{ "navigate_to_pose" } );
    opts.append_parameter_override( "navigate_to_pose.autonomy_type", "autonomy" );
    opts.append_parameter_override( "navigate_to_pose.timeout", 300.0 );

    // Controller monitoring
    opts.append_parameter_override( "controller_manager_activity_topic",
                                    "/controller_manager/controller_manager_activity" );
    opts.append_parameter_override( "joint_states_topic", "/joint_states" );
    opts.append_parameter_override( "joint_velocity_threshold", 0.01 );
    opts.append_parameter_override( "observed_controllers",
                                    std::vector<std::string>{ "arm_controller" } );
    opts.append_parameter_override( "arm_controller.autonomy_type", "teleop" );
    opts.append_parameter_override( "arm_controller.require_motion", true );

    manager_ = std::make_shared<autonomy_manager::AutonomyManager>( opts );
    executor_->add_node( manager_ );

    // Create parameter client
    params_ = std::make_unique<RemoteParameterClient>( tester_node_, "autonomy_manager" );
    ASSERT_TRUE( params_->wait_for_service( *executor_, 5s ) );

    // Subscribe to the output topic
    autonomy_sub_ = tester_node_->create_test_subscription<AutonomyMode>(
        "/autonomy_mode", rclcpp::QoS( 10 ).transient_local() );

    ASSERT_TRUE( autonomy_sub_->wait_for_publishers( *executor_, 1, 5s ) );
  }

  void TearDown() override
  {
    autonomy_sub_.reset();
    params_.reset();
    manager_.reset();
    HectorTestFixtureWithContext::TearDown();
  }

  /// Helper: spin the executor for a fixed duration.
  void spinFor( std::chrono::milliseconds duration )
  {
    auto deadline = std::chrono::steady_clock::now() + duration;
    executor_->spin_until( [&deadline]() { return std::chrono::steady_clock::now() >= deadline; },
                           duration );
  }

  /// Helper: wait until the autonomy mode matches the expected value.
  /// Checks last_message() first in case the message was already received.
  bool waitForMode( uint8_t expected_mode, std::chrono::seconds timeout = 5s )
  {
    auto last = autonomy_sub_->last_message();
    if ( last.has_value() && last->autonomy_mode == expected_mode ) {
      return true;
    }
    return autonomy_sub_->wait_for_message(
        *executor_, timeout,
        [expected_mode]( const AutonomyMode &msg ) { return msg.autonomy_mode == expected_mode; } );
  }

  std::shared_ptr<autonomy_manager::AutonomyManager> manager_;
  std::unique_ptr<RemoteParameterClient> params_;
  std::shared_ptr<hector_testing_utils::TestSubscription<AutonomyMode>> autonomy_sub_;
};

// =============================================================================
// Topic Monitor — Dynamic Reconfiguration
// =============================================================================

TEST_F( DynamicReconfigurationTest, TopicMonitor_ReconfigureNodes )
{
  // Initially: cmd_vel with teleop_node → teleop
  // Publish a message from teleop_node → should get teleop mode
  // The topic monitor uses generic subscription with publisher_gid matching.
  // In integration tests we can't easily spoof the publisher node name in the GID.
  // Instead, test that dynamic parameter changes are accepted.

  // Must update nodes and autonomy_types together (validation requires same length)
  std::vector<rclcpp::Parameter> batch = {
      rclcpp::Parameter( "cmd_vel.nodes", std::vector<std::string>{ "teleop_node", "nav2_node" } ),
      rclcpp::Parameter( "cmd_vel.autonomy_types",
                         std::vector<std::string>{ "teleop", "autonomy" } ) };
  auto result = params_->set_parameters( *executor_, batch );
  EXPECT_TRUE( result.success ) << result.reason;

  // Verify the parameters were actually set
  auto nodes = params_->get_parameter<std::vector<std::string>>( *executor_, "cmd_vel.nodes" );
  ASSERT_TRUE( nodes.has_value() );
  ASSERT_EQ( nodes->size(), 2u );
  EXPECT_EQ( ( *nodes )[0], "teleop_node" );
  EXPECT_EQ( ( *nodes )[1], "nav2_node" );

  auto types =
      params_->get_parameter<std::vector<std::string>>( *executor_, "cmd_vel.autonomy_types" );
  ASSERT_TRUE( types.has_value() );
  ASSERT_EQ( types->size(), 2u );
  EXPECT_EQ( ( *types )[0], "teleop" );
  EXPECT_EQ( ( *types )[1], "autonomy" );
}

TEST_F( DynamicReconfigurationTest, TopicMonitor_ReconfigureTimeout )
{
  // Initial timeout is 5 seconds
  auto initial = params_->get_parameter<double>( *executor_, "cmd_vel.timeout" );
  ASSERT_TRUE( initial.has_value() );
  EXPECT_DOUBLE_EQ( initial.value(), 5.0 );

  // Change timeout to 10.5
  auto result = params_->set_parameter( *executor_, "cmd_vel.timeout", 10.5 );
  EXPECT_TRUE( result.success ) << result.reason;

  // Verify it changed
  auto updated = params_->get_parameter<double>( *executor_, "cmd_vel.timeout" );
  ASSERT_TRUE( updated.has_value() );
  EXPECT_DOUBLE_EQ( updated.value(), 10.5 );
}

// =============================================================================
// Action Monitor — Dynamic Reconfiguration
// =============================================================================

TEST_F( DynamicReconfigurationTest, ActionMonitor_ReconfigureTimeout )
{
  // Initial timeout is 300 seconds
  auto initial = params_->get_parameter<double>( *executor_, "navigate_to_pose.timeout" );
  ASSERT_TRUE( initial.has_value() );
  EXPECT_DOUBLE_EQ( initial.value(), 300.0 );

  // Change timeout to 60.5
  auto result = params_->set_parameter( *executor_, "navigate_to_pose.timeout", 60.5 );
  EXPECT_TRUE( result.success ) << result.reason;

  // Verify it changed
  auto updated = params_->get_parameter<double>( *executor_, "navigate_to_pose.timeout" );
  ASSERT_TRUE( updated.has_value() );
  EXPECT_DOUBLE_EQ( updated.value(), 60.5 );
}

TEST_F( DynamicReconfigurationTest, ActionMonitor_StatusTriggersMode )
{
  // Publish action status with an executing goal
  auto status_pub = tester_node_->create_publisher<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", 10 );
  ASSERT_TRUE( hector_testing_utils::wait_for_subscribers(
      *executor_, tester_node_, "/navigate_to_pose/_action/status", 1, 5s ) );

  action_msgs::msg::GoalStatusArray status_msg;
  action_msgs::msg::GoalStatus goal;
  goal.status = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  status_msg.status_list.push_back( goal );
  status_pub->publish( status_msg );

  ASSERT_TRUE( waitForMode( AutonomyMode::AUTONOMOUS ) );

  // Goal succeeds → inactive
  status_msg.status_list[0].status = action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
  status_pub->publish( status_msg );

  ASSERT_TRUE( waitForMode( AutonomyMode::INACTIVE ) );
}

// =============================================================================
// Controller Monitor — Dynamic Reconfiguration
// =============================================================================

TEST_F( DynamicReconfigurationTest, ControllerMonitor_ReconfigureRequireMotion )
{
  // Initially require_motion is true
  auto initial = params_->get_parameter<bool>( *executor_, "arm_controller.require_motion" );
  ASSERT_TRUE( initial.has_value() );
  EXPECT_TRUE( initial.value() );

  // Make the controller active (publish activity message)
  auto activity_pub =
      tester_node_->create_publisher<controller_manager_msgs::msg::ControllerManagerActivity>(
          "/controller_manager/controller_manager_activity", rclcpp::QoS( 10 ).transient_local() );
  ASSERT_TRUE( hector_testing_utils::wait_for_subscribers(
      *executor_, tester_node_, "/controller_manager/controller_manager_activity", 1, 5s ) );

  controller_manager_msgs::msg::ControllerManagerActivity activity;
  controller_manager_msgs::msg::NamedLifecycleState ctrl;
  ctrl.name = "arm_controller";
  ctrl.state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  activity.controllers.push_back( ctrl );
  activity_pub->publish( activity );
  spinFor( 200ms );

  // With require_motion=true and no joints discovered / no motion → should be INACTIVE
  // (ListControllers service not available in integration test → joints empty)

  // Now set require_motion to false → controller should be "commanding" just by being active
  auto result = params_->set_parameter( *executor_, "arm_controller.require_motion", false );
  EXPECT_TRUE( result.success ) << result.reason;

  // Verify parameter changed
  auto updated = params_->get_parameter<bool>( *executor_, "arm_controller.require_motion" );
  ASSERT_TRUE( updated.has_value() );
  EXPECT_FALSE( updated.value() );

  // Now the controller is active with require_motion=false → should publish teleop
  ASSERT_TRUE( waitForMode( AutonomyMode::TELEOPERATED ) );
}

TEST_F( DynamicReconfigurationTest, ControllerMonitor_ReconfigureVelocityThreshold )
{
  // Initial threshold is 0.01
  auto initial = params_->get_parameter<double>( *executor_, "joint_velocity_threshold" );
  ASSERT_TRUE( initial.has_value() );
  EXPECT_DOUBLE_EQ( initial.value(), 0.01 );

  // Change threshold to 0.5
  auto result = params_->set_parameter( *executor_, "joint_velocity_threshold", 0.5 );
  EXPECT_TRUE( result.success ) << result.reason;

  // Verify
  auto updated = params_->get_parameter<double>( *executor_, "joint_velocity_threshold" );
  ASSERT_TRUE( updated.has_value() );
  EXPECT_DOUBLE_EQ( updated.value(), 0.5 );
}

// =============================================================================
// E-Stop Monitor — Functional Test
// =============================================================================

TEST_F( DynamicReconfigurationTest, EStop_HardEStopOverridesAll )
{
  auto estop_qos = rclcpp::QoS( 1 ).transient_local();
  auto hard_estop_pub =
      tester_node_->create_publisher<std_msgs::msg::Bool>( "/hard_estop", estop_qos );
  ASSERT_TRUE( hector_testing_utils::wait_for_subscribers( *executor_, tester_node_, "/hard_estop",
                                                           1, 5s ) );

  // Activate hard e-stop
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  hard_estop_pub->publish( estop_msg );

  ASSERT_TRUE( waitForMode( AutonomyMode::EMERGENCY_STOP_HARD ) );

  // Deactivate
  estop_msg.data = false;
  hard_estop_pub->publish( estop_msg );

  ASSERT_TRUE( waitForMode( AutonomyMode::INACTIVE ) );
}

TEST_F( DynamicReconfigurationTest, EStop_SoftEStopActivation )
{
  auto estop_qos = rclcpp::QoS( 1 ).transient_local();
  auto soft_estop_pub =
      tester_node_->create_publisher<std_msgs::msg::Bool>( "/soft_estop", estop_qos );
  ASSERT_TRUE( hector_testing_utils::wait_for_subscribers( *executor_, tester_node_, "/soft_estop",
                                                           1, 5s ) );

  // Activate soft e-stop
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  soft_estop_pub->publish( estop_msg );

  ASSERT_TRUE( waitForMode( AutonomyMode::EMERGENCY_STOP_SOFT ) );
}

TEST_F( DynamicReconfigurationTest, EStop_HardOverridesSoft )
{
  auto estop_qos = rclcpp::QoS( 1 ).transient_local();
  auto hard_estop_pub =
      tester_node_->create_publisher<std_msgs::msg::Bool>( "/hard_estop", estop_qos );
  auto soft_estop_pub =
      tester_node_->create_publisher<std_msgs::msg::Bool>( "/soft_estop", estop_qos );
  ASSERT_TRUE( hector_testing_utils::wait_for_subscribers( *executor_, tester_node_, "/hard_estop",
                                                           1, 5s ) );
  ASSERT_TRUE( hector_testing_utils::wait_for_subscribers( *executor_, tester_node_, "/soft_estop",
                                                           1, 5s ) );

  // Activate soft first
  std_msgs::msg::Bool msg;
  msg.data = true;
  soft_estop_pub->publish( msg );
  ASSERT_TRUE( waitForMode( AutonomyMode::EMERGENCY_STOP_SOFT ) );

  // Then hard — should override
  hard_estop_pub->publish( msg );
  ASSERT_TRUE( waitForMode( AutonomyMode::EMERGENCY_STOP_HARD ) );
}

// =============================================================================
// Combined — Reconfiguration Under Load
// =============================================================================

TEST_F( DynamicReconfigurationTest, ReconfigureWhileMonitorsActive )
{
  // Start with an active action (autonomy mode)
  auto status_pub = tester_node_->create_publisher<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", 10 );
  ASSERT_TRUE( hector_testing_utils::wait_for_subscribers(
      *executor_, tester_node_, "/navigate_to_pose/_action/status", 1, 5s ) );

  action_msgs::msg::GoalStatusArray status_msg;
  action_msgs::msg::GoalStatus goal;
  goal.status = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
  status_msg.status_list.push_back( goal );
  status_pub->publish( status_msg );

  ASSERT_TRUE( waitForMode( AutonomyMode::AUTONOMOUS ) );

  // While the action is active, reconfigure the timeout
  auto result = params_->set_parameter( *executor_, "navigate_to_pose.timeout", 10.0 );
  EXPECT_TRUE( result.success ) << result.reason;

  // Node should still report autonomy mode (action is still executing)
  auto last = autonomy_sub_->last_message();
  ASSERT_TRUE( last.has_value() );
  EXPECT_EQ( last->autonomy_mode, AutonomyMode::AUTONOMOUS );

  // Now override with hard e-stop
  auto hard_estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>(
      "/hard_estop", rclcpp::QoS( 1 ).transient_local() );
  ASSERT_TRUE( hector_testing_utils::wait_for_subscribers( *executor_, tester_node_, "/hard_estop",
                                                           1, 5s ) );

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  hard_estop_pub->publish( estop_msg );

  ASSERT_TRUE( waitForMode( AutonomyMode::EMERGENCY_STOP_HARD ) );

  // Clear e-stop — should fall back to action's autonomy mode
  estop_msg.data = false;
  hard_estop_pub->publish( estop_msg );

  ASSERT_TRUE( waitForMode( AutonomyMode::AUTONOMOUS ) );
}
