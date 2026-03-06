#include <autonomy_manager/autonomy_manager.h>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rtest/publisher_mock.hpp>
#include <rtest/service_client_mock.hpp>
#include <rtest/subscription_mock.hpp>

using namespace autonomy_manager;
using AutonomyMode = autonomy_manager_msgs::msg::AutonomyMode;
using ListControllers = controller_manager_msgs::srv::ListControllers;

class ControllerMonitorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override( "stale_mode", "inactive" );
    opts.append_parameter_override( "hard_e_stop_topic", "" );
    opts.append_parameter_override( "soft_e_stop_topic", "" );
    opts.append_parameter_override( "observed_topics", std::vector<std::string>{} );
    opts.append_parameter_override( "observed_actions", std::vector<std::string>{} );
    opts.append_parameter_override( "controller_manager_activity_topic",
                                    "/controller_manager/controller_manager_activity" );
    opts.append_parameter_override( "list_controllers_service",
                                    "/controller_manager/list_controllers" );
    opts.append_parameter_override( "joint_states_topic", "/joint_states" );
    opts.append_parameter_override( "joint_velocity_threshold", 0.01 );
    opts.append_parameter_override( "observed_controllers",
                                    std::vector<std::string>{ "arm_teleop_controller" } );
    opts.append_parameter_override( "arm_teleop_controller.autonomy_type", "teleop" );
    opts.append_parameter_override( "arm_teleop_controller.require_motion", true );

    node_ = std::make_shared<AutonomyManager>( opts );

    activity_sub_ = rtest::findSubscription<controller_manager_msgs::msg::ControllerManagerActivity>(
        node_, "/controller_manager/controller_manager_activity" );
    joint_state_sub_ =
        rtest::findSubscription<sensor_msgs::msg::JointState>( node_, "/joint_states" );
    pub_mock_ = rtest::findPublisher<AutonomyMode>( node_, "/autonomy_mode" );
    // Use the 2-arg overload to avoid rtest's slash-stripping bug in the dispatcher overload
    service_mock_ = rtest::findServiceClient<ListControllers>(
        node_->get_fully_qualified_name(), "/controller_manager/list_controllers" );
  }

  controller_manager_msgs::msg::ControllerManagerActivity makeActivity( const std::string &name,
                                                                        uint8_t state_id )
  {
    controller_manager_msgs::msg::ControllerManagerActivity msg;
    controller_manager_msgs::msg::NamedLifecycleState ctrl;
    ctrl.name = name;
    ctrl.state.id = state_id;
    msg.controllers.push_back( ctrl );
    return msg;
  }

  sensor_msgs::msg::JointState makeJointState( const std::vector<std::string> &names,
                                               const std::vector<double> &velocities )
  {
    sensor_msgs::msg::JointState msg;
    msg.name = names;
    msg.velocity = velocities;
    return msg;
  }

  using ListControllersClient = rclcpp::Client<ListControllers>;

  /// Set up the service mock to return joints for a controller when queried.
  void mockListControllersResponse( const std::string &controller_name,
                                    const std::vector<std::string> &claimed_interfaces )
  {
    ASSERT_TRUE( service_mock_ );

    ON_CALL( *service_mock_, service_is_ready() ).WillByDefault( testing::Return( true ) );

    ON_CALL( *service_mock_, async_send_request_with_callback( testing::_, testing::_ ) )
        .WillByDefault( testing::Invoke(
            [controller_name, claimed_interfaces]( ListControllers::Request::SharedPtr,
                                                   ListControllersClient::CallbackType callback )
                -> ListControllersClient::SharedFutureAndRequestId {
              // Build response
              auto response = std::make_shared<ListControllers::Response>();
              controller_manager_msgs::msg::ControllerState ctrl_state;
              ctrl_state.name = controller_name;
              ctrl_state.state = "active";
              ctrl_state.claimed_interfaces = claimed_interfaces;
              response->controller.push_back( ctrl_state );

              // Create a fulfilled promise/future
              auto promise = std::make_shared<std::promise<ListControllers::Response::SharedPtr>>();
              promise->set_value( response );
              auto future = promise->get_future().share();

              // Invoke the callback synchronously
              callback( future );

              return ListControllersClient::SharedFutureAndRequestId( future, 0 );
            } ) );
  }

  std::shared_ptr<AutonomyManager> node_;
  std::shared_ptr<rclcpp::Subscription<controller_manager_msgs::msg::ControllerManagerActivity>> activity_sub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> joint_state_sub_;
  std::shared_ptr<rtest::PublisherMock<AutonomyMode>> pub_mock_;
  std::shared_ptr<rtest::ServiceClientMock<ListControllers>> service_mock_;
};

TEST_F( ControllerMonitorTest, SubscriptionsCreated )
{
  ASSERT_TRUE( activity_sub_ );
  ASSERT_TRUE( joint_state_sub_ );
  ASSERT_TRUE( pub_mock_ );
  ASSERT_TRUE( service_mock_ );
}

TEST_F( ControllerMonitorTest, ActiveControllerWithMovingJointsIsTeleop )
{
  // Mock service to return joint1 and joint2 for arm_teleop_controller
  mockListControllersResponse( "arm_teleop_controller", { "joint1/position", "joint2/position" } );

  // Make controller active — triggers joint discovery via service call
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  auto activity =
      makeActivity( "arm_teleop_controller", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE );
  activity_sub_->handle_message( activity );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Move joints — should trigger teleop
  AutonomyMode teleop;
  teleop.autonomy_mode = AutonomyMode::TELEOPERATED;
  EXPECT_CALL( *pub_mock_, publish( teleop ) ).Times( 1 );

  auto js = makeJointState( { "joint1", "joint2" }, { 0.5, 0.0 } );
  joint_state_sub_->handle_message( js );
}

TEST_F( ControllerMonitorTest, ActiveControllerWithStillJointsIsInactive )
{
  mockListControllersResponse( "arm_teleop_controller", { "joint1/position", "joint2/position" } );

  // Make controller active
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  auto activity =
      makeActivity( "arm_teleop_controller", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE );
  activity_sub_->handle_message( activity );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Joints not moving — should NOT publish teleop (stays inactive)
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( 0 );

  auto js = makeJointState( { "joint1", "joint2" }, { 0.0, 0.001 } );
  joint_state_sub_->handle_message( js );
}

TEST_F( ControllerMonitorTest, InactiveControllerWithMovingJointsIsInactive )
{
  // Controller is inactive — no service call expected (joints not queried)
  auto activity =
      makeActivity( "arm_teleop_controller", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE );
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  activity_sub_->handle_message( activity );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Even if joints move, controller is not active
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( 0 );

  auto js = makeJointState( { "joint1", "joint2" }, { 1.0, 1.0 } );
  joint_state_sub_->handle_message( js );
}

TEST_F( ControllerMonitorTest, OnlyDiscoveredJointsMatter )
{
  // Mock: controller only claims joint1
  mockListControllersResponse( "arm_teleop_controller", { "joint1/position" } );

  // Make controller active
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  auto activity =
      makeActivity( "arm_teleop_controller", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE );
  activity_sub_->handle_message( activity );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Move an unclaimed joint — should NOT trigger teleop
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( 0 );

  auto js = makeJointState( { "unrelated_joint" }, { 5.0 } );
  joint_state_sub_->handle_message( js );
}

TEST_F( ControllerMonitorTest, ServiceNotReadyFallsBackGracefully )
{
  ASSERT_TRUE( service_mock_ );
  ON_CALL( *service_mock_, service_is_ready() ).WillByDefault( testing::Return( false ) );

  // Make controller active — service not ready, joints won't be discovered
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( testing::AnyNumber() );
  auto activity =
      makeActivity( "arm_teleop_controller", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE );
  activity_sub_->handle_message( activity );
  testing::Mock::VerifyAndClearExpectations( pub_mock_.get() );

  // Joints moving but controller has no joints discovered → require_motion can't match
  EXPECT_CALL( *pub_mock_, publish( testing::_ ) ).Times( 0 );

  auto js = makeJointState( { "joint1", "joint2" }, { 1.0, 1.0 } );
  joint_state_sub_->handle_message( js );
}
