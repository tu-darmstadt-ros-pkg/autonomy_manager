#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autonomy_manager_msgs/msg/autonomy_mode.hpp>
#include <autonomy_manager/autonomy_manager.h>

// a class that publishes on the cmd_vel topic
class CmdVelPublisher : public rclcpp::Node
{
public:
  explicit CmdVelPublisher( const std::string &name ) : Node( name + nameSpecifier )
  {
    pub_ = create_publisher<geometry_msgs::msg::Twist>( "cmd_vel", 10 );
  }

  void publish()
  {
    geometry_msgs::msg::Twist msg;
    pub_->publish( msg );
  }

  static const std::string nameSpecifier;

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

// a class that publishes on the arm_goal_pose topic
class PosePublisher : public rclcpp::Node
{
public:
  explicit PosePublisher( const std::string &name ) : Node( name + nameSpecifier )
  {
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>( "arm_goal_pose", 10 );
  }

  void publish()
  {
    geometry_msgs::msg::PoseStamped msg;
    pub_->publish( msg );
  }

  static const std::string nameSpecifier;

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

const std::string CmdVelPublisher::nameSpecifier = "_cmd_vel";
const std::string PosePublisher::nameSpecifier = "_pose";


// a class that subscribes to the autonomy_mode topic
class AutonomySubscriber : public rclcpp::Node
{
public:
  explicit AutonomySubscriber( const std::string &name ) : Node( name )
  {
    sub_ = create_subscription<autonomy_manager_msgs::msg::AutonomyMode>( "/autonomy_mode", 10,
      std::bind( &AutonomySubscriber::callback,
                 this, std::placeholders::_1 ) );
  }

  void callback( const autonomy_manager_msgs::msg::AutonomyMode::SharedPtr msg )
  {
    autonomy_mode_ = *msg;
    autonomy_mode_received_ = true;
  }

  void reset() { autonomy_mode_received_ = false; }

  bool receivedMsg() const { return autonomy_mode_received_; }


  autonomy_manager_msgs::msg::AutonomyMode autonomy_mode_;

private:
  bool autonomy_mode_received_{false};
  rclcpp::Subscription<autonomy_manager_msgs::msg::AutonomyMode>::SharedPtr sub_;
};

/**
 * Test class for the AutonomyManager
 * It creates a node for each autonomy type and publishes on the cmd_vel topic and the arm_goal_pose topic.
 * It also creates a subscriber for the autonomy_mode topic.
 */
class AutonomyManagerTest : public ::testing::Test
{
protected:
  std::map<std::string, std::shared_ptr<CmdVelPublisher>> nodesCmdVelPub_;
  std::map<std::string, std::shared_ptr<PosePublisher>> nodesPosePub_;
  std::shared_ptr<AutonomySubscriber> autonomy_subscriber_;
  autonomy_manager::AutonomyManager::SharedPtr autonomy_manager_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  // autonomy Topic subscriber


  void SetUp() override
  {
    rclcpp::init( 0, nullptr );
    // get all autonomyTypes as string
    std::vector<std::string> autonomyTypeNames = {"autonomy", "teleop", "semi_autonomy", "unknown"};
    std::vector<autonomy_manager_msgs::msg::AutonomyMode> autonomyTypes;
    autonomyTypes.reserve( autonomyTypeNames.size() );
    for ( const auto &autonomyTypeName : autonomyTypeNames ) {
      autonomyTypes.push_back( autonomy_manager::fromString( autonomyTypeName ) );
    }
    for ( const auto &autonomyType : autonomyTypeNames ) {
      nodesCmdVelPub_[autonomyType] = std::make_shared<CmdVelPublisher>( autonomyType );
      nodesPosePub_[autonomyType] = std::make_shared<PosePublisher>( autonomyType );
    }
    // remove unknown type
    autonomyTypeNames.pop_back();
    autonomyTypes.pop_back();

    autonomy_subscriber_ = std::make_shared<AutonomySubscriber>( "autonomy_subscriber" );
    std::vector<std::string> nodesVelPubNames, nodesPosePubNames;
    // iterate autonomyTypeNames and adapt PubNames by adding "_cmd_vel" and "_pose"
    for ( const auto &autonomyTypeName : autonomyTypeNames ) {
      nodesVelPubNames.push_back( autonomyTypeName + CmdVelPublisher::nameSpecifier );
      nodesPosePubNames.push_back( autonomyTypeName + PosePublisher::nameSpecifier );
    }
    autonomy_manager::ObservedTopicInfo observedTopicInfoCmd(
        "/cmd_vel", nodesVelPubNames, autonomyTypeNames, 0 );
    autonomy_manager::ObservedTopicInfo observedTopicInfoPose( "/arm_goal_pose", nodesPosePubNames,
                                                               autonomyTypeNames, 3 );
    std::vector<autonomy_manager::ObservedTopicInfo> observedTopicInfos = {observedTopicInfoCmd,
      observedTopicInfoPose};
    autonomy_manager_ = std::make_shared<autonomy_manager::AutonomyManager>( observedTopicInfos );
    // setup executor
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node( autonomy_manager_ );
    executor_->add_node( autonomy_subscriber_ );
    for ( const auto &node : nodesCmdVelPub_ ) { executor_->add_node( node.second ); }
    for ( const auto &node : nodesPosePub_ ) { executor_->add_node( node.second ); }

    waitForAutonomyMsg();
    autonomy_subscriber_->reset();
  }

  void waitFor( const int seconds )
  {
    rclcpp::Rate rate( 1 );
    for ( int i = 0; i < seconds; i++ ) {
      executor_->spin_some( std::chrono::seconds( 1 ) );
      rate.sleep();
    }
  }

  void TearDown() override { rclcpp::shutdown(); }

  void waitForAutonomyMsg()
  {
    int counter = 0;
    while ( !autonomy_subscriber_->receivedMsg() && counter < 30 ) {
      executor_->spin_some( std::chrono::milliseconds( 100 ) );
      counter++;
    }
  }
};

TEST_F( AutonomyManagerTest, TestStartUp )
{
  // without receiving any message, the autonomy mode should be "inactive"
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::INACTIVE );
}

TEST_F( AutonomyManagerTest, TestDetectAutonomyNode )
{
  autonomy_subscriber_->reset();
  nodesCmdVelPub_["autonomy"]->publish();
  waitForAutonomyMsg();
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::AUTONOMOUS );
}

TEST_F( AutonomyManagerTest, TestDetectTeleopNode )
{
  autonomy_subscriber_->reset();
  nodesCmdVelPub_["teleop"]->publish();
  waitForAutonomyMsg();
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::TELEOPERATED );
}

TEST_F( AutonomyManagerTest, TestDetectSemiAutonomyNode )
{
  autonomy_subscriber_->reset();
  nodesCmdVelPub_["semi_autonomy"]->publish();
  waitForAutonomyMsg();
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS );
}

TEST_F( AutonomyManagerTest, TestDetectUnknownNode )
{
  // since mode is already in unknown, we need to first publish a different mode
  autonomy_subscriber_->reset();
  nodesCmdVelPub_["semi_autonomy"]->publish();
  waitForAutonomyMsg();
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS );

  autonomy_subscriber_->reset();
  nodesCmdVelPub_["unknown"]->publish();
  waitForAutonomyMsg();
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN );
}

TEST_F( AutonomyManagerTest, TestTimeoutNoTimeoutSet )
{
  autonomy_subscriber_->reset();
  nodesCmdVelPub_["semi_autonomy"]->publish();
  waitForAutonomyMsg();
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS );
  // wait for 10 seconds no msg should arrive
  autonomy_subscriber_->reset();
  rclcpp::Rate rate( 1 );
  for ( int i = 0; i < 10; i++ ) {
    executor_->spin_some( std::chrono::seconds( 1 ) );
    rate.sleep();
    RCLCPP_INFO( autonomy_manager_->get_logger(), "Waiting for timeout: %d", i );
  }
  EXPECT_FALSE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS );
}

TEST_F( AutonomyManagerTest, TestTimeoutWithTimeoutSet )
{
  autonomy_subscriber_->reset();
  nodesPosePub_["semi_autonomy"]->publish();
  waitForAutonomyMsg();
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS );
  // wait for 10 seconds, msg should arrive after 3
  autonomy_subscriber_->reset();
  rclcpp::Rate rate( 1 );
  for ( int i = 0; i < 10; i++ ) {
    executor_->spin_some( std::chrono::seconds( 1 ) );
    rate.sleep();
    RCLCPP_INFO( autonomy_manager_->get_logger(), "Waiting for timeout: %d", i );
    if ( autonomy_subscriber_->receivedMsg() ) { break; }
  }
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::INACTIVE );
}

TEST_F( AutonomyManagerTest, ObservingSeveralTopicsTeleop_SA )
{
  autonomy_subscriber_->reset();
  nodesPosePub_["teleop"]->publish();
  nodesCmdVelPub_["semi_autonomy"]->publish(); // mode should remain teleop
  waitFor( 3 );
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::TELEOPERATED );
}

TEST_F( AutonomyManagerTest, ObservingSeveralTopicsTeleop_A )
{
  autonomy_subscriber_->reset();
  nodesPosePub_["teleop"]->publish();
  nodesCmdVelPub_["autonomy"]->publish(); // mode should remain teleop
  waitFor( 3 );
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::TELEOPERATED );
}

TEST_F( AutonomyManagerTest, ObservingSeveralTopicsTeleop_U )
{
  autonomy_subscriber_->reset();
  nodesPosePub_["teleop"]->publish();
  nodesCmdVelPub_["unknown"]->publish();
  waitFor( 3 );
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN );
}


TEST_F( AutonomyManagerTest, ObservingSeveralTopicsSemiAutonomy_A )
{
  autonomy_subscriber_->reset();
  nodesPosePub_["semi_autonomy"]->publish();
  nodesCmdVelPub_["autonomy"]->publish(); // mode should remain teleop
  waitFor( 3 );
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS );
}

TEST_F( AutonomyManagerTest, ObservingSeveralTopicsSemiAutonomy_U )
{
  autonomy_subscriber_->reset();
  nodesPosePub_["semi_autonomy"]->publish();
  nodesCmdVelPub_["unknown"]->publish();
  waitFor( 3 );
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN );
}

TEST_F( AutonomyManagerTest, ObservingSeveralTopicsAutonomy_U )
{
  autonomy_subscriber_->reset();
  nodesPosePub_["autonomy"]->publish();
  nodesCmdVelPub_["unknown"]->publish();
  waitFor( 3 );
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN );
}

TEST_F( AutonomyManagerTest, ObserversSeveralTopicsAutonomy_1Inactive )
{
  // make sure that the mode is inactive
  {
    autonomy_subscriber_->reset();
    nodesPosePub_["semi_autonomy"]->publish();
    waitForAutonomyMsg();
    EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
    EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
               autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS );
    // wait for 10 seconds, msg should arrive after 3
    autonomy_subscriber_->reset();
    rclcpp::Rate rate( 1 );
    for ( int i = 0; i < 10; i++ ) {
      executor_->spin_some( std::chrono::seconds( 1 ) );
      rate.sleep();
      RCLCPP_INFO( autonomy_manager_->get_logger(), "Waiting for timeout: %d", i );
      if ( autonomy_subscriber_->receivedMsg() ) { break; }
    }
    EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
    EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
               autonomy_manager_msgs::msg::AutonomyMode::INACTIVE );
  }
  // now test the actual test
  autonomy_subscriber_->reset();
  nodesCmdVelPub_["autonomy"]->publish();
  waitForAutonomyMsg();
  EXPECT_TRUE( autonomy_subscriber_->receivedMsg() );
  EXPECT_EQ( autonomy_subscriber_->autonomy_mode_.autonomy_mode,
             autonomy_manager_msgs::msg::AutonomyMode::AUTONOMOUS );
}

// sTART THE TESTS
int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}