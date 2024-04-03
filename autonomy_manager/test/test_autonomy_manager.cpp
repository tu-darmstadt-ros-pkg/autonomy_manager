#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <autonomy_manager_msgs/msg/autonomy_mode.hpp>
#include <autonomy_manager/autonomy_manager.h>

// a class that publishes on the cmd_vel topic
class CmdVelPublisher : public rclcpp::Node {
public:
    explicit CmdVelPublisher(const std::string &name) : Node(name) {
        pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    }

    void publish() {
        geometry_msgs::msg::Twist msg;
        pub_->publish(msg);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

// a class that subscribes to the autonomy_mode topic
class AutonomySubscriber : public rclcpp::Node {
public:
    explicit AutonomySubscriber(const std::string &name) : Node(name) {
        sub_ = create_subscription<autonomy_manager_msgs::msg::AutonomyMode>("/autonomy_mode", 10,
                                                                             std::bind(&AutonomySubscriber::callback,
                                                                                       this, std::placeholders::_1));
    }

    void callback(const autonomy_manager_msgs::msg::AutonomyMode::SharedPtr msg) {
        autonomy_mode_ = *msg;
        autonomy_mode_received_ = true;
    }

    void reset() {
        autonomy_mode_received_ = false;
    }

    bool receivedMsg() const {
        return autonomy_mode_received_;
    }


    autonomy_manager_msgs::msg::AutonomyMode autonomy_mode_;
private:
    bool autonomy_mode_received_{false};
    rclcpp::Subscription<autonomy_manager_msgs::msg::AutonomyMode>::SharedPtr sub_;
};

class AutonomyManagerTest : public ::testing::Test {
protected:
    std::map<std::string, std::shared_ptr<CmdVelPublisher>> nodes_;
    std::shared_ptr<AutonomySubscriber> autonomy_subscriber_;
    autonomy_manager::AutonomyManager::SharedPtr autonomy_manager_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    // autonomy Topic subscriber


    void SetUp() override {
        rclcpp::init(0, nullptr);
        // get all autonomyTypes as string
        std::vector<std::string> autonomyTypeNames = {"autonomy", "teleop", "semi_autonomy", "unknown"};
        std::vector<autonomy_manager_msgs::msg::AutonomyMode> autonomyTypes;
        autonomyTypes.reserve(autonomyTypeNames.size());
        for (const auto &autonomyTypeName: autonomyTypeNames) {
            autonomyTypes.push_back(autonomy_manager::fromString(autonomyTypeName));
        }
        for (const auto &autonomyType: autonomyTypeNames) {
            nodes_[autonomyType] = std::make_shared<CmdVelPublisher>(autonomyType);
        }
        // remove unknown type
        autonomyTypeNames.pop_back();
        autonomyTypes.pop_back();

        autonomy_subscriber_ = std::make_shared<AutonomySubscriber>("autonomy_subscriber");
        autonomy_manager::ObservedTopicInfo observedTopicInfo("/cmd_vel", autonomyTypeNames, autonomyTypeNames, 0);
        std::vector<autonomy_manager::ObservedTopicInfo> observedTopicInfos = {observedTopicInfo};
        autonomy_manager_ = std::make_shared<autonomy_manager::AutonomyManager>(observedTopicInfos);
        // setup executor
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(autonomy_manager_);
        executor_->add_node(autonomy_subscriber_);
        for (const auto &node: nodes_) {
            executor_->add_node(node.second);
        }

        waitForAutonomyMsg();
        autonomy_subscriber_->reset();

    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    void waitForAutonomyMsg() {
        int counter = 0;
        while (!autonomy_subscriber_->receivedMsg() && counter < 30) {
            executor_->spin_some(std::chrono::milliseconds(100));
            counter++;
        }
    }
};

TEST_F(AutonomyManagerTest, TestStartUp) {
    // without receiving a cmd_vel message, the autonomy mode should be "unknown"
    EXPECT_EQ(autonomy_subscriber_->autonomy_mode_.autonomy_mode, autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN);
}

TEST_F(AutonomyManagerTest, TestDetectAutonomyNode) {
    autonomy_subscriber_->reset();
    nodes_["autonomy"]->publish();
    waitForAutonomyMsg();
    EXPECT_TRUE(autonomy_subscriber_->receivedMsg());
    EXPECT_EQ(autonomy_subscriber_->autonomy_mode_.autonomy_mode, autonomy_manager_msgs::msg::AutonomyMode::AUTONOMOUS);
}

TEST_F(AutonomyManagerTest, TestDetectTeleopNode) {
    autonomy_subscriber_->reset();
    nodes_["teleop"]->publish();
    waitForAutonomyMsg();
    EXPECT_TRUE(autonomy_subscriber_->receivedMsg());
    EXPECT_EQ(autonomy_subscriber_->autonomy_mode_.autonomy_mode,
              autonomy_manager_msgs::msg::AutonomyMode::TELEOPERATED);
}

TEST_F(AutonomyManagerTest, TestDetectSemiAutonomyNode) {
    autonomy_subscriber_->reset();
    nodes_["semi_autonomy"]->publish();
    waitForAutonomyMsg();
    EXPECT_TRUE(autonomy_subscriber_->receivedMsg());
    EXPECT_EQ(autonomy_subscriber_->autonomy_mode_.autonomy_mode,
              autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS);
}

TEST_F(AutonomyManagerTest, TestDetectUnknownNode) {
    // since mode is already in unknown, we need to first publish a different mode
    autonomy_subscriber_->reset();
    nodes_["semi_autonomy"]->publish();
    waitForAutonomyMsg();
    EXPECT_TRUE(autonomy_subscriber_->receivedMsg());
    EXPECT_EQ(autonomy_subscriber_->autonomy_mode_.autonomy_mode,
              autonomy_manager_msgs::msg::AutonomyMode::SEMI_AUTONOMOUS);

    autonomy_subscriber_->reset();
    nodes_["unknown"]->publish();
    waitForAutonomyMsg();
    EXPECT_TRUE(autonomy_subscriber_->receivedMsg());
    EXPECT_EQ(autonomy_subscriber_->autonomy_mode_.autonomy_mode, autonomy_manager_msgs::msg::AutonomyMode::UNKNOWN);
}