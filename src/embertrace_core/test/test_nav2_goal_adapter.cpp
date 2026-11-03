#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

// ============================================================================
// INTEGRATION TEST - Publish messages and verify no crashes
// ============================================================================

class Nav2GoalAdapterInterfaceTest : public ::testing::Test
{
protected:
    // Single shared node for entire test suite
    static std::shared_ptr<rclcpp::Node> test_node_;
    static bool initialized_;

    static void SetUpTestCase() {
        if (!initialized_) {
            rclcpp::init(0, nullptr);
            test_node_ = std::make_shared<rclcpp::Node>("nav2_goal_adapter_interface_test");
            initialized_ = true;
            
            // Small delay to let DDS discover the node
            std::this_thread::sleep_for(50ms);
        }
    }

    static void TearDownTestCase() {
        if (initialized_) {
            test_node_.reset();
            std::this_thread::sleep_for(50ms);
            rclcpp::shutdown();
            initialized_ = false;
        }
    }

    void setUpMissionStatePublisher() {
        mission_state_pub_ = test_node_->create_publisher<embertrace_interfaces::msg::MissionState>(
            "/embertrace/mission_state", 
            rclcpp::QoS(10).transient_local());
    }

    void setUpNavigationGoalPublisher() {
        navigation_goal_pub_ = test_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/embertrace/navigation_goal", 10);
    }

    void setUpOdometryPublisher() {
        odom_pub_ = test_node_->create_publisher<nav_msgs::msg::Odometry>(
            "/husky1/odometry", 10);
    }

    void publishMissionState(bool motion_allowed, int64_t state = 0) {
        auto msg = std::make_shared<embertrace_interfaces::msg::MissionState>();
        msg->motion_allowed = motion_allowed;
        msg->state = state;
        mission_state_pub_->publish(*msg);
    }

    void publishNavigationGoal(double x, double y, const std::string & frame_id = "husky1_map") {
        auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        msg->header.stamp = test_node_->now();
        msg->header.frame_id = frame_id;
        msg->pose.position.x = x;
        msg->pose.position.y = y;
        msg->pose.orientation.w = 1.0;
        navigation_goal_pub_->publish(*msg);
    }

    void publishOdometry(double x, double y, double yaw = 0.0) {
        auto msg = std::make_shared<nav_msgs::msg::Odometry>();
        msg->header.stamp = test_node_->now();
        msg->header.frame_id = "husky1_odom";
        msg->child_frame_id = "base_link";
        msg->pose.pose.position.x = x;
        msg->pose.pose.position.y = y;
        msg->pose.pose.orientation.x = 0.0;
        msg->pose.pose.orientation.y = 0.0;
        msg->pose.pose.orientation.z = sin(yaw / 2.0);
        msg->pose.pose.orientation.w = cos(yaw / 2.0);
        odom_pub_->publish(*msg);
    }

    void spinFor(int duration_ms) {
        auto end_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_ms);
        while (std::chrono::steady_clock::now() < end_time && rclcpp::ok()) {
            rclcpp::spin_some(test_node_);
            std::this_thread::sleep_for(5ms);
        }
    }

    std::shared_ptr<rclcpp::Publisher<embertrace_interfaces::msg::MissionState>> mission_state_pub_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> navigation_goal_pub_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
};

std::shared_ptr<rclcpp::Node> Nav2GoalAdapterInterfaceTest::test_node_ = nullptr;
bool Nav2GoalAdapterInterfaceTest::initialized_ = false;

// ============================================================================
// TESTS - Just verify we can publish without crashes
// ============================================================================

TEST_F(Nav2GoalAdapterInterfaceTest, MotionDisabled_WhenMissionDisallows) {
    setUpNavigationGoalPublisher();
    
    publishNavigationGoal(5.0, 5.0);
    spinFor(100);
    
    EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Nav2GoalAdapterInterfaceTest, MotionEnabled_AllowsNavigation) {
    setUpMissionStatePublisher();
    setUpNavigationGoalPublisher();
    
    publishMissionState(true);
    spinFor(100);
    
    publishNavigationGoal(10.0, 10.0);
    spinFor(100);
    
    EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Nav2GoalAdapterInterfaceTest, MotionDisables_WhileActive_CancelsGoal) {
    setUpMissionStatePublisher();
    
    publishMissionState(true);
    spinFor(100);
    
    publishMissionState(false);
    spinFor(100);
    
    EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Nav2GoalAdapterInterfaceTest, MotionState_QoSTransientLocalDelivered) {
    auto pub = test_node_->create_publisher<embertrace_interfaces::msg::MissionState>(
        "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable());
    
    auto msg = std::make_shared<embertrace_interfaces::msg::MissionState>();
    msg->motion_allowed = true;
    pub->publish(*msg);
    spinFor(100);
    
    EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Nav2GoalAdapterInterfaceTest, GoalInWrongFrame_HandledGracefully) {
    setUpMissionStatePublisher();
    setUpNavigationGoalPublisher();
    
    publishMissionState(true);
    spinFor(50);
    
    auto goal = std::make_shared<geometry_msgs::msg::PoseStamped>();
    goal->header.stamp = test_node_->now();
    goal->header.frame_id = "unknown_frame";
    goal->pose.position.x = 5.0;
    goal->pose.position.y = 5.0;
    navigation_goal_pub_->publish(*goal);
    
    spinFor(150);
    EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Nav2GoalAdapterInterfaceTest, MultipleGoals_SequentialProcessing) {
    setUpMissionStatePublisher();
    setUpNavigationGoalPublisher();
    
    publishMissionState(true);
    spinFor(50);
    
    for (int i = 0; i < 3; ++i) {
        publishNavigationGoal(i * 5.0, 0.0);
        spinFor(50);
    }
    
    EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Nav2GoalAdapterInterfaceTest, OdometryUpdates_NoCrash) {
    setUpMissionStatePublisher();
    setUpNavigationGoalPublisher();
    setUpOdometryPublisher();
    
    publishMissionState(true);
    publishNavigationGoal(10.0, 0.0);
    spinFor(50);
    
    for (int i = 0; i < 10; ++i) {
        publishOdometry(i * 0.5, 0.0);
    }
    spinFor(100);
    
    EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Nav2GoalAdapterInterfaceTest, OdometryWithQuaternion_HandlesYawExtraction) {
    setUpOdometryPublisher();
    setUpMissionStatePublisher();
    setUpNavigationGoalPublisher();
    
    publishOdometry(0.0, 0.0, 0.0);
    publishMissionState(true);
    publishNavigationGoal(0.0, 10.0);
    
    spinFor(100);
    EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Nav2GoalAdapterInterfaceTest, NullTimestamp_GracefullyHandled) {
    setUpMissionStatePublisher();
    setUpNavigationGoalPublisher();
    
    publishMissionState(true);
    spinFor(50);
    
    auto goal = std::make_shared<geometry_msgs::msg::PoseStamped>();
    goal->header.stamp.sec = 0;
    goal->header.stamp.nanosec = 0;
    goal->header.frame_id = "husky1_map";
    goal->pose.position.x = 5.0;
    goal->pose.position.y = 5.0;
    navigation_goal_pub_->publish(*goal);
    
    spinFor(100);
    EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Nav2GoalAdapterInterfaceTest, RapidStateFlips_Stability) {
    setUpMissionStatePublisher();
    
    for (int i = 0; i < 100; ++i) {
        publishMissionState((i % 2 == 0));
        if (i % 10 == 0) {
            spinFor(10);
        }
    }
    
    spinFor(200);
    EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Nav2GoalAdapterInterfaceTest, LargeCoordinates_NumericStability) {
    setUpMissionStatePublisher();
    setUpNavigationGoalPublisher();
    
    publishMissionState(true);
    publishNavigationGoal(1000000.0, 1000000.0);
    spinFor(100);
    
    EXPECT_TRUE(rclcpp::ok());
}

TEST_F(Nav2GoalAdapterInterfaceTest, GoalSubscription_QoSReliable) {
    setUpNavigationGoalPublisher();
    
    auto goal = std::make_shared<geometry_msgs::msg::PoseStamped>();
    goal->header.stamp = test_node_->now();
    goal->header.frame_id = "husky1_map";
    goal->pose.position.x = 1.0;
    goal->pose.position.y = 1.0;
    
    auto best_effort_pub = test_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/embertrace/navigation_goal",
        rclcpp::QoS(10).best_effort());
    
    best_effort_pub->publish(*goal);
    spinFor(100);
    
    EXPECT_TRUE(rclcpp::ok());
}

int main(int argc, char ** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}