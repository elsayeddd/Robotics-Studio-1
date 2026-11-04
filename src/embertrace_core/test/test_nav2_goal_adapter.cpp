#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include "embertrace_core/nav2_goal_adapter_node.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;
using State = embertrace_interfaces::msg::MissionState;
using Navigate = nav2_msgs::action::NavigateToPose;
using Handle = rclcpp_action::ServerGoalHandle<Navigate>;

class AdapterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("adapter_test");
    auto options = rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("max_goal_age_s", 0.5),
      rclcpp::Parameter("pending_timeout_s", 0.8)});
    adapter = std::make_shared<embertrace_core::Nav2GoalAdapterNode>(options);
    executor.add_node(node); executor.add_node(adapter);
    state = node->create_publisher<State>("/embertrace/mission_state", rclcpp::QoS(1).transient_local());
    goals = node->create_publisher<geometry_msgs::msg::PoseStamped>("/embertrace/navigation_goal", 10);
    status = node->create_subscription<std_msgs::msg::String>("/embertrace/navigation_status", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {statuses.push_back(msg->data);});
    tf = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node->now(); transform.header.frame_id = "husky1_map";
    transform.child_frame_id = "husky1_odom"; transform.transform.rotation.w = 1;
    transform.transform.translation.x = 2;
    tf->sendTransform(transform);
    ASSERT_TRUE(until([this] {return state->get_subscription_count() > 0 &&
      goals->get_subscription_count() > 0 && status->get_publisher_count() > 0;}));
  }
  void TearDown() override
  {
    executor.remove_node(adapter); executor.remove_node(node);
    adapter.reset(); server.reset(); status.reset(); goals.reset(); state.reset(); tf.reset(); node.reset();
    rclcpp::shutdown();
  }
  bool until(const std::function<bool()> & predicate, double timeout = 3)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    do {executor.spin_some(); if (predicate()) {return true;} std::this_thread::sleep_for(5ms);}
    while (std::chrono::steady_clock::now() < deadline);
    return predicate();
  }
  void pump(double seconds) {until([] {return false;}, seconds);}
  void start_server(bool reject = false)
  {
    server = rclcpp_action::create_server<Navigate>(node, "/husky1/navigate_to_pose",
      [this, reject](const rclcpp_action::GoalUUID &, std::shared_ptr<const Navigate::Goal> goal) {
        received.push_back(goal->pose);
        return reject ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<Handle>) {++cancels; return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<Handle> handle) {handles.push_back(handle);});
    pump(0.15);
  }
  void mission(bool allowed, std::uint8_t value = State::PATROL)
  {
    State msg; msg.header.stamp = node->now(); msg.state = value; msg.motion_allowed = allowed;
    state->publish(msg); pump(0.1);
  }
  void goal(double x = 5, const std::string & frame = "husky1_map", double age = 0)
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = frame; msg.header.stamp = node->now() - rclcpp::Duration::from_seconds(age);
    msg.pose.position.x = x; msg.pose.orientation.w = 1; goals->publish(msg);
  }
  bool saw(const std::string & expected) const
  {return std::find(statuses.begin(), statuses.end(), expected) != statuses.end();}
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<embertrace_core::Nav2GoalAdapterNode> adapter;
  rclcpp_action::Server<Navigate>::SharedPtr server;
  rclcpp::Publisher<State>::SharedPtr state;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goals;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf;
  std::vector<geometry_msgs::msg::PoseStamped> received;
  std::vector<std::shared_ptr<Handle>> handles;
  std::vector<std::string> statuses;
  int cancels{0};
};

TEST_F(AdapterTest, DisallowedMotionNeverReachesActionServer)
{
  start_server(); goal();
  ASSERT_TRUE(until([this] {return saw("REJECTED");})); EXPECT_TRUE(received.empty());
}
TEST_F(AdapterTest, TransformsGoalAndReportsRealActionSuccess)
{
  start_server(); mission(true); goal(5, "husky1_odom");
  ASSERT_TRUE(until([this] {return !handles.empty();}));
  EXPECT_EQ(received[0].header.frame_id, "husky1_map");
  EXPECT_NEAR(received[0].pose.position.x, 7, 1e-6);
  handles[0]->succeed(std::make_shared<Navigate::Result>());
  EXPECT_TRUE(until([this] {return saw("SUCCEEDED");}));
}
TEST_F(AdapterTest, StopCancelsActiveAction)
{
  start_server(); mission(true); goal();
  ASSERT_TRUE(until([this] {return !handles.empty();}));
  mission(false, State::EMERGENCY_STOP);
  EXPECT_TRUE(until([this] {return cancels == 1;})); EXPECT_TRUE(saw("CANCEL_REQUESTED"));
}
TEST_F(AdapterTest, RejectsStaleAndOutOfBoundsGoals)
{
  start_server(); mission(true); goal(5, "husky1_map", 2);
  ASSERT_TRUE(until([this] {return saw("STALE");}));
  goal(100);
  ASSERT_TRUE(until([this] {return saw("OUT_OF_BOUNDS");})); EXPECT_TRUE(received.empty());
}
TEST_F(AdapterTest, ServerRejectionIsPublished)
{
  start_server(true); mission(true); goal();
  EXPECT_TRUE(until([this] {return saw("REJECTED");})); EXPECT_EQ(received.size(), 1U);
}
TEST_F(AdapterTest, PendingGoalExpiresWithoutNavigationServer)
{
  mission(true); goal();
  EXPECT_TRUE(until([this] {return saw("STALE");}));
  start_server(); pump(0.7); EXPECT_TRUE(received.empty());
}
TEST_F(AdapterTest, PauseDiscardsPendingGoal)
{
  mission(true); goal(); pump(0.1); mission(false, State::PAUSED);
  start_server(); pump(0.7); EXPECT_TRUE(received.empty());
}
TEST_F(AdapterTest, NewGoalCancelsPreviousAction)
{
  start_server(); mission(true); goal();
  ASSERT_TRUE(until([this] {return handles.size() == 1;}));
  goal(8);
  ASSERT_TRUE(until([this] {return handles.size() == 2 && cancels == 1;}));
  EXPECT_NEAR(received.back().pose.position.x, 8, 1e-6);
}
