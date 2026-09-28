/**
 * THIS IS ADAPTED FROM SCAFFOLDS
 * STARTER FILE — intentionally not included in CMakeLists.txt yet.
 *
 * Purpose:
 *   Convert /embertrace/navigation_goal messages into Nav2 NavigateToPose actions.
 *
 * Before enabling this file:
 *   1. Add rclcpp_action, nav2_msgs, tf2_ros and tf2_geometry_msgs dependencies.
 *   2. Transform goals from husky1_odom into husky1_map.
 *   3. Cancel the active action whenever MissionState.motion_allowed is false.
 *   4. Publish result status for the patrol and mission managers.
 *   5. Add the executable to embertrace_core/CMakeLists.txt.
 *
 * This safe starter subscribes and logs only. It cannot accidentally command motion.
 */

#include <memory>

#include "embertrace_interfaces/msg/mission_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
// #include "tf2_ros/tf2_ros/"
#include "tf2_geometry_msgs.hpp"
#include <cmath.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace embertrace_core
{

class Nav2GoalAdapterNode : public rclcpp::Node
{
public:
  Nav2GoalAdapterNode()
  : Node("nav2_goal_adapter_node")
  {
    state_subscription_ = create_subscription<embertrace_interfaces::msg::MissionState>(
      "/embertrace/mission_state", rclcpp::QoS(1).transient_local().reliable(),
      [this](const embertrace_interfaces::msg::MissionState::SharedPtr message) {
        state_ = message.state;
        motion_allowed_ = message->motion_allowed;
        if (!motion_allowed_) {
          // TODO(student): call async_cancel_goal on the active Nav2 goal.
          RCLCPP_WARN(get_logger(), "Motion disabled; Nav2 goal must be cancelled");
        }
      });

    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/embertrace/navigation_goal", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        goal_ = goal;
        if (!motion_allowed_) {
          RCLCPP_WARN(get_logger(), "Ignoring goal because the mission does not permit motion");
          return;
        }
        RCLCPP_INFO(
          get_logger(), "TODO: send Nav2 goal (%.2f, %.2f) from frame %s",
          goal->pose.position.x, goal->pose.position.y, goal->header.frame_id.c_str());
        // TODO(student): transform goal, create NavigateToPose::Goal and call async_send_goal.
      });

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/husky1/odometry", 1000, /*
      [this](nav_msgs::msg::Odometry::SharedPtr odom){
        double dist_to_goal = hypot(odom->pose.pose.position.x - goal.
      }
        /*/std::bind(&nav2_goal_adapter_node::odoCallback,this,std::placeholders::_1;));
  }

private:
  bool motion_allowed_{false};
  rclcpp::Subscription<embertrace_interfaces::msg::MissionState>::SharedPtr state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  embertrace_interfaces::msg::MissionState state_;//! copied state from subscriber.

  geometry_msgs::msg::Pose pose_;//!< The current pose of platform
  std::mutex poseMtx_; //!< Mutex for controlling access to pose

  geometry_msgs::msg::PoseStamped goal_; //goal copied from subscriber
  std::mutex goalMtx_;

  double dist_to_goal; //!< distance from platform to goal
  double angle_to_goal; //!< angle from platform to goal
  const double angleTolerance = 0.2; // the acceptable difference between the yaw and angleToGoal
  const double calibratedTurningSpeed_ = 0.75; // used in reachGoal to avoid exceding time

  //need goalCallback
  /*
  {
      std::lock_guard<std::mutex> lock(goalMtx_);
      goal_ = msg.pose.pose;
    }
  */

  //Assuming angle range is 0 to 2pi
  //Will need to change if range is -pi to pi
  void normaliseAngle(double &angle)
  {
    if (angle < 0.0){
      angle = angle + 2*M_PI;
    }
    else if (angle > 2*M_PI){
      angle = angle - 2*M_PI;
    }
  }

  
  void nav2_goal_adapter_node::odoCallback(const nav_msgs::msg::Odometry& msg){
    {
      std::unique_lock<std::mutex> lock(poseMtx_);
      pose_ = msg.pose.pose;
    }    

    double dist_to_goal = hypot((pose_->pose.pose.position.x - goal_.pose.point.x) + (pose_->pose.pose.position.y - goal_.pose.point.y));

    normaliseAngle(pose_->pose.orientation);
    angle_to_goal = atan2(goal_.pose.point.y - pose_->pose.pose.position.y, goal_.pose.point.x - pose_->pose.pose.position.x); 
    normaliseAngle(angle_to_goal);


    //copy over skidsteer if necessary
    if (state_.motion_allowed == false){
      // set cmd to 0
    }else{
      if (fabs(odometry_.yaw - angle) <= angleTolerance)
      { 
        //set turning to 0; drive forward
        // action_ = IDLE;
        // cmd_.at(1) = 0;
      }
      else{

        if (odometry_.yaw < angle){
          if ((angle-odometry_.yaw)<M_PI){
            // cmd_.at(1) = calibratedTurningSpeed_; // left = +ve
          }
          else{
            // cmd_.at(1) = -calibratedTurningSpeed_; // right = -ve
          }
        }
        else{
          if ((odometry_.yaw-angle)<M_PI){
            // cmd_.at(1) = -calibratedTurningSpeed_; // right
          }
          else{
            // cmd_.at(1) = calibratedTurningSpeed_; // left 
          }
        }
      }
    }

  }
}; 

}  // namespace embertrace_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embertrace_core::Nav2GoalAdapterNode>());
  rclcpp::shutdown();
  return 0;
}

