#include "embertrace_core/waypoint_patrol.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace embertrace_core
{

namespace
{

PatrolRoute route_from_node(const YAML::Node & root, const std::string & source_description)
{
  PatrolRoute route;
  if (root["frame_id"]) {
    route.frame_id = root["frame_id"].as<std::string>();
  }

  const YAML::Node list = root["waypoints"];
  if (!list || !list.IsSequence() || list.size() == 0) {
    throw std::runtime_error(
      "Patrol waypoints source '" + source_description + "' defines no waypoints");
  }

  route.waypoints.reserve(list.size());
  for (std::size_t i = 0; i < list.size(); ++i) {
    const YAML::Node & entry = list[i];
    if (!entry["x"] || !entry["y"]) {
      throw std::runtime_error(
        "Waypoint " + std::to_string(i) + " in '" + source_description + "' is missing x/y");
    }
    Waypoint waypoint;
    waypoint.name = entry["name"] ?
      entry["name"].as<std::string>() :
      ("waypoint_" + std::to_string(i));
    waypoint.x = entry["x"].as<double>();
    waypoint.y = entry["y"].as<double>();
    waypoint.yaw = entry["yaw"] ? entry["yaw"].as<double>() : 0.0;
    route.waypoints.push_back(waypoint);
  }
  return route;
}

}  // namespace

PatrolRoute parse_patrol_route(const std::string & yaml_text)
{
  YAML::Node root;
  try {
    root = YAML::Load(yaml_text);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(std::string("Failed to parse patrol waypoints YAML: ") + error.what());
  }
  return route_from_node(root, "<in-memory YAML>");
}

PatrolRoute load_patrol_route(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error(
      "Failed to read patrol waypoints file '" + yaml_path + "': " + error.what());
  }
  return route_from_node(root, yaml_path);
}

WaypointPatrol::WaypointPatrol(const double arrival_tolerance_m)
: arrival_tolerance_m_(arrival_tolerance_m)
{
}

void WaypointPatrol::set_waypoints(
  std::vector<Waypoint> waypoints, const std::string & base_waypoint_name)
{
  waypoints_ = std::move(waypoints);
  current_index_ = 0;
  loops_completed_ = 0;
  mode_ = Mode::kInactive;
  returned_to_base_ = false;
  base_goal_succeeded_ = false;
  near_base_since_.reset();
  base_index_ = 0;
  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    if (waypoints_[i].name == base_waypoint_name) {
      base_index_ = i;
      break;
    }
  }
}

void WaypointPatrol::set_home_pose(double x, double y)
{
  if (waypoints_.empty() || !std::isfinite(x) || !std::isfinite(y)) return;
  waypoints_[base_index_].x = x;
  waypoints_[base_index_].y = y;
}

std::size_t WaypointPatrol::nearest_index(const double x, const double y) const
{
  std::size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    const double distance = std::hypot(x - waypoints_[i].x, y - waypoints_[i].y);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

std::optional<Waypoint> WaypointPatrol::issue(const bool is_retry)
{
  goal_issued_at_ = now_s_;
  retry_due_at_.reset();
  attempts_ = is_retry ? attempts_ + 1 : 1;
  return mode_ == Mode::kReturning ? waypoints_[base_index_] : waypoints_[current_index_];
}

std::optional<Waypoint> WaypointPatrol::advance_and_issue()
{
  const std::size_t previous_index = current_index_;
  current_index_ = (current_index_ + 1) % waypoints_.size();
  if (current_index_ <= previous_index) {
    ++loops_completed_;  // wrapped back to the front: one lap of the route is done
  }
  return issue(false);
}

std::optional<Waypoint> WaypointPatrol::retry_or_skip()
{
  if (mode_ == Mode::kPatrolling && attempts_ >= policy_.max_goal_attempts) {
    ++skipped_waypoints_;  // give up on this waypoint and carry on with the loop
    return advance_and_issue();
  }
  return issue(true);
}

std::optional<Waypoint> WaypointPatrol::update_pose(const double x, const double y)
{
  pose_x_ = x;
  pose_y_ = y;
  have_pose_ = true;

  if (mode_ == Mode::kReturning) {
    if (!waypoints_.empty() &&
      std::hypot(x - waypoints_[base_index_].x, y - waypoints_[base_index_].y) <=
      arrival_tolerance_m_)
    {
      returned_to_base_ = true;
      retry_due_at_.reset();
      if (!near_base_since_) {
        near_base_since_ = now_s_;
      }
    } else {
      near_base_since_.reset();
    }
    return std::nullopt;
  }

  if (mode_ != Mode::kPatrolling || waypoints_.empty()) {
    return std::nullopt;
  }

  const Waypoint & goal = waypoints_[current_index_];
  const double distance_to_goal = std::hypot(x - goal.x, y - goal.y);
  if (distance_to_goal > arrival_tolerance_m_) {
    return std::nullopt;  // still travelling toward the current goal
  }
  return advance_and_issue();
}

std::optional<Waypoint> WaypointPatrol::on_mission_state(const PatrolTrigger trigger)
{
  if (waypoints_.empty()) {
    return std::nullopt;
  }

  switch (trigger) {
    case PatrolTrigger::kPatrol:
      if (mode_ == Mode::kPatrolling) {
        return std::nullopt;  // already patrolling; ignore the repeated state message
      }
      mode_ = Mode::kPatrolling;
      returned_to_base_ = false;
      current_index_ = have_pose_ ? nearest_index(pose_x_, pose_y_) : base_index_;
      return issue(false);

    case PatrolTrigger::kReturnToBase:
      if (mode_ == Mode::kReturning) {
        return std::nullopt;  // already returning; ignore the repeated state message
      }
      mode_ = Mode::kReturning;
      returned_to_base_ = false;
      base_goal_succeeded_ = false;  // a fresh journey: forget any earlier arrival
      near_base_since_.reset();
      return issue(false);

    case PatrolTrigger::kHold:
    default:
      // Freeze current_index_ so PATROL can resume sensibly afterward instead
      // of restarting the loop from scratch, and drop any pending retry.
      mode_ = Mode::kInactive;
      retry_due_at_.reset();
      return std::nullopt;
  }
}

bool WaypointPatrol::return_complete() const
{
  if (mode_ != Mode::kReturning) {
    return false;
  }
  if (base_goal_succeeded_) {
    return true;
  }
  return near_base_since_.has_value() && (now_s_ - *near_base_since_) >= policy_.base_settle_s;
}

std::optional<Waypoint> WaypointPatrol::tick(const double now_s)
{
  now_s_ = now_s;
  if (waypoints_.empty() || mode_ == Mode::kInactive ||
    (mode_ == Mode::kReturning && returned_to_base_))
  {
    return std::nullopt;
  }
  if (!retry_due_at_ && (now_s_ - goal_issued_at_) > policy_.goal_timeout_s) {
    retry_due_at_ = now_s_;  // a goal that never completes counts as a failure
  }
  if (retry_due_at_ && now_s_ >= *retry_due_at_) {
    return retry_or_skip();
  }
  return std::nullopt;
}

std::optional<Waypoint> WaypointPatrol::on_nav_result(const NavResult result)
{
  if (waypoints_.empty() || mode_ == Mode::kInactive) {
    return std::nullopt;
  }
  const bool settled = (mode_ == Mode::kReturning && returned_to_base_);

  switch (result) {
    case NavResult::kSucceeded:
      if ((now_s_ - goal_issued_at_) < policy_.min_success_age_s) {
        return std::nullopt;  // almost certainly the previous goal's result
      }
      if (mode_ == Mode::kReturning) {
        returned_to_base_ = true;
        base_goal_succeeded_ = true;
        retry_due_at_.reset();
        return std::nullopt;
      }
      return advance_and_issue();

    case NavResult::kFailed:
      if (!settled && !retry_due_at_) {
        retry_due_at_ = now_s_ + policy_.retry_delay_s;
      }
      return std::nullopt;

    case NavResult::kOutOfBounds:
    default:
      if (mode_ == Mode::kPatrolling) {
        ++skipped_waypoints_;
        return advance_and_issue();
      }
      return std::nullopt;
  }
}

}  // namespace embertrace_core
