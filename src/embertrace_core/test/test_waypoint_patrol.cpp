#include <gtest/gtest.h>

#include "embertrace_core/waypoint_patrol.hpp"

namespace embertrace_core
{

namespace
{

// Verbatim contents of embertrace_bringup/config/patrol_waypoints.yaml, kept
// in sync here so this test fails loudly if the real file's shape changes.
constexpr const char * kRealPatrolWaypointsYaml = R"(
frame_id: husky1_map
waypoints:
  - {name: ranger_base, x: -14.0, y: -14.0, yaw: 0.7854}
  - {name: west_south, x: -10.0, y: -8.0, yaw: 1.5708}
  - {name: west_centre, x: -10.0, y: 0.0, yaw: 1.5708}
  - {name: west_north, x: -10.0, y: 10.0, yaw: 0.0}
  - {name: north_centre, x: 0.0, y: 10.0, yaw: 0.0}
  - {name: north_east, x: 10.0, y: 10.0, yaw: -1.5708}
  - {name: east_centre, x: 10.0, y: 4.0, yaw: -1.5708}
  - {name: junction, x: 10.0, y: 0.0, yaw: 3.1416}
  - {name: centre, x: 0.0, y: 0.0, yaw: 3.1416}
  - {name: west_centre_return, x: -10.0, y: 0.0, yaw: -1.5708}
)";

std::vector<Waypoint> square_route()
{
  return {
    {"a", 0.0, 0.0, 0.0},
    {"b", 1.0, 0.0, 0.0},
    {"c", 1.0, 1.0, 0.0},
    {"d", 0.0, 1.0, 0.0},
  };
}

}  // namespace

TEST(PatrolRouteYaml, ParsesTheRealBringupWaypointsFile)
{
  const PatrolRoute route = parse_patrol_route(kRealPatrolWaypointsYaml);
  ASSERT_EQ(route.frame_id, "husky1_map");
  ASSERT_EQ(route.waypoints.size(), 10U);
  EXPECT_EQ(route.waypoints.front().name, "ranger_base");
  EXPECT_DOUBLE_EQ(route.waypoints.front().x, -14.0);
  EXPECT_DOUBLE_EQ(route.waypoints.front().y, -14.0);
  EXPECT_EQ(route.waypoints.back().name, "west_centre_return");
  EXPECT_DOUBLE_EQ(route.waypoints[5].yaw, -1.5708);
}

TEST(PatrolRouteYaml, ThrowsWhenNoWaypointsAreDefined)
{
  EXPECT_THROW(parse_patrol_route("frame_id: map\nwaypoints: []\n"), std::runtime_error);
  EXPECT_THROW(parse_patrol_route("frame_id: map\n"), std::runtime_error);
}

TEST(PatrolRouteYaml, ThrowsWhenAWaypointIsMissingCoordinates)
{
  EXPECT_THROW(
    parse_patrol_route("waypoints:\n  - {name: only_a_name}\n"), std::runtime_error);
}

TEST(WaypointPatrol, SendsWaypointsInSequenceWhilePatrolling)
{
  WaypointPatrol patrol;
  patrol.set_waypoints(square_route(), "a");

  const auto first_goal = patrol.on_mission_state(PatrolTrigger::kPatrol);
  ASSERT_TRUE(first_goal.has_value());
  EXPECT_EQ(first_goal->name, "a");  // no pose known yet: falls back to the base waypoint

  const auto second_goal = patrol.update_pose(0.0, 0.0);  // arrives at "a"
  ASSERT_TRUE(second_goal.has_value());
  EXPECT_EQ(second_goal->name, "b");

  const auto still_travelling = patrol.update_pose(0.3, 0.0);  // short of "b", outside tolerance
  EXPECT_FALSE(still_travelling.has_value());

  const auto third_goal = patrol.update_pose(1.0, 0.0);  // arrives at "b"
  ASSERT_TRUE(third_goal.has_value());
  EXPECT_EQ(third_goal->name, "c");
}

TEST(WaypointPatrol, CompletesTwoLoopsByCyclingThroughTheRoute)
{
  WaypointPatrol patrol;
  patrol.set_waypoints(square_route(), "a");
  patrol.on_mission_state(PatrolTrigger::kPatrol);  // starts at "a"

  const std::vector<std::pair<double, double>> stops = {
    {0.0, 0.0},  // a
    {1.0, 0.0},  // b
    {1.0, 1.0},  // c
    {0.0, 1.0},  // d -> wraps to a: loop 1 complete
    {0.0, 0.0},  // a
    {1.0, 0.0},  // b
    {1.0, 1.0},  // c
    {0.0, 1.0},  // d -> wraps to a: loop 2 complete
  };
  for (const auto & [x, y] : stops) {
    patrol.update_pose(x, y);
  }

  EXPECT_EQ(patrol.loops_completed(), 2U);
  EXPECT_EQ(patrol.current_index(), 0U);
  EXPECT_TRUE(patrol.is_patrolling());
}

TEST(WaypointPatrol, PausesOnAnyNonPatrolNonReturnStateAndStopsIssuingGoals)
{
  WaypointPatrol patrol;
  patrol.set_waypoints(square_route(), "a");
  patrol.on_mission_state(PatrolTrigger::kPatrol);
  patrol.update_pose(0.0, 0.0);  // now heading to "b" (index 1)
  ASSERT_EQ(patrol.current_index(), 1U);

  const auto hold_result = patrol.on_mission_state(PatrolTrigger::kHold);  // e.g. INVESTIGATE
  EXPECT_FALSE(hold_result.has_value());
  EXPECT_FALSE(patrol.is_patrolling());

  // While paused, pose updates (the rover chasing a plume) must not advance
  // or otherwise disturb the frozen patrol index.
  const auto ignored = patrol.update_pose(5.0, 5.0);
  EXPECT_FALSE(ignored.has_value());
  EXPECT_EQ(patrol.current_index(), 1U);
}

TEST(WaypointPatrol, ResumesAtNearestWaypointAfterADismissedFalseAlarm)
{
  WaypointPatrol patrol;
  patrol.set_waypoints(square_route(), "a");
  patrol.on_mission_state(PatrolTrigger::kPatrol);
  patrol.update_pose(0.0, 0.0);              // now heading to "b" (index 1)
  patrol.on_mission_state(PatrolTrigger::kHold);  // smoke detected -> INVESTIGATE
  patrol.update_pose(0.05, 0.95);            // chased the lead far from the loop, near "d"

  const auto resumed_goal = patrol.on_mission_state(PatrolTrigger::kPatrol);  // false alarm dismissed
  ASSERT_TRUE(resumed_goal.has_value());
  EXPECT_EQ(resumed_goal->name, "d");  // nearest waypoint to (0.05, 0.95), not the frozen index "b"
  EXPECT_EQ(patrol.current_index(), 3U);
}

TEST(WaypointPatrol, IgnoresRepeatedIdenticalMissionStateMessages)
{
  // mission_manager_node republishes MissionState every 500ms even when
  // nothing changed; the patrol must not re-publish or re-pick a waypoint
  // on every one of those repeats.
  WaypointPatrol patrol;
  patrol.set_waypoints(square_route(), "a");
  patrol.on_mission_state(PatrolTrigger::kPatrol);
  patrol.update_pose(0.0, 0.0);  // heading to "b"
  ASSERT_EQ(patrol.current_index(), 1U);

  const auto repeat = patrol.on_mission_state(PatrolTrigger::kPatrol);
  EXPECT_FALSE(repeat.has_value());
  EXPECT_EQ(patrol.current_index(), 1U);
}

TEST(WaypointPatrol, ReturnsToTheNamedBaseWaypointOnReturn)
{
  auto waypoints = square_route();
  waypoints.push_back({"ranger_base", -5.0, -5.0, 0.0});  // base is not index 0
  WaypointPatrol patrol;
  patrol.set_waypoints(waypoints, "ranger_base");

  patrol.on_mission_state(PatrolTrigger::kPatrol);
  patrol.update_pose(1.0, 1.0);  // now somewhere mid-loop, far from base

  const auto return_goal = patrol.on_mission_state(PatrolTrigger::kReturnToBase);
  ASSERT_TRUE(return_goal.has_value());
  EXPECT_EQ(return_goal->name, "ranger_base");
  EXPECT_TRUE(patrol.is_returning());
  EXPECT_FALSE(patrol.is_patrolling());
  EXPECT_FALSE(patrol.has_reached_base());

  const auto repeat = patrol.on_mission_state(PatrolTrigger::kReturnToBase);
  EXPECT_FALSE(repeat.has_value());  // still returning; do not re-issue the same goal

  patrol.update_pose(-5.0, -5.0);
  EXPECT_TRUE(patrol.has_reached_base());
}

TEST(WaypointPatrol, LeavingAndResumingPatrolRepeatedlyStaysConsistent)
{
  // Exercises "can leave/resume patrol cleanly": several pause/resume and
  // false-alarm cycles interleaved with genuine progress, ending back on a
  // clean, predictable loop count. Uses a tight tolerance so intermediate,
  // not-yet-arrived poses are unambiguous.
  WaypointPatrol patrol(0.2);
  patrol.set_waypoints(square_route(), "a");

  patrol.on_mission_state(PatrolTrigger::kPatrol);
  patrol.update_pose(0.0, 0.0);   // arrives at "a" -> now heading to "b"
  patrol.on_mission_state(PatrolTrigger::kHold);   // interrupted en route to "b"
  patrol.update_pose(0.6, 0.0);   // rover holds partway to "b" while investigating
  patrol.on_mission_state(PatrolTrigger::kPatrol); // false alarm dismissed, nearest is "b"
  EXPECT_EQ(patrol.current_index(), 1U);

  patrol.update_pose(1.0, 0.0);   // arrives at "b" -> now heading to "c"
  patrol.update_pose(1.0, 1.0);   // arrives at "c" -> now heading to "d"
  patrol.update_pose(0.3, 1.0);   // partway to "d" (closer to "d" than "c", not yet arrived)
  patrol.on_mission_state(PatrolTrigger::kHold);   // paused again right before completing lap 1
  patrol.on_mission_state(PatrolTrigger::kPatrol); // resumed with no further movement: nearest is "d"
  EXPECT_EQ(patrol.current_index(), 3U);

  patrol.update_pose(0.0, 1.0);   // arrives at "d" -> wraps to "a": loop 1 complete
  EXPECT_EQ(patrol.loops_completed(), 1U);
  EXPECT_EQ(patrol.current_index(), 0U);
  EXPECT_TRUE(patrol.is_patrolling());
}


// ---------------------------------------------------------------------------
// Nav2 integration behaviour (nav2_goal_adapter_node status feedback)
// ---------------------------------------------------------------------------

TEST(WaypointPatrolNav2, KeepsPatrollingWhenNav2StopsInsideItsGoalTolerance)
{
  // nav2_params_husky1.yaml sets xy_goal_tolerance = 1.0, so Nav2 declares a
  // goal reached and stops the rover up to 1 m short of the waypoint. A patrol
  // arrival tolerance below that value stalls forever; 1.2 m does not.
  const double nav2_stop_distance = 0.9;
  auto drive = [&](WaypointPatrol & patrol, const Waypoint & goal) {
      // rover stops nav2_stop_distance short of `goal`, approaching along +x
      return patrol.update_pose(goal.x - nav2_stop_distance, goal.y);
    };

  WaypointPatrol stalled(0.5);
  stalled.set_waypoints(square_route(), "a");
  auto goal = stalled.on_mission_state(PatrolTrigger::kPatrol);
  ASSERT_TRUE(goal.has_value());
  EXPECT_FALSE(drive(stalled, *goal).has_value());  // never advances

  WaypointPatrol patrol(1.2);
  patrol.set_waypoints(square_route(), "a");
  goal = patrol.on_mission_state(PatrolTrigger::kPatrol);
  for (int leg = 0; leg < 8; ++leg) {  // two laps of the four-waypoint square
    ASSERT_TRUE(goal.has_value());
    goal = drive(patrol, *goal);
  }
  EXPECT_EQ(patrol.loops_completed(), 2U);
}

TEST(WaypointPatrolNav2, SuccessResultAdvancesWhenProximityNeverFires)
{
  WaypointPatrol patrol(0.5);
  patrol.set_waypoints(square_route(), "a");
  patrol.tick(10.0);
  ASSERT_EQ(patrol.on_mission_state(PatrolTrigger::kPatrol)->name, "a");

  patrol.tick(12.0);  // two seconds later Nav2 reports success
  const auto next = patrol.on_nav_result(NavResult::kSucceeded);
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(next->name, "b");
}

TEST(WaypointPatrolNav2, IgnoresASuccessThatArrivesRightAfterANewGoal)
{
  WaypointPatrol patrol(0.5);
  patrol.set_waypoints(square_route(), "a");
  patrol.tick(10.0);
  patrol.on_mission_state(PatrolTrigger::kPatrol);

  patrol.tick(10.2);  // 0.2 s after issuing "a": this is the previous goal's result
  EXPECT_FALSE(patrol.on_nav_result(NavResult::kSucceeded).has_value());
  EXPECT_EQ(patrol.current_index(), 0U);
}

TEST(WaypointPatrolNav2, ResendsTheSameWaypointAfterAFailureThenSkipsIt)
{
  RecoveryPolicy policy;
  policy.retry_delay_s = 2.0;
  policy.max_goal_attempts = 3;
  WaypointPatrol patrol(0.5);
  patrol.set_recovery_policy(policy);
  patrol.set_waypoints(square_route(), "a");
  patrol.tick(0.0);
  ASSERT_EQ(patrol.on_mission_state(PatrolTrigger::kPatrol)->name, "a");  // attempt 1

  // e.g. the adapter reported REJECTED because motion was not yet allowed
  EXPECT_FALSE(patrol.on_nav_result(NavResult::kFailed).has_value());
  EXPECT_FALSE(patrol.tick(1.0).has_value());          // retry delay not elapsed
  const auto retry_1 = patrol.tick(2.1);               // attempt 2
  ASSERT_TRUE(retry_1.has_value());
  EXPECT_EQ(retry_1->name, "a");

  patrol.on_nav_result(NavResult::kFailed);
  const auto retry_2 = patrol.tick(4.5);               // attempt 3
  ASSERT_TRUE(retry_2.has_value());
  EXPECT_EQ(retry_2->name, "a");
  EXPECT_EQ(patrol.goal_attempts(), 3);

  patrol.on_nav_result(NavResult::kFailed);
  const auto skipped = patrol.tick(7.0);               // out of attempts: move on
  ASSERT_TRUE(skipped.has_value());
  EXPECT_EQ(skipped->name, "b");
  EXPECT_EQ(patrol.skipped_waypoints(), 1U);
  EXPECT_EQ(patrol.goal_attempts(), 1);
}

TEST(WaypointPatrolNav2, OutOfBoundsWaypointIsSkippedImmediately)
{
  WaypointPatrol patrol;
  patrol.set_waypoints(square_route(), "a");
  patrol.on_mission_state(PatrolTrigger::kPatrol);
  const auto next = patrol.on_nav_result(NavResult::kOutOfBounds);
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(next->name, "b");
  EXPECT_EQ(patrol.skipped_waypoints(), 1U);
}

TEST(WaypointPatrolNav2, WatchdogResendsAGoalThatNeverCompletes)
{
  RecoveryPolicy policy;
  policy.goal_timeout_s = 30.0;
  WaypointPatrol patrol(0.5);
  patrol.set_recovery_policy(policy);
  patrol.set_waypoints(square_route(), "a");
  patrol.tick(0.0);
  patrol.on_mission_state(PatrolTrigger::kPatrol);

  EXPECT_FALSE(patrol.tick(29.0).has_value());
  const auto resent = patrol.tick(31.0);
  ASSERT_TRUE(resent.has_value());
  EXPECT_EQ(resent->name, "a");
  EXPECT_EQ(patrol.goal_attempts(), 2);
}

TEST(WaypointPatrolNav2, NothingIsResentWhilePausedForAnInvestigation)
{
  WaypointPatrol patrol(0.5);
  patrol.set_waypoints(square_route(), "a");
  patrol.tick(0.0);
  patrol.on_mission_state(PatrolTrigger::kPatrol);
  patrol.on_nav_result(NavResult::kFailed);          // retry now pending
  patrol.on_mission_state(PatrolTrigger::kHold);     // smoke detected -> INVESTIGATE

  EXPECT_FALSE(patrol.tick(100.0).has_value());      // no resend, no timeout resend
  EXPECT_FALSE(patrol.on_nav_result(NavResult::kSucceeded).has_value());
  EXPECT_FALSE(patrol.on_nav_result(NavResult::kOutOfBounds).has_value());
  EXPECT_EQ(patrol.current_index(), 0U);
}

TEST(WaypointPatrolNav2, ReturnToBaseRetriesUntilNav2ReportsSuccess)
{
  auto waypoints = square_route();
  waypoints.push_back({"ranger_base", -5.0, -5.0, 0.0});
  WaypointPatrol patrol(0.5);
  patrol.set_waypoints(waypoints, "ranger_base");
  patrol.tick(0.0);
  patrol.on_mission_state(PatrolTrigger::kPatrol);
  patrol.tick(5.0);
  ASSERT_EQ(patrol.on_mission_state(PatrolTrigger::kReturnToBase)->name, "ranger_base");

  patrol.on_nav_result(NavResult::kFailed);
  const auto retry = patrol.tick(7.5);
  ASSERT_TRUE(retry.has_value());
  EXPECT_EQ(retry->name, "ranger_base");

  patrol.tick(10.0);
  EXPECT_FALSE(patrol.on_nav_result(NavResult::kSucceeded).has_value());
  EXPECT_TRUE(patrol.has_reached_base());
  EXPECT_FALSE(patrol.tick(200.0).has_value());      // arrived: nothing more to send
}

TEST(WaypointPatrol, ReturnsToObservedLaunchPose)
{
  WaypointPatrol patrol(0.5);
  patrol.set_waypoints({{"ranger_base", -5.0, -5.0, 0.0},
                        {"survey", 5.0, 5.0, 0.0}}, "ranger_base");
  patrol.set_home_pose(-13.4, -12.7);
  const auto home = patrol.on_mission_state(PatrolTrigger::kReturnToBase);
  ASSERT_TRUE(home.has_value());
  EXPECT_DOUBLE_EQ(home->x, -13.4);
  EXPECT_DOUBLE_EQ(home->y, -12.7);
}

}  // namespace embertrace_core