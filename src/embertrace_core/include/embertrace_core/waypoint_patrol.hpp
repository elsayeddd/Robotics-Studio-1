#ifndef EMBERTRACE_CORE__WAYPOINT_PATROL_HPP_
#define EMBERTRACE_CORE__WAYPOINT_PATROL_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace embertrace_core
{

/// A single patrol stop, expressed in the frame the route was authored in
/// (husky1_map for patrol_waypoints.yaml).
struct Waypoint
{
  std::string name;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// A loaded route: the frame the coordinates are expressed in, plus the
/// ordered stops.
struct PatrolRoute
{
  std::string frame_id{"map"};
  std::vector<Waypoint> waypoints;
};

/// Parses patrol-waypoints YAML content already read into a string. Kept
/// separate from file I/O so it can be unit tested without touching disk.
/// Throws std::runtime_error if `yaml_text` is malformed or defines no
/// waypoints.
PatrolRoute parse_patrol_route(const std::string & yaml_text);

/// Reads and parses a patrol_waypoints.yaml file (see parse_patrol_route).
/// Throws std::runtime_error if the file cannot be read or parsed.
PatrolRoute load_patrol_route(const std::string & yaml_path);

/// The mission-state categories that matter to the patrol sequencer. The
/// node maps embertrace_interfaces::msg::MissionState::state onto this enum
/// so that this class stays free of any ROS/message dependency.
enum class PatrolTrigger
{
  kPatrol,        ///< MissionState::PATROL
  kReturnToBase,  ///< MissionState::RETURN_TO_BASE
  kHold,          ///< anything else: INVESTIGATE, TRACK, RECOVER, VERIFY,
                  ///< ALERT, PAUSED, IDLE, EMERGENCY_STOP
};

/// Outcome of a navigation goal as reported by nav2_goal_adapter_node on
/// /embertrace/navigation_status. The node maps the adapter's status strings
/// onto this enum so the library stays free of ROS types.
enum class NavResult
{
  kSucceeded,    ///< "SUCCEEDED": Nav2 finished the goal
  kFailed,       ///< "ABORTED" / "REJECTED" / "STALE": the goal was lost; retry
  kOutOfBounds,  ///< "OUT_OF_BOUNDS": retrying can never help; skip the waypoint
};

/// How the patrol reacts when a goal is dropped, rejected or never finishes.
struct RecoveryPolicy
{
  double retry_delay_s{2.0};       ///< wait before re-sending after a failure
  double goal_timeout_s{90.0};     ///< treat a goal that never completes as failed
  int max_goal_attempts{3};        ///< after this many sends, skip the waypoint
  double min_success_age_s{1.0};   ///< ignore a "SUCCEEDED" that arrives this soon
                                   ///< after a new goal was issued (it is stale)
  double base_settle_s{6.0};       ///< RETURN_TO_BASE fallback: if Nav2 never reports
                                   ///< SUCCEEDED, the return still counts as complete once
                                   ///< the rover has stayed inside the arrival tolerance
                                   ///< this long. Long enough for a 180 degree turn onto
                                   ///< the base heading (~5 s at max_vel_theta 0.8 rad/s).
};

/// Pure sequencing/geometry logic for the waypoint patrol: no ROS types, no
/// I/O beyond parse_patrol_route/load_patrol_route above, so it can be
/// exercised directly by unit tests. PatrolManagerNode is a thin ROS wrapper
/// around this class: it feeds mission_state and odometry in, and publishes
/// whatever goal comes back out.
///
/// Design notes:
///  - mission_manager_node republishes MissionState unconditionally every
///    500ms even when nothing changed, so on_mission_state() only reacts to
///    a genuine change of trigger category (a fresh entry into PATROL or
///    RETURN_TO_BASE); repeated identical triggers are ignored rather than
///    re-publishing the same goal or re-picking a waypoint mid-transit.
///  - Entering PATROL always resumes at whichever loaded waypoint is
///    geometrically nearest to the last known pose (falling back to the
///    named base waypoint if no pose has been observed yet). This one rule
///    covers a plain operator PAUSE/RESUME (the rover barely moved, so the
///    nearest waypoint is the one it was already headed to), an operator
///    dismissing a false alarm with START while INVESTIGATE/TRACK/RECOVER/
///    VERIFY was active (the rover may be far from the patrol loop after
///    chasing a lead), and the very first patrol start.
class WaypointPatrol
{
public:
  /// `arrival_tolerance_m` is how close (in metres) the rover must get to
  /// the current goal before the patrol advances to the next waypoint.
  explicit WaypointPatrol(double arrival_tolerance_m = 0.5);

  /// Loads a new route and resets all sequencing state. `base_waypoint_name`
  /// selects which waypoint RETURN_TO_BASE targets and which waypoint is
  /// used as the fallback start point when no pose is known yet; if the name
  /// is not found, waypoint 0 is used instead.
  void set_waypoints(std::vector<Waypoint> waypoints, const std::string & base_waypoint_name);

  bool has_waypoints() const { return !waypoints_.empty(); }

  /// Records the rover's latest known position (e.g. from odometry). If the
  /// patrol is currently active and this pose is within tolerance of the
  /// current goal, advances to the next waypoint (wrapping back to the
  /// front of the list, which counts as completing one loop) and returns
  /// the new goal to publish. Returns std::nullopt otherwise -- including
  /// while paused or returning to base, when no further goal is issued
  /// here.
  std::optional<Waypoint> update_pose(double x, double y);

  /// Feed every mission_state update here. Returns the goal to publish
  /// immediately if this call represents a genuine transition into PATROL
  /// or RETURN_TO_BASE; returns std::nullopt otherwise (including repeated
  /// messages for a state we are already acting on).
  std::optional<Waypoint> on_mission_state(PatrolTrigger trigger);

  void set_recovery_policy(const RecoveryPolicy & policy) { policy_ = policy; }

  /// Tells the patrol what time it is (any monotonic clock, in seconds) so
  /// goal ages can be measured. tick() does this and also runs the watchdog.
  void set_time(double now_s) { now_s_ = now_s; }

  /// Call regularly (e.g. 10 Hz). Re-sends the current goal once a retry is
  /// due or the goal has been outstanding longer than goal_timeout_s. After
  /// max_goal_attempts sends of the same waypoint it is skipped. Returns the
  /// goal to publish, or std::nullopt.
  std::optional<Waypoint> tick(double now_s);

  /// Feed every goal result reported by the Nav2 adapter. A success advances
  /// the patrol (a safety net for when proximity alone never fires); a
  /// failure schedules a retry; out-of-bounds skips the waypoint. Ignored
  /// while paused.
  std::optional<Waypoint> on_nav_result(NavResult result);

  std::size_t current_index() const { return current_index_; }
  std::size_t loops_completed() const { return loops_completed_; }
  bool is_patrolling() const { return mode_ == Mode::kPatrolling; }
  bool is_returning() const { return mode_ == Mode::kReturning; }
  bool has_reached_base() const { return returned_to_base_; }
  bool return_complete() const;
  std::size_t skipped_waypoints() const { return skipped_waypoints_; }
  int goal_attempts() const { return attempts_; }

private:
  enum class Mode
  {
    kInactive,
    kPatrolling,
    kReturning,
  };

  std::size_t nearest_index(double x, double y) const;
  std::optional<Waypoint> issue(bool is_retry);
  std::optional<Waypoint> advance_and_issue();
  std::optional<Waypoint> retry_or_skip();

  double arrival_tolerance_m_;
  std::vector<Waypoint> waypoints_;
  std::size_t base_index_{0};
  std::size_t current_index_{0};
  std::size_t loops_completed_{0};
  Mode mode_{Mode::kInactive};
  bool have_pose_{false};
  bool returned_to_base_{false};
  bool base_goal_succeeded_{false};
  std::optional<double> near_base_since_;
  double pose_x_{0.0};
  double pose_y_{0.0};
  RecoveryPolicy policy_;
  double now_s_{0.0};
  double goal_issued_at_{0.0};
  std::optional<double> retry_due_at_;
  int attempts_{0};
  std::size_t skipped_waypoints_{0};
};

}  // namespace embertrace_core

#endif  // EMBERTRACE_CORE__WAYPOINT_PATROL_HPP_