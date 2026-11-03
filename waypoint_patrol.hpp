#ifndef EMBERTRACE_CORE__WAYPOINT_PATROL_HPP_
#define EMBERTRACE_CORE__WAYPOINT_PATROL_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace embertrace_core {

/// A single patrol stop, expressed in the frame the route was authored in
/// (husky1_map for patrol_waypoints.yaml).
struct Waypoint {
  std::string name;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// A loaded route: the frame the coordinates are expressed in, plus the
/// ordered stops.
struct PatrolRoute {
  std::string frame_id{"map"};
  std::vector<Waypoint> waypoints;
};

/// Parses patrol-waypoints YAML content already read into a string. Kept
/// separate from file I/O so it can be unit tested without touching disk.
/// Throws std::runtime_error if `yaml_text` is malformed or defines no
/// waypoints.
PatrolRoute parse_patrol_route(const std::string& yaml_text);

/// Reads and parses a patrol_waypoints.yaml file (see parse_patrol_route).
/// Throws std::runtime_error if the file cannot be read or parsed.
PatrolRoute load_patrol_route(const std::string& yaml_path);

/// The mission-state categories that matter to the patrol sequencer. The
/// node maps embertrace_interfaces::msg::MissionState::state onto this enum
/// so that this class stays free of any ROS/message dependency.
enum class PatrolTrigger {
  kPatrol,        ///< MissionState::PATROL
  kReturnToBase,  ///< MissionState::RETURN_TO_BASE
  kHold,          ///< anything else: INVESTIGATE, TRACK, RECOVER, VERIFY,
                  ///< ALERT, PAUSED, IDLE, EMERGENCY_STOP
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
class WaypointPatrol {
 public:
  /// `arrival_tolerance_m` is how close (in metres) the rover must get to
  /// the current goal before the patrol advances to the next waypoint.
  explicit WaypointPatrol(double arrival_tolerance_m = 0.5);

  /// Loads a new route and resets all sequencing state. `base_waypoint_name`
  /// selects which waypoint RETURN_TO_BASE targets and which waypoint is
  /// used as the fallback start point when no pose is known yet; if the name
  /// is not found, waypoint 0 is used instead.
  void set_waypoints(std::vector<Waypoint> waypoints, const std::string& base_waypoint_name);

  /// Use the measured launch point for the base waypoint, including custom spawns.
  void set_home_pose(double x, double y);

  bool has_waypoints() const {
    return !waypoints_.empty();
  }

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

  std::size_t current_index() const {
    return current_index_;
  }
  std::size_t loops_completed() const {
    return loops_completed_;
  }
  bool is_patrolling() const {
    return mode_ == Mode::kPatrolling;
  }
  bool is_returning() const {
    return mode_ == Mode::kReturning;
  }
  bool has_reached_base() const {
    return returned_to_base_;
  }

 private:
  enum class Mode {
    kInactive,
    kPatrolling,
    kReturning,
  };

  std::size_t nearest_index(double x, double y) const;

  double arrival_tolerance_m_;
  std::vector<Waypoint> waypoints_;
  std::size_t base_index_{0};
  std::size_t current_index_{0};
  std::size_t loops_completed_{0};
  Mode mode_{Mode::kInactive};
  bool have_pose_{false};
  bool returned_to_base_{false};
  double pose_x_{0.0};
  double pose_y_{0.0};
};

}  // namespace embertrace_core

#endif  // EMBERTRACE_CORE__WAYPOINT_PATROL_HPP_
