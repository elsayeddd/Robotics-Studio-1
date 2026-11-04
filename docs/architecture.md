# System architecture

The simulator knows the hidden source solely to produce independent local
measurements. Perception, mission and search consume measurements, never source
truth or tree-state messages. Evaluation reads truth separately to score outcomes.

## Runtime responsibilities

| Component | Responsibility |
| --- | --- |
| Gazebo + bridges | Two vehicle models, RGB/depth/IR, LiDAR/IMU, odometry and clock |
| mission_simulator.py | Shared stamped wind and simulated per-vehicle battery |
| fire_state_manager.py | Healthy/burning/burnt visual, thermal and gas-source consistency through actual Fortress scene services |
| environment_simulator_node (each vehicle) | Noisy local smoke, scalar temperature and independent CO2 field at measured pose; altitude attenuation and controlled dropouts |
| camera_verifier_node (each vehicle) | Connected RGB colour regions with stamped validity and frame persistence |
| thermal_verifier_node (each vehicle) | Radiometric decoding and persistent connected hot regions |
| perception_node (each vehicle) | EMA/hysteresis, gas/heat/image fusion, required-channel validity, time alignment and watchdogs |
| sensor_health_node (each vehicle) | Required bridge-stream acquisition and monotonic receipt health |
| mission_manager_node + MissionLogic | Wind dispatch, selected-local confirmation, mission progression, timeouts, operator service and explicit alerts |
| search_planner_node | Selected-local bounded history; numerical source fit; upwind/crosswind goals with rover map clearance or aerial flight envelope |
| patrol_manager_node + Nav2 adapter | Rover patrol/return, action cancellation, retries, bounded goals and completion/failure reports |
| velocity_guard_node | Sole rover output; selected/health/mission/command gates and manual/autonomous separation |
| drone_scout_node | Aerial ascent, sweep, local investigation, return/landing, obstacle stop and heartbeat/envelope guards |
| heatmap_node | Bounded selected-platform samples for operator awareness |
| operator_dashboard | Map, both-platform telemetry/health, alerts/ACK, commands, hold/release manual input, wind/tree controls |
| mission_event_logger.py | Flushed bounded CSV of transitions and alerts |
| recorder/evaluator/campaign scripts | Independent truth joins, replay/tuning and retained actual trial metrics |

## Mission ownership

Only the mission manager publishes authoritative mission state. Stable low wind
selects the drone; higher safe wind selects the rover; unsafe or missing wind
prevents deployment. Only selected-platform local evidence drives investigation
and confirmation. The unselected platform remains held but its sensors stay visible.
A mid-mission platform/sensor availability change pauses motion; operator RESUME
re-checks inputs and starts a new epoch. No random platform selector is used.

Patrol progresses through investigate, track/recover, verify and alert. Verification
holds motion while gathering separate confirmation samples. An alert automatically
leads to return after a hold. Completed coverage or low battery also returns. Arrival
ends the mission; failed or timed-out navigation pauses for review. E_STOP remains
latched across clock rewind and only RESET clears it.

## Evidence and motion boundaries

Acquisition timestamps reject old/future/repeated evidence. Monotonic receipt age
also expires streams if simulation time freezes. Mission epochs clear old motion
commands, goals and source fits. The action adapter cancels when selected platform,
state, epoch or permission changes. Both command paths need fresh mission and sensor
heartbeats. Manual input has its own timeout and cannot bypass selection or E-stop.

IR peaks never overwrite local scalar temperature or sample pose. A local detection
can be valid even when source fitting is unresolved. Alerts explicitly distinguish
a fitted location from the detection sample. No detector subscribes to TrialTruth.

The drone is an ideal velocity-controlled simulation proxy. Its bounded flight
layer and LiDAR stop do not represent a physical flight controller or rotor model.
The smoke/CO2 fields are educational mathematical models, not fluid dynamics.

See [interface-contract.md](interface-contract.md), [completion-handover.md](completion-handover.md)
and [perception-vm-startup.md](perception-vm-startup.md) for interfaces, acceptance
status and reproducible commands.
