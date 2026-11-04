# Final system acceptance

The implementation phase includes navigation, waypoint patrol, source estimation,
RGB/IR/CO2 perception, operator controls and both standalone vehicle missions.
The old node scaffolds are built and connected. The outstanding work is determined
by measured acceptance results, not by the existence of source files.

| Requirement | Implementation | Acceptance evidence |
| --- | --- | --- |
| Clean target-platform build | Five Humble/Fortress packages; bounded build concurrency | Full workspace build |
| Navigation execution | Namespaced action client, TF conversion, bounds, cancellation | Mock action-server regressions and actual rover motion |
| Ground patrol | YAML route, proximity/action feedback, retries and return | Two complete measured loops, pause/resume and return |
| Aerial mission | Independent sweep, local investigation, source fit and return | Standalone flight, investigation, alert, return and stop |
| Automatic deployment | Stable wind selects one available vehicle and its local evidence | Selected vehicle moves and returns; the unselected vehicle remains held |
| Search/source localisation | Bounded local history, fitted covariance, free-space ground goals | Ten seeded autonomous trials; median resolved-source error below 2 m |
| Perception | Timestamp-aware smoke/heat/CO2 fusion and native IR/RGB evidence | Recorded confusion matrix, availability and latency for at least three scenarios |
| Mission lifecycle | Search timeout, verification timeout, dropout pause, latched stop | ROS transport state-transition tests and live mission controls |
| Operator interface | Freshness, IR/CO2 values, controls and explicit E-stop release | Qt startup and operator-facing mission checks |
| Wind policy | Configured initial dispatch and live sensor-based motion gating | Boundary and missing-status tests for both vehicles |

[Integration review](integration-review.md) records the exact branch heads, issues
found and completed checks. [Testing](testing.md) contains reproducible commands.
Do not mark physical sensor calibration, flight dynamics or real-fire detection
complete on the strength of these mathematical-field simulation trials.

## Future extensions

Terrain slopes, real camera classifiers, physical sensor drivers, flight-autopilot
integration, multiple fire sources, physical battery modelling and measured wind
remain extensions. Simulated battery depletion and live wind parameter changes
are already integrated.
