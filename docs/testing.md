# Testing strategy

## Test layers

### 1. Static repository validation

```bash
./scripts/validate_repo.py
```

This checks package manifests, XML/SDF syntax, launch-file Python syntax, required files and accidental references to the invalid original package name.

### 2. C++ unit tests

```bash
colcon test --packages-select embertrace_core
colcon test-result --verbose
```

The suite covers the plume model, source estimation, patrol geometry, timestamp-aware
perception and image classification. The ROS transport test launches camera,
perception and mission processes and checks multisensor confirmation, vision-only
suppression, stale vision, dropout pause and the E-stop latch.

### 3. Node-level tests

Inspect the stamped visual cue and local fused measurements:

```bash
ros2 topic echo /embertrace/visual_cue
ros2 topic echo /embertrace/environment
```

### 4. Integrated simulation trials

With ROS and the workspace sourced, run the full startup/motion check:

```bash
python3 scripts/ros_demo_smoke.py
```

It waits for fresh vehicle sensors, map/TF and active Nav2 lifecycle nodes, starts
the mission, requires actual odometry movement from both vehicles, and checks
zero command output after E-stop. The launch log is retained in `log/demo-smoke/`.

Run the recorded perception campaign separately:

```bash
python3 scripts/ros_perception_campaign.py
```

It records six stationary near-source scenarios for both vehicles with separate
training and held-out seeds, checks visible visual-decoy evidence and dropout,
then writes threshold-sweep and confusion-matrix/latency CSVs. See
[Perception validation](perception-validation.md) for metric definitions and
[Demo guide](demo-guide.md) for interactive trials. For headless execution, prefix
either command with `LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a`.

Longer operational trials should record each random seed and parameters. Measure:

- time from plume entry to detection;
- time from detection to alert;
- distance from estimated source to configured ground truth;
- path length;
- number of lost-plume recoveries;
- number of collisions or Nav2 failures;
- false alert rate.

## Fair evaluation

The autonomy nodes must not access source coordinates. Ground truth may be used only after a trial to calculate error. At least one person should prepare blind source locations for final testing.

