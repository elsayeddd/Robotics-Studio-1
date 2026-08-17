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

The initial unit test checks physical relationships in the plume model. Add tests for filters, mission transitions and search geometry as those components mature.

### 3. Node-level tests

Publish controlled messages and inspect outputs. Example:

```bash
ros2 topic pub --once /embertrace/visual_fire_cue std_msgs/msg/Float32 "{data: 0.8}"
ros2 topic echo /embertrace/environment
```

### 4. Integrated simulation trials

Record each trial's random seed and parameters. Measure:

- time from plume entry to detection;
- time from detection to alert;
- distance from estimated source to configured ground truth;
- path length;
- number of lost-plume recoveries;
- number of collisions or Nav2 failures;
- false alert rate.

## Fair evaluation

The autonomy nodes must not access source coordinates. Ground truth may be used only after a trial to calculate error. At least one person should prepare blind source locations for final testing.

