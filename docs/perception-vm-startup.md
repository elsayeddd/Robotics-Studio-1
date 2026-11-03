# Linux VM, GitHub and perception testing — step by step

This guide accompanies the October 20 code update based on the newly uploaded main
branch ZIP. Run the commands **inside the Linux VM**, except for downloading/transferring
the patch ZIP. Commands assume Bash and a repository at `~/projects/Robotics-Studio-1`.
Replace GitHub OWNER/email placeholders with your own values. No repository URL was
included in the upload, so none is assumed here.

## 1. Prepare the Linux VM

Use **Ubuntu Desktop 22.04 LTS (64-bit), ROS 2 Humble and Gazebo Fortress** for this
repository. Do not mix Ubuntu 24.04/Jazzy/Gazebo Harmonic instructions into this setup.
The official Humble binary packages target Ubuntu 22.04:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Practical starting resources: 4 CPU cores, 8 GB RAM and 60 GB disk, if your host can
spare them. Use NAT networking for downloads/GitHub, and install your VM platform's
supported guest display tools. Enable supported graphics acceleration if available.
A VM rendering issue is not necessarily a perception failure. Begin with the small
world, no Nav2, and one robot, then enable the drone/full stack.

Open a terminal in Ubuntu:

```bash
lsb_release -ds
sudo apt update
sudo apt install -y git openssh-client curl build-essential unzip python3-pip python3-yaml
```

Expected OS: Ubuntu 22.04. Keep the repository in the VM's Linux home filesystem,
not a Windows shared folder, to avoid symlink/permissions issues during colcon builds.

## 2. Link GitHub to the VM

First ensure your GitHub account has access to the group's repository. A private
repository invitation must be accepted before cloning.

```bash
git config --global user.name "Vasili Giaras"
git config --global user.email "YOUR_GITHUB_EMAIL"
ls -al ~/.ssh
```

If a key already exists and is registered to your GitHub account, reuse it. Otherwise
create a dedicated key (do not overwrite an existing key):

```bash
mkdir -p ~/.ssh
chmod 700 ~/.ssh
ssh-keygen -t ed25519 -C "YOUR_GITHUB_EMAIL" -f ~/.ssh/id_ed25519_embertrace
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519_embertrace
cat ~/.ssh/id_ed25519_embertrace.pub
```

Choose a passphrase when prompted. Copy the **public** `.pub` line into GitHub:
Settings -> SSH and GPG keys -> New SSH key -> Authentication key. Name it
`Ubuntu robotics VM`. Keep the private key inside the VM.

Because this uses a dedicated filename, open `~/.ssh/config` in a text editor and add
or update the existing github.com block (do not duplicate an existing Host block):

```text
Host github.com
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_embertrace
    IdentitiesOnly yes
```

Then test:

```bash
ssh -T git@github.com
```

The expected message says you successfully authenticated and that GitHub provides
no shell access; an exit code of 1 can be normal for this test. Check a new host-key
fingerprint against GitHub's published fingerprint before accepting it.

Official SSH instructions:
https://docs.github.com/en/authentication/connecting-to-github-with-ssh

## 3. Clone the group project and make a perception branch

On the group's GitHub page choose Code -> SSH and copy its URL. Substitute that exact
URL for the example below:

```bash
mkdir -p ~/projects
git clone git@github.com:OWNER/Robotics-Studio-1.git ~/projects/Robotics-Studio-1
cd ~/projects/Robotics-Studio-1
git remote -v
git switch main
git pull --ff-only origin main
git switch -c feature/perception-ir-co2
```

If you already have a clone, begin with `git status` and save unrelated uncommitted
work on its own branch before updating main. Do not run `git init` inside an existing
clone and do not replace its `.git` directory with a downloaded ZIP.

## 4. Apply this update

Transfer/download `EmberTrace-Perception-Oct20-Changes.zip` into the VM and extract it
into `~/Downloads/embertrace-perception-update`. The archive contains:

- `files/`: repository-relative versions of added/modified files.
- `perception-oct20.patch`: diff against the supplied main ZIP commit.
- `CHANGED_FILES.txt`: exact list.
- `START_HERE.md`: handover summary.

From your cloned repository:

```bash
cd ~/projects/Robotics-Studio-1
git apply --check ~/Downloads/embertrace-perception-update/perception-oct20.patch
git apply ~/Downloads/embertrace-perception-update/perception-oct20.patch
git status
git diff --stat
git diff --check
```

If `git apply --check` fails, your clone may contain newer group edits. Compare and
merge the files listed in CHANGED_FILES manually, especially launch/configuration,
mission_manager_node.cpp, interfaces and CMakeLists. Do not force-overwrite teammates'
changes. Use either patch application or file copying, not both. The full updated
repository ZIP is an alternative standalone snapshot for inspecting/testing the exact
provided version; it is not your authenticated Git clone.

## 5. Install ROS 2 Humble and project dependencies

If `/opt/ros/humble/setup.bash` exists, skip the ROS installation and run the project
dependency script below. Otherwise follow the **official Humble Ubuntu deb guide**
through locale, Universe repository, ROS apt-source configuration and system updates:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

After configuring the ROS apt repository in that guide:

```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools
source /opt/ros/humble/setup.bash
cd ~/projects/Robotics-Studio-1
bash scripts/install_dependencies.sh
sudo apt install -y ros-humble-rqt-image-view xvfb libgl1-mesa-dri
```

The project script installs ros-ign compatibility packages, Nav2, SLAM Toolbox,
robot_localization, xacro, teleop, RViz, Qt, colcon and rosdep dependencies. Keep
Fortress; the project deliberately uses the `ign gazebo` / `ros_ign_*` interfaces.

Check:

```bash
printenv ROS_DISTRO
ros2 --help
ign gazebo --versions
```

Expected ROS distribution: `humble`; Gazebo Fortress is the Gazebo 6 generation.
`ros2 --version` is not a required command. If `ign gazebo` is missing, inspect the
project dependency installation output before proceeding.

## 6. Build all packages

The update changes message definitions. Build the entire workspace so every C++
and Python consumer agrees about the new fields.

```bash
cd ~/projects/Robotics-Studio-1
source /opt/ros/humble/setup.bash
python3 scripts/validate_repo.py
bash scripts/validate_perception.sh
colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 interface show embertrace_interfaces/msg/ThermalCue
ros2 interface show embertrace_interfaces/msg/EnvironmentalReading
```

Use two parallel workers initially to limit VM RAM use. If interfaces still show the
old fields, open a fresh shell, source Humble then this workspace, and confirm the
build succeeded. Do not source a second old EmberTrace workspace into that shell.

## 7. Run ROS tests before Gazebo

Stop other EmberTrace launches. Run transport tests on an unused DDS domain so their
synthetic publishers do not collide with a real demonstration:

```bash
cd ~/projects/Robotics-Studio-1
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=73 colcon test --packages-select embertrace_core \
  --event-handlers console_direct+
colcon test-result --verbose
```

This includes the pure regression executable and actual ROS perception/mission test.
You can run the ROS test directly to see its logs:

```bash
ROS_DOMAIN_ID=73 python3 scripts/ros_perception_integration_test.py
```

Expected: no failed tests. The integration test sends known RGB/IR/gas data, checks
vision-only rejection, required-IR loss, fire confirmation, dropout pause and E-stop
latching. It does not launch Gazebo or prove renderer/bridge functionality.

## 8. Start the simulation with no autonomous navigation

Every new terminal must use the same working directory, sources and ROS domain.
Use these lines at the top of **each simulation terminal**:

```bash
cd ~/projects/Robotics-Studio-1
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=26
```

Terminal 1 — start the smaller baseline with both platforms:

```bash
ros2 launch embertrace_bringup embertrace_demo.launch.py \
  use_nav2:=false use_drone:=true use_thermal:=true \
  use_rviz:=true use_dashboard:=true scenario:=no_fire
```

This single launch starts Gazebo, both robot models/bridges, clock, local environmental
simulators, RGB and IR verifiers, perception, health, mission manager, search/patrol
nodes, velocity guard, RViz and dashboard. It does **not** start Nav2 in this command.
Do not separately launch Gazebo or duplicate core nodes. The robots initially wait
for START. Some RViz map displays will be absent without SLAM/Nav2; that is expected.

If your VM struggles, first repeat with `use_drone:=false use_rviz:=false`; that is a
setup diagnostic, not evidence of completing the dual-platform demonstration.

Terminal 2 — verify sensors and perception:

```bash
ros2 node list
ros2 topic hz /husky1/odometry
ros2 topic hz /husky1/thermal/image
ros2 topic hz /drone1/thermal/image
```

Each `hz` command keeps running; press Ctrl+C before the next one. IR aims for 5 Hz;
a VM may run slower. Next inspect:

```bash
ros2 topic echo /embertrace/raw_environment --once
ros2 topic echo /embertrace/thermal_cue --once
ros2 topic echo /embertrace/perception_status --once
ros2 topic echo /drone1/perception_status --once
ros2 topic echo /diagnostics --once
ros2 run embertrace_core demo_preflight.py --no-nav2 --timeout 60
```

In full mode, look for valid CO2, fresh IR, fresh environmental data, and ready sensor
streams on both robots. A no-fire scene should not produce a confirmed fire. Ambient
CO2 should be near 420 ppm with noise; IR peaks should be below the hotspot threshold.
The dashboard shows smoke, local temperature, wind, confidence, CO2 and IR peak.

To inspect actual thermal image encoding:

```bash
ros2 topic echo /husky1/thermal/image --field encoding --once
ros2 run rqt_image_view rqt_image_view
```

Choose `/husky1/thermal/image` or `/drone1/thermal/image`. The viewer may normalise
brightness: brightness alone is not Celsius. Use `thermal_cue.peak_temperature_c`
for the decoded radiometric value. Expected integer image encoding is mono16/16UC1.

If no IR messages arrive, inspect Terminal 1 for thermal plugin/bridge errors and:

```bash
ign topic -l
ros2 topic info /husky1/thermal/image --verbose
```

Verify the Gazebo topic `/model/husky1/thermal/image`, plugin names, Sensors/Ogre2,
and bridge type. Do not treat the ordinary RGB camera as an IR substitute. Update
Fortress dependencies through apt if your image bridge cannot convert L_INT16.

## 9. Test a visible nearby fire before testing navigation

Stop Terminal 1 with Ctrl+C and wait until that launch closes. Restart it with:

```bash
ros2 launch embertrace_bringup embertrace_demo.launch.py \
  use_nav2:=false use_drone:=true use_thermal:=true \
  use_rviz:=true use_dashboard:=true scenario:=fire \
  rover_x:=10.5 rover_y:=5.5 drone_x:=12.0 drone_y:=7.0
```

These spawn positions are an explicit test fixture near the world fire, not source
coordinates supplied to perception/autonomy. They isolate sensor testing from search.
The rover faces approximately northeast and the drone camera points down.

After preflight passes, use the dashboard START button or Terminal 2:

```bash
ros2 service call /embertrace/operator_command \
  embertrace_interfaces/srv/OperatorCommand "{command: 'START'}"
ros2 topic echo /embertrace/environment --once
ros2 topic echo /embertrace/fire_alert
```

Expected after warmup/persistence: elevated CO2 and smoke, a valid IR hotspot, fused
confidence, and progression through investigation/verification to ALERT. Source
location can be unresolved in a stationary test; that is separate from fire detection.
If not, inspect every input and status flag before tuning thresholds. Ctrl+C stops
topic echo, not the running simulation.

## 10. Record the required scenario evidence

Run this in a third sourced simulation terminal while a trial is running:

```bash
ros2 run embertrace_core record_perception.py \
  --robot husky1 --trial heldout_fire_142 \
  --output log/manual/heldout_fire_142.csv --duration 30
```

Run another recorder for `--robot drone1` with a different output filename. Use fresh
launches with `scenario:=no_fire`, `visual_decoy`, `dust`, `hot_surface`, `fire` and
`dropout` as appropriate. At least three scenarios are required; test all six if
possible. Keep failed trials. Do not label training and held-out runs identically.

Optional raw bags preserve images for checking/replaying the IR/RGB methods:

```bash
ros2 bag record -o log/bag_fire_142 \
  /clock /husky1/odometry /husky1/camera/image /husky1/thermal/image \
  /embertrace/raw_environment /embertrace/environment \
  /embertrace/visual_cue /embertrace/thermal_cue /embertrace/perception_status \
  /embertrace/validation/truth /embertrace/mission_state /embertrace/fire_alert
```

The truth topic is evaluation-only and must not feed perception or decision nodes.
Record the independent geometric exposure rule with the results. Raw bag playback
alone is not a complete isolated detector replay setup; avoid duplicate live clocks
and sensor publishers when designing a replay test.

Evaluate a recorded CSV:

```bash
ros2 run embertrace_core evaluate_perception log/manual/heldout_fire_142.csv
```

For an automated recorded six-scenario campaign, **stop the manual simulation first**:

```bash
ROS_DOMAIN_ID=74 python3 scripts/ros_perception_campaign.py \
  --output log/gazebo-perception-final --duration 12
```

It starts and stops its own Gazebo launches and records both platforms for training
and held-out seeds. Use a new output directory when repeating it. It checks explicit
CO2 and IR presence and produces confusion matrices, availability, missed events and
latency. If your graphics setup needs a virtual X display, use:

```bash
ROS_DOMAIN_ID=74 LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a \
  python3 scripts/ros_perception_campaign.py \
  --output log/gazebo-perception-software --duration 12
```

Software rendering can be slow. A campaign timeout or poor visibility is a failed
trial to diagnose, not a reason to silently replace measurements with synthetic data.

## 11. Test dropout and recovery manually

Start a fresh launch near the fire as above with:

```text
scenario:=dropout dropout_start_s:=10.0 dropout_duration_s:=5.0
```

Watch perception_status and sensors_ready. Environmental loss should become unavailable
within the configured 2 s timeout, and active mission motion should pause. Restored
samples require warmup again. Use RESUME only after sensors are healthy. Historical
ALERT messages remain historical events; they are not erased by sensor loss.
Also test IR-only interruption, RGB-only interruption, repeated timestamps and pausing
Gazebo's clock. RGB loss should remove visual contribution; required IR loss suppresses
new fire confirmation. Steady watchdogs detect missing data even when simulation time
is paused. These checks are important in a VM.

## 12. Enable the full autonomous stack and map/TF trial

After sensor checks, stop the earlier launch and run:

```bash
ros2 launch embertrace_bringup embertrace_demo.launch.py \
  use_nav2:=true use_drone:=true use_thermal:=true \
  use_rviz:=true use_dashboard:=true scenario:=fire
```

This restores normal base spawn positions. Terminal 2:

```bash
ros2 run embertrace_core demo_preflight.py --timeout 90
ros2 run tf2_ros tf2_echo husky1_map husky1_odom \
  --ros-args -r /tf:=/husky1/tf -r /tf_static:=/husky1/tf_static
```

Stop tf2_echo with Ctrl+C. Preflight must report map, TF, active Nav2 and a valid first
patrol path before START. A blocked/unknown path is a navigation/map issue to resolve
with the integration team. Once ready:

```bash
ros2 service call /embertrace/operator_command \
  embertrace_interfaces/srv/OperatorCommand "{command: 'START'}"
```

Observe both vehicles, sensor detections, mission transitions, rover investigation,
location estimation and operator alert. Do not start `platform_select.launch.py` on
top of this launch: it can create another goal adapter and currently uses a random
platform choice rather than measured wind. The new wind status output needs the
team's deployment-policy integration before C1 R13 can be claimed complete.

If teleoperation is needed with `use_nav2:=false`, keep the safety guard in the path:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/husky1/autonomy_cmd_vel
```

START and ready sensors are needed for the guard to permit commands. Do not publish
directly to `/husky1/cmd_vel` while demonstrating PAUSE/E-stop/health protection.

Run at least ten autonomous repetitions with distinct `random_seed` values and
record seed, sensor performance, failures, source error and mission behaviour with
Adam. The perception campaign's static fixtures do not replace these C3 runs.
Only after the small world works, add:

```text
world_file:=embertrace_forest_dense.sdf
```

Re-check visibility, false positives, obstacle navigation and VM performance there.

## 13. Commit and share the reviewed change

After the tests you can actually run:

```bash
git status
git diff --check
```

Stage the files listed in CHANGED_FILES after reviewing them (use VS Code Source
Control or `git add` with explicit filenames). Exclude build/, install/, log/,
recordings and private keys. Then:

```bash
git diff --cached --stat
git commit -m "Complete IR and CO2 perception integration and validation tools"
git push -u origin feature/perception-ir-co2
```

Open a pull request into main and report the tests **actually run**, outstanding VM
checks and AI assistance. Do not call the sensor system validated from synthetic
results alone. Your teammates should review shared interfaces, launch/configuration,
mission integration and UI changes before merging.
