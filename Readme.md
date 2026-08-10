# Dual-Arm Robot Teleoperation via Webcam Hand Tracking

Control two 6-DOF robotic arms in real time using a standard webcam and
your bare hands. Your left and right hands drive the left and right arms
respectively; making a fist closes the matching gripper.

Built on ROS 2 Jazzy, MoveIt 2, ros2_control and MediaPipe.

---

## 1. Architecture

Each arm is planned by its **own `move_group` instance, in its own ROS
namespace**. The arms are fully independent: they plan in parallel, at
their own rates, and a failure on one does not affect the other.

```
                         GLOBAL NAMESPACE
  ┌───────────────────────────────────────────────────────────────┐
  │  robot_state_publisher    full 12-DOF URDF  →  /tf            │
  │  static_transform_pub     world → base_link                   │
  │  ros2_control_node        ONE controller_manager @ 100 Hz     │
  │    ├ manipulator_robot_one_controller   (JointTrajectory)     │
  │    ├ manipulator_robot_two_controller   (JointTrajectory)     │
  │    ├ gripper_robot_one_controller       (GripperAction)       │
  │    ├ gripper_robot_two_controller       (GripperAction)       │
  │    └ joint_state_broadcaster            →  /joint_states      │
  │  hand_tracking_node       webcam → /left_hand_state,          │
  │                                     /right_hand_state         │
  │  hand_to_robot_controller two MoveGroup clients, two gates    │
  │  rviz2                                                        │
  └───────────────────────────────────────────────────────────────┘
        │                                        │
   ┌────┴──────────────┐              ┌──────────┴─────────┐
   │   /robot_one      │              │   /robot_two       │
   │   move_group      │              │   move_group       │
   │   plans           │              │   plans            │
   │   manipulator_    │              │   manipulator_     │
   │   robot_one       │              │   robot_two        │
   └───────────────────┘              └────────────────────┘
```

Both `move_group` instances deliberately load the **full dual-arm URDF
and SRDF** and monitor the global `/joint_states`, so each one still sees
the other arm in its planning scene. Only `move_group` is namespaced -
see §7 for why the controllers are not.

### Data flow for one hand

```
webcam ──▶ MediaPipe (21 landmarks)
       ──▶ HandState @ 10 Hz on /left_hand_state
       ──▶ TargetMapper    relative offset from reference pose,
                           clamped to a ±8 cm box
       ──▶ significance + cooldown gates
       ──▶ ArmGate         one goal in flight, newest target wins
       ──▶ /robot_one/move_action
       ──▶ move_group      OMPL / RRTConnect, plan + execute
       ──▶ manipulator_robot_one_controller
       ──▶ mock hardware ──▶ /joint_states ──▶ /tf ──▶ (loop closes)
```

Grippers bypass MoveIt entirely: a fist sends a `GripperCommand` goal
straight to `/gripper_robot_one_controller/gripper_cmd`.

---

## 2. Packages

| Package | Role |
| ------- | ---- |
| `hand_tracking` | Webcam capture, MediaPipe detection, gesture and relative-pose logic |
| `robot_interfaces` | Custom `HandState` message |
| `robot_imitation` | Hand→target mapping, per-arm flow control, MoveIt and gripper clients |
| `robot_moveit` | MoveIt configuration: URDF/SRDF, kinematics, controllers, launch |
| `robot_main` | Robot description, top-level launch, RViz config |

### `robot_imitation` module layout

Everything is split so the non-ROS logic is unit-testable on its own.

| Module | Contains | ROS imports |
| ------ | -------- | ----------- |
| `params.py` | `ImitationParams` — every tunable, plus frame/action name helpers | no |
| `pose_mapping.py` | `TargetMapper`, `movement_is_significant` | no |
| `arm_gate.py` | `ArmGate` — per-arm "one goal in flight, newest wins" | no |
| `ros_conversions.py` | numpy ↔ `geometry_msgs` helpers | msgs only |
| `goal_constraints.py` | Builds MoveIt `Constraints` for pose goals | yes |
| `planning_client.py` | `ArmPlanner` (one per arm), `PlannerFleet` | yes |
| `gripper_client.py` | `GripperFleet` — non-blocking gripper actions | yes |
| `ee_tracker.py` | `EndEffectorTracker` — current EE pose from TF | yes |
| `imitation_node.py` | `HandToRobotController` — thin wiring layer | yes |

---

## 3. The robot

Two identical 6-DOF arms with parallel grippers, mounted on a shared
`base_link`:

| | Arm 1 | Arm 2 |
| --- | --- | --- |
| Driven by | left hand | right hand |
| Base origin | `0 -0.75 0.1` | `0 0.75 0.1` |
| Base frame | `base_link_robot_1` | `base_link_robot_2` |
| Tip link | `link6_robot_1` | `link6_robot_2` |
| Arm joints | `joint1_robot_1` … `joint6_robot_1` | `joint1_robot_2` … `joint6_robot_2` |
| Gripper joint (active) | `left_gripper_joint_robot_1` | `left_gripper_joint_robot_2` |
| Gripper joint (mimic) | `right_gripper_joint_robot_1` | `right_gripper_joint_robot_2` |
| SRDF planning group | `manipulator_robot_one` | `manipulator_robot_two` |
| SRDF gripper group | `gripper_robot_one` | `gripper_robot_two` |

The bases sit **1.5 m apart** on the y axis. The SRDF virtual joint
`base_joint` fixes `world` → `base_link`.

Both arms home to `(0, 0.3, 1.1, 0, -0.5, 0)`, putting each end effector
near `(0.45, 0, 0.42)` in its own base frame; chosen to leave reach
margin in every direction so the whole teleop box is plannable.

The right finger of each gripper is a **passive mimic joint**: the mimic
relationship lives in the URDF `<mimic>` tag, and `ros2_control` on Jazzy
requires activated mimic joints to expose state interfaces only, no
command interface. Mock hardware fills their position from the mimicked
joint, which is what keeps the finger links valid in TF and RViz.

Hardware is `mock_components/GenericSystem` (`FakeSystem`). Kinematics is
KDL per manipulator group.

> The SRDF still defines an unused `both_manipulators` group spanning all
> 12 joints. It is harmless and is kept so a single-`move_group` layout
> can be restored by reverting the launch files alone.

---

## 4. Installation

### Docker (recommended)

```bash
git clone https://github.com/EbyGunner/camera_robot_teleoperation.git
cd camera_robot_teleoperation
xhost +local:                  # allow the container to open RViz
docker compose up --build      # first build takes a while; cached afterwards
```

RViz opens with both arms. `Ctrl-C` stops everything; `xhost -local:`
revokes display access again. Camera indices, black RViz windows, NVIDIA
GPUs and a dev-mode source mount are covered in `DOCKER.md`.

### Native

Requires ROS 2 Jazzy, MoveIt 2, ros2_control/ros2_controllers, and
Python 3 with `mediapipe`.

```bash
mkdir -p ~/teleop_ws/src && cd ~/teleop_ws/src
git clone https://github.com/EbyGunner/camera_robot_teleoperation.git
cd ~/teleop_ws
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y \
  --skip-keys "warehouse_ros_mongo opencv"
sudo apt install ros-jazzy-moveit ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-gripper-controllers
```

MediaPipe (pick one):

```bash
pip3 install --break-system-packages mediapipe          # simplest
# or keep it out of the system Python:
cd src/camera_robot_teleoperation/hand_tracking
mkdir -p external_libraries && pip3 install mediapipe --target=external_libraries
# or point the loader anywhere:
export HAND_TRACKING_MEDIAPIPE_DIR=/path/to/site-packages
```

Build:

```bash
cd ~/teleop_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 5. Running

```bash
ros2 launch robot_main robot_main.launch.py
```

One command brings up the description, controllers, both `move_group`
instances, the hand tracker, the teleop node, and RViz.

Startup order: `robot_state_publisher` and `ros2_control_node` first,
controller spawners next, both `move_group` instances at **+3 s** (so
`/joint_states` exists before MoveIt looks for it), RViz at **+6 s**.

### Controls

| You do | Robot does |
| ------ | ---------- |
| Show a hand to the camera | That arm captures its reference pose (`reference pose set` in the log) |
| Move the hand left/right | End effector moves along the arm's y axis |
| Move the hand up/down | End effector moves along z |
| Move both hands | Both arms plan and move independently, in parallel |
| Make a fist / open the hand | Gripper closes / opens |
| Hide the hand | That arm's reference resets; it re-captures when the hand returns |

Motion is clamped to a ±8 cm box around each arm's reference pose. Depth
control (hand toward/away from camera) exists but ships disabled,
monocular depth from apparent hand size is too noisy (`scale_x = 0.0`).

### Launch arguments

| Argument | Description | Default |
| -------- | ----------- | ------- |
| `robot1_namespace` | Namespace of the `move_group` planning for arm 1 | `robot_one` |
| `robot2_namespace` | Namespace of the `move_group` planning for arm 2 | `robot_two` |
| `planning_time` | Allowed planning time (s) | `5.0` |
| `velocity_scaling` | Velocity scaling factor (0.0–1.0) | `0.5` |
| `acceleration_scaling` | Acceleration scaling factor (0.0–1.0) | `0.5` |
| `workspace_extent` | Half-size (m) of the teleop box | `0.08` |
| `plan_cooldown` | Minimum seconds between re-plans for one arm | `1.0` |
| `use_orientation_constraint` | Set `false` when using position-only IK | `true` |
| `diagnostic_mode` | `''` = follow hands; `exact_current_pose` / `fixed_offset` for planner debugging | `''` |
| `debug` | Verbose logging | `false` |

`robot1_namespace` and `robot2_namespace` are passed to **both** the
`move_group` instances and the teleop node, so changing one argument
keeps them in agreement.

---

## 6. Configuration reference

Everything below is a ROS parameter on the `hand_to_robot_controller`
node, declared in `robot_imitation/robot_imitation/params.py`. Only a
subset is exposed as a launch argument; the rest are set with
`--ros-args -p name:=value` or by editing the defaults.

### Identity and namespaces

| Parameter | Default | Meaning |
| --------- | ------- | ------- |
| `robot1_name` / `robot2_name` | `robot_one` / `robot_two` | Internal arm keys; also select the SRDF group `manipulator_<name>` |
| `robot1_namespace` / `robot2_namespace` | `robot_one` / `robot_two` | `move_group` namespace → `/<ns>/move_action` |
| `controller_namespace` | `''` | Namespace of the ros2_control controllers. Empty = global, which is how the stack ships |

### Hand → target mapping

| Parameter | Default | Meaning |
| --------- | ------- | ------- |
| `scale_x` | `0.0` | hand depth → robot x. Zero disables depth control |
| `scale_y` | `-0.30` | hand horizontal → robot y (m per unit) |
| `scale_z` | `-0.30` | hand vertical → robot z |
| `workspace_extent` | `0.08` | Half-size (m) of the clamp box around the reference pose |
| `max_missing_frames` | `5` | Frames without a hand before the reference resets |

Flip a sign if a direction feels inverted on your setup.

### Re-planning gates

| Parameter | Default | Meaning |
| --------- | ------- | ------- |
| `position_threshold` | `0.02` | m of movement before a re-plan is worth it |
| `orientation_threshold` | `0.2` | rad, same idea |
| `plan_cooldown` | `1.0` | s between re-plans, **per arm** |

`plan_cooldown` is now the only thing limiting how often an arm
re-plans — under the old single-`move_group` layout it also had to absorb
the other arm's planning time. Try `0.4`–`0.6` for snappier tracking once
the setup is stable.

### MoveIt request

| Parameter | Default | Meaning |
| --------- | ------- | ------- |
| `planning_time` | `5.0` | Allowed planning time (s) |
| `num_planning_attempts` | `4` | Attempts per goal |
| `velocity_scaling` / `acceleration_scaling` | `0.5` | Trajectory scaling |
| `planner_id` | `''` | Empty = pipeline default (RRTConnect) |
| `position_tolerance` | `0.02` | Goal sphere radius (m) |
| `use_orientation_constraint` | `true` | Set `false` for position-only IK |
| `orientation_tolerance` | `0.5` | rad per axis. `1.0` wanders too far |
| `goal_timeout` | `20.0` | s before the watchdog cancels a stuck goal |

### Gripper

| Parameter | Default |
| --------- | ------- |
| `gripper_open_position` | `0.0` |
| `gripper_closed_position` | `0.06` |
| `gripper_effort` | `0.5` |

---

## 7. Design notes

### Why one `move_group` per arm

A single `move_group` executes **one motion goal at a time**. With both
arms sharing one instance, per-arm goals could only ever alternate, so
simultaneous motion required merging both arms' targets into one goal for
a combined 12-DOF planning group, with a merge window during which a
single moving hand waited on the chance that the other hand was about to
produce a target.

Splitting into two namespaced instances removes all of that:

- **No merge window.** One moving hand no longer pays latency waiting for
  the other.
- **No shared failure.** A `NO_IK_SOLUTION` on one arm used to abort the
  combined goal and stop *both* arms. Now it stops one.
- **No alternating goals.** Each arm re-plans on its own cooldown.

### Why the controllers stay global

The mock hardware covers all 12 joints in a single `<ros2_control>`
block. Two controller managers would each instantiate that whole system
and produce two competing `/joint_states` streams, which would then need
splitting the URDF and merging the streams back; complexity for no
benefit. So one `controller_manager` owns all four controllers, and each
namespaced `move_group` reaches out to the ones it owns.

Two mechanisms keep that safe:

1. **Per-arm controller configs.** `moveit_controllers_robot_one.yaml`
   and `moveit_controllers_robot_two.yaml` each list only that arm's
   trajectory and gripper controller. Without this, both instances would
   accept trajectories for both arms and their
   `TrajectoryExecutionManager`s would race. Confirm it worked by looking
   for `Returned 2 controllers in list` in each instance's startup log.

2. **`moveit_manage_controllers: false`.** Two instances sharing one
   controller manager must never activate or deactivate controllers, or
   they will switch off each other's. Confirmed by
   `Trajectory execution is not managing controllers` at startup.


### Remapping

`move_group_ns.launch.py` applies three groups of remap rules.

**Controller actions.** `MoveItSimpleControllerManager` builds action
names as `<controller_name>/<action_ns>` *relative to `move_group`'s
namespace*, so inside `/robot_one` it would look for
`/robot_one/manipulator_robot_one_controller/follow_joint_trajectory`.
The controllers are global, so each action is remapped back out. `rclcpp`
has no single-name action remap, so all five underlying entities are
remapped individually, `send_goal`, `cancel_goal`, `get_result`,
`feedback`, `status`, giving 10 rules per arm. The launch file derives
them by parsing the per-arm controllers YAML, so the two cannot drift
apart.

**TF.** Written `('tf', '/tf')`, not `('/tf', 'tf')`. A remap rule's
match side is expanded with the node namespace before comparison, so this
form fires if `tf2_ros` used a relative name and is a harmless no-op if
it used an absolute one. Either way `move_group` lands on the global tree.

**joint_states.** Remapped to the global topic so both instances read the
single `joint_state_broadcaster`.

### Collision checking: what you get, and what you don't

Because both instances load the full dual-arm model and monitor all 12
joints, each planner **does** see the other arm as an obstacle at its
current position. That is state-based avoidance.

What is lost relative to a single combined-group plan is **time**
synchronisation: the two trajectories are produced by separate processes
and are not checked against each other's *future* paths. Arm A plans
around where arm B is at t=0, not where B will be at t=0.4.

With bases 1.5 m apart and targets clamped to a ±8 cm box, the arms
cannot reach each other from the home pose, and the SRDF marks every
cross-arm link pair `reason="Never"`. **Revisit this if you move the
bases closer or widen `workspace_extent` substantially.**

### Per-arm flow control

`ArmGate` allows one goal in flight per arm. Targets arriving while a
goal runs replace the pending one rather than queueing, so the arm always
chases the freshest hand position. Stale intermediate targets are
dropped on purpose, since the hand has already moved past them. A
watchdog cancels goals that overrun `goal_timeout`.

`clear()` (called when a hand disappears) drops the *pending* target but
deliberately leaves an in-flight goal alone: letting the current motion
finish is less surprising than aborting when a hand flickers out for a
few frames.

---

## 8. Verifying a healthy system

```bash
# Two namespaced action servers
ros2 action list | grep move_action
#   /robot_one/move_action
#   /robot_two/move_action

# Each move_group must reach the GLOBAL controller actions.
# If you see a /robot_one prefix here, the remaps did not apply and
# goals will be accepted then fail with CONTROL_FAILED.
ros2 node info /robot_one/move_group | grep follow_joint_trajectory

# One publisher, ~100 Hz
ros2 topic hz /joint_states

# Hand tracking alive
ros2 topic echo /left_hand_state
```

On startup the teleop node announces its wiring:

```
Arms: robot_one -> /robot_one/move_action, robot_two -> /robot_two/move_action
robot_one: move_group ready on /robot_one/move_action
robot_two: move_group ready on /robot_two/move_action
```

Run with `debug:=true` to see per-arm plan lines. Under the current
layout they interleave freely rather than appearing in pairs.

---

## 9. Webcam tips

- Use a well-lit environment and face the camera directly.
- Keep both hands fully inside the frame.
- Avoid gloves or accessories that obscure hand features.

---

## 10. Limitations and next steps

**Stepwise motion.** The pipeline is plan-and-execute at roughly one
second per motion, not continuous servoing. Fresh targets replace each
other while a plan runs. This is the largest remaining limitation, and it
is not really the right control mode for following a hand.

**Two control axes per hand.** Depth is disabled by default (noisy
monocular estimate), and wrist orientation is tracked but not yet mapped
to the robot.

**No time-synchronised cross-arm avoidance.** See §7.

**Lighting sensitivity.** Poor light degrades detection; a lost hand
pauses that arm until it reappears.

---

## License

Apache-2.0
