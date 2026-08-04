# Dual Arm Robot Teleoperation via Webcam Hand Tracking

Control two 6-DOF robotic arms in real time using nothing but a standard
webcam and your bare hands. Your **left** and **right** hands drive the
**left** and **right** robots respectively. Both arms move
**simultaneously** through a single collision-checked MoveIt plan, and
making a fist closes the matching gripper.

Built on ROS 2, MoveIt 2, ros2_control, and MediaPipe. Ideal for learning
teleoperation, motion planning, and real-time perception, or as a base for
human–robot interaction experiments.

## ✨ Features

* **Markerless hand tracking** -- MediaPipe reads 21 landmarks per hand
  from a single webcam; no gloves, controllers, or depth camera.
* **Simultaneous dual-arm motion** -- both hands' targets are merged into
  one goal for a combined 12-DOF planning group, so the arms plan and
  execute together and are collision-checked *against each other*.
* **Gesture grippers** -- fist closes, open hand opens; the passive finger
  mirrors via a mimic joint so the full gripper tracks in RViz and TF.
* **Safe relative mapping** -- hand motion is applied relative to where
  your hand first appeared, clamped to a workspace box around the arm's
  reference pose; losing a hand resets its reference.
* **Robust planning pipeline** -- freshest-target batching, per-arm
  re-plan gates, and a watchdog that cancels stuck goals.
* **Modular, testable code** -- the mapping, batching, and gesture logic
  are pure Python modules with no ROS imports, unit-testable on their own.
* **One-command Docker setup** -- the entire stack (ROS 2 + MoveIt +
  MediaPipe + this workspace) builds into a single container.

## 🛠️ Requirements

* Linux host with a webcam (`/dev/video0` by default)
* **Docker route:** just Docker Engine, nothing else, not even ROS
* **Native route:** ROS 2 Jazzy (the tested distro), MoveIt 2,
  ros2_control/ros2_controllers, Python 3 with `mediapipe`

## 🐳 Quick Start with Docker (recommended)

```bash
git clone https://github.com/EbyGunner/camera_robot_teleoperation.git
cd camera_robot_teleoperation
xhost +local:                  # allow the container to open RViz
docker compose up --build      # first build takes a while; cached afterwards
```

RViz opens with both arms; step in front of the camera and they follow
your hands. `Ctrl-C` stops everything, and `xhost -local:` revokes the
display access again.

Useful extras:

```bash
docker compose exec teleop bash          # second shell inside the container
ros2 topic echo /left_hand_state         # works from the HOST too (host networking)
```

Camera on a different index, black RViz window, NVIDIA GPUs, and a
dev-mode source mount are all covered in [DOCKER.md](DOCKER.md).

## ✅ Native Installation

```bash
mkdir -p ~/teleop_ws/src && cd ~/teleop_ws/src
git clone https://github.com/EbyGunner/camera_robot_teleoperation.git   # directly inside src, no sub-folder
cd ~/teleop_ws
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "warehouse_ros_mongo opencv"
sudo apt install ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gripper-controllers
```

Install MediaPipe (pick one):

```bash
pip3 install --break-system-packages mediapipe          # simplest
# or, to keep it out of the system Python:
cd src/camera_robot_teleoperation/hand_tracking
mkdir -p external_libraries && pip3 install mediapipe --target=external_libraries
# or point the loader anywhere: export HAND_TRACKING_MEDIAPIPE_DIR=/path/to/site-packages
```

Build and run:

```bash
cd ~/teleop_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Running

One command brings up everything: MoveIt, controllers, RViz, the hand
tracker, and the teleoperation node:

```bash
ros2 launch robot_main robot_main.launch.py
```

### 🖐️ Controls

| You do | Robot does |
| ------ | ---------- |
| Show a hand to the camera | Matching arm captures its reference pose ("reference pose set" in the log) |
| Move the hand left/right | End effector moves along the arm's y axis |
| Move the hand up/down | End effector moves along z |
| Move **both** hands | **Both arms plan and move simultaneously** |
| Make a fist / open the hand | Gripper closes / opens |
| Hide the hand | That arm's reference resets; it re-captures when the hand returns |

Motion is clamped to a box (default ±8 cm) around each arm's reference
pose. Depth control (hand toward/away from camera) exists but ships
disabled; monocular depth from hand size is noisy (`scale_x`).

### Launch arguments

| Argument | Description | Default |
| -------- | ----------- | ------- |
| `planning_time` | Allowed planning time in seconds | `5.0` |
| `velocity_scaling` | Velocity scaling factor (0.0–1.0) | `0.5` |
| `acceleration_scaling` | Acceleration scaling factor (0.0–1.0) | `0.5` |
| `workspace_extent` | Half-size (m) of the teleop box | `0.08` |
| `use_orientation_constraint` | Set `false` when using position-only IK | `true` |
| `diagnostic_mode` | `''` = follow hands; `exact_current_pose` / `fixed_offset` for planner debugging | `''` |
| `debug` | Verbose logging | `false` |

Many more knobs (`scale_x/y/z`, `merge_window`, `plan_cooldown`,
`position_threshold`, gripper positions, …) are ROS parameters on the
`hand_to_robot_controller` node; see `robot_imitation/robot_imitation/params.py`.

## 📸 Webcam Tips

* Use a well-lit environment and face the camera directly.
* Keep both hands fully inside the frame.
* Avoid gloves or accessories that obscure hand features.

## 🧩 How It Works

```
webcam ─▶ MediaPipe (hand_tracking) ─▶ /left_hand_state, /right_hand_state
       ─▶ imitation node (robot_imitation): relative mapping + target merging
       ─▶ MoveIt 2 /move_action: one collision-checked plan (single arm or
          combined 12-DOF group) ─▶ ros2_control trajectory + gripper
          controllers ─▶ RViz
```

| Package | Role |
| ------- | ---- |
| `hand_tracking` | Webcam capture, MediaPipe detection, gesture + relative-pose logic |
| `robot_interfaces` | Custom `HandState` message |
| `robot_imitation` | Hand→target mapping, goal batching, MoveIt + gripper clients |
| `robot_moveit` | MoveIt configuration (URDF/SRDF, controllers, RViz) |
| `robot_main` | Robot description and the top-level launch |

## ⚠️ Current Limitations

* **Stepwise motion**: the pipeline is plan-and-execute (~1 s per
  motion), not continuous servoing; fresh targets queue while a plan runs.
* **Two control axes per hand**: depth is disabled by default (noisy
  monocular estimate), and wrist orientation is tracked but not yet
  mapped to the robot.
* **Lighting sensitivity**: poor light degrades detection; a lost hand
  pauses that arm until it reappears.

## 🔧 Project Notes

* [DOCKER.md](DOCKER.md): container details and troubleshooting.

## 📄 License

Apache-2.0