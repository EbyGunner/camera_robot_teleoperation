# camera_robot_teleoperation — full stack in one image
# Base: ROS 2 Jazzy desktop (Ubuntu 24.04) — ships RViz2, matching the
# distro this project is developed against.
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# --- system + ROS dependencies -------------------------------------------
# MoveIt / ros2_control stacks installed explicitly (belt and braces on top
# of rosdep below), plus OpenCV/scipy for the Python nodes and v4l-utils
# for webcam debugging inside the container.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-moveit \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gripper-controllers \
    ros-jazzy-cv-bridge \
    python3-pip \
    python3-opencv \
    python3-scipy \
    python3-numpy \
    libopencv-dev \
    v4l-utils \
 && rm -rf /var/lib/apt/lists/*

# mediapipe has no apt/rosdep package - pip it into the image (this replaces
# the repo's external_libraries sys.path workaround; the loader's plain
# `import mediapipe` succeeds first). The install is deliberately TWO-STEP
# and fully pinned - verified end-to-end on this exact Python 3.12 /
# Ubuntu 24.04 stack with the Debian-managed numpy/scipy in place:
#   * mediapipe 0.10.14 - a battle-tested release that still ships the
#     legacy mp.solutions API this project uses; newer releases (0.10.3x)
#     removed it ("module 'mediapipe' has no attribute 'solutions'").
#   * --no-deps + explicit list: mediapipe declares jax/jaxlib, which the
#     hands pipeline never touches but whose scipy>=1.12 requirement would
#     make pip try to uninstall the Debian-managed scipy 1.11 - impossible
#     (no RECORD file, "installed by debian") and undesirable anyway.
#     Skipping auto-deps and listing the real runtime set keeps apt's
#     numpy 1.26 / scipy 1.11 exactly as the ROS nodes expect and saves
#     ~170 MB of jax/jaxlib. (numpy and matplotlib come from apt above.)
RUN pip3 install --no-cache-dir --break-system-packages --no-deps \
      'mediapipe==0.10.14' \
 && pip3 install --no-cache-dir --break-system-packages \
      'numpy<2' \
      'opencv-contrib-python==4.11.0.86' \
      'absl-py==2.5.0' \
      'flatbuffers==25.12.19' \
      'protobuf==4.25.9' \
      'sounddevice==0.5.5'

# Fail the BUILD - not the runtime - if this set ever stops providing a
# working hands pipeline: runs one frame through the actual TFLite graph.
RUN python3 -c "import mediapipe as mp, numpy as np; \
h = mp.solutions.hands.Hands(static_image_mode=True); \
h.process(np.zeros((48, 64, 3), dtype=np.uint8)); h.close(); \
print('mediapipe hands pipeline OK')"

WORKDIR /ws

# --- dependency resolution (cached) --------------------------------------
# Copy only the package manifests first so this expensive layer is reused
# on every code edit and re-runs only when a package.xml changes.
COPY robot_interfaces/package.xml    src/camera_robot_teleoperation/robot_interfaces/package.xml
COPY hand_tracking/package.xml       src/camera_robot_teleoperation/hand_tracking/package.xml
COPY robot_imitation/package.xml     src/camera_robot_teleoperation/robot_imitation/package.xml
COPY robot_main/package.xml          src/camera_robot_teleoperation/robot_main/package.xml
COPY robot_moveit/config/package.xml src/camera_robot_teleoperation/robot_moveit/config/package.xml

# skip-keys: warehouse_ros_mongo is a Setup Assistant template leftover that
# was never released for Jazzy; opencv is a legacy key already covered by
# the explicit apt install above.
RUN apt-get update && rosdep update --rosdistro jazzy \
 && rosdep install --from-paths src --ignore-src -y \
      --skip-keys "warehouse_ros_mongo opencv" \
 && rm -rf /var/lib/apt/lists/*

# --- build the workspace --------------------------------------------------
COPY . src/camera_robot_teleoperation
RUN . /opt/ros/jazzy/setup.sh && colcon build --event-handlers console_cohesion+

# --- runtime --------------------------------------------------------------
# Entrypoint written inline so the image needs nothing beyond the repo
# itself (a separate docker/entrypoint.sh file proved too easy to miss).
RUN printf '%s\n' \
      '#!/usr/bin/env bash' \
      'set -e' \
      'source /opt/ros/jazzy/setup.bash' \
      'source /ws/install/setup.bash' \
      'exec "$@"' > /entrypoint.sh \
 && chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "robot_main", "robot_main.launch.py"]