# Running the teleoperation stack in Docker

## Quickstart (Linux host with a webcam)

```bash
xhost +local:                 # allow the container to open RViz on your display
docker compose up --build     # first build takes a while; cached afterwards
```

Ctrl-C stops everything. After editing code, `docker compose up --build`
rebuilds only what changed (dependency layers are cached; a code edit
re-runs just the colcon build layer).

## What the pieces are

`Dockerfile` is the recipe: Ubuntu 24.04 + ROS 2 Jazzy + MoveIt +
ros2_control + mediapipe + this workspace, compiled. Building it produces
an *image* - a frozen, self-contained filesystem. `docker compose up`
starts a *container*, a running instance of that image, and
`docker-compose.yml` records every run-time flag as code: which camera
device to pass in, GPU access, X11 display, host networking so ROS tools
on your host can still see the topics.

Because mediapipe is pip-installed inside the image, the
`external_libraries` workaround is unnecessary in Docker (the loader's
plain `import mediapipe` succeeds first) - and the matplotlib Axes3D
warning disappears with it.

## Requirements and knobs

* Native Docker on Linux. Docker Desktop on macOS/Windows cannot pass a
  webcam through as `/dev/video0`, and X11 forwarding there is painful -
  this setup targets a Linux host.
* Webcam on a different index: change the mapping in `docker-compose.yml`
  to e.g. `/dev/video2:/dev/video0` (the node inside always reads
  `/dev/video0`).
* NVIDIA GPU instead of Intel/AMD: install `nvidia-container-toolkit`,
  remove the `/dev/dri` line, and add `gpus: all` to the service.
* Everything runs as one container; because of `network_mode: host` you
  can still run `ros2 topic echo /left_hand_state`, `rqt`, or a second
  RViz directly on the host while it's up.

## Troubleshooting

* **RViz window never appears / "could not connect to display"** - run
  `xhost +local:` again (it resets on logout), and confirm `echo $DISPLAY`
  is non-empty in the shell where you run compose. Wayland desktops work
  through XWayland automatically.
* **RViz opens but renders black or garbled** - uncomment
  `LIBGL_ALWAYS_SOFTWARE: "1"` in the compose file (software rendering;
  slower but reliable).
* **"Could not open video device"** - check the host index with
  `v4l2-ctl --list-devices` and adjust the device mapping; make sure no
  other app is holding the camera.
* **Slow first build** - normal: the image pulls ~2 GB of ROS/MoveIt debs.
  Subsequent builds reuse cached layers.

## Why bother

One command reproduces the exact environment - same ROS distro, same
MoveIt, same mediapipe - on any Linux machine, whether or not it has ROS
installed, and without touching the host system. That means contributors
and reviewers can run the demo in minutes, CI can build and test the
image on every push, and "works on my machine" stops being a sentence you
ever have to say about this project.
