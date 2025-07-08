# Dual Arm Robot Teleoperation via Webcam Hand Tracking

This ROS 2 project enables real-time control of two 6-DOF robotic arms using a standard webcam and your hands. The system mimics the motions of your **left** and **right** hands to control the **left** and **right** robotic arms respectively. The end-effectors follow the hand movements, and the grippers open or close based on the state of the respective hands (open or closed).

This setup is ideal for remote manipulation, human-robot interaction experiments, or learning teleoperation and motion planning with ROS 2, MoveIt, and real-time perception.

---

## 📦 Repository

**GitHub Link**: [https://github.com/EbyGunner/camera_robot_teleoperation.git](https://github.com/EbyGunner/camera_robot_teleoperation.git)

---

## 🛠️ Requirements

- ROS 2 Humble
- MoveIt 2
- `ros2_control`
- `ros2_controllers`
- Python3 packages: `mediapipe`

---

## ✅ Installation Steps

### 1. Setup a ROS 2 Humble Workspace

```bash
mkdir -p ~/teleop_ws/src
cd ~/teleop_ws/src
```
---

### 2. Clone the Repository

⚠️ DO NOT create a sub-folder; clone the repository directly inside `src`.

```bash
git clone https://github.com/EbyGunner/camera_robot_teleoperation.git
```
---
### 3. Install Dependencies
Install required ROS 2 packages:
```bash
sudo apt update
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
```
---
### 4. Install Required Python Packages
Local installation of mediapipe in project directory:

To avoid installing mediapipe globally:
```bash
cd ~/teleop_ws/src/camera_robot_teleoperation/hand_tracking
mkdir -p external_libraries
pip3 install mediapipe --target=external_libraries
```
This will install `mediapipe` in the `hand_tracking/external_libraries` folder. The teleoperation script is already configured to import it from this local path.

---
### 5. Build the Workspace
```bash
cd ~/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
---
### 6. Source the Workspace
```bash
source ~/teleop_ws/install/setup.bash
```
---

## 🚀 Running the Teleoperation System

### 1. Launch the Robot Simulation (in RViz)

Launch the robot simulation using:
```bash
ros2 launch robot_main robot_main.launch.py
```
This will bring up two MoveIt-controlled arms with working joint_state_broadcasters and gripper controllers.

---

### 2. Start the Webcam Hand Tracker and Controller Node
```bash
ros2 launch robot_imitation robot_imitation.launch.py 
```
This node:

* Uses your tracked hands messages.

* Publishes end-effector goals based on hand poses.

* Controls grippers based on hand openness (closed hand → close gripper, open hand → open gripper).

#### Optional Launch Arguments
You can customize the behavior of the teleoperation node with the following launch arguments:

| Argument      | Description | Default     |
| -----------   | ----------- | ----------- |
| planning_time        | Allowed planning time in seconds       |5.0        |
| velocity_scaling        | Velocity scaling factor (0.0–1.0)       |0.5        |
| acceleration_scaling        | Acceleration scaling factor (0.0–1.0)       |0.5        |
| world_frame        | World frame used for TF       |world        |
| debug        | Enable debug logging output       |false        |

Example: Enable Debug Mode
```bash
ros2 launch robot_imitation robot_imitation.launch.py debug:=true
 
```
Example: Set custom velocity and planning time
```bash
ros2 launch robot_imitation robot_imitation.launch.py velocity_scaling:=0.3 planning_time:=10.0 
```

---
## ⚠️ Known Issues

* Current Limitation: The robot does not yet follow the end-effector pose goals correctly via trajectory execution.

This is due to a **TF transform issues**, which is being actively worked on. Motion planning works, but actual trajectory following might not reflect the target poses yet.


##  📸 Webcam Setup

* Use a well-lit environment.

* Place your webcam directly in front of your hands.

* Keep your hands movements within the camera frame.

* Avoid wearing gloves or accessories that obscure hand features.

## 📂 Project Structure
```bash
camera_robot_teleoperation/
├── .devcontainer
|   └── Dockerfile
├── hand_tracking
│   ├── external_libraries
│   ├── handtracking
|   ├── launch
|   ├── resource
|   ├── test
|   ├── CMakeLists.txt
|   ├── package.xml
|   ├── setup.cfg
|   └── setup.py
├── robot_imitation
│   ├── launch
│   ├── resource
|   ├── robot_imitation
|   ├── test
|   ├── LICENSE
|   ├── package.xml
|   ├── setup.cfg
|   └── setup.py
├── robot_interfaces
│   ├── msg
|   ├── CMakeLists.txt
|   └── package.xml
├── robot_moveit
│   ├── config
│   ├── launch
|   ├── .setup_assistant
|   ├── CMakeLists.txt
|   └── package.xml
├── robot_main
│   ├── src
│   ├── LICENSE
|   ├── .setup_assistant
|   ├── CMakeLists.txt
|   └── package.xml
├── .gitignore
└── Readme.md
```

## 🧠 Future Improvements

* Fix TF and trajectory following to enable true hand-to-end-effector mimicry.

* Introduce orientation of gripper based on hand orientation


# Setting up Docker for ROS2 Humble

This is the step by step instructions on how to create a ros2 humble docker image and container with host webcam and graphincal interface access and run the above program.

Follow the instructions shown [here](https://docs.docker.com/engine/install/ubuntu/) to install docker in your system.

## Creating docker ros2 humble image


### 1. Create Dockerfile

Create a file called `Dockerfile` and copy the contents from `.devcontainer/Dockerfile` into this file.

---

### 2. Create docker image

Open a terminal in your machine and run the following command:

```bash
docker build -t {docker_image_name} {Dockerfile address}

```
Replace `{docker_image_name}` with a name of your choice and `{Dockerfile address}` with the address to your `Dockerfile`.

note: `{Dockerfile address}` should be until the `Dockerfile` and not including it. That is, if the file is at `src/camera_robot_teleoperation/.devcontainer/Dockerfile`, the `{Dockerfile address}` should be `src/camera_robot_teleoperation/.devcontainer/`.

---
### 3. Create docker container
Once the image is set up, docker needs access to your X server which can be granted using the following command:
```bash
xhost +local:docker
```
After the program is run, this can be reversed using the following command:
```bash
xhost -local:docker
```
Once the access is granted, the docker container can be created
```bash
docker run -it \
  --name {container_name} \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device=/dev/video0:/dev/video0 \
  --privileged \
  {docker_image_name}
```
Where `{container_name}` should be replaced with the a name for the container and `{docker_image_name}` should be replaced with the image name set in the previous step. If the video device is not the webcam, replace the device number with the correct number.

---
### 4. Setting up the project
The above step will start a terminal in the container and the steps for `Dual Arm Robot Teleoperation via Webcam Hand Tracking` can be followed. To exit from the container, press `ctrl + d`.

## Container operations
To start a new terminal in the container:
```bash
docker exec -it {container_name} /bin/bash
```
Once the container is closed, to restart the container, the following command can be used:
```bash
docker start -i {container_name}
```
