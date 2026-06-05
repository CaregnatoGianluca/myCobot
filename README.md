# myCobot 280 — ROS 2 Workspace

ROS 2 Jazzy workspace for the **Elephant Robotics myCobot 280 Raspberry Pi 2023** (6-DOF, 280 mm reach, 250 g payload).

Supports three modes via two launch files:
| Mode | Command |
|---|---|
| Gazebo simulation + MoveIt2 | `ros2 launch mycobot_bringup sim.launch.py` |
| Mock hardware (offline, no Gazebo) | `ros2 launch mycobot_bringup real.launch.py use_mock_hardware:=true` |
| Real robot (serial) | `ros2 launch mycobot_bringup real.launch.py` |

---

## Prerequisites

- **OS**: Ubuntu 24.04 (native or WSL2 on Windows 11)
- **ROS 2**: Jazzy Jalisco
- **Gazebo**: Harmonic (gz-sim 8.x)

### Install ROS 2 Jazzy

Follow the [official ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then install the full desktop and required extras:

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-moveit \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-position-controllers \
  ros-jazzy-gripper-controllers \
  ros-jazzy-moveit-ros-move-group \
  ros-jazzy-moveit-planners-ompl \
  ros-jazzy-moveit-kinematics \
  ros-jazzy-moveit-simple-controller-manager \
  ros-jazzy-pilz-industrial-motion-planner \
  ros-jazzy-stomp-moveit \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-generate-parameter-library \
  python3-colcon-common-extensions
```

### Install pymycobot (for real robot)

```bash
sudo apt install python3-pip
python3 -m pip install pymycobot pyserial --break-system-packages
```

---

## Clone and Build

```bash
# Clone this workspace
git clone <this-repo-url> ~/myCobot
cd ~/myCobot

# Build (skip broken demo packages from the upstream repo)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select \
    mycobot_interfaces \
    mycobot_description \
    mycobot_gazebo \
    mycobot_moveit_config \
    mycobot_hardware \
    mycobot_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# Source the workspace
source install/local_setup.bash
```

Add to `~/.bashrc` so you don't have to source every session:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/myCobot/install/local_setup.bash 2>/dev/null || true' >> ~/.bashrc
echo "alias mycobot_build='colcon build --symlink-install \
  --packages-select mycobot_interfaces mycobot_description mycobot_gazebo \
    mycobot_moveit_config mycobot_hardware mycobot_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3'" >> ~/.bashrc
```

---

## Running

### Simulation (Gazebo + MoveIt2 + RViz2)

```bash
export LIBGL_ALWAYS_SOFTWARE=1        # required on WSL2 (no GPU passthrough)
export MESA_GL_VERSION_OVERRIDE=4.5   # required on WSL2
ros2 launch mycobot_bringup sim.launch.py
```

Optional arguments:
```bash
ros2 launch mycobot_bringup sim.launch.py use_camera:=false world_file:=pick_and_place_demo.world
```

### Mock hardware (offline development, no Gazebo)

Useful for testing MoveIt2 planning and URDF changes without Gazebo running.

```bash
ros2 launch mycobot_bringup real.launch.py use_mock_hardware:=true
```

### Real robot

Connect the myCobot 280 Pi via USB-C, then:

```bash
ros2 launch mycobot_bringup real.launch.py
```

The hardware interface communicates over `/dev/ttyAMA0` at 1 Mbaud (configured in [src/mycobot_hardware/config/hardware_params.yaml](src/mycobot_hardware/config/hardware_params.yaml)).

---

## Project Structure

```
src/
├── mycobot_ros2/                     # based on automaticaddison/mycobot_ros2 (modified, see below)
│   ├── mycobot_description/          # URDF/xacro, meshes, launch for robot_state_publisher
│   ├── mycobot_bringup/              # top-level launch files (sim.launch.py, real.launch.py)
│   ├── mycobot_gazebo/               # Gazebo world, models, bridge config
│   ├── mycobot_moveit_config/        # MoveIt2 SRDF, kinematics, planning pipelines
│   └── mycobot_interfaces/           # custom ROS 2 message/service definitions
└── mycobot_hardware/                 # custom ros2_control hardware interface (serial → real robot)
```

Packages excluded from the build (upstream compilation issues with Miniconda Python):
- `mycobot_moveit_demos`
- `mycobot_mtc_demos`
- `mycobot_system_tests`

---

## Modifications from upstream

This workspace includes [automaticaddison/mycobot_ros2](https://github.com/automaticaddison/mycobot_ros2) with the following modifications:

### `mycobot_description`

**[`urdf/control/mycobot_280_ros2_control.urdf.xacro`](src/mycobot_ros2/mycobot_description/urdf/control/mycobot_280_ros2_control.urdf.xacro)**
Added `use_mock_hardware` parameter. Hardware plugin is now selected at launch time:
- `use_gazebo=true` → `gz_ros2_control/GazeboSimSystem`
- `use_mock_hardware=true` → `mock_components/GenericSystem` (offline dev, no hardware needed)
- neither → `mycobot_hardware/MyCobotHardwareInterface` (real robot via serial)

**[`urdf/robots/mycobot_280.urdf.xacro`](src/mycobot_ros2/mycobot_description/urdf/robots/mycobot_280.urdf.xacro)**
Added `use_mock_hardware` xacro arg, forwarded to the control macro.

**[`launch/robot_state_publisher.launch.py`](src/mycobot_ros2/mycobot_description/launch/robot_state_publisher.launch.py)**
- Replaced hardcoded `~/ros2_ws` paths with `ament_index_python` so the file works from any workspace location.
- Added `use_mock_hardware` launch argument forwarded into xacro.

### `mycobot_gazebo`

**[`launch/mycobot.gazebo.launch.py`](src/mycobot_ros2/mycobot_gazebo/launch/mycobot.gazebo.launch.py)**
Added `condition=IfCondition(use_camera)` to the `ros_gz_image` bridge node — without this, launching with `use_camera:=false` crashes if `ros_gz_image` is not installed.

### `mycobot_moveit_config`

**[`launch/load_ros2_controllers.launch.py`](src/mycobot_ros2/mycobot_moveit_config/launch/load_ros2_controllers.launch.py)**
Replaced the `TimerAction(10s) + ros2 control load_controller` CLI approach with `controller_manager/spawner` nodes. Spawners wait up to 30 s for the controller_manager (essential on WSL2 where Gazebo is slow to start).

### `mycobot_bringup`

**[`CMakeLists.txt`](src/mycobot_ros2/mycobot_bringup/CMakeLists.txt)** — added `launch` to install directories.

**[`package.xml`](src/mycobot_ros2/mycobot_bringup/package.xml)** — added `exec_depend` for all sibling packages.

**[`launch/sim.launch.py`](src/mycobot_ros2/mycobot_bringup/launch/sim.launch.py)** *(new)*
Top-level simulation entry point: starts Gazebo, spawns robot, launches MoveIt2 + RViz2.

**[`launch/real.launch.py`](src/mycobot_ros2/mycobot_bringup/launch/real.launch.py)** *(new)*
Top-level real/mock entry point: starts `robot_state_publisher`, loads controllers, launches MoveIt2 + RViz2.

### `mycobot_hardware` *(new package)*

Custom `ros2_control` hardware interface plugin for the real robot, exported as `mycobot_hardware/MyCobotHardwareInterface`. Currently a stub — the serial communication with `pymycobot` is the next step when the physical robot arrives.

---

## WSL2 Notes

**Display**: WSLg on Windows 11 provides `DISPLAY=:0` and `WAYLAND_DISPLAY=wayland-0` automatically — no VcXsrv needed.

**OpenGL**: Software rendering is required since there is no GPU passthrough. Set these before launching Gazebo:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=4.5
```

**Python conflict**: If Miniconda is installed alongside system Python 3.12, force colcon to use the system interpreter:
```bash
colcon build ... --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

---

## Roadmap

- [ ] Complete real robot serial driver in `mycobot_hardware/`
- [ ] Add camera pipeline (Intel RealSense D435 via `ros_gz_image`)
- [ ] Integrate [LeRobot](https://github.com/huggingface/lerobot) for dataset collection
- [ ] Train and deploy a VLA (SmolVLA / ACT) for pick-and-place tasks
