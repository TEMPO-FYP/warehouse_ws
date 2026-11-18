# Clearpath Husky A200 + UR5e — Setup & Launch Guide

## Prerequisites

- Ubuntu 24.04
- ROS2 Jazzy installed
- Gazebo Harmonic installed

## Step 1: Add Clearpath apt Sources

```bash
wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'
sudo apt update
```

Add Clearpath rosdep rules:

```bash
sudo wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list \
    -O /etc/ros/rosdep/sources.list.d/50-clearpath.list
rosdep update
```

## Step 2: Install Required Packages

```bash
sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-clearpath-desktop
sudo apt install ros-jazzy-clearpath-simulator
sudo apt install ros-jazzy-ur
```

## Step 3: Extract the Clearpath Config

Unzip the provided `clearpath.zip` and place it in your home directory:

```bash
cd ~
unzip clearpath.zip
```

This gives you the pre-configured `~/clearpath/` folder containing:
- `robot.yaml` — Husky A200 + UR5e configuration
- `setup.bash` — generated environment setup
- `robot.urdf.xacro` — generated full robot description
- `robot.srdf` — generated MoveIt semantic description
- `manipulators/` — MoveIt2 arm configuration
- `platform/` — ros2_control base configuration

## Step 4: Source the Setup

Add the following to your `~/.bashrc`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/clearpath/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 5: Launch the Simulation

**Terminal 1 — Launch Gazebo with the Husky + UR5e:**

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/clearpath world:=/path/to/your/custom_world.sdf
```

**Replace `/path/to/your/custom_world.sdf` with the actual path to your world SDF file.**

Wait until Gazebo fully loads and the robot spawns before proceeding.

**Terminal 2 — Launch MoveIt2 for the arm:**

```bash
source ~/clearpath/setup.bash
ros2 launch clearpath_manipulators moveit.launch.py setup_path:=$HOME/clearpath use_sim_time:=true
```

Wait until you see `MoveGroup context initialization complete` before proceeding.

**Terminal 3 — Launch RViz:**

```bash
source ~/clearpath/setup.bash
ros2 launch clearpath_viz view_moveit.launch.py namespace:=a200_0000 use_sim_time:=True
```

## Step 6: Verify Everything is Running

In a new terminal confirm the robot topics are available:

```bash
ros2 topic list | grep a200
```

You should see topics including:
/a200_0000/cmd_vel
/a200_0000/joint_states
/a200_0000/arm_0_joint_trajectory_controller/joint_trajectory

## Controlling the Robot

### Drive the Husky Base via Keyboard

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/a200_0000/cmd_vel
```

### Drive the Husky Base via Topic

```bash
# Move forward
ros2 topic pub --once /a200_0000/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Turn left
ros2 topic pub --once /a200_0000/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"

# Stop
ros2 topic pub --once /a200_0000/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

### Move the Arm via RViz

1. RViz opens automatically in Terminal 3
2. In the MotionPlanning panel set **Planning Group** to `arm_0`
3. Set **Goal State** to `home` or `up` from the dropdown
4. Click **Plan & Execute**
5. Watch the arm move in both RViz and Gazebo simultaneously

### Move the Arm via Topic

```bash
ros2 topic pub --once /a200_0000/arm_0_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [arm_0_shoulder_pan_joint, arm_0_shoulder_lift_joint, arm_0_elbow_joint, arm_0_wrist_1_joint, arm_0_wrist_2_joint, arm_0_wrist_3_joint],
  points: [{positions: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0], time_from_start: {sec: 2}}]
}"
```