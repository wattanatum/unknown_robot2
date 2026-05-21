# unknown_robot2

`unknown_robot2` is a ROS 2 mobile robot project for simulation, SLAM, localization, and autonomous navigation using Nav2.

This project contains the robot description, Gazebo simulation files, navigation configuration, maps, launch files, and RViz setup for a custom differential-drive mobile robot.

## What Can You Do With It?

- Simulate a custom mobile robot in Gazebo
- Visualize robot model, TF, laser scan, odometry, and map in RViz2
- Build a map using SLAM Toolbox
- Navigate autonomously using Nav2
- Test global planner and local planner behavior
- Tune costmaps, inflation layer, controller, and planner parameters

## Features

- ROS 2 Jazzy / Ubuntu 24.04 support
- Gazebo simulation
- Differential-drive robot model
- LiDAR sensor support
- Camera sensor support
- IMU support
- SLAM Toolbox mapping
- Nav2 autonomous navigation
- AMCL localization
- RViz2 visualization

## Workspace Structure

```text
unknown_robot2_ws/
├── src/
│   ├── unknown_robot2_description/
│   │   ├── urdf/
│   │   ├── meshes/
│   │   └── launch/
│   ├── unknown_robot2_navigation/
│   │   ├── config/
│   │   ├── launch/
│   │   ├── maps/
│   │   └── rviz/
│   ├── unknown_robot2_bringup/
│   │   └── launch/
│   └── ...
├── README.md
├── LICENSE
└── .gitignore
```

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Nav2
- SLAM Toolbox
- RViz2
- `ros_gz` bridge packages

Install common dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro
```

## Installation

Clone the repository:

```bash
cd ~
git clone git@github.com:wattanatum/unknown_robot2.git unknown_robot2_ws
cd unknown_robot2_ws
```

Install dependencies:

```bash
rosdep update
rosdep install --from-paths src -y --ignore-src
```

Build the workspace:

```bash
colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

Optional: add to `.bashrc`:

```bash
echo "source ~/unknown_robot2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Quickstart

### 1. Launch Gazebo Simulation

```bash
ros2 launch unknown_robot2_bringup gazebo.launch.py
```

### 2. Launch RViz2

```bash
ros2 launch unknown_robot2_navigation rviz.launch.py
```

### 3. Run SLAM Mapping

```bash
ros2 launch unknown_robot2_navigation slam_toolbox_mapping.launch.py
```

### 4. Save Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/unknown_robot2_ws/src/unknown_robot2_navigation/maps/unknown_map
```

### 5. Run Navigation with Saved Map

```bash
ros2 launch unknown_robot2_navigation nav2.launch.py
```

## Main Launch Files

| Launch File | Description |
|---|---|
| `gazebo.launch.py` | Start robot simulation in Gazebo |
| `rviz.launch.py` | Open RViz2 visualization |
| `slam_toolbox_mapping.launch.py` | Start SLAM Toolbox mapping |
| `nav2.launch.py` | Start Nav2 navigation |
| `cartographer_mapping.launch.py` | Optional Cartographer mapping |

## Important ROS 2 Topics

| Topic | Description |
|---|---|
| `/cmd_vel` | Robot velocity command |
| `/odom` | Odometry data |
| `/scan` | LiDAR scan |
| `/imu` | IMU data |
| `/tf` | Dynamic transforms |
| `/tf_static` | Static transforms |
| `/map` | Occupancy grid map |
| `/goal_pose` | Navigation goal pose |

## Navigation Stack

This project uses Nav2 for autonomous navigation.

Main Nav2 components:

- Planner Server
- Controller Server
- Behavior Tree Navigator
- AMCL
- Map Server
- Global Costmap
- Local Costmap
- Recovery Behaviors
- Velocity Smoother

## Costmap Tuning

Inflation layer example:

```yaml
inflation_layer:
  plugin: nav2_costmap_2d::InflationLayer
  inflation_radius: 0.25
  cost_scaling_factor: 3.0
```

Lower `inflation_radius` reduces the obstacle safety area.

Higher `cost_scaling_factor` makes obstacle cost drop faster.

## Build Again After Editing Code

```bash
cd ~/unknown_robot2_ws
colcon build --symlink-install
source install/setup.bash
```

## Push Updates to GitHub

```bash
cd ~/unknown_robot2_ws
git add README.md src .gitignore
git commit -m "Update README"
git push
```

## Troubleshooting

### Robot does not move

Check:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 run tf2_tools view_frames
```

### No map in RViz2

Check:

```bash
ros2 topic list | grep map
ros2 topic echo /map --once
```

### No LiDAR scan

Check:

```bash
ros2 topic echo /scan
```

### TF problem

Check:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

## License

This project is licensed under the terms of the license file included in this repository.

## Author

Kasiphat Uppaphak  
GitHub: [wattanatum](https://github.com/wattanatum)
