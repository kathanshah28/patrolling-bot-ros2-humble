# Patrolling Bot ROS2 Humble

A ROS2 Humble project for autonomous patrol navigation using a robot and Nav2. It includes a waypoint recording utility and a patrol node that navigates through saved poses in a loop.

## Overview

This repository contains two ROS2 packages:

- `patrol_robot`: runs the autonomous patrolling logic using Nav2's `BasicNavigator`
- `waypoint_saver`: records the robot's current pose as a waypoint and saves it to a JSON file

The workflow is:

1. Start ROS2 simulation and navigation stack
2. Drive the robot to desired locations
3. Save waypoints using the `waypoint_saver` node
4. Run the `patrol_robot` node to navigate through the saved route repeatedly

## Repository Structure

```text
patrolling-bot-ros2-humble/
├── README.md
├── .gitignore
├── .gitmodules
├── src/
│   ├── patrol_robot/
│   │   ├── patrol_robot/
│   │   │   ├── __init__.py
│   │   │   └── patrol.py
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   ├── setup.py
│   │   └── resource/
│   ├── waypoint_saver/
│   │   ├── waypoint_saver/
│   │   │   ├── __init__.py
│   │   │   └── save_waypoints.py
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   ├── setup.py
│   │   └── resource/
│   └── turtlebot3_simulations/   # simulation dependency/submodule
```

## Dependencies

This project depends on the following components:

### ROS2 packages
- `rclpy`
- `geometry_msgs`
- `nav_msgs`
- `std_msgs`
- `nav2`
- `nav2_simple_commander`
- `amcl`
- `turtlebot3_bringup`
- `turtlebot3_gazebo` (simulation)

### Python packages / libraries
- Python 3
- `setuptools`
- Standard library modules: `json`, `os`, `threading`, `sys`, `termios`, `tty`

## Prerequisites

- Ubuntu 22.04 or newer
- ROS2 Humble installed
- `colcon` build tool installed
- TurtleBot3 packages available in the workspace or environment

## Installation

1. Create a ROS2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone the repository:

```bash
git clone https://github.com/kathanshah28/patrolling-bot-ros2-humble.git
```

3. Update submodules if present:

```bash
cd ~/ros2_ws/src/patrolling-bot-ros2-humble
git submodule update --init --recursive
```

4. Build the workspace:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Required ROS2 Setup

To run the robot in a simulated environment, start the TurtleBot3 simulation and Nav2 stack.

### Terminal 1: Gazebo simulation

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2: TurtleBot3 bringup / localization stack

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_bringup robot.launch.py use_sim_time:=true
```

### Terminal 3: Nav2 navigation

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
```

If your setup uses a different launch file structure, match the equivalent TurtleBot3 + Nav2 bringup for ROS2 Humble.

## Recording Waypoints

The `waypoint_saver` node records the robot's current pose and saves it to a JSON file.

### Run the waypoint saver

```bash
source ~/ros2_ws/install/setup.bash
ros2 run waypoint_saver save_waypoints
```

### Controls
- Press `s` to save the current pose
- Press `q` to quit

### Output file

The script writes waypoints to:

```bash
~/waypoints.json
```

Example output:

```json
[
  {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0
  },
  {
    "x": 1.5,
    "y": 2.0,
    "z": 0.0,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.707,
    "qw": 0.707
  }
]
```

## Running the Patrol Robot

Once the waypoints have been recorded, launch the patrol node:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run patrol_robot patrol
```

### Behavior

The patrol script:

- loads waypoints from `~/waypoints.json`
- converts each waypoint into a `PoseStamped`
- sets the initial pose based on the first point
- waits for Nav2 to become active
- navigates to each waypoint in sequence
- reverses the route after reaching the end
- repeats continuously until shutdown

## Script Details

### `src/waypoint_saver/waypoint_saver/save_waypoints.py`

This node subscribes to `/amcl_pose` and records the current robot pose when the user presses `s`.

Key features:
- listens for ROS2 pose updates
- reads keyboard input in a background thread
- saves each waypoint in JSON format
- stores data under the home directory as `~/waypoints.json`

### `src/patrol_robot/patrol_robot/patrol.py`

This node loads the saved waypoints and uses `nav2_simple_commander.BasicNavigator` to navigate through the route.

Key features:
- loads the saved waypoint list from JSON
- creates `PoseStamped` goals from waypoint values
- calls `navigator.goToPose(goal)` for each target
- waits until navigation is complete
- reverses the route for the return trip

## Package Metadata

Both packages use a standard ROS2 Python package layout and define their entry points in `setup.py`.

### `waypoint_saver`

```python
entry_points={
    'console_scripts': [
        'save_waypoints = waypoint_saver.save_waypoints:main',
    ],
}
```

### `patrol_robot`

```python
entry_points={
    'console_scripts': [
        'patrol = patrol_robot.patrol:main',
    ],
}
```

## Troubleshooting

### Nav2 or AMCL does not start
- Make sure the robot simulation is running
- Check that your TurtleBot3 model is exported correctly
- Verify that ROS2 environment variables are sourced

### Waypoints file is missing
- Run the `save_waypoints` node before `patrol`
- Confirm the file exists at `~/waypoints.json`

### Navigation fails at a waypoint
- Ensure the target pose is reachable in the map
- Check for obstacles or localization issues
- Verify the map and AMCL are initialized
